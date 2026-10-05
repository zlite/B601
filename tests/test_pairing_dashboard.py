import tempfile
import struct
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import numpy as np
from leader_read import fresh_angles, monitor_request, parse_monitor
from pairing_dashboard import Workbench, direction_feedback, joint_points, stable_median


class PairingTests(unittest.TestCase):
    def test_monitor_protocol_requires_complete_matching_checksummed_reply(self):
        # Near-zero angle is valid, provided the actual packet is fresh.
        payload=bytes([1])+struct.pack('<HHHHBi',12000,30,360,1800,0,-1)
        packet=bytes([5,28,22,len(payload)])+payload
        packet+=bytes([sum(packet)&255])
        buf=bytearray(b'noise'+packet[:9])
        self.assertIsNone(parse_monitor(buf,1))
        buf.extend(packet[9:])
        self.assertEqual(parse_monitor(buf,1),-.1)
        with self.assertRaises(RuntimeError): parse_monitor(bytearray(packet),2)
        bad=bytearray(packet);bad[-1]^=1
        with self.assertRaises(RuntimeError): parse_monitor(bad,1)
        self.assertEqual(monitor_request(1),bytes([18,76,22,1,1,118]))
        with self.assertRaises(ValueError): monitor_request(7)

    def test_cached_or_nonfinite_leader_reply_rejected(self):
        readings = {i: SimpleNamespace(angle_deg=i, reliable=True) for i in range(7)}
        self.assertEqual(fresh_angles(readings),list(range(7)))
        readings[2].reliable = False
        with self.assertRaises(RuntimeError): fresh_angles(readings)
        readings[2].reliable = True
        readings[2].angle_deg = float('nan')
        with self.assertRaises(RuntimeError): fresh_angles(readings)

    def test_stable_capture_requires_time_freshness_and_stillness(self):
        h=[(t,[0.]*7) for t in np.linspace(8.8,10.,10)]
        self.assertEqual(stable_median(h,10.),[0.]*7)
        with self.assertRaises(ValueError): stable_median(h,10.6)
        with self.assertRaises(ValueError): stable_median(h[-4:],10.)
        h[-1]=(10.,[1.]*7)
        with self.assertRaises(ValueError): stable_median(h,10.)

    def test_geometry_matches_existing_wrist_transform(self):
        w=Workbench()
        raw=np.array(w.geometry.profile['reference_raw_rad'])+np.array([.1,-.2,-.3,.2,.1,.2])
        model=np.degrees(raw*w.geometry.signs+w.geometry.offsets)
        shape=joint_points(w.geometry,model)
        T=w.geometry.transform(raw)
        np.testing.assert_allclose(shape['points'][-1],T[:3,3],atol=1e-9)
        np.testing.assert_allclose(shape['axes'][0],T[:3,3]+T[:3,0]*.07,atol=1e-9)

    def populate(self,w,q):
        now=time.monotonic()
        w.sources['leader']={'time':now,'angles':q}
        w.history.clear()
        w.history.extend((now-t,list(q)) for t in np.linspace(1.3,0.,12))

    def test_reference_direction_and_link_loss(self):
        w=Workbench()
        with tempfile.TemporaryDirectory() as temp:
            w.output=Path(temp)/'candidate.json'
            self.populate(w,[0.]*7)
            with self.assertRaises(ValueError): w.action({'action':'reference'})
            w.action({'action':'reference','pose_confirmed':True})
            w.action({'action':'begin_direction','joint':2})
            w.guided['began']-=2
            self.populate(w,[0.,8.,0.,0.,0.,0.,0.])
            w.action({'action':'record_direction','direction_confirmed':True})
            self.assertEqual(w.signs[1],-1)
            self.assertFalse(w.snapshot()['motion_ready'])
            w.error('leader','connection lost')
            self.assertIsNone(w.reference)
            self.assertEqual(w.checks,{})
            with self.assertRaises(ValueError): w.action({'action':'reference','pose_confirmed':True})

    def test_coupled_check_from_any_pose_and_flip_only_commits_on_confirmation(self):
        w=Workbench()
        with tempfile.TemporaryDirectory() as temp:
            w.output=Path(temp)/'candidate.json'
            w.reference=[0.]*7
            # No requirement to return to the folded pose.
            baseline=[14.,23.,-35.,18.,-12.,15.,30.]
            self.populate(w,baseline)
            w.action({'action':'begin_direction','joint':2})
            w.guided['began']-=2
            w.action({'action':'flip_direction'})
            self.assertEqual(w.signs[1],-1)
            self.assertEqual(w.guided['preview_sign'],1)
            # Larger motion elsewhere is allowed; no dominant-joint assumption.
            self.populate(w,[26.,31.,-29.,23.,-8.,12.,80.])
            self.assertTrue(w.snapshot()['direction_feedback']['ready'])
            with self.assertRaises(ValueError): w.action({'action':'record_direction'})
            w.action({'action':'record_direction','direction_confirmed':True})
            self.assertEqual(w.signs[1],1)
            self.assertEqual(w.reference,[0.]*7)
            self.assertEqual(w.checks['2']['delta_deg'],[12.,8.,6.,5.,4.,-3.])
            self.assertFalse(w.snapshot()['motion_ready'])

    def test_selected_joint_must_move_and_pause_but_other_joints_can_drift(self):
        guided={'joint':2,'baseline':[0.]*7,'began':8.}
        times=np.linspace(9.4,10.,8)
        rows=[(t,[i*3.,8.,i*2.,0.,0.,0.,i*5.]) for i,t in enumerate(times)]
        result=direction_feedback(rows,guided,10.)
        self.assertTrue(result['ready'])
        self.assertEqual(result['most_active_joint'],1)
        rows=[(t,[i*3.,0.,i*2.,0.,0.,0.,i*5.]) for i,t in enumerate(times)]
        self.assertFalse(direction_feedback(rows,guided,10.)['ready'])
        rows=[(t,[0.,8.+i,0.,0.,0.,0.,0.]) for i,t in enumerate(times)]
        self.assertFalse(direction_feedback(rows,guided,10.)['ready'])
        self.assertFalse(direction_feedback(rows,guided,11.)['ready'])

    def test_cancel_discards_preview_flip_and_stale_read_cannot_save(self):
        w=Workbench();w.reference=[0.]*7
        self.populate(w,[10.]*7)
        w.action({'action':'begin_direction','joint':1})
        w.action({'action':'flip_direction'})
        w.action({'action':'cancel_direction'})
        self.assertEqual(w.signs[0],-1)
        self.assertEqual(w.checks,{})
        self.assertIsNone(w.guided)
        w.action({'action':'begin_direction','joint':1})
        w.sources['leader']['time']-=3
        with self.assertRaises(ValueError):
            w.action({'action':'record_direction','direction_confirmed':True})

    def test_stale_models_not_displayed(self):
        w=Workbench();w.reference=[0.]*7
        w.sources['leader']={'time':time.monotonic()-3,'angles':[0.]*7}
        self.assertNotIn('leader_shape',w.snapshot())


if __name__=='__main__': unittest.main()
