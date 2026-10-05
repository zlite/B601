import tempfile
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch
import numpy as np
from plate_carry import check_retained,carry_and_return


class CarryTests(unittest.TestCase):
    def test_retention_rejects_slip_rotation_and_invalid_pose(self):
        reference=np.eye(4)
        self.assertEqual(check_retained(reference,reference)['relative_translation_m'],0.)
        slipped=reference.copy();slipped[0,3]=.003
        turned=reference.copy();a=np.radians(4);turned[:2,:2]=[[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]]
        for bad in (slipped,turned,np.full((4,4),np.nan),np.eye(3)):
            with self.assertRaises(ValueError):check_retained(reference,bad)

    def trial(self,fail_lateral=False):
        zero=[0.]*6;up=[1.]*6;side=[2.]*6;events=[]
        r=SimpleNamespace(targets=dict(enumerate(zero)),rows=[{'raw_deg':zero}],limits={},
            low=np.full(6,-10.),high=np.full(6,10.),speed=32.,acceleration=288.,pacing_lag=2.4,
            vision_ready=lambda:True,tick=lambda:None)
        def blend(points,visited,returning=False):
            events.append(('reverse' if returning else 'forward',points))
            if fail_lateral and not returning and points[-1]==side:
                partial=[1.5]*6;r.targets=dict(enumerate(partial));visited.append(partial)
                raise ValueError('Injected vision loss')
            r.targets=dict(enumerate(points[-1]))
            if not returning:visited.append(points[-1])
            return {}
        def retrace(points):
            events.append(('partial',points));r.targets=dict(enumerate(points[-1]));return {}
        def settle(goal,returning=False):self.assertEqual(list(r.targets.values()),goal)
        r.blend=blend;r.retrace=retrace;r.settle=settle
        w=SimpleNamespace(holder_evidence=list(range(10)),geometry=None)
        observer=SimpleNamespace(surface_required=True,get=lambda:{'T_camera_b_plate':np.eye(4).tolist(),'surface':{}})
        with tempfile.TemporaryDirectory() as path, \
            patch('plate_carry.table_geometry',return_value=(np.zeros(3),np.array([0,0,1]),np.array([0,1,0]))), \
            patch('plate_carry.cartesian_segment',side_effect=[([zero,up],.03),([up,side],.04)]), \
            patch('plate_carry.checked_blend_segments',side_effect=lambda points,*args:[points]):
            result=carry_and_return(r,w,observer,Path(path),lambda name:None)
        self.assertTrue(result['returned_to_source']);self.assertEqual(r.speed,32.)
        return result,events

    def test_complete_carry_reverses_each_original_segment(self):
        result,events=self.trial()
        self.assertTrue(result['destination_reached'])
        reverse=[points for kind,points in events if kind=='reverse']
        self.assertEqual(reverse,[[[2.]*6,[1.]*6],[[1.]*6,[0.]*6]])

    def test_vision_stop_retraces_partial_before_completed_segment(self):
        result,events=self.trial(True)
        self.assertFalse(result['destination_reached'])
        self.assertEqual(events[-2:], [('partial',[[1.5]*6,[1.]*6]),('reverse',[[1.]*6,[0.]*6])])
