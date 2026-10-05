import unittest
from unittest.mock import patch
from types import SimpleNamespace
from tempfile import TemporaryDirectory
from pathlib import Path
import threading
import numpy as np
from plate_recovery import (withdraw_open_gripper,check_withdrawal_load,
    at_open_recovery_anchor,check_jaw_recovery_placement,open_jaws_at_placement)


class PlacementRecoveryTests(unittest.TestCase):
    def test_known_open_anchor_accepts_small_readback_error(self):
        self.assertTrue(at_open_recovery_anchor([.1]*6,[0.]*6,-1.17))

    def test_anchor_rejects_closed_jaws_drift_and_nonfinite_pose(self):
        for pose,q in [([0.]*6,-.4),([.31,0,0,0,0,0],-1.17),
                       ([float('nan')]*6,-1.17),([0.]*6,float('nan'))]:
            with self.subTest(pose=pose,q=q):
                self.assertFalse(at_open_recovery_anchor(pose,[0.]*6,q))

    def test_jaw_recovery_rejects_moved_arm_or_plate(self):
        check_jaw_recovery_placement([0.]*6,[0.]*6,np.eye(4),np.eye(4))
        with self.assertRaisesRegex(RuntimeError,'placement pose'):
            check_jaw_recovery_placement([1.]*6,[0.]*6,np.eye(4),np.eye(4))
        moved=np.eye(4);moved[0,3]=.003
        with self.assertRaisesRegex(RuntimeError,'Plate shifted'):
            check_jaw_recovery_placement([0.]*6,[0.]*6,np.eye(4),moved)

    def exercise(self,*,fault=False,shifted=False,opening_stalls=False):
        class Grip:
            def __init__(self):
                self.q=-.4;self.mode=1;self.timeout=10000;self.enabled=False
                self.ever_enabled=False;self.commands=[]
            def get_register_f32(self,*args):return self.q
            def get_register_u32(self,reg,*args):return self.mode if reg==10 else self.timeout
            def ensure_mode(self,mode,*args):self.mode=mode
            def write_register_u32(self,reg,value):self.timeout=value
            def request_feedback(self):pass
            def get_state(self):
                return SimpleNamespace(status_code=8 if fault else int(self.enabled),
                    vel=0.,torq=0.,t_mos=25.,t_rotor=25.)
            def send_force_pos(self,q,*args):
                self.commands.append(q)
                if self.enabled and not opening_stalls:self.q=q
            def enable(self):self.enabled=True;self.ever_enabled=True
            def disable(self):self.enabled=False
        self.grip=grip=Grip();clock=[0.]
        def tick():clock[0]+=.025
        runner=SimpleNamespace(gripper=grip,tick=tick,vision_ready=lambda:True,
            rows=[{'raw_deg':[0.]*6}],arm=SimpleNamespace(ctrl=SimpleNamespace(poll_feedback_once=lambda:None)))
        pose=np.eye(4)
        if shifted:pose[0,3]=.003
        context={'placed':[0.]*6,'opening_rad':-1.17,'reference_surface':np.eye(4)}
        with patch('plate_recovery.time.monotonic',side_effect=lambda:clock[0]), \
             patch('plate_surface.plate_in_surface_frame',return_value=pose):
            return open_jaws_at_placement(runner,SimpleNamespace(get=lambda:{}),context)

    def test_partial_opening_completes_and_restores_disabled_motor(self):
        result=self.exercise()
        self.assertTrue(result['opened_at_verified_placement'])
        self.assertFalse(self.grip.enabled)
        self.assertEqual(self.grip.mode,1)
        self.assertAlmostEqual(self.grip.q,-1.17,delta=.015)
        self.assertTrue(all(a>=b for a,b in zip(self.grip.commands,self.grip.commands[1:])))

    def test_faulted_motor_is_never_enabled(self):
        with self.assertRaisesRegex(RuntimeError,'no fault reset'):self.exercise(fault=True)
        self.assertFalse(self.grip.ever_enabled)

    def test_unseated_plate_is_never_released(self):
        with self.assertRaisesRegex(RuntimeError,'Plate shifted'):self.exercise(shifted=True)
        self.assertFalse(self.grip.ever_enabled)

    def test_stalled_opening_times_out_and_restores_motor(self):
        with self.assertRaisesRegex(RuntimeError,'timed out'):self.exercise(opening_stalls=True)
        self.assertFalse(self.grip.enabled)
        self.assertEqual(self.grip.mode,1)

    def test_verified_clear_return_does_not_need_wells_at_anchor(self):
        from plate_recovery import recover_to_standoff
        clock=[0.];anchor=[1.]*6;views=[]
        w=SimpleNamespace(lock=threading.Lock(),sources={},plate_speed_scale=2.)
        runner=SimpleNamespace(speed=10.,acceleration=144.,minimum_duration=.125,vision_ready=lambda:True,rows=[],
            gripper=SimpleNamespace(get_register_f32=lambda *args:-1.17))
        def tick(*args,**kwargs):
            clock[0]+=.1
            runner.rows.append({'time':clock[0],'raw_deg':anchor,'motor_torques_nm':[0.]*6})
            for role in ('wrist','tripod'):w.sources[role]={'time':clock[0],'frame_age_s':0.}
        runner.tick=tick;runner.go=lambda goal:tick();tick()
        calls=[]
        def observation():
            calls.append(True)
            if len(calls)>1:raise ValueError('Wells have left view at anchor')
            return {'T_camera_b_plate':np.eye(4).tolist()}
        observer=SimpleNamespace(get=observation)
        with TemporaryDirectory() as folder, \
             patch('plate_recovery.time.monotonic',side_effect=lambda:clock[0]), \
             patch('plate_recovery.withdraw_open_gripper',return_value={'phase':'clear','separation_verified':True}):
            result=recover_to_standoff(runner,w,observer,Path(folder),anchor,views.append,'test')
        self.assertTrue(result['returned_to_standoff'])
        self.assertEqual(len(calls),1)
        self.assertIn('recovery_standoff',views)
        self.assertEqual((runner.speed,runner.acceleration,runner.minimum_duration),(10.,144.,.125))


class RecoveryTests(unittest.TestCase):
    def test_recorded_clear_upward_load_is_allowed(self):
        check_withdrawal_load(np.zeros(6),[.232,4.171,-2.099,.083,.01,-.078],True)
    def test_near_contact_retains_original_load_bound(self):
        with self.assertRaises(ValueError):check_withdrawal_load(np.zeros(6),[.232,4.171,-2.099,.083,.01,-.078],False)
    def test_seated_plate_allows_recorded_upward_reversal_with_absolute_cap(self):
        baseline=np.array([0.,7.39,-5.5,0.,0.,0.])
        check_withdrawal_load(baseline,baseline+[0.,4.185,-1.026,.186,.005,0.],False,True)
        for delta in ([0,6.1,0,0,0,0],[0,-4.1,0,0,0,0],[0,0,-4.1,0,0,0],[0,0,0,1.6,0,0]):
            with self.assertRaises(RuntimeError):check_withdrawal_load(baseline,baseline+delta,False,True)
        with self.assertRaisesRegex(RuntimeError,'Absolute shoulder'):
            check_withdrawal_load([0,10,0,0,0,0],[0,13.6,0,0,0,0],False,True)

    def test_stationary_check_in_recovery_rejects_shift_before_motion(self):
        from plate_recovery import recover_to_standoff
        clock=[0.];moves=[]
        runner=SimpleNamespace(speed=10.,acceleration=144.,minimum_duration=.125,
            vision_ready=lambda:True,rows=[],gripper=SimpleNamespace(get_register_f32=lambda *args:-1.17))
        def tick(*args,**kwargs):
            clock[0]+=.1
            runner.rows.append({'time':clock[0],'raw_deg':[0.]*6,'motor_torques_nm':[0.]*6})
        runner.tick=tick;runner.go=lambda goal:moves.append(goal);tick()
        w=SimpleNamespace(plate_recovery_context={'seated_plate_surface':np.eye(4).tolist()})
        pose=np.eye(4);pose[0,3]=.003
        observer=SimpleNamespace(get=lambda:{'T_camera_b_plate':pose.tolist(),
            'surface':{'T_camera_b_reference':np.eye(4).tolist()}})
        with TemporaryDirectory() as folder,patch('plate_recovery.time.monotonic',side_effect=lambda:clock[0]):
            with self.assertRaisesRegex(RuntimeError,'Plate shifted'):
                recover_to_standoff(runner,w,observer,Path(folder),[0.]*6,lambda *args:None,'test')
        self.assertEqual(moves,[])
    def test_excessive_clear_withdrawal_load_still_stops(self):
        with self.assertRaises(RuntimeError):check_withdrawal_load(np.zeros(6),[0,7,0,0,0,0],True)
    def exercise(self,open_jaws=True,fresh=True,retained=False,sideways=False,fault=False):
        position=np.array([.025,0.,.008]);moves=[];saved=[]
        def sample():
            return {'center_m':position.tolist(),'min_height_m':position[2],
                    'load_nm':[0.]*6,'jaws_open':open_jaws,'vision_fresh':fresh}
        def move(distance):
            moves.append(distance)
            if fault:raise RuntimeError('Drive feedback failed')
            if not retained:position[2]+=distance
            if sideways:position[1]+=.005
        try:return withdraw_open_gripper(sample,move,saved.append)
        finally:self.moves=moves;self.saved=saved

    def test_open_empty_gripper_rises_clear_before_return(self):
        result=self.exercise()
        self.assertEqual(result['phase'],'clear')
        self.assertTrue(result['separation_verified'])
        self.assertGreaterEqual(result['samples'][-1]['min_height_m'],.030)
        self.assertTrue(all(x==.0025 for x in self.moves))

    def test_closed_jaws_never_move(self):
        with self.assertRaisesRegex(RuntimeError,'open jaws'):self.exercise(open_jaws=False)
        self.assertEqual(self.moves,[])

    def test_stale_camera_never_moves(self):
        with self.assertRaisesRegex(RuntimeError,'fresh cameras'):self.exercise(fresh=False)
        self.assertEqual(self.moves,[])

    def test_retained_plate_stops_after_bounded_probe(self):
        with self.assertRaisesRegex(RuntimeError,'possible retained plate'):self.exercise(retained=True)
        self.assertAlmostEqual(sum(self.moves),.005)

    def test_sideways_motion_aborts_before_second_step(self):
        with self.assertRaisesRegex(RuntimeError,'vertical corridor'):self.exercise(sideways=True)
        self.assertEqual(len(self.moves),1)

    def test_drive_fault_is_not_retried(self):
        with self.assertRaisesRegex(RuntimeError,'Drive feedback'):self.exercise(fault=True)
        self.assertEqual(len(self.moves),1)
