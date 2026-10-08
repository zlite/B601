import tempfile
import time
from pathlib import Path
import unittest
from unittest.mock import Mock, patch
import numpy as np
from blended_motion import BlendedPath
from replay_timing import TimedCurve, derivative_bounds, ACCELERATION_LIMITS
from printer_demo_replay import ReversedCurve
from printer_teach import TeachWorkbench

PAIRING=Path('calibration/leader_pairing_20261002T202428971124Z.json')

class TimingTests(unittest.TestCase):
    def test_bounds_cover_dense_analytic_samples_and_time_scaling_keeps_geometry(self):
        points=np.array([[0]*6,[2,4,-1,8,1,2],[5,-3,2,10,-2,3],[8,1,0,-1,4,0]],float)
        path=BlendedPath(points,3,9,max_deviation=10);v,a=derivative_bounds(path)
        u=np.linspace(0,1,20001);s=u**3*(10-15*u+6*u*u)
        sp=30*u*u*(1-u)**2; spp=60*u*(1-u)*(1-2*u)
        actual_v=path.spline(s,1)*sp[:,None]/path.duration
        actual_a=(path.spline(s,2)*sp[:,None]**2+path.spline(s,1)*spp[:,None])/path.duration**2
        self.assertTrue(np.all(np.max(abs(actual_v),axis=0)<=v+1e-9))
        self.assertTrue(np.all(np.max(abs(actual_a),axis=0)<=a+1e-9))
        timed=TimedCurve(path,4)
        self.assertTrue(np.all(timed.acceleration_bounds<=ACCELERATION_LIMITS*.95+1e-8))
        self.assertLessEqual(timed.scale,4)
        reverse=ReversedCurve(timed,timed.duration)
        for f in np.linspace(0,1,101):
            np.testing.assert_allclose(timed.at(f*timed.duration),path.at(f*path.duration),atol=1e-10)
            np.testing.assert_allclose(reverse.at(f*timed.duration),timed.at((1-f)*timed.duration),atol=1e-10)
        for bad in (0,4.1,float('nan'),float('inf')):
            with self.assertRaises(ValueError):TimedCurve(path,bad)

class DemoTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.addCleanup(self.tmp.cleanup)
        self.w=TeachWorkbench(PAIRING,Path(self.tmp.name)/'recording')
        d=self.w.demo;d.config={'enabled':True};d.expected=np.zeros(6)
        self.w.cameras_ready=Mock(return_value=True)
        self.w.publish('follower',angles=[0]*6)
        self.w.rail.connected=True;self.w.rail.state='Idle';self.w.rail.error=None
        self.w.rail.status_time=time.monotonic();self.w.rail.x_mm=0;self.w.rail.commands=0
        self.seq=0
    def action(self,op,**kw):
        self.seq+=1
        self.w.action(dict(action=op,client='test',seq=self.seq,**kw))
    def start(self):self.action('demo_start',rest_confirmed=True,scene_unchanged=True,rail_clear_confirmed=True)
    def test_confirmation_stale_pose_rail_and_motor_interlocks(self):
        self.assertIsNone(self.w.demo.reason())
        with self.assertRaisesRegex(ValueError,'Confirm'):self.action('demo_start')
        self.w.publish('follower',angles=[0,1,0,0,0,0])
        with self.assertRaisesRegex(ValueError,'resting'):self.start()
        self.w.publish('follower',angles=[0]*6);self.w.powered=True
        with self.assertRaisesRegex(ValueError,'motor power'):self.start()
        self.w.powered=False;self.w.rail.commands=1
        with self.assertRaisesRegex(ValueError,'Rail'):self.start()
    def test_demo_excludes_follow_and_rail_and_cancel_is_before_enable(self):
        self.start();self.assertTrue(self.w.demo.pending)
        for action in ('press','rail_start','select','realign'):
            with self.assertRaisesRegex(ValueError,'demo'):self.action(action)
        self.action('demo_pause');self.assertFalse(self.w.demo.busy())
    def test_heartbeat_requires_owner_and_expiry_blocks_motion(self):
        self.start();d=self.w.demo;d.heartbeat-=1
        self.assertFalse(d.attended())
        self.w.action(dict(action='demo_heartbeat',client='other',seq=1))
        self.assertFalse(d.attended())
        self.action('demo_heartbeat');self.assertTrue(d.attended())
        with self.assertRaisesRegex(ValueError,'Stale'):self.w.action(dict(action='demo_start',client='test',seq=self.seq))
    def test_supported_disable_and_explicit_pause_resume(self):
        self.start();d=self.w.demo;d.pending=False;d.active=True;d.runner=Mock();d.paused=True
        with self.assertRaises(ValueError):self.action('disable',support_confirmed=False)
        with self.assertRaisesRegex(ValueError,'Confirm'):self.action('demo_resume')
        self.action('demo_return',clearance_confirmed=True);self.assertEqual(d.decision,'return')
        self.action('disable',support_confirmed=True);self.assertTrue(d.stop_requested)
    def test_repeat_runs_next_cycle_and_stop_finishes_at_rest(self):
        self.start();arm=Mock();arm.active=False;arm.read.return_value=[0]*6
        cycles=[]
        def cycle(_):
            cycles.append(self.w.demo.cycle)
            if len(cycles)==2:self.action('demo_stop')
        clock=[100.]
        def now():clock[0]+=.25;return clock[0]
        with patch.object(self.w.demo,'prepare_demo'), patch.object(self.w.demo,'rail_cycle',return_value=True) as rail, patch.object(self.w.demo,'_arm_cycle',side_effect=cycle), patch.object(self.w.demo,'attended',return_value=True), patch('printer_demo_ui.time.monotonic',side_effect=now), patch('printer_demo_ui.time.sleep'):
            self.w.demo.run_pending(arm)
        self.assertEqual(cycles,[1,2]);self.assertEqual(rail.call_count,2)
        self.assertFalse(self.w.demo.busy());self.assertFalse(self.w.powered)

    def test_rest_capture_uses_actual_postrail_pose_without_fixed_wait(self):
        self.start();self.w.rail.busy=Mock(return_value=False)
        arm=Mock();arm.active=False;arm.read.return_value=[0.,0.,0.,0.,.4,-.4]
        with patch.object(self.w.demo,'attended',return_value=True), patch('printer_demo_ui.time.sleep') as sleep:
            q=self.w.demo.settled_disabled_pose(arm)
        self.assertEqual(q,arm.read.return_value);arm.read.assert_called_once();sleep.assert_not_called()
        arm.enable_group.assert_not_called()
        self.w.rail.busy.return_value=True
        with self.assertRaisesRegex(ValueError,'Rail must be stopped'):self.w.demo.settled_disabled_pose(arm)

    def test_plan_failure_never_enables_and_restores_worker(self):
        self.start();arm=Mock();arm.active=False;arm.speed_limits={};arm.read.return_value=[0]*6
        arm.add_motor.return_value.get_register_f32.return_value=-1.
        arm.add_motor.return_value.get_state.return_value.status_code=0
        self.w.demo.config['source']='unused'
        with patch.object(self.w.demo,'rail_cycle',return_value=True), patch.object(self.w.demo,'settled_disabled_pose',return_value=[0]*6), patch('printer_demo_ui.PreparedDemoPlan',side_effect=ValueError('clearance failed')):
            self.assertTrue(self.w.demo.run_pending(arm))
        arm.enable_group.assert_not_called();self.assertFalse(self.w.demo.busy())
        self.assertEqual(self.w.demo.error,'clearance failed')

    def test_gripper_registration_reused_across_cycles_and_restarts_with_new_owner(self):
        first=Mock();second=Mock();d=self.w.demo
        self.assertIs(d.gripper_for(first),first.add_motor.return_value)
        self.assertIs(d.gripper_for(first),first.add_motor.return_value)
        first.add_motor.assert_called_once_with(7,23,'4310')
        self.assertIs(d.gripper_for(second),second.add_motor.return_value)
        second.add_motor.assert_called_once_with(7,23,'4310')

    def test_fresh_entry_failure_stays_disabled_and_closes_prepared_checker(self):
        self.start();arm=Mock();arm.active=False;arm.speed_limits={};arm.read.return_value=[0]*6
        arm.add_motor.return_value.get_register_f32.return_value=-1.
        arm.add_motor.return_value.get_state.return_value.status_code=0
        prepared=Mock();prepared.refresh.side_effect=ValueError('fresh clearance failed')
        def prepare(_):self.w.demo.prepared=prepared
        with patch.object(self.w.demo,'prepare_demo',side_effect=prepare), patch.object(self.w.demo,'rail_cycle',return_value=True), patch.object(self.w.demo,'settled_disabled_pose',return_value=[0]*6):
            self.assertTrue(self.w.demo.run_pending(arm))
        arm.enable_group.assert_not_called();prepared.close.assert_called_once()
        self.assertEqual(self.w.demo.error,'fresh clearance failed');self.assertFalse(self.w.demo.busy())

if __name__=='__main__':unittest.main()
