import json
import math
from pathlib import Path
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

from arm_geometry import Geometry
from axis_follow import AxisArm, AxisWorkbench, PROFILES, axis_bounds, raw_signs, starting_targets, AxisGate, ClutchedTarget, ReferencedTarget
from wrist_follow import WristTrajectory

PAIRING = Path('calibration/leader_pairing_20261002T202428971124Z.json')


class BoundsTests(unittest.TestCase):
    def test_idle_boundary_jitter_never_expands_commanded_range(self):
        limits={1:(46.,56.),3:(-15.,15.)}
        q=[0.,56.005,0.,-15.05,0.,0.]
        self.assertEqual(starting_targets(q,limits),{1:56.,3:-15.})
        q[1]=56.11
        with self.assertRaisesRegex(RuntimeError,'Shoulder outside starting envelope'):
            starting_targets(q,limits)
        q[1]=math.nan
        with self.assertRaises(RuntimeError):starting_targets(q,limits)

    def test_shoulder_near_upper_limit_can_only_move_inward(self):
        g = Geometry()
        upper_raw = -math.degrees(g.offsets[1])
        low,high = axis_bounds(g,1,upper_raw+.4)
        self.assertAlmostEqual(high,upper_raw+.4)
        self.assertAlmostEqual(low,upper_raw+.4-PROFILES[1]["range"])
        trajectory = WristTrajectory(high,high,low=low,high=high,speed=6,acceleration=20)
        for _ in range(100): self.assertEqual(trajectory.step(high+10,.05),high)
        for _ in range(100): trajectory.step(low-10,.05)
        self.assertTrue(low<=trajectory.position<high)
        with self.assertRaises(ValueError): axis_bounds(g,1,upper_raw+1.1)

    def test_all_six_profiles_have_bounded_smooth_motion(self):
        g = Geometry()
        for i,cfg in enumerate(PROFILES):
            model = -30 if i in (1,2) else 0
            raw = (model-math.degrees(g.offsets[i]))/g.signs[i]
            low,high = axis_bounds(g,i,raw)
            t = WristTrajectory(raw,raw,low=low,high=high,speed=cfg['speed'],acceleration=cfg['acceleration'],response_time=.08)
            for k in range(500):
                old,vel=t.position,t.velocity
                t.step(raw+(100 if k%100<50 else -100),.05)
                self.assertTrue(low-1e-8<=t.position<=high+1e-8)
                self.assertLessEqual(abs(t.position-old),cfg['speed']*.05+1e-8)
                self.assertLessEqual(abs(t.velocity-vel),cfg['acceleration']*.05+1e-8)


class AxisHardwareTests(unittest.TestCase):
    def arm(self,i):
        arm=AxisArm();arm.ctrl=Mock();arm.motors=[Mock() for _ in range(6)]
        arm._handles=arm.motors[:];arm.read=Mock(return_value=[0.]*6)
        registers={9:0,10:1,25:.1,26:.2,27:60.,28:1.}
        for motor in arm.motors:
            state=registers.copy()
            motor.get_register_u32.side_effect=lambda rid,*_,s=state:s[rid]
            motor.get_register_f32.side_effect=lambda rid,*_,s=state:s[rid]
            motor.write_register_u32.side_effect=lambda rid,value,s=state:s.__setitem__(rid,value)
            motor.write_register_f32.side_effect=lambda rid,value,s=state:s.__setitem__(rid,value)
            motor.set_can_timeout_ms.side_effect=lambda ms,s=state:s.__setitem__(9,20*ms)
            motor.ensure_mode.side_effect=lambda mode,*_,s=state:s.__setitem__(10,mode)
        arm.select(i)
        return arm

    def test_only_selected_axis_written_and_settings_restored(self):
        for i in range(6):
            arm=self.arm(i);selected=arm.motors[i]
            arm.prepare();arm.enable(12);arm.command(13);arm.disable()
            self.assertEqual(selected.get_register_u32(10),PROFILES[i]['mode'])
            self.assertEqual(selected.get_register_u32(9),10000)
            if i==5: selected.send_mit.assert_called()
            else: selected.send_pos_vel.assert_called()
            arm.__exit__(None,None,None)
            self.assertEqual(selected.get_register_u32(10),1)
            self.assertEqual(selected.get_register_u32(9),0)
            if i<3: self.assertAlmostEqual(selected.get_register_f32(25),.1)
            for j,m in enumerate(arm.motors):
                m.store_parameters.assert_not_called()
                if j!=i:
                    m.enable.assert_not_called();m.disable.assert_not_called()
                    m.send_mit.assert_not_called();m.send_pos_vel.assert_not_called()
                    m.ensure_mode.assert_not_called();m.write_register_f32.assert_not_called()
            arm.ctrl.close_bus.assert_called_once()

    def test_transient_speed_ceiling_is_bounded_and_preserves_take_up(self):
        arm=self.arm(1);arm.command_speed_limits={1:60.}
        arm.command_axis(1,10.)
        self.assertAlmostEqual(arm.motors[1].send_pos_vel.call_args.args[1],math.radians(60.))
        arm.command_axis(1,10.,take_up=True)
        self.assertEqual(arm.motors[1].send_pos_vel.call_args.args[1],.01)
        for invalid in (0.,61.,float('nan')):
            arm.command_speed_limits={1:invalid};arm.motors[1].reset_mock()
            with self.assertRaises(ValueError):arm.command_axis(1,10.)
            arm.motors[1].send_pos_vel.assert_not_called()

    def test_partial_enable_failure_disables_selected_motor(self):
        arm=self.arm(1);arm.prepare()
        arm.motors[1].enable.side_effect=RuntimeError('transport failure')
        with self.assertRaises(RuntimeError):arm.enable(10)
        arm.__exit__(None,None,None)
        arm.motors[1].disable.assert_called_once()
        for i in (0,2,3,4,5):arm.motors[i].disable.assert_not_called()

    def test_partial_gain_write_failure_restores_prior_values(self):
        arm=self.arm(2);m=arm.motors[2]
        normal=m.write_register_f32.side_effect
        count=0
        def write(rid,value):
            nonlocal count
            count+=1
            if count==2:raise RuntimeError('write failed')
            normal(rid,value)
        m.write_register_f32.side_effect=write
        with self.assertRaises(RuntimeError):arm.prepare()
        arm.__exit__(None,None,None)
        self.assertEqual(m.get_register_f32(25),.1)
        m.enable.assert_not_called()

    def test_cannot_switch_enabled_axis(self):
        arm=self.arm(4);arm.active=True
        with self.assertRaises(ValueError):arm.select(1)

    def test_group_enables_and_disables_all_six_without_gripper(self):
        arm=self.arm(4)
        arm.prepare_group(range(6))
        arm.enable_group({i:0. for i in range(6)})
        self.assertEqual(arm.enabled_indices,set(range(6)))
        for m in arm.motors:m.enable.assert_called_once()
        arm.disable()
        for m in arm.motors:m.disable.assert_called_once()
        self.assertFalse(arm.active)
        arm.__exit__(None,None,None)
        for m in arm.motors:
            self.assertEqual(m.get_register_u32(9),0)
            self.assertEqual(m.get_register_u32(10),1)

    def test_partial_group_enable_cleans_up_every_attempted_motor(self):
        arm=self.arm(4)
        arm.prepare_group(range(6))
        arm.motors[2].enable.side_effect=RuntimeError('enable failed')
        with self.assertRaises(RuntimeError):arm.enable_group({i:0. for i in range(6)})
        arm.__exit__(None,None,None)
        for m in arm.motors[:3]:m.disable.assert_called_once()
        for m in arm.motors[3:]:m.enable.assert_not_called()

    def test_disable_failure_does_not_prevent_other_motors_stopping(self):
        arm=self.arm(4)
        arm.enable_group({i:0. for i in range(6)})
        arm.motors[1].disable.side_effect=RuntimeError('USB error')
        with self.assertRaises(RuntimeError):arm.disable()
        for m in arm.motors:m.disable.assert_called_once()
        self.assertEqual(arm.enabled_indices,{1})

    def test_hold_all_commands_stationary_axes_instead_of_disabling(self):
        arm=self.arm(4)
        targets={i:float(i) for i in range(6)}
        arm.enable_group(targets)
        targets[4]+=1
        arm.command_group(targets)
        for i in range(5):
            self.assertAlmostEqual(arm.motors[i].send_pos_vel.call_args.args[0],math.radians(targets[i]))
        self.assertAlmostEqual(arm.motors[5].send_mit.call_args.args[0],math.radians(5))
        arm.disable()


class SelectionTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.path=Path(self.temp.name)/'pairing.json'
        self.path.write_bytes(PAIRING.read_bytes())
        self.w=AxisWorkbench(self.path)
        self.w.record=Mock()

    def tearDown(self):self.temp.cleanup()

    def test_direction_can_be_reversed_and_saved_without_changing_other_axes(self):
        before=self.w.follow_signs[:]
        self.w.action({'action':'reverse','revision':0})
        self.assertEqual(self.w.follow_signs[:4],before[:4])
        self.assertEqual(self.w.follow_signs[4],-before[4])
        self.assertEqual(self.w.follow_signs[5],before[5])
        saved=json.loads(self.path.read_text())
        self.assertFalse(saved['relative_follow_checks']['5']['corrected_direction_physically_confirmed'])
        with self.assertRaises(ValueError):self.w.action({'action':'confirm_direction','revision':1})
        self.w.observed.add(4)
        self.w.action({'action':'confirm_direction','revision':1})
        self.assertTrue(json.loads(self.path.read_text())['relative_follow_checks']['5']['corrected_direction_physically_confirmed'])

    def test_switch_reverse_and_confirm_blocked_during_hold(self):
        self.w.gate.action({'action':'press','client':'a','seq':1},time.monotonic())
        for op in ('select','reverse','confirm_direction','set_control_mode'):
            with self.assertRaises(ValueError):self.w.action({'action':op,'revision':0,'joint':2})

    def test_mode_change_invalidates_old_browser_press(self):
        self.w.action({'action':'set_control_mode','control_mode':'follow_all','revision':0})
        self.assertEqual(self.w.control_mode,'follow_all')
        self.assertIsNone(self.w.bounds)
        with self.assertRaises(ValueError):
            self.w.action({'action':'press','revision':0,'joint':5,'client':'a','seq':1,'clearance_confirmed':True})
        self.assertIsNone(self.w.gate.lease)

    def test_stale_browser_cannot_enable_wrong_joint(self):
        self.w.action({'action':'select','revision':0,'joint':2})
        with self.assertRaises(ValueError):
            self.w.action({'action':'press','revision':0,'joint':5,'client':'a','seq':1,'clearance_confirmed':True})
        self.assertIsNone(self.w.gate.lease)

    def test_no_motion_on_startup_and_release_during_setup_cancels_enable(self):
        for cancel in (False,True):
            w=AxisWorkbench(self.path);w.record=Mock()
            w.publish('leader',angles=[0.]*7);w.publish('wrist');w.publish('tripod')
            arm=Mock();arm.active=False;arm.selected=4
            arm.__enter__=Mock(return_value=arm);arm.__exit__=Mock()
            arm.select.side_effect=lambda i:setattr(arm,'selected',i)
            n=0
            def read():
                nonlocal n
                n+=1
                if n==2:
                    if cancel:w.gate.action({'action':'press','client':'a','seq':1},time.monotonic())
                    else:w.stop.set()
                return [0.]*6
            arm.read.side_effect=read
            def prepare():
                w.gate.action({'action':'release','client':'a','seq':2},time.monotonic())
                w.stop.set()
            arm.prepare_group.side_effect=lambda indices:prepare()
            w.motor_worker(lambda:arm)
            arm.enable_group.assert_not_called()
            if not cancel:arm.prepare_group.assert_not_called()

    def test_invalid_disabled_start_keeps_reading_and_recovers_without_enable(self):
        w=self.w;w.record=Mock();w.cameras_ready=Mock(return_value=True)
        arm=Mock();arm.active=False;arm.selected=4
        arm.__enter__=Mock(return_value=arm);arm.__exit__=Mock()
        arm.select.side_effect=lambda i:setattr(arm,'selected',i)
        count=0;blocked=[]
        def read():
            nonlocal count
            count+=1
            if count in (2,3):blocked.append((w.ready,w.gate.lease,w.fault))
            if count==4:w.stop.set()
            w.publish('leader',angles=[0.]*7)
            return [0.]*6
        def bounds(*args):
            if count<3:raise ValueError('Shoulder starting pose is outside its model limits')
            return (-5.,5.)
        arm.read.side_effect=read
        w.gate.action({'action':'press','client':'a','seq':1},time.monotonic())
        with patch('axis_follow.axis_bounds',side_effect=bounds):w.motor_worker(lambda:arm)
        self.assertEqual(count,4)
        self.assertEqual(blocked,[(False,None,None),(False,None,None)])
        self.assertIsNone(w.fault)
        self.assertIsNotNone(w.bounds)
        arm.enable_group.assert_not_called();arm.prepare_group.assert_not_called()

    def test_all_follow_accepts_coupled_motion_and_release_keeps_torque(self):
        w=self.w;w.control_mode='follow_all'
        g=w.geometry
        q=[(float(-20 if i in (1,2) else 0)-math.degrees(g.offsets[i]))/g.signs[i] for i in range(6)]
        initial=q[:]
        arm=Mock();arm.active=False;arm.selected=4;arm.joint_velocities_deg_s=[0.]*6
        arm.__enter__=Mock(return_value=arm);arm.__exit__=Mock()
        arm.select.side_effect=lambda i:setattr(arm,'selected',i)
        tick=100.;count=0
        def command(targets,**kwargs):
            for i,v in targets.items():q[i]=v
        def enable(targets):
            arm.active=True;command(targets)
        arm.enable_group.side_effect=enable
        arm.command_group.side_effect=command
        arm.disable.side_effect=lambda:setattr(arm,'active',False)
        def read():
            nonlocal tick,count
            tick+=.05;count+=1
            delta=min(4.,max(0.,(count-20)*.3))
            w.publish('leader',angles=[delta,-delta,-delta,delta,delta,delta,0])
            w.publish('wrist');w.publish('tripod')
            if count==2:w.gate.action({'action':'press','client':'a','seq':1},tick)
            elif w.gate.lease:w.gate.lease['heartbeat']=tick
            if count==38:w.gate.lease=None
            if count>=41:w.stop.set()
            return q[:]
        arm.read.side_effect=read
        with patch('time.monotonic',side_effect=lambda:tick):w.motor_worker(lambda:arm)
        self.assertIsNone(w.fault)
        self.assertEqual(set(arm.enable_group.call_args.args[0]),set(range(6)))
        self.assertGreater(abs(q[0]-initial[0]),1)
        self.assertGreater(abs(q[4]-initial[4]),1)
        arm.disable.assert_not_called()  # mock context deliberately does no shutdown cleanup
        self.assertTrue(arm.active)
        self.assertTrue(any(c.args[0]=='paused' for c in w.record.call_args_list))




class ClutchTests(unittest.TestCase):
    def test_excess_travel_discarded_and_reverse_immediate_both_signs(self):
        for sign in (-1,1):
            t=ClutchedTarget(0,0,sign,-10,10)
            self.assertEqual(t.update(100*sign),10)
            self.assertTrue(t.limited)
            self.assertEqual(t.update(200*sign),10)
            self.assertEqual(t.update(199*sign),9)
            self.assertFalse(t.limited)
            with self.assertRaises(RuntimeError):t.update(math.nan)

    def test_gate_reason_survives_late_release_and_expiry(self):
        g=AxisGate()
        g.action({'action':'press','client':'a','seq':1},0)
        g.action({'action':'release','client':'a','seq':3,'reason':'window_blur'},.1)
        g.action({'action':'release','client':'a','seq':2,'reason':'old'},.2)
        self.assertEqual(g.reason,'window_blur')
        g.action({'action':'press','client':'a','seq':4},.3)
        self.assertFalse(g.valid(.8))
        self.assertEqual(g.reason,'heartbeat_expired')
        g.action({'action':'heartbeat','client':'a','seq':5,'press_id':4},.9)
        self.assertIsNone(g.lease)


class ControllerTransitionsTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        path=Path(self.temp.name)/'pairing.json';path.write_bytes(PAIRING.read_bytes())
        self.w=AxisWorkbench(path);self.w.record=Mock()
        self.w.stop.wait=Mock(return_value=False)
        self.tick=100.;self.count=0;self.seq=0
        self.q=[(-90. if i in (1,2) else 0.)-math.degrees(self.w.geometry.offsets[i]) for i in range(6)]
        self.commands=[];self.states=[]
        self.arm=Mock();self.arm.active=False;self.arm.joint_velocities_deg_s=[0.]*6
        self.arm.__enter__=Mock(return_value=self.arm)
        self.arm.__exit__=Mock(side_effect=lambda *_:self.arm.disable())
        self.arm.disable.side_effect=lambda:setattr(self.arm,'active',False)
        def command(targets,**kw):
            self.commands.append((self.count,dict(targets)))
            for j,value in targets.items():self.q[j]=value
        def enable(targets):
            self.arm.active=True;command(targets)
        self.arm.command_group.side_effect=command;self.arm.enable_group.side_effect=enable

    def tearDown(self):self.temp.cleanup()

    def request(self,action,**kw):
        self.seq+=1
        self.w.action(dict(action=action,client='test',seq=self.seq,joint=5,revision=self.w.revision,
                           clearance_confirmed=True,**kw))

    def run_scenario(self,scenario,stop=75):
        def read():
            self.tick+=.05;self.count+=1
            self.states.append((self.count,self.w.powered,self.w.active_id,self.w.pause_reason))
            self.w.publish('leader',angles=[max(0,self.count-20)*.2]*7)
            self.w.publish('wrist');self.w.publish('tripod')
            if self.count==2:self.request('press')
            if self.w.gate.lease:self.w.gate.lease['heartbeat']=self.tick
            scenario(self.count)
            if self.count>=stop:self.w.stop.set()
            return self.q[:]
        self.arm.read.side_effect=read
        with patch('time.monotonic',side_effect=lambda:self.tick):self.w.motor_worker(lambda:self.arm)

    def events(self,name):return [c.kwargs for c in self.w.record.call_args_list if c.args[0]==name]

    def test_reference_preserving_mapper_survives_limit_in_motor_loop(self):
        self.w.target_mapper = ReferencedTarget
        self.w.response_time = .04
        initial = self.q[3]
        def scenario(n):
            leader = [0.]*7
            if 20 <= n < 70:
                leader[3] = 80.  # Exceeds the unchanged 45 degree local limit.
            self.w.publish('leader', angles=leader)
        self.run_scenario(scenario, stop=180)
        self.assertIsNone(self.w.fault)
        self.assertAlmostEqual(self.q[3], initial, places=4)
        self.assertLessEqual(max(abs(target[3]-initial) for _,target in self.commands),45.+1e-8)
        self.arm.enable_group.assert_called_once()

    def test_release_holds_despite_moving_leader_resume_rebases_without_reenable(self):
        def scenario(n):
            if n==30:self.request('release',reason='pointerup')
            if n==50:self.request('press')
        self.run_scenario(scenario)
        self.assertIsNone(self.w.fault)
        held=[v for n,v in self.commands if 31<=n<50]
        self.assertGreater(len(held),10)
        self.assertTrue(all(v==held[0] for v in held))
        self.arm.enable_group.assert_called_once()
        self.arm.disable.assert_called_once()  # server shutdown only
        self.assertEqual(self.events('paused')[0]['reason'],'pointerup')
        resumed=self.events('resumed')[0]
        self.assertAlmostEqual(resumed['leader_baseline'][0],6.)
        self.assertEqual({j:resumed['follower_baseline'][j] for j in range(6)},held[0])
        self.assertFalse(self.w.powered)

    def test_expired_heartbeat_pauses_and_recovery_does_not_resume(self):
        def scenario(n):
            if n==30:self.w.gate.lease['heartbeat']=self.tick-1
        self.run_scenario(scenario)
        self.assertIsNone(self.w.fault)
        self.assertEqual(self.events('paused')[0]['reason'],'heartbeat_expired')
        self.assertFalse(self.events('resumed'))
        self.assertTrue(all(active is None for n,powered,active,reason in self.states if n>30))
        self.assertTrue(all(powered for n,powered,active,reason in self.states if n>30))

    def test_stale_inputs_pause_without_torque_off_or_automatic_resume(self):
        for source,reason in [('leader','leader_stale'),('wrist','camera_stale')]:
            with self.subTest(source=source):
                # Each subcase gets fresh controller and motor state.
                self.tearDown();self.setUp()
                def scenario(n):
                    if n==30:self.w.sources[source]['time']=self.tick-2
                self.run_scenario(scenario)
                self.assertIsNone(self.w.fault)
                self.assertEqual(self.events('paused')[0]['reason'],reason)
                self.assertFalse(self.events('resumed'))
                self.arm.disable.assert_called_once()  # shutdown, not input loss

    def test_explicit_supported_torque_off_disables_and_logs_cause(self):
        def scenario(n):
            if n==30:
                with self.assertRaises(ValueError):self.request('disable')
                self.assertFalse(self.w.disable_requested)
                self.request('disable',support_confirmed=True)
        self.run_scenario(scenario)
        self.assertIsNone(self.w.fault)
        self.assertEqual(self.events('disabled')[0]['reason'],'operator_torque_off')
        self.assertFalse(any(powered for n,powered,*_ in self.states if n>31))
        self.arm.enable_group.assert_called_once()

    def test_motor_feedback_failure_disables_with_explicit_fault_record(self):
        def scenario(n):
            if n==30:raise RuntimeError('Joint 3 unexpected status')
        self.run_scenario(scenario)
        self.assertIn('Joint 3',self.w.fault)
        self.arm.disable.assert_called_once()
        self.assertEqual(self.events('shutdown_requested')[0]['reason'],'motor_or_control_fault')
        self.assertTrue(self.events('fault')[0]['torque_off_attempted'])

    def test_primary_error_survives_cleanup_error_and_logging_follows_cleanup(self):
        events=[]
        self.w.run_motor=Mock(side_effect=RuntimeError('feedback deadline'))
        self.w.record=Mock(side_effect=lambda *a,**k:events.append('log'))
        self.w.after_motor_shutdown=Mock(side_effect=lambda:events.append('reports'))
        def close(*args):
            events.append('disable')
            self.arm.active=False
            raise RuntimeError('secondary status fault')
        self.arm.__exit__.side_effect=close
        self.w.powered=True
        self.w.motor_worker(lambda:self.arm)
        self.assertEqual(events[0],'disable')
        self.assertEqual(events[-1],'reports')
        self.assertEqual(self.w.fault,'feedback deadline')
        self.assertEqual(self.events('fault')[0]['cleanup_error'],'secondary status fault')
        self.assertIsNone(self.w.powered)
        self.assertFalse(self.w.ready)

    def test_configuration_blocked_during_powered_pause(self):
        def scenario(n):
            if n==30:self.request('release',reason='escape')
            if n==35:
                for op in ('select','reverse','confirm_direction','set_control_mode'):
                    with self.assertRaises(ValueError):self.request(op)
        self.run_scenario(scenario)
        self.assertIsNone(self.w.fault)

    def test_startup_settings_do_not_block_http_cancellation(self):
        import threading
        def prepare(_):
            t=threading.Thread(target=lambda:self.request('release',reason='setup_cancelled'))
            t.start();t.join(timeout=1)
            self.assertFalse(t.is_alive(), 'setup must not hold the HTTP lock')
        self.arm.prepare_group.side_effect=prepare
        self.run_scenario(lambda n:None,stop=10)
        self.assertIsNone(self.w.fault)
        self.arm.enable_group.assert_not_called()


if __name__=='__main__':unittest.main()
