import math
from pathlib import Path
import tempfile
import threading
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from axis_follow import AxisGate
from rail_jog import (RailJog,parse_status,validate_settings,jog_command,
                      LOOKAHEAD_MM,FEED_MM_MIN,ACCEL_MM_S2)


class RailTests(unittest.TestCase):
    def setUp(self):
        self.tmp=tempfile.TemporaryDirectory();self.tick=100.
        self.clock=patch('time.monotonic',side_effect=lambda:self.tick);self.clock.start()
        self.w=SimpleNamespace(recording_root=Path(self.tmp.name),powered=False,active_id=None,
            fault=None,gate=AxisGate(),geometry=SimpleNamespace(profile={'reference_raw_rad':[0.]*6}),
            sources={'follower':{'angles':[0.]*6,'time':self.tick},'wrist':{'time':self.tick},'tripod':{'time':self.tick}},
            cameras_ready=lambda:True,lock=threading.RLock(),stop=threading.Event())
        self.r=RailJog(self.w);self.r.connected=True;self.r.state='Idle';self.r.x_mm=0.;self.r.status_time=self.tick
        self.port=Mock();self.seq=0

    def tearDown(self):self.clock.stop();self.tmp.cleanup()

    def action(self,op,**kw):
        self.seq+=1
        self.r.action(dict(action=op,client='test',seq=self.seq,clearance_confirmed=True,
                           **{'direction':'left',**kw}))

    def fresh(self,dt):
        self.tick+=dt;self.r.status_time=self.tick
        for source in self.w.sources.values():source['time']=self.tick

    def test_mapping_and_required_idle_power_off(self):
        self.assertIn(b'X2.000',jog_command('left'))
        self.assertIn(b'X-2.000',jog_command('right'))
        self.assertIn(b'F1920',jog_command('left'))
        settings={1:0.,3:1.,5:0.,13:0.,20:0.,21:1.,22:0.,100:5.,110:1920.,120:40.}
        validate_settings(settings)
        for key,value in ((1,255),(21,0),(100,80),(3,0),(120,1),(120,20),(110,960)):
            with self.assertRaises(ValueError):validate_settings({**settings,key:value})
        with self.assertRaises(ValueError):parse_status('<Idle|WPos:0,0,0>')
        self.assertEqual(parse_status('<Idle|MPos:1,0,0|Pn:X>')['pins'],'X')

    def test_interlock_requires_rest_and_no_arm_power_or_follow_request(self):
        for change in ('powered','moving','unparked','stale','limit'):
            with self.subTest(change=change):
                self.w.powered=False;self.w.gate.lease=None;self.w.sources['follower']['angles']=[0.]*6
                self.r.pins='';self.w.sources['follower']['time']=self.tick
                if change=='powered':self.w.powered=True
                if change=='moving':self.w.gate.action(dict(action='press',client='arm',seq=1),self.tick)
                if change=='unparked':self.w.sources['follower']['angles'][2]=3.
                if change=='stale':self.w.sources['follower']['time']-=1.
                if change=='limit':self.r.pins='X'
                with self.assertRaises(ValueError):self.action('rail_start')
                self.port.write.assert_not_called()

    def test_release_cancels_again_after_ack_and_waits_for_idle(self):
        self.action('rail_start');self.r.tick(self.port)
        self.action('rail_stop');self.fresh(.02);self.r.tick(self.port)
        self.assertEqual(self.port.write.call_args.args[0],b'\x85')
        self.assertIsNone(self.r.cancelled_at)  # Still awaiting original ack.
        self.r.accept_line('ok');self.r.accept_line('<Jog|MPos:0.2,0,0>')
        self.fresh(.11);self.r.tick(self.port)
        self.assertIsNotNone(self.r.cancelled_at);self.assertTrue(self.r.busy())
        self.fresh(.12);self.r.accept_line('<Idle|MPos:0.4,0,0>');self.r.tick(self.port)
        self.assertFalse(self.r.busy())
        self.assertEqual(sum(c.args[0].startswith(b'$J') for c in self.port.write.call_args_list),1)

    def test_heartbeat_loss_cancels_and_cannot_be_revived_by_late_heartbeat(self):
        self.action('rail_start');press=self.seq;self.r.tick(self.port)
        self.fresh(.31);self.action('rail_heartbeat',press_id=press);self.r.tick(self.port)
        self.assertIsNone(self.r.lease)
        self.assertIn(b'\x85',[c.args[0] for c in self.port.write.call_args_list])

    def test_waits_for_ack_and_bounds_queued_distance(self):
        self.action('rail_start');self.r.tick(self.port)
        self.r.tick(self.port);self.r.tick(self.port)
        self.assertEqual(self.r.commands,1)
        for _ in range(12):
            self.r.accept_line('ok');self.r.tick(self.port)
        self.assertEqual(self.r.commands,9)
        self.assertEqual(self.r.target,LOOKAHEAD_MM)
        # Keep moving: next segment is sent during Jog, without waiting for Idle.
        self.fresh(.1);self.r.accept_line('<Jog|MPos:2.0,0,0>');self.r.tick(self.port)
        self.assertEqual(self.r.commands,10)
        self.assertLessEqual(self.r.target-self.r.x_mm,LOOKAHEAD_MM)

    def test_demo_300mm_endpoint_and_return_do_not_enqueue_past_target(self):
        for endpoint in (300.,0.):
            self.r.begin_demo_leg(endpoint)
            for _ in range(400):
                self.fresh(.05)
                if self.r.lease:self.r.lease['heartbeat']=self.tick
                self.r.tick(self.port)
                if self.r.awaiting_ack:self.r.accept_line('ok')
                target=self.r.target
                self.fresh(.01);self.r.accept_line(f'<Idle|MPos:{target},0,0>')
                self.r.tick(self.port)
                if not self.r.busy():break
            self.assertFalse(self.r.busy())
            self.assertAlmostEqual(self.r.x_mm,endpoint)
            self.assertAlmostEqual(self.r.travel,300.)
            self.assertIsNone(self.r.lease)
        self.assertEqual(self.r.commands,300)
        self.assertNotIn(b'\x85',[c.args[0] for c in self.port.write.call_args_list])

    def test_demo_endpoint_still_cancels_on_lost_heartbeat_and_rejects_long_travel(self):
        with self.assertRaises(ValueError):self.r.begin_demo_leg(301.)
        self.r.begin_demo_leg(300.);self.r.tick(self.port)
        self.fresh(.31);self.r.tick(self.port)
        self.assertIsNone(self.r.lease)
        self.assertIn(b'\x85',[c.args[0] for c in self.port.write.call_args_list])

    def run_long_hold(self,direction):
        sign=1 if direction=='left' else -1
        self.action('rail_start',direction=direction);press=self.seq
        x=0.;queued=0.;cruise=False
        for i in range(1500):
            self.fresh(.01)
            if i%10==0:self.action('rail_heartbeat',press_id=press)
            # Emulate planner motion and 10 Hz status at 0.2 mm step resolution.
            # Acceleration initially ramps to the requested feed; then brakes.
            speed=min(FEED_MM_MIN/60.,i*.01*ACCEL_MM_S2,
                      math.sqrt(max(0,2*ACCEL_MM_S2*(queued-x))))
            x=min(queued,x+speed*.01)
            if abs(x-queued)<.001:x=queued
            if i%10==0:
                moving=x<queued
                self.r.accept_line(f'<{"Jog" if moving else "Idle"}|MPos:{sign*round(x*5)/5},0,0>')
            previous=self.r.commands
            self.r.tick(self.port)
            if self.r.commands>previous:
                queued=sign*self.r.target;self.r.accept_line('ok')
                self.assertLessEqual(queued-x,LOOKAHEAD_MM+.101)
            if x>18:
                cruise=True
                self.assertGreater(queued-x,12.8)  # Remains beyond braking distance.
                self.assertAlmostEqual(speed,32.)
            self.assertIsNotNone(self.r.lease)
        self.assertTrue(cruise)
        self.assertGreater(x,400.)
        self.assertGreater(self.r.commands,200)
        self.assertTrue(self.r.busy())
        self.assertIsNone(self.r.snapshot()['max_hold_mm'])
        self.assertNotIn(b'\x85',[c.args[0] for c in self.port.write.call_args_list])
        commands=self.r.commands
        self.action('rail_stop');self.fresh(.02);self.r.tick(self.port)
        self.assertEqual(self.port.write.call_args.args[0],b'\x85')
        self.fresh(.12);self.r.accept_line(f'<Idle|MPos:{sign*x},0,0>');self.r.tick(self.port)
        self.assertFalse(self.r.busy())
        self.assertEqual(self.r.commands,commands)

    def test_left_hold_continues_past_old_cap_until_release(self):
        self.run_long_hold('left')

    def test_right_hold_continues_past_old_cap_until_release(self):
        self.run_long_hold('right')

    def test_release_purges_stream_without_replenishing_or_reversing(self):
        self.action('rail_start')
        for _ in range(3):
            self.r.tick(self.port);self.r.accept_line('ok')
        self.r.accept_line('<Jog|MPos:0.2,0,0>')
        self.action('rail_stop');self.fresh(.02);self.r.tick(self.port)
        with self.assertRaises(ValueError):self.action('rail_start')
        self.fresh(.12);self.r.accept_line('<Idle|MPos:0.6,0,0>');self.r.tick(self.port)
        self.assertEqual(self.r.commands,3)
        self.assertFalse(self.r.busy())

    def test_ack_and_progress_timeouts_do_not_send_more_blocks(self):
        self.action('rail_start');self.r.tick(self.port)
        self.fresh(.51);self.r.lease['heartbeat']=self.tick
        with self.assertRaisesRegex(RuntimeError,'acknowledgement'):self.r.tick(self.port)
        self.assertEqual(self.r.commands,1)
        self.r.accept_line('ok')
        self.fresh(1.6);self.r.lease['heartbeat']=self.tick
        with self.assertRaisesRegex(RuntimeError,'progress'):self.r.tick(self.port)
        self.assertEqual(self.r.commands,1)

    def test_lost_interlocks_cancel_a_buffered_stream(self):
        self.action('rail_start');self.r.tick(self.port);self.r.accept_line('ok')
        self.r.tick(self.port);self.r.accept_line('ok')
        self.r.accept_line('<Jog|MPos:0.2,0,0>')
        self.w.powered=True
        self.r.tick(self.port)
        self.assertIsNone(self.r.lease)
        self.assertEqual(self.port.write.call_args.args[0],b'\x85')
        self.assertEqual(self.r.commands,2)

    def test_alarm_is_reported_as_alarm_and_blocks_restart(self):
        with self.assertRaisesRegex(RuntimeError,'ALARM:1'):self.r.accept_line('ALARM:1')
        self.assertEqual(self.r.state,'Alarm')
        with self.assertRaises(ValueError):self.action('rail_start')

    def test_monitor_only_never_jogs_and_requires_fresh_stationary_status(self):
        self.r.control_enabled=False
        with self.assertRaisesRegex(ValueError,'jogging disabled'):self.action('rail_start')
        self.r.tick(self.port);self.port.write.assert_not_called()
        self.assertFalse(self.r.busy())
        self.r.accept_line('ALARM:1')
        self.assertTrue(self.r.busy())  # Alarm text alone is not fresh status.
        self.r.accept_line('<Alarm|MPos:0,0,0|Pn:X>')
        self.assertFalse(self.r.busy())
        self.assertFalse(self.r.snapshot()['ready'])
        self.tick+=.41
        self.assertTrue(self.r.busy())
        self.r.accept_line('<Jog|MPos:0.2,0,0>')
        self.assertTrue(self.r.busy())
        self.r.accept_line('<Idle|MPos:0.4,0,0>')
        self.assertFalse(self.r.busy())
        self.r.connected=False
        self.assertTrue(self.r.busy())

    def test_late_start_after_key_release_is_ignored(self):
        self.r.action(dict(action='rail_stop',client='test',seq=2))
        self.r.action(dict(action='rail_start',client='test',seq=1,direction='left',clearance_confirmed=True))
        self.r.tick(self.port);self.port.write.assert_not_called()


if __name__=='__main__':unittest.main()
