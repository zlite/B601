import tempfile
import math
from pathlib import Path
import time
import unittest
from unittest.mock import Mock, patch

from axis_follow import ReferencedTarget
from printer_teach import TeachWorkbench, teaching_profiles
from wrist_follow import WristTrajectory

PAIRING = Path('calibration/leader_pairing_20261002T202428971124Z.json')


class TeachingTests(unittest.TestCase):
    def test_monitor_only_rail_blocks_arm_until_stopped_and_on_stale_status(self):
        with tempfile.TemporaryDirectory() as tmp:
            w=TeachWorkbench(PAIRING,Path(tmp)/'recording',rail_monitor_only=True)
            for role in ('wrist','tripod'):w.publish(role)
            w.recorder_time=time.monotonic()
            self.assertFalse(w.cameras_ready())
            w.rail.connected=True
            w.rail.accept_line('<Alarm|MPos:0,0,0|Pn:X>')
            self.assertTrue(w.cameras_ready())
            w.rail.accept_line('<Jog|MPos:0.2,0,0>')
            self.assertFalse(w.cameras_ready())
            w.rail.accept_line('<Idle|MPos:0.4,0,0>')
            self.assertTrue(w.cameras_ready())
            w.rail.status_time-=1.
            self.assertFalse(w.cameras_ready())

    def test_ninety_degree_offset_and_reclutch_do_not_command_a_turn(self):
        target = ReferencedTarget(2., -90., -1, -58., 62.)
        self.assertEqual(target.update(-90.), 2.)
        self.assertEqual(target.update(-92.), 4.)
        target = ReferencedTarget(4., 45., -1, -58., 62.)
        self.assertEqual(target.update(45.), 4.)
        self.assertEqual(target.update(44.), 5.)

    def test_profiles_preserve_other_ranges_modes_and_speed_ceilings(self):
        from axis_follow import PROFILES
        for i, (original, slow) in enumerate(zip(PROFILES, teaching_profiles())):
            if i != 1:
                self.assertEqual(slow['range'], original['range'])
            self.assertEqual(slow['mode'], original['mode'])
            self.assertLessEqual(slow['speed'], original['speed'])
            self.assertLessEqual(slow['acceleration'], original['acceleration'])

    def test_recorded_shoulder_descent_reaches_request_and_preserves_model_limit(self):
        from arm_geometry import Geometry
        from axis_follow import axis_bounds
        geometry = Geometry()
        # October 7: raw shoulder start 56.0378, leader moved +128.4 degrees.
        # The old command stuck at raw -3.9622 (only 60 degrees from start).
        start, leader = 56.037802101938546, 11.3
        old_low, _ = axis_bounds(geometry, 1, start)
        self.assertAlmostEqual(old_low, start-60.)
        with patch('axis_follow.PROFILES', teaching_profiles()):
            low, high = axis_bounds(geometry, 1, start)
        target = ReferencedTarget(start, leader, -1, low, high)
        self.assertAlmostEqual(target.update(leader+128.4), start-128.4)
        self.assertFalse(target.limited)
        trajectory = WristTrajectory(start, start, low=low, high=high,
            speed=18., acceleration=60., response_time=.02, brake_at_target=True)
        for _ in range(200):
            previous, velocity = trajectory.position, trajectory.velocity
            trajectory.step(target.position, .05)
            self.assertTrue(low <= trajectory.position <= high)
            self.assertLessEqual(abs(trajectory.position-previous), 18.*.05+1e-8)
            self.assertLessEqual(abs(trajectory.velocity-velocity), 60.*.05+1e-8)
        self.assertAlmostEqual(trajectory.position, start-128.4)
        self.assertEqual(target.update(leader+250.), low)
        self.assertTrue(target.limited)
        model_low = math.degrees(float(geometry.joints[1].find('limit').get('lower')))
        self.assertAlmostEqual(low+math.degrees(geometry.offsets[1]), model_low+.25)
        self.assertEqual(target.update(leader-20.), high)
        self.assertEqual(high, start)  # Existing inward-only tolerance at rest.
        self.assertAlmostEqual(target.update(leader), start)

    def test_joint_four_overtravel_returns_to_reference_without_offset(self):
        # Recorded first trial: +78.5 degrees leader travel exceeded +45,
        # then returning to -0.9 left the previous incremental mapping at -33.3.
        for sign in (-1, 1):
            t = TeachWorkbench.target_mapper(-2.8, 2.5, sign, -47.8, 42.2)
            for delta in (0., 14., 72.7, 78.5, 23.4, 4.9, -12.2, -.6, -.9):
                t.update(2.5+sign*delta)
            self.assertAlmostEqual(t.position, -3.7)
            self.assertFalse(t.limited)
            self.assertAlmostEqual(t.update(2.5), -2.8)
            # Repeated saturation in both directions cannot accumulate bias.
            for _ in range(5):
                t.update(2.5+sign*200)
                t.update(2.5-sign*200)
                self.assertAlmostEqual(t.update(2.5), -2.8)

    def test_load_take_up_rebases_without_moving_the_follower(self):
        t = ReferencedTarget(10., 90., 1, -20., 40.)
        t.rebase_leader(95.)
        self.assertEqual(t.update(95.), 10.)
        self.assertEqual(t.update(96.), 11.)

    def test_hand_speed_ramp_no_longer_builds_multi_second_backlog(self):
        for i, profile in enumerate(teaching_profiles()):
            def simulate(speed, acceleration, response):
                trajectory = WristTrajectory(0., 0., low=-45., high=45.,
                    speed=speed, acceleration=acceleration, response_time=response,
                    brake_at_target=True)
                for n in range(1, 41):
                    trajectory.step(n*.5, .05)  # 10 degree/s hand motion for 2s
                return 20.-trajectory.position
            old_error = simulate(3., 8., .08)
            new_error = simulate(profile['speed'], profile['acceleration'], TeachWorkbench.response_time)
            self.assertGreater(old_error, 14.)
            self.assertLess(new_error, 1.)

    def test_teaching_trajectory_brakes_at_requested_pose_without_step_overshoot(self):
        for goal in (-4.,4.):
            t=WristTrajectory(0.,0.,low=-45.,high=45.,speed=24.,acceleration=80.,
                response_time=TeachWorkbench.response_time,brake_at_target=TeachWorkbench.brake_at_target)
            for _ in range(100):
                previous,velocity=t.position,t.velocity
                t.step(goal,.05)
                self.assertTrue(min(0.,goal)-1e-8<=t.position<=max(0.,goal)+1e-8)
                self.assertLessEqual(abs(t.position-previous),24.*.05+1e-8)
                self.assertLessEqual(abs(t.velocity-velocity),80.*.05+1e-8)
            self.assertAlmostEqual(t.position,goal,places=5)

    def test_recorder_failure_pauses_without_requesting_torque_off(self):
        with tempfile.TemporaryDirectory() as tmp:
            w = TeachWorkbench(PAIRING, Path(tmp)/'recording')
            w.gate.action({'action':'press', 'client':'test', 'seq':1}, time.monotonic())
            with patch.object(Path, 'open', side_effect=OSError('Disk full')):
                w.recording_worker()
            self.assertIsNone(w.gate.lease)
            self.assertEqual(w.recorder_error, 'Disk full')
            self.assertFalse(w.disable_requested)
            self.assertFalse(w.cameras_ready())

    def test_stalled_recorder_blocks_following_even_with_live_cameras(self):
        with tempfile.TemporaryDirectory() as tmp:
            w = TeachWorkbench(PAIRING, Path(tmp)/'recording')
            for role in ('wrist', 'tripod'):
                w.publish(role)
            self.assertFalse(w.cameras_ready())
            w.recorder_time = time.monotonic()
            self.assertTrue(w.cameras_ready())
            w.recorder_time -= 1.
            self.assertFalse(w.cameras_ready())

    def test_pause_preserves_reference_and_mismatch_requires_explicit_realign(self):
        with tempfile.TemporaryDirectory() as tmp:
            w = TeachWorkbench(PAIRING, Path(tmp)/'recording')
            w.ready = True; w.recorder_time = time.monotonic()
            w.powered = True
            w.teaching_mappers = {3:ReferencedTarget(5.,10.,1,-45.,45.)}
            w.publish('leader', angles=[0,0,0,14.,0,0,0])
            w.publish('follower', angles=[0,0,0,5.,0,0])
            press = dict(action='press',client='test',seq=1,joint=5,revision=0,clearance_confirmed=True)
            with self.assertRaisesRegex(ValueError,'Match leader'):
                w.action(press)
            self.assertIsNone(w.gate.lease)
            self.assertEqual(w.teaching_mappers[3].leader_reference,10.)
            w.action({'action':'realign'})
            self.assertEqual(w.teaching_mappers[3].leader_reference,14.)
            self.assertEqual(w.teaching_mappers[3].position,5.)
            self.assertIsNone(w.gate.lease)  # Re-align never starts following.
            w.action(press)
            self.assertIsNotNone(w.gate.lease)
            with self.assertRaisesRegex(ValueError,'Pause following'):
                w.action({'action':'realign'})

    def test_press_time_reference_survives_setup_and_take_up(self):
        with tempfile.TemporaryDirectory() as tmp:
            w = TeachWorkbench(PAIRING, Path(tmp)/'recording')
            w.press_leader_reference = [0,0,0,10.,0,0]
            mapped = w.make_target_mappers([0,0,0,5.,0,0], [0,0,0,13.,0,0], [3], {3:(-45.,45.)})
            w.update_take_up_reference(mapped, [0,0,0,14.,0,0], [3])
            self.assertEqual(mapped[3].update(14.),9.)
            self.assertEqual(mapped[3].update(10.),5.)

    def test_leader_movement_between_resume_press_and_motor_read_cancels_resume(self):
        with tempfile.TemporaryDirectory() as tmp:
            w = TeachWorkbench(PAIRING, Path(tmp)/'recording')
            w.teaching_mappers = {3:ReferencedTarget(5.,10.,1,-45.,45.)}
            w.gate.action(dict(action='press',client='test',seq=1),time.monotonic())
            w.make_target_mappers([0,0,0,5.,0,0],[0,0,0,12.,0,0],[3],{3:(-45.,45.)})
            self.assertIsNone(w.gate.lease)
            self.assertEqual(w.teaching_mappers[3].leader_reference,10.)

    def test_motor_loop_preserves_startup_motion_and_only_realigns_explicitly(self):
        with tempfile.TemporaryDirectory() as tmp:
            w = TeachWorkbench(PAIRING, Path(tmp)/'recording')
            w.record = Mock(); w.stop.wait = Mock(return_value=False)
            q = [math.degrees(v) for v in w.geometry.profile['reference_raw_rad']]
            initial = q[:]; tick=100.; count=0; seq=0; at_pause=[]
            arm=Mock(); arm.active=False; arm.joint_velocities_deg_s=[0.]*6
            arm.__enter__=Mock(return_value=arm)
            arm.__exit__=Mock(side_effect=lambda *_:setattr(arm,'active',False))
            def command(targets, **kwargs):
                for j,v in targets.items():q[j]=v
            arm.command_group.side_effect=command
            def enable(targets):
                arm.active=True;command(targets)
            arm.enable_group.side_effect=enable
            def request(op):
                nonlocal seq
                seq+=1
                w.action(dict(action=op,client='test',seq=seq,joint=5,revision=0,clearance_confirmed=True))
            def read():
                nonlocal count,tick
                count+=1;tick+=.05
                leader=[0.]*7
                leader[3]=0. if count<5 else (4. if count<70 else (6. if count<90 else 8.))
                w.publish('leader',angles=leader)
                w.publish('wrist');w.publish('tripod');w.recorder_time=tick
                # The real loop publishes after read; keep the mocked latest
                # measurement available to HTTP request validation as well.
                w.publish('follower',angles=list(q))
                if w.gate.lease:w.gate.lease['heartbeat']=tick
                if count==2:request('press')
                if count==60:
                    at_pause.append(q[3]);request('release')
                if count==70:
                    with self.assertRaisesRegex(ValueError,'Match leader'):request('press')
                if count==75:request('realign')
                if count==80:request('press')
                if count>=140:w.stop.set()
                return q[:]
            arm.read.side_effect=read
            with patch('time.monotonic',side_effect=lambda:tick):
                w.motor_worker(lambda:arm)
            self.assertIsNone(w.fault)
            self.assertAlmostEqual(at_pause[0]-initial[3],4.,places=3)
            self.assertAlmostEqual(q[3]-initial[3],6.,places=3)
            arm.enable_group.assert_called_once()


if __name__ == '__main__':
    unittest.main()
