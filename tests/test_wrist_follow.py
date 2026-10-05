import math
import random
import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from wrist_follow import HoldGate, mapped_target, WristTrajectory, check_envelope, relative_wrist_sign, FollowArm, WristWorkbench


class HoldTests(unittest.TestCase):
    def request(self, gate, op, seq, now, **kw):
        gate.action({'action': op, 'client': 'a', 'seq': seq, **kw}, now)

    def test_release_cancels_even_when_press_arrives_late(self):
        gate = HoldGate()
        self.request(gate, 'release', 2, 0)
        self.request(gate, 'press', 1, .1)
        self.assertFalse(gate.valid(.1))

    def test_heartbeat_cannot_resurrect_expired_or_released_press(self):
        for released in (False, True):
            gate = HoldGate()
            self.request(gate, 'press', 1, 0)
            if released:
                self.request(gate, 'release', 2, .1)
            self.request(gate, 'heartbeat', 3, .5, press_id=1)
            self.assertFalse(gate.valid(.5))

    def test_old_or_other_browser_heartbeat_cannot_extend_new_hold(self):
        gate = HoldGate()
        self.request(gate, 'press', 1, 0)
        self.request(gate, 'release', 2, .1)
        self.request(gate, 'press', 3, .2)
        self.request(gate, 'heartbeat', 4, .4, press_id=1)
        gate.action({'action':'heartbeat','client':'b','seq':1,'press_id':3}, .5)
        self.assertFalse(gate.valid(.61))

    def test_no_duration_cap_while_heartbeats_continue(self):
        gate = HoldGate()
        self.request(gate, 'press', 1, 0)
        for i in range(1, 6001):
            self.request(gate, 'heartbeat', i+1, i/10, press_id=1)
            self.assertTrue(gate.valid(i/10))
        self.assertTrue(gate.valid(600))
        self.assertFalse(gate.valid(600.41))

    def test_new_press_cannot_replace_existing_hold(self):
        gate = HoldGate()
        self.request(gate, 'press', 1, 0)
        with self.assertRaises(ValueError):
            self.request(gate, 'press', 2, .1)


class MotionBoundsTests(unittest.TestCase):
    def test_physical_mapping_overrides_preview_without_double_inversion(self):
        saved = {'direction_checks': {'6': {'sign': 1}}}
        self.assertEqual(relative_wrist_sign(saved, 1), 1)
        self.assertEqual(relative_wrist_sign(saved, -1), -1)
        saved['relative_follow_checks'] = {'6': {'raw_leader_to_raw_follower_sign': -1}}
        self.assertEqual(relative_wrist_sign(saved, 1), -1)
        self.assertEqual(relative_wrist_sign(saved, -1), -1)
        saved['relative_follow_checks']['6']['raw_leader_to_raw_follower_sign'] = 0
        with self.assertRaises(ValueError):
            relative_wrist_sign(saved, 1)

    def test_coupling_is_irrelevant_and_both_signs_are_bounded(self):
        for sign in (-1, 1):
            trajectory = WristTrajectory(-6, -6)
            for _ in range(1000):
                previous = trajectory.position
                velocity = trajectory.velocity
                target = trajectory.step(mapped_target(-6, -6, 50, 100, sign), .02)
                self.assertLessEqual(abs(target-previous), .24000001)
                self.assertLessEqual(abs(target+6), 15.00000001)
                self.assertLessEqual(abs(trajectory.velocity-velocity), .80000001)
            self.assertAlmostEqual(trajectory.position, -6+sign*15)

    def test_three_degree_step_settles_within_one_second(self):
        trajectory = WristTrajectory(0, 0)
        for _ in range(50):
            trajectory.step(3, .02)
        self.assertLess(abs(trajectory.position-3), .1)

    def test_reclutch_does_not_extend_global_envelope(self):
        trajectory = WristTrajectory(0, 14.9)
        for _ in range(100):
            trajectory.step(mapped_target(0, 14.9, 0, 20, 1), .1)
        self.assertAlmostEqual(trajectory.position, 15)

    def test_reversals_jitter_and_moving_targets_keep_all_bounds(self):
        rng = random.Random(90210)
        trajectory = WristTrajectory(-4, -4)
        for i in range(3000):
            desired = rng.uniform(-60,60) if i%3 else (100 if i%2 else -100)
            dt = rng.uniform(.01,.3)
            position, velocity = trajectory.position, trajectory.velocity
            trajectory.step(desired, dt)
            self.assertLessEqual(abs(trajectory.position+4),15+1e-8)
            self.assertLessEqual(abs(trajectory.velocity),12+1e-8)
            self.assertLessEqual(abs(trajectory.position-position),12*dt+1e-8)
            self.assertLessEqual(abs(trajectory.velocity-velocity),40*dt+1e-8)

    def test_stall_and_nonfinite_rejected(self):
        for dt in (0, -.1, .31, math.inf, math.nan):
            with self.assertRaises(ValueError):
                WristTrajectory(0,0).step(1,dt)
        with self.assertRaises(ValueError):
            mapped_target(0,0,0,math.nan,1)
        with self.assertRaises(ValueError):
            WristTrajectory(0,16)

    def test_supported_joints_and_wrist_must_stay_in_envelope(self):
        for q in ([1.1,0,0,0,0,0], [0,0,0,0,0,15.01]):
            with self.assertRaises(RuntimeError):
                check_envelope(q, [0]*6)


class HardwareCleanupTests(unittest.TestCase):
    def test_moderate_speed_no_longer_trips_but_overspeed_still_stops(self):
        arm = FollowArm()
        arm.ctrl = Mock()
        arm.active = True
        arm.motors = [Mock() for _ in range(6)]
        for i,m in enumerate(arm.motors):
            m.get_register_f32.return_value = 0.
            m.get_state.return_value = SimpleNamespace(status_code=1 if i==5 else 0,
                vel=math.radians(9) if i==5 else 0., torq=0., t_mos=30., t_rotor=30.)
        with patch('wrist_follow.time.sleep'):
            self.assertEqual(arm.read(),[0.]*6)
            self.assertAlmostEqual(arm.wrist_velocity_deg_s,9)
            arm.motors[5].get_state.return_value.vel = math.radians(-36)
            with self.assertRaisesRegex(RuntimeError,'-36.0 degrees/second exceeds 35'):
                arm.read()

    def arm(self):
        arm = FollowArm()
        arm.ctrl = Mock()
        arm.motors = [Mock() for _ in range(6)]
        arm._handles = arm.motors[:]
        arm.read = Mock(return_value=[0]*6)
        return arm

    def test_partial_enable_failure_disables_only_wrist(self):
        arm = self.arm()
        arm.motors[5].enable.side_effect = RuntimeError('USB fault')
        with self.assertRaises(RuntimeError):
            arm.enable(0)
        arm.__exit__(None, None, None)
        arm.motors[5].disable.assert_called_once()
        for motor in arm.motors[:5]:
            motor.enable.assert_not_called()
            motor.disable.assert_not_called()
            motor.send_mit.assert_not_called()

    def test_timeout_restored_after_disable_not_to_flash(self):
        arm = self.arm()
        arm.active = True
        arm.timeout_saved = 1234
        arm.timeout_changed = True
        arm.motors[5].get_register_u32.return_value = 1234
        arm.__exit__(None, None, None)
        arm.motors[5].write_register_u32.assert_called_once_with(9,1234)
        arm.motors[5].store_parameters.assert_not_called()
        arm.ctrl.close_bus.assert_called_once()

    def test_failed_timeout_setup_never_enables_and_is_restored(self):
        arm = self.arm()
        arm.motors[5].get_register_u32.side_effect = [1, 1234, 0, 1234]
        with self.assertRaises(RuntimeError):
            arm.prepare()
        arm.__exit__(None, None, None)
        arm.motors[5].enable.assert_not_called()
        arm.motors[5].write_register_u32.assert_called_once_with(9,1234)

    def test_500_ms_uses_10000_raw_ticks_and_restores_raw_units(self):
        arm = self.arm()
        arm.motors[5].get_register_u32.side_effect = [1, 12340, 10000, 12340]
        arm.prepare()
        arm.motors[5].set_can_timeout_ms.assert_called_once_with(500)
        arm.motors[5].enable.assert_not_called()
        arm.__exit__(None, None, None)
        arm.motors[5].write_register_u32.assert_called_once_with(9,12340)

    def test_500_raw_ticks_is_not_a_500_ms_timeout(self):
        arm = self.arm()
        arm.motors[5].get_register_u32.side_effect = [1, 12340, 500, 12340]
        with self.assertRaisesRegex(RuntimeError, 'expected 10000 ticks, got 500'):
            arm.prepare()
        arm.motors[5].enable.assert_not_called()
        arm.__exit__(None, None, None)

    def test_disable_failure_still_closes_transport(self):
        arm = self.arm()
        arm.active = True
        arm.motors[5].disable.side_effect = RuntimeError('USB disconnected')
        with self.assertRaises(RuntimeError):
            arm.__exit__(None,None,None)
        arm.ctrl.close_bus.assert_called_once()


class WorkerTests(unittest.TestCase):
    def run_worker(self, action):
        workbench = WristWorkbench('calibration/leader_pairing_20261002T202428971124Z.json')
        workbench.record = Mock()
        workbench.publish('leader', angles=[0]*7)
        workbench.publish('wrist')
        workbench.publish('tripod')
        arm = Mock()
        arm.active = False
        arm.__enter__ = Mock(return_value=arm)
        arm.__exit__ = Mock(side_effect=lambda *args: setattr(arm, 'active', False))
        count = 0
        def read():
            nonlocal count
            count += 1
            if count == 2:
                if action != 'idle':
                    workbench.gate.action({'action':'press','client':'a','seq':1}, time.monotonic())
                else:
                    workbench.stop.set()
            if count > 4:
                workbench.stop.set()
            return [0]*6
        arm.read.side_effect = read
        def prepare():
            if action == 'release_during_setup':
                workbench.gate.action({'action':'release','client':'a','seq':2}, time.monotonic())
                workbench.stop.set()
        arm.prepare.side_effect = prepare
        def enable(_):
            arm.active = True
            if action == 'leader_lost':
                workbench.error('leader', RuntimeError('disconnected'))
        arm.enable.side_effect = enable
        workbench.motor_worker(lambda: arm)
        return workbench, arm

    def test_startup_never_writes_or_enables(self):
        _, arm = self.run_worker('idle')
        arm.prepare.assert_not_called()
        arm.enable.assert_not_called()
        arm.command.assert_not_called()

    def test_release_during_setup_prevents_enable(self):
        _, arm = self.run_worker('release_during_setup')
        arm.prepare.assert_called_once()
        arm.enable.assert_not_called()

    def test_leader_loss_latches_and_cleans_up_after_enable(self):
        w, arm = self.run_worker('leader_lost')
        arm.enable.assert_called_once()
        self.assertIsNotNone(w.fault)
        self.assertIsNone(w.gate.lease)
        self.assertFalse(arm.active)
        arm.__exit__.assert_called_once()

if __name__ == '__main__':
    unittest.main()
