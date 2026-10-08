import unittest
import numpy as np

from pairing_dashboard import Workbench
from printer_target_tracker import compare_reference, require_current_reference, stationary_joint_pose, validate_stationary_capture


class TargetTrackingTests(unittest.TestCase):
    def test_capture_requires_stationary_arm_jaw_and_bracketing_telemetry(self):
        rows = [{'start_s':t, 'end_s':t+.05, 'arm_raw_rad':[.1]*6,
                 'gripper_raw_rad':-1.15} for t in [10.,10.2,10.4,10.6]]
        self.assertTrue(validate_stationary_capture(rows,[10.1,10.5])['stationary'])
        for times in [[9.9,10.5],[10.1,10.7]]:
            with self.assertRaises(ValueError):validate_stationary_capture(rows,times)
        from copy import deepcopy
        for key in ['arm_raw_rad','gripper_raw_rad']:
            moved = deepcopy(rows)
            if key=='arm_raw_rad':moved[1][key][2]+=.002
            else:moved[1][key]+=.004
            with self.assertRaises(ValueError):validate_stationary_capture(moved,[10.1,10.5])

    def test_changed_bed_position_requires_replan(self):
        before = np.eye(4); before[:3, 3] = [.1, .4, .05]
        after = before.copy(); after[1, 3] += .053
        result = compare_reference(before, after)
        self.assertTrue(result['requires_replan'])
        self.assertAlmostEqual(result['translation_change_m'], .053)
        obs = {'valid': True, 'session': 'current', 'observation_time': 10.,
               'T_base_tag_candidate': after.tolist()}
        with self.assertRaisesRegex(ValueError, 'discard the old path'):
            require_current_reference(before, obs, 10.1, 'current')
        self.assertFalse(require_current_reference(after, obs, 10.1, 'current')['requires_replan'])

    def test_lost_stale_future_and_restarted_references_are_rejected(self):
        obs = {'valid': True, 'session': 'run', 'observation_time': 10.,
               'T_base_tag_candidate': np.eye(4).tolist()}
        for changed, now, session in [({'valid': False}, 10.1, 'run'),
                                      ({}, 10.3, 'run'), ({}, 9.9, 'run'),
                                      ({}, 10.1, 'another'),
                                      ({'observation_time': float('nan')}, 10.1, 'run')]:
            with self.assertRaises(ValueError):
                require_current_reference(np.eye(4), {**obs, **changed}, now, session)

    def test_moving_or_disconnected_arm_cannot_supply_base_pose(self):
        history = [(t, [5.]*6) for t in [9.8, 9.9, 10.]]
        np.testing.assert_allclose(stationary_joint_pose(history, 10.), np.radians([5.]*6))
        history[-1] = (10., [5.1]*6)
        with self.assertRaises(ValueError): stationary_joint_pose(history, 10.)
        with self.assertRaises(ValueError): stationary_joint_pose([], 10.)
        with self.assertRaises(ValueError): stationary_joint_pose(history, 11.)

    def test_dashboard_expires_target_and_clears_failed_joint_history(self):
        w = Workbench()
        w.publish('printer_target', valid=True, observation_time=0., session='old')
        self.assertFalse(w.snapshot()['sources']['printer_target']['valid'])
        w.publish('follower', angles=[0.]*6)
        self.assertTrue(w.follower_history)
        w.error('follower', 'link lost')
        self.assertFalse(w.follower_history)


if __name__ == '__main__':
    unittest.main()
