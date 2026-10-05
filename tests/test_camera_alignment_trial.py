import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from arm_geometry import Geometry
from camera_alignment_trial import plan, recovery_target, Trial


class CameraTrialTests(unittest.TestCase):
    def setUp(self):
        self.geometry = Geometry()
        self.start = [math.degrees(x) for x in self.geometry.profile['reference_raw_rad']]

    def test_wrist_path_preserves_supported_links_and_reverses_to_start(self):
        poses = plan(self.geometry,self.start)
        self.assertEqual(poses[0],self.start)
        for q in poses:
            self.assertEqual(q[:3],self.start[:3])
        for before,after,j in zip(poses,poses[1:],[5,4,3]):
            self.assertAlmostEqual(abs(after[j]-before[j]),8.)
            self.assertEqual([x for i,x in enumerate(after) if i!=j],
                             [x for i,x in enumerate(before) if i!=j])
        self.assertEqual(list(reversed(poses[:-1]))[-1],self.start)

    def test_nonfolded_or_invalid_start_rejected(self):
        for joint in (1,2):
            q = self.start.copy();q[joint] -= 5
            with self.assertRaises(ValueError):plan(self.geometry,q)
        q = self.start.copy();q[4] = math.nan
        with self.assertRaises(ValueError):plan(self.geometry,q)

    def test_bend_repeat_only_changes_bend_and_returns_in_between(self):
        poses=plan(self.geometry,self.start,bend_repeat=True)
        for a,b in zip(poses[0],poses[2]):self.assertAlmostEqual(a,b)
        for a,b in zip(poses[1],poses[3]):self.assertAlmostEqual(a,b)
        for q in poses:
            self.assertEqual(q[:3]+q[4:],self.start[:3]+self.start[4:])
        self.assertAlmostEqual(poses[1][3]-poses[0][3],-8.)

    def test_wrist_limit_not_expanded_for_diagnostic(self):
        joint = self.geometry.joints[5].find('limit')
        q = self.start.copy()
        q[5] = (math.degrees(float(joint.get('upper')))-math.degrees(self.geometry.offsets[5]))/self.geometry.signs[5]-.5
        with self.assertRaises(ValueError):plan(self.geometry,q)

    def test_motor_feedback_failure_prevents_next_command(self):
        for index,delta in ((0,1.1),(4,3.1)):
            arm=Mock();arm.joint_velocities_deg_s=[0.]*6
            work=SimpleNamespace(publish=Mock(),camera_check=Mock())
            with patch('camera_alignment_trial.time.monotonic',return_value=100.):
                trial=Trial(work,arm,[0.]*6)
            q=[0.]*6;q[index]=delta;arm.read.return_value=q
            with patch('camera_alignment_trial.time.monotonic',return_value=100.05):
                with self.assertRaises(RuntimeError):trial.tick()
            arm.command_group.assert_not_called()

    def test_loop_stall_prevents_command(self):
        arm=Mock();arm.read.return_value=[0.]*6
        with patch('camera_alignment_trial.time.monotonic',return_value=100.):
            trial=Trial(Mock(),arm,[0.]*6)
        with patch('camera_alignment_trial.time.monotonic',return_value=100.4):
            with self.assertRaisesRegex(RuntimeError,'stalled'):trial.tick()
        arm.command_group.assert_not_called()

    def test_recovery_keeps_large_joints_fixed_and_rejects_large_movements(self):
        current=self.start.copy();current[5]+=7.5;current[2]-=.05
        target=recovery_target(self.geometry,current,{'start_raw_deg':self.start})
        self.assertEqual(target[:3],current[:3])
        self.assertEqual(target[3:],self.start[3:])
        current[5]+=3
        with self.assertRaises(ValueError):recovery_target(self.geometry,current,{'start_raw_deg':self.start})
        current=self.start.copy();current[1]-=.5
        with self.assertRaises(ValueError):recovery_target(self.geometry,current,{'start_raw_deg':self.start})


if __name__ == '__main__':unittest.main()
