import threading
import time
import unittest
from types import SimpleNamespace

import numpy as np
from arm_geometry import Geometry
from printer_observation_lift import checked_poses, LiftRunner


class ObservationLiftTests(unittest.TestCase):
    def setUp(self):
        self.g = Geometry()
        self.start = np.degrees(self.g.profile['reference_raw_rad'])

    def test_only_elbow_moves_inward(self):
        poses, _ = checked_poses(self.g, self.start, self.start)
        np.testing.assert_allclose(np.array(poses[1])-poses[0], [0, 0, -8, 0, 0, 0])
        np.testing.assert_allclose(poses[0], self.start)

    def test_changed_pose_rejected(self):
        changed = self.start.copy()
        changed[4] += .06
        with self.assertRaises(ValueError):
            checked_poses(self.g, changed, self.start)

    def test_nonrest_pose_rejected_even_if_saved(self):
        changed = self.start.copy()
        changed[0] += 15
        with self.assertRaises(ValueError):
            checked_poses(self.g, changed, changed)

    def test_nonfinite_rejected(self):
        changed = self.start.copy()
        changed[2] = np.nan
        with self.assertRaises(ValueError):
            checked_poses(self.g, changed, self.start)

    def test_live_images_required_but_tag_absence_expected(self):
        runner = object.__new__(LiftRunner)
        runner.w = SimpleNamespace(lock=threading.RLock(),
            sources={r: {'time': time.monotonic(), 'frame_age_s': .02}
                     for r in ('wrist', 'tripod')}, images={'wrist': b'image', 'tripod': b'image'})
        runner.w.sources['printer_target'] = {'valid': False, 'reason': 'Tag outside view'}
        self.assertTrue(runner.vision_ready())
        runner.w.sources['tripod']['time'] -= .5
        self.assertFalse(runner.vision_ready())
        runner.w.sources['tripod']['time'] = time.monotonic()
        runner.w.sources['wrist']['error'] = 'Disconnected'
        self.assertFalse(runner.vision_ready())


if __name__ == '__main__':
    unittest.main()
