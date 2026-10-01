import unittest
import cv2
import numpy as np
from tag_pose import estimate


class PoseTests(unittest.TestCase):
    def test_metric_pose_with_tilt_and_distortion(self):
        half = .03
        points = np.array([[-half, half, 0], [half, half, 0], [half, -half, 0], [-half, -half, 0]])
        K = np.array([[800., 0, 640], [0, 800, 400], [0, 0, 1]])
        distortion = np.array([.1, -.03, .001, -.002, 0.])
        translation = np.array([.04, -.02, .35])
        corners, _ = cv2.projectPoints(points, np.array([2.8, .1, .2]), translation, K, distortion)
        poses = estimate(corners.reshape(4, 2), K, distortion, .06)
        self.assertTrue(poses)
        np.testing.assert_allclose(poses[0]['translation_camera_m'], translation, atol=1e-5)
        self.assertLess(poses[0]['reprojection_rms_px'], 1e-4)
