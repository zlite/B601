import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch, MagicMock

import cv2
import numpy as np

import calibrate_camera
import camera_selection
from revalidate_setup import compare_tag_depth


class DepthComparisonTests(unittest.TestCase):
    def test_tilted_plane_uses_each_pixel_ray_and_ignores_missing_depth(self):
        K = np.array([[100., 0, 50], [0, 100., 50], [0, 0, 1]])
        rotation = np.array([.3, -.2, .1])
        translation = np.array([.03, .02, .6])
        y, x = np.indices((100, 100))
        rays = np.linalg.solve(K, np.vstack([x.ravel(), y.ravel(), np.ones(10000)])).T
        normal = cv2.Rodrigues(rotation)[0][:, 2]
        z = ((normal @ translation) / (rays @ normal)).reshape(100, 100)
        depth = np.rint(z*1000).astype(np.uint16)
        depth[:, :50] = 0
        result = compare_tag_depth(depth, [[20,20],[80,20],[80,80],[20,80]], K,
                                   {'rotation_vector': rotation, 'translation_camera_m': translation})
        self.assertLess(result['median_absolute_error_m'], .0005)
        self.assertGreater(result['valid_fraction'], .4)
        self.assertLess(result['valid_fraction'], .6)

    def test_missing_depth_does_not_claim_distance_agreement(self):
        result = compare_tag_depth(np.zeros((100,100),np.uint16),
                                   [[20,20],[80,20],[80,80],[20,80]], np.eye(3), {})
        self.assertEqual(result['valid_fraction'], 0)
        self.assertNotIn('median_absolute_error_m', result)


class CameraSessionTests(unittest.TestCase):
    def test_focus_differing_from_eeprom_prevents_camera_setup(self):
        pipeline = MagicMock()
        pipeline.getDefaultDevice().readCalibration().getLensPosition.return_value = 90
        with patch.object(camera_selection, 'wrist_camera_config', return_value={'manual_focus': 76}):
            with self.assertRaisesRegex(RuntimeError, 'differs from camera calibration'):
                camera_selection.build_wrist_rgb(pipeline)
        pipeline.create.assert_not_called()

    def test_legacy_or_different_camera_samples_cannot_resume(self):
        class Geometry:
            fingerprint = 'geometry'
        camera = {'device_id': 'wrist', 'model': 'OAK-D-LITE'}
        with tempfile.TemporaryDirectory() as folder:
            path = Path(folder)/'samples.json'
            with patch.object(calibrate_camera, 'DATA', path), patch.object(calibrate_camera, 'wrist_camera_config', return_value=camera):
                data = {'geometry_fingerprint': 'geometry', 'tag': calibrate_camera.TASK_CONFIG['tag'], 'samples': []}
                for identity in [None, {'device_id': 'old', 'model': 'OAK-1'}]:
                    data['wrist_camera'] = identity
                    path.write_text(json.dumps(data))
                    with self.assertRaisesRegex(ValueError, 'Camera changed'):
                        calibrate_camera.load_data(Geometry())
                data['wrist_camera'] = camera
                path.write_text(json.dumps(data))
                self.assertEqual(calibrate_camera.load_data(Geometry())['wrist_camera'], camera)
