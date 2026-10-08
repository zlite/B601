import threading
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from pairing_dashboard import Workbench
from uvc_camera import camera_worker, frame_timing


class UVCCameraTests(unittest.TestCase):
    def test_timestamp_must_be_recent_monotonic_and_advancing(self):
        self.assertAlmostEqual(frame_timing(9950., 10., 9.8)['frame_age_s'], .05)
        for stamp in (0., float('nan'), float('inf'), 9000., 11000.):
            with self.subTest(stamp=stamp), self.assertRaises(RuntimeError):
                frame_timing(stamp, 10.)
        for stamp in (9950., 9940.):
            with self.assertRaises(RuntimeError):
                frame_timing(stamp, 10., 9.95)

    def test_overview_keeps_native_aspect_ratio_and_has_no_metric_calibration(self):
        w = Workbench()
        w.survey_mode = True
        w.camera_sizes = {'tripod': (1280, 800)}  # Old OAK requests must not stretch UVC.
        config = {'usage': 'overview_only', 'device_id': 'test', 'model': 'test',
                  'size': [640, 480], 'fps': 10, 'fourcc': 'MJPG'}

        class Capture:
            def __init__(self): self.props = {}; self.count = 0; self.released = False
            def isOpened(self): return True
            def set(self, prop, value): self.props[prop] = value; return True
            def get(self, prop):
                if prop == cv2.CAP_PROP_POS_MSEC: return 9600. + self.count * 10.
                return self.props[prop]
            def read(self): self.count += 1; return True, np.zeros((480, 640, 3), np.uint8)
            def release(self): self.released = True

        capture = Capture()
        original_publish = w.publish
        def publish(role, **info):
            original_publish(role, **info)
            w.stop.set()
        w.publish = publish
        with patch('uvc_camera.validate_device', return_value='/dev/test'), \
                patch('uvc_camera.cv2.VideoCapture', return_value=capture), \
                patch('uvc_camera.time.monotonic', return_value=10.):
            camera_worker(w, 'tripod', config)
        frame = w.survey_frames['tripod']
        self.assertEqual(frame['image'].shape, (480, 640, 3))
        self.assertNotIn('camera_matrix', frame)
        self.assertNotIn('distortion', frame)
        self.assertEqual(frame['calibration_status'], 'uncalibrated')
        self.assertTrue(capture.released)

    def test_uvc_cannot_replace_calibrated_wrist(self):
        w = Workbench()
        with patch('uvc_camera.cv2.VideoCapture') as capture:
            camera_worker(w, 'wrist', {'usage': 'overview_only'})
        capture.assert_not_called()
        self.assertIn('error', w.sources['wrist'])


if __name__ == '__main__':
    unittest.main()
