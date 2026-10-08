import unittest
import cv2
import numpy as np

from printer_plate_vision import detect_wells, fit_lattice, stereo_plate_pose, well_model


class PlateVisionTests(unittest.TestCase):
    def setUp(self):
        self.K = np.array([[600., 0, 320], [0, 600., 240], [0, 0, 1]])
        self.cal = dict(camera_matrix=self.K.tolist(), distortion=[0.]*5)
        self.baseline = np.eye(4); self.baseline[0, 3] = -.075
        self.R = cv2.Rodrigues(np.array([.2, -.1, .15]))[0]
        self.xyz = well_model() @ self.R.T + [.012, -.008, .4]

    def pixels(self, xyz):
        return cv2.projectPoints(xyz, np.zeros(3), np.zeros(3), self.K, np.zeros(5))[0].reshape(-1, 2)

    def solve(self, xyz):
        return stereo_plate_pose(self.pixels(xyz), self.pixels(xyz+[-.075, 0, 0]),
                                 self.cal, self.cal, self.baseline)

    def test_fresh_grid_tracks_plate_translation_and_rotation_without_bed_offset(self):
        for shift in ([0., 0, 0], [.012, .027, .01]):
            result = self.solve(self.xyz+shift)
            pose = np.array(result['T_camera_b_plate'])
            np.testing.assert_allclose(pose[:3, 3], np.mean(self.xyz, axis=0)+shift, atol=1e-8)
            self.assertLess(pose[:3, 2] @ pose[:3, 3], 0)
            self.assertFalse(result['motion_ready'])
            self.assertFalse(result['well_identity_validated'])

    def test_stereo_ordering_is_recovered_and_wrong_correspondence_rejected(self):
        b = self.pixels(self.xyz); c = self.pixels(self.xyz+[-.075, 0, 0])
        for other in (c[::-1], c.reshape(8, 12, 2)[:, ::-1].reshape(-1, 2)):
            result = stereo_plate_pose(b, other, self.cal, self.cal, self.baseline)
            np.testing.assert_allclose(np.array(result['T_camera_b_plate'])[:3, 3], self.xyz.mean(0), atol=1e-8)
        with self.assertRaises(ValueError):
            stereo_plate_pose(b, np.roll(c, 1, axis=0), self.cal, self.cal, self.baseline)

    def test_partial_grid_bad_spacing_warp_and_nonfinite_fail_closed(self):
        for xyz in (self.xyz[:-1], self.xyz*1.2,
                    self.xyz+np.eye(96, 3)*.01, self.xyz*np.nan):
            with self.assertRaises(ValueError):
                fit_lattice(xyz)
        with self.assertRaises(ValueError):
            self.solve(self.xyz*1.2)

    def test_image_acquisition_finds_full_lattice_and_rejects_blank(self):
        image = np.full((480, 640), 255, np.uint8)
        for p in self.pixels(self.xyz):
            cv2.circle(image, tuple(np.round(p).astype(int)), 4, 0, -1)
        self.assertEqual(detect_wells(image).shape, (96, 2))
        with self.assertRaises(ValueError):
            detect_wells(np.full_like(image, 255))


if __name__ == '__main__': unittest.main()
