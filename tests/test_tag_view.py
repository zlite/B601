import unittest
import cv2
import numpy as np
from tag_view import detect, FAMILIES


class TagTests(unittest.TestCase):
    def test_id_zero_for_each_supported_family(self):
        for family, dictionary_id in FAMILIES.items():
            with self.subTest(family=family):
                dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
                marker = cv2.aruco.generateImageMarker(dictionary, 0, 200)
                canvas = np.full((300, 300), 255, np.uint8)
                canvas[50:250, 50:250] = marker
                frame = cv2.cvtColor(canvas, cv2.COLOR_GRAY2BGR)
                found = detect(frame, family)
                self.assertEqual(len(found), 1)
                self.assertEqual(found[0]['id'], 0)
                self.assertEqual(detect(frame, family, 1), [])

    def test_blank_frame_has_no_tag(self):
        self.assertEqual(detect(np.full((400, 640, 3), 255, np.uint8)), [])
