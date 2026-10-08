import unittest
import numpy as np
from printer_observation_route import review_action, check_opening_recovery


class ObservationReviewTests(unittest.TestCase):
    def test_requires_exact_stopped_waypoint_and_explicit_image_review(self):
        self.assertEqual(review_action({'completed_stage': 4, 'action': 'advance',
                                        'images_reviewed': True}, 4), 'advance')
        for value in [None, {}, {'completed_stage': 3, 'action': 'advance', 'images_reviewed': True},
                      {'completed_stage': 4, 'action': 'advance'},
                      {'completed_stage': 4, 'action': 'grasp', 'images_reviewed': True}]:
            with self.assertRaises(ValueError):
                review_action(value, 4)

    def test_return_does_not_require_camera_approval(self):
        self.assertEqual(review_action({'completed_stage': 4, 'action': 'return'}, 4), 'return')

    def test_supported_rest_contact_only_escapes_inward_in_first_point_two_degrees(self):
        source = {'rows': [{'stage': 'unfold_8', 'raw_rad': [0, 0, np.radians(-d), 0, 0, 0]}
                           for d in [0, .1, .2, 2]]}
        result = {'rows': [{'below_margin': [{'a': 'link2', 'b': 'link5',
                    'distance_m': d, 'intersects': hit}]} for d, hit in [(0, True), (0, True), (.001, False)]]
                  + [{'below_margin': []}]}
        check_opening_recovery(source, result, supported_contact=True)
        with self.assertRaises(ValueError):
            check_opening_recovery(source, result)
        source['rows'][1]['raw_rad'][2] = np.radians(-.3)
        with self.assertRaises(ValueError):
            check_opening_recovery(source, result, supported_contact=True)
        source['rows'][1]['raw_rad'][2] = np.radians(-.1)
        result['rows'][1]['below_margin'][0]['b'] = 'link4'
        with self.assertRaises(ValueError):
            check_opening_recovery(source, result, supported_contact=True)


if __name__ == '__main__':
    unittest.main()
