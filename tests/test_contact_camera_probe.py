import unittest
from contact_camera_probe_v2 import metric_tag_definition


class ProbeDefinitionTests(unittest.TestCase):
    def test_discovery_does_not_assign_old_reference_size(self):
        self.assertIsNone(metric_tag_definition(all_tags=True))
        self.assertEqual(metric_tag_definition()['id'], 0)

    def test_new_tag_requires_explicit_valid_size(self):
        for kwargs in ({'tag_id': 18}, {'tag_size': .03},
                       {'tag_id': 18, 'tag_size': 0},
                       {'tag_id': 18, 'tag_size': float('nan')}):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                metric_tag_definition(**kwargs)
        self.assertEqual(metric_tag_definition(True, 18, .03),
                         {'family': '36h11', 'id': 18, 'size_m': .03})


if __name__ == '__main__':
    unittest.main()
