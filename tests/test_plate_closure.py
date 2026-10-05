import unittest
from plate_closure import closure_action

class ClosureTests(unittest.TestCase):
    def test_recorded_false_contact_requires_relief_not_lift(self):
        self.assertEqual(closure_action(-.852161,0),'relieve_half_mm')
        with self.assertRaisesRegex(ValueError,'bounded relief'):closure_action(-.85,3)

    def test_recorded_verified_grasps_are_accepted(self):
        for q in (-.362257,-.361717,-.336268):self.assertEqual(closure_action(q,0),'grasp')

    def test_invalid_position_cannot_authorize_grasp(self):
        for q in (float('nan'),float('inf'),-2.,1.):
            with self.assertRaises(ValueError):closure_action(q,0)
