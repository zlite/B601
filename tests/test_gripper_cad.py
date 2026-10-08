import json
import unittest

import numpy as np

from gripper_cad import CAD, COLLISION_PARTS, finger_registration, registered_boxes, installed_pad_bounds


class GripperCADTests(unittest.TestCase):
    tips = np.array([[-.00764567, .00084545, .1555204], [.08689201, .0008138, .15233901]])

    def test_registration_maps_each_named_seam_and_preserves_handedness(self):
        for height in (.008, .013):
            transforms = finger_registration(self.tips, height)
            for name, i, sign in [('LEFT', 0, 1), ('RIGHT', 1, -1)]:
                T = transforms[name]
                np.testing.assert_allclose((T@[-.0325, sign*.04374, height, 1])[:3], self.tips[i])
                self.assertAlmostEqual(np.linalg.det(T[:3, :3]), 1.)
        with self.assertRaises(ValueError): finger_registration(self.tips, .020)

    def test_reference_plate_is_not_a_robot_collision_part(self):
        boxes = registered_boxes(self.tips)
        self.assertEqual({b['component'] for b in boxes}, set(COLLISION_PARTS))
        self.assertEqual(len(boxes), 30)
        self.assertTrue(all(np.all(b['half_size'] > 0) for b in boxes))

    def test_section_boxes_cover_exported_body_meshes(self):
        report = json.loads((CAD/'inspection.json').read_text())
        for name in ('LEFT', 'RIGHT'):
            vertices = np.load(CAD/'derived'/(name+'.npz'))['vertices_mm']
            covered = np.zeros(len(vertices), bool)
            for box in report['components'][name]['section_boxes']:
                covered |= np.all((vertices >= np.array(box['min_mm'])-.05) &
                                  (vertices <= np.array(box['max_mm'])+.05), axis=1)
            self.assertTrue(covered.all(), name)

    def test_anchor_envelope_contains_both_tested_hypotheses(self):
        envelope = registered_boxes(self.tips)
        for h in (.008, .013):
            boxes = registered_boxes(self.tips, (h, h))
            for outer, inner in zip(envelope, boxes):
                offset = outer['axes'].T@(inner['center']-outer['center'])
                self.assertTrue(np.all(abs(offset)+inner['half_size'] <= outer['half_size']+1e-12))

    def test_actual_pads_replace_cad_pads_and_are_mirrored_inward(self):
        left_lo, left_hi = installed_pad_bounds('LEFT_PAD')
        right_lo, right_hi = installed_pad_bounds('RIGHT_PAD')
        for low, high in ((left_lo, left_hi), (right_lo, right_hi)):
            np.testing.assert_allclose(high-low, [.060, .003, .010])
            self.assertAlmostEqual(low[0], -.0325)
        self.assertAlmostEqual(left_lo[1], -right_hi[1])
        self.assertAlmostEqual(left_hi[1], -right_lo[1])
        # Same assembly opening: 2 mm extra rubber on each inward face.
        self.assertAlmostEqual(left_lo[1]-right_hi[1], .08148)
        boxes = registered_boxes(self.tips, (.013, .013))
        for box in boxes:
            if box['component'].endswith('_PAD'):
                np.testing.assert_allclose(2*box['half_size'], [.060, .003, .010])


if __name__ == '__main__':
    unittest.main()
