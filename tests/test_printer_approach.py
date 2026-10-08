import unittest
import cv2
import numpy as np

from arm_geometry import Geometry
from printer_approach import box_gap, link_transforms, plan, rigid, stock_path_clearance
from printer_grasp_clearance import full_face_height_window, pad_boxes, pad_path_clearance


class SideApproachTests(unittest.TestCase):
    def test_link_chain_matches_existing_calibrated_kinematics(self):
        g = Geometry()
        for q in (np.array(g.profile['reference_raw_rad']), np.array([1.26, .585, 1.684, .224, .057, -.068])):
            np.testing.assert_allclose(link_transforms(g, q)['link6'], g.transform(q), atol=1e-12)

    def test_rotated_boxes_and_contact_are_not_treated_as_free(self):
        box = {'center': np.zeros(3), 'axes': np.eye(3), 'half_size': np.ones(3)}
        near = {**box, 'center': np.array([2., 0., 0.])}
        self.assertAlmostEqual(box_gap(box, near), 0.)
        far = {**box, 'center': np.array([3., 0., 0.])}
        self.assertAlmostEqual(box_gap(box, far), 1.)
        rotated = {**near, 'axes': cv2.Rodrigues(np.array([0., 0., np.pi/4]))[0]}
        self.assertLess(box_gap(box, rotated), 0.)

    def test_bad_transforms_and_empty_scene_fail_closed(self):
        bad = np.eye(4); bad[0, 0] = 2.
        with self.assertRaises(ValueError): rigid(bad)
        with self.assertRaises(ValueError):
            stock_path_clearance(Geometry(), np.zeros((2, 6)), [])

    def test_new_plate_orientation_does_not_force_a_wrist_flip(self):
        # Regression geometry from the printer-bed observation, retained here
        # only to test IK math. This test is not physical route validation.
        g = Geometry()
        q = np.array([1.2599951, .5850691, 1.6840857, .2242678, .0569354, -.0684954])
        import json
        from pathlib import Path
        X = np.array(json.loads(Path('calibration/stereo_sweep_handeye_20261004.json').read_text())['T_wrist_camera'])
        tips = np.array([[-.00764567, .00084545, .1555204], [.08689201, .0008138, .15233901]])
        P = np.array([[-.3088438, -.95111258, .00062819, .05941167],
                      [-.55681634, .18134392, .81059851, .03844215],
                      [-.77108430, .24999852, -.58560192, .23855605], [0, 0, 0, 1]])
        result = plan(g, q, X, P, tips)
        path = np.radians(result['waypoints_raw_deg'])
        self.assertLess(np.max(abs(np.diff(path, axis=0))), np.deg2rad(2.))
        base_plate = np.array(result['T_base_plate_candidate'])
        final = g.transform(path[-1])@X
        point = np.linalg.solve(base_plate, final@np.r_[tips.mean(0), 1])[:3]
        np.testing.assert_allclose(point, [.025, 0., .04], atol=.0002)
        angle = np.linalg.norm(cv2.Rodrigues((g.transform(q)@X)[:3, :3].T@final[:3, :3])[0])
        self.assertLess(angle, np.deg2rad(30.))
        np.testing.assert_allclose(result['return_waypoints_raw_deg'], result['waypoints_raw_deg'][::-1])
        self.assertFalse(result['motion_ready'])
        # The measured pad axis must be parallel to the new plate plane.
        for pad in pad_boxes(tips, length=.060, height=.010, width=.003):
            axis_plate = base_plate[:3, :3].T@final[:3, :3]@pad['axes'][:, 0]
            self.assertLess(abs(axis_plate[2]), 1e-5)

    def test_exposed_edge_window_accounts_for_pitch_and_error(self):
        dimensions = {'pad_height': .008, 'pad_length': .061}
        flat = full_face_height_window(.010, **dimensions)
        self.assertAlmostEqual(flat['minimum_center_above_rim_m'], .004)
        self.assertAlmostEqual(flat['maximum_center_above_rim_m'], .006)
        self.assertTrue(flat['positive_full_face_window'])
        self.assertFalse(full_face_height_window(.010, **dimensions, pitch_deg=2)['positive_full_face_window'])
        self.assertFalse(full_face_height_window(.010, **dimensions, pitch_deg=1,
                         height_uncertainty=.0005)['positive_full_face_window'])
        with self.assertRaises(ValueError):
            full_face_height_window(.010, **dimensions, height_uncertainty=-.001)

    def test_pad_check_detects_collision_between_waypoints(self):
        class LinearGeometry:
            def transform(self, q):
                T = np.eye(4); T[0, 3] = q[0]
                return T
        q = np.zeros((2, 6)); q[:, 0] = [-.02, .02]
        pad = {'name': 'pad', 'center': np.zeros(3), 'axes': np.eye(3),
               'half_size': np.full(3, .002)}
        obstacle = {'name': 'rim', 'T_base_obstacle': np.eye(4), 'size_m': [.002]*3}
        result = pad_path_clearance(LinearGeometry(), q, np.eye(4), [pad], [obstacle], .005)
        self.assertFalse(result['sampled_pads_clear'])
        self.assertGreater(result['closest']['fraction'], 0)
        self.assertLess(result['closest']['fraction'], 1)
        self.assertFalse(result['motion_ready'])

    def test_actual_ten_mm_pads_have_no_positive_full_face_height_window(self):
        flat = full_face_height_window(.010, pad_height=.010, pad_length=.060)
        self.assertAlmostEqual(flat['window_width_m'], 0.)
        self.assertFalse(flat['positive_full_face_window'])
        tilted = full_face_height_window(.010, pad_height=.010, pad_length=.060, pitch_deg=1)
        self.assertLess(tilted['window_width_m'], -.001)

    def test_current_near_limit_elbow_starts_in_place_and_recovers_margin(self):
        # Settled operator-selected pose, October 6. The elbow is within its
        # nominal bound but already inside the extra planning margin.
        g = Geometry()
        q = np.array([1.21942436695, .50556075573, 1.76453387737,
                      -.04934072495, .08159148693, .02881484106])
        P = np.array([[-.4065880207,-.9117019966,.0590394006,.0585281645],
                      [-.4256615474,.2462173952,.8707406281,.0397268656],
                      [-.8083924966,.3289019059,-.488185526,.2767603257], [0,0,0,1.]])
        import json
        from pathlib import Path
        X = json.loads(Path('calibration/stereo_sweep_handeye_20261004.json').read_text())['T_wrist_camera']
        tips = np.array([[-.00764567,.00084545,.1555204],[.08689201,.0008138,.15233901]])
        result = plan(g,q,X,P,tips,max_joint_excursion=1.2)
        path = np.radians(result['waypoints_raw_deg'])
        np.testing.assert_allclose(path[0],q,atol=1e-12)
        self.assertLessEqual(path[:,2].max(),q[2]+1e-9)
        for i,joint in enumerate(g.joints):
            lo,hi = sorted((float(joint.find('limit').get(k))-g.offsets[i])/g.signs[i]
                           for k in ['lower','upper'])
            self.assertTrue(np.all(path[:,i]>=lo))
            self.assertTrue(np.all(path[:,i]<=hi))
            self.assertTrue(lo+.005 <= path[-1,i] <= hi-.005)
        self.assertFalse(result['motion_ready'])
        bad = q.copy();bad[2]+=.01
        self.assertTrue(result['near_limit_start_recovery'])
        self.assertEqual(result['return_destination'],'raised outside-fixture pose')
        np.testing.assert_allclose(result['return_waypoints_raw_deg'][-1],
            result['waypoints_raw_deg'][result['stages'][0]['last_waypoint']])
        with self.assertRaisesRegex(ValueError,'outside the nominal model limits'):
            plan(g,bad,X,P,tips,max_joint_excursion=1.2)


if __name__ == '__main__':
    unittest.main()
