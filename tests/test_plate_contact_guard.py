import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from plate_contact_guard import check_descent_load,check_touchdown_load,check_plate_release,check_release_separation

class DescentLoadTests(unittest.TestCase):
    def test_recorded_unexpected_loading_stops(self):
        with self.assertRaisesRegex(ValueError,'Unexpected arm load'):
            check_descent_load([-.47,11.59,-6.43,-1.67,.07,.17],[-4.55,-1.22,1.91,2.71,1.65,.22])
    def test_small_settled_variation_is_allowed(self):
        check_descent_load([-.47,11.59,-6.43,-1.67,.07,.17],[-.2,11.,-6.,-1.5,.1,.2])
    def test_invalid_feedback_stops(self):
        with self.assertRaises(ValueError):check_descent_load([0]*6,[float('nan')]*6)

    def test_touchdown_directional_allowance_requires_stationary_plate(self):
        baseline=np.array([0.,11.,-7.,0.,0.,0.])
        current=baseline+[.232,-4.007,3.2,.122,.039,-.01]
        check_touchdown_load(baseline,current,True)
        with self.assertRaises(ValueError):check_touchdown_load(baseline,current,False)

    def test_touchdown_keeps_other_directions_and_absolute_load_bounded(self):
        baseline=np.array([0.,9.,-7.,0.,0.,0.])
        for delta in ([0,-6.1,0,0,0,0],[0,4.1,0,0,0,0],[0,0,6.1,0,0,0],
                      [0,0,-4.1,0,0,0],[0,0,0,1.6,0,0],[0,0,0,0,.81,0]):
            with self.assertRaises(ValueError):check_touchdown_load(baseline,baseline+delta,True)
        with self.assertRaises(ValueError):check_touchdown_load([0,11,0,0,0,0],[0,13.6,0,0,0,0],True)
        with self.assertRaises(ValueError):check_touchdown_load(baseline,[float('nan')]*6,True)

class ReleaseTests(unittest.TestCase):
    def test_retained_plate_following_open_gripper_blocks_return(self):
        with self.assertRaisesRegex(RuntimeError,'did not separate'):
            check_release_separation(np.eye(4),np.eye(4),[0,0,1])
    def test_stationary_plate_separates_during_camera_withdrawal(self):
        after=np.eye(4);after[2,3]=-.004
        self.assertAlmostEqual(check_release_separation(np.eye(4),after,[0,0,1])['release_observed_separation_m'],.004)
    def test_small_pose_noise_allows_retreat(self):
        pose=np.eye(4);pose[0,3]=.0005
        check_plate_release(np.eye(4),pose)
    def test_plate_skew_prevents_retreat_even_with_open_jaws(self):
        pose=np.eye(4);pose[:3,:3]=Rotation.from_euler('z',8,degrees=True).as_matrix()
        with self.assertRaisesRegex(RuntimeError,'hold before retreat'):check_plate_release(np.eye(4),pose)
    def test_displaced_plate_prevents_retreat(self):
        pose=np.eye(4);pose[0,3]=.006
        with self.assertRaises(RuntimeError):check_plate_release(np.eye(4),pose)
    def test_nonfinite_release_pose_prevents_retreat(self):
        pose=np.eye(4);pose[0,3]=float('nan')
        with self.assertRaises(RuntimeError):check_plate_release(np.eye(4),pose)
