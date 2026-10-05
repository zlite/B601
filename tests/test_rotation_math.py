import unittest
import numpy as np
from scipy.spatial.transform import Rotation
from rotation_math import rotation_vector
from arm_geometry import Geometry

class RotationMathTests(unittest.TestCase):
    def test_preserves_microradian_rotation(self):
        v=np.array([2.,-3.,4.])*1e-7
        np.testing.assert_allclose(rotation_vector(Rotation.from_rotvec(v).as_matrix()),v,rtol=1e-9,atol=1e-14)
    def test_all_revolute_joints_have_unit_angular_derivative(self):
        g=Geometry();q=np.radians([1.,-50.,30.,9.,5.,-3.]);base=g.transform(q)
        for i in range(6):
            p=q.copy();p[i]+=1e-5
            derivative=rotation_vector(base[:3,:3].T@g.transform(p)[:3,:3])/1e-5
            self.assertAlmostEqual(np.linalg.norm(derivative),1.,places=7)
