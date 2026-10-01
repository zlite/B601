import unittest
import cv2
import numpy as np
from arm_geometry import Geometry
from calibrate_camera import solve_samples


def rigid(r,t):
    T=np.eye(4);T[:3,:3]=cv2.Rodrigues(np.array(r,dtype=float))[0];T[:3,3]=t
    return T


class HandEyeTests(unittest.TestCase):
    def samples(self):
        rng=np.random.default_rng(15)
        X=rigid([.2,-.1,.5],[.03,-.02,.07])
        Y=rigid([.1,.2,-.3],[.5,.1,.2])
        rows=[]
        for _ in range(25):
            A=rigid(rng.uniform(-.6,.6,3),rng.uniform(-.2,.2,3))
            B=np.linalg.inv(X)@np.linalg.inv(A)@Y
            rows.append({'T_base_wrist':A.tolist(),'T_camera_tag':B.tolist()})
        return rows,X

    def test_recovers_known_transform_using_heldout_views(self):
        rows,X=self.samples();r=solve_samples(rows)
        np.testing.assert_allclose(r['T_wrist_camera'],X,atol=1e-7)
        self.assertTrue(r['validation_passed'])
        self.assertFalse(r['motion_ready'])

    def test_heldout_bad_measurement_fails_validation(self):
        rows,_=self.samples();rows[4]['T_camera_tag'][0][3]+=.05
        self.assertFalse(solve_samples(rows)['validation_passed'])

    def test_degenerate_rotations_rejected(self):
        rows,_=self.samples()
        for row in rows:row['T_base_wrist']=np.eye(4).tolist()
        with self.assertRaises(ValueError):solve_samples(rows)

    def test_reference_offsets_produce_same_geometry(self):
        g=Geometry();T=g.transform(g.profile['reference_raw_rad'])
        self.assertTrue(np.isfinite(T).all())
        self.assertAlmostEqual(np.linalg.det(T[:3,:3]),1)
        # First joint rotates all downstream geometry around vertical base axis.
        q=g.profile['reference_raw_rad'].copy();q[0]+=.1
        shifted=g.transform(q)
        self.assertAlmostEqual(T[2,3],shifted[2,3],places=6)
