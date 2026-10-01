import unittest
import cv2
import numpy as np
from refine_handeye import refine
from test_handeye import rigid


class RefinementTests(unittest.TestCase):
    def test_known_transform_and_heldout_isolation(self):
        rng=np.random.default_rng(21)
        X=rigid([.2,.1,-.15],[.02,-.03,.06]);Y=rigid([.1,.1,.1],[0,0,.8])
        K=np.array([[900.,0,640],[0,900,400],[0,0,1]])
        obj=np.array([[-.03,.03,0],[.03,.03,0],[.03,-.03,0],[-.03,-.03,0]])
        data={'tag':{'size_m':.06},'geometry_fingerprint':'synthetic','samples':[]}
        for _ in range(20):
            A=rigid(rng.uniform(-.4,.4,3),rng.uniform(-.1,.1,3));B=np.linalg.inv(X)@np.linalg.inv(A)@Y
            pts,_=cv2.projectPoints(obj,cv2.Rodrigues(B[:3,:3])[0],B[:3,3],K,np.zeros(5))
            data['samples'].append({'T_base_wrist':A.tolist(),'T_camera_tag':B.tolist(),
                'camera_matrix':K.tolist(),'distortion':[0]*5,'corners_px':pts.reshape(4,2).tolist()})
        result=refine(data)
        np.testing.assert_allclose(result['T_wrist_camera'],X,atol=1e-6)
        self.assertFalse(result['motion_ready'])
        data['samples'][4]['corners_px'][0][0]+=20
        changed=refine(data)
        np.testing.assert_allclose(result['T_wrist_camera'],changed['T_wrist_camera'],atol=1e-8)
        self.assertGreater(changed['heldout_corner_rms_px'][0],9)
        self.assertFalse(changed['validation_passed'])
