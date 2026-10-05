import unittest
from unittest.mock import patch
import numpy as np
from plate_pregrasp import wait_pregrasp

class PregraspTests(unittest.TestCase):
    def test_transient_boundary_miss_requires_three_new_accepted_frames(self):
        tips=np.array([[.025,-.047,.008],[.025,.047,.008]])
        pose=np.eye(4);bad=pose.copy();bad[2,3]=.0021
        stamps=iter([{'received':i,'T_camera_b_plate':P} for i,P in enumerate((bad,pose,pose,pose))])
        saved=[]
        result=wait_pregrasp(lambda:next(stamps),lambda:None,tips,saved.append)
        self.assertEqual(result['received'],3);self.assertFalse(saved[0]['accepted'])
        self.assertEqual(len(saved),4)

    def test_repeated_same_frame_cannot_satisfy_stationarity(self):
        tips=np.array([[.025,-.047,.008],[.025,.047,.008]])
        with patch('plate_pregrasp.time.monotonic',side_effect=[0,.1,.2,.3,3.]):
            with self.assertRaisesRegex(ValueError,'stabilize'):
                wait_pregrasp(lambda:{'received':1,'T_camera_b_plate':np.eye(4)},lambda:None,tips,lambda s:None)
