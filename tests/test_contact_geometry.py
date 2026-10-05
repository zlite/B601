import unittest
import cv2
import numpy as np
from contact_geometry import triangulate,check_tag_triplet,stereo_tag_pose

class GeometryTests(unittest.TestCase):
 def test_known_points_and_invalid_baseline(self):
  K=np.array([[600.,0,320],[0,600,240],[0,0,1]]);D=np.zeros(5)
  points=np.array([[-.03,.03,.5],[.03,.03,.5],[.03,-.03,.5],[-.03,-.03,.5]])
  T=np.eye(4);T[0,3]=-.075
  def camera(x):
   px=cv2.projectPoints(x,np.zeros(3),np.zeros(3),K,D)[0].reshape(-1,2).tolist()
   return {'camera_matrix':K.tolist(),'distortion':D.tolist(),'tags':[{'corners_px':px}],'poses':[{'rotation_vector':[0.,0.,0.],'translation_camera_m':[0.,0.,.5]}]}
  b=camera(points);c=camera(points+T[:3,3]);frame={'cameras':{'A':b,'B':b,'C':c}}
  actual,error=triangulate(b['tags'][0]['corners_px'],c['tags'][0]['corners_px'],b,c,T)
  np.testing.assert_allclose(actual,points,atol=1e-8)
  self.assertTrue(check_tag_triplet(frame,{'B_C':T,'B_A':np.eye(4)})['passed'])
  pose=stereo_tag_pose(frame,{'B_C':T})
  np.testing.assert_allclose(np.array(pose['T_camera_b_tag'])[:3,3],[0,0,.5],atol=1e-8)
  np.testing.assert_allclose(np.array(pose['T_camera_b_tag'])[:3,:3],np.eye(3),atol=1e-8)
  T[0,3]*=1.2
  self.assertFalse(check_tag_triplet(frame,{'B_C':T,'B_A':np.eye(4)})['passed'])
  with self.assertRaises(ValueError):stereo_tag_pose(frame,{'B_C':T})
