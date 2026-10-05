import unittest
import numpy as np
from stereo_servo import geometry_error,mean_pose,step,TIP_B,TAG_POINT
from arm_geometry import Geometry

class StereoServoTests(unittest.TestCase):
 def pose(self,gap):
  T=np.eye(4);T[:3,:3]=np.diag([-1.,1.,-1.]);T[:3,3]=TIP_B+np.array([0,0,gap])-T[:3,:3]@TAG_POINT
  return T
 def test_signed_clearance_and_stationarity(self):
  e,gap,other,lateral=geometry_error(self.pose(.03))
  np.testing.assert_allclose(e,0,atol=1e-9);self.assertAlmostEqual(gap,.03);self.assertGreater(other,0);self.assertAlmostEqual(lateral,0)
  rows=[{'T_camera_b_tag':self.pose(.03).tolist()} for _ in range(7)]
  np.testing.assert_allclose(mean_pose(rows),self.pose(.03))
  rows[-1]['T_camera_b_tag'][0][3]+=.01
  with self.assertRaises(ValueError):mean_pose(rows)
 def test_contact_and_unsafe_clearance_are_rejected(self):
  g=Geometry();q=np.degrees(g.profile['reference_raw_rad']);limits={i:(q[i]-10,q[i]+10) for i in range(6)}
  with self.assertRaises(ValueError):step(g,q,self.pose(.03),np.eye(4),limits,standoff=0.)
  with self.assertRaises(ValueError):step(g,q,self.pose(.01),np.eye(4),limits)

 def test_contact_requires_explicit_mode_and_never_targets_penetration(self):
  g=Geometry();q=np.degrees(g.profile['reference_raw_rad']);limits={i:(q[i]-10,q[i]+10) for i in range(6)}
  with self.assertRaises(ValueError):step(g,q,self.pose(.005),np.eye(4),limits,standoff=-.001,contact=True)
  with self.assertRaises(ValueError):step(g,q,self.pose(-.002),np.eye(4),limits,standoff=0.,contact=True)
  with self.assertRaises(ValueError):step(g,q,self.pose(.005),np.eye(4),limits,standoff=0.,travel_m=.021,contact=True)
  with self.assertRaises(ValueError):step(g,q,self.pose(.005),np.eye(4),limits,standoff=0.,travel_m=.002,contact=True)
