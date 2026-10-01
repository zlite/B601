import json
import unittest
import numpy as np
from rise_approach import Planner,safe_observation,PROFILE

class LiftPlannerTests(unittest.TestCase):
 def setUp(self):self.planner=Planner(json.loads(PROFILE.read_text())['reference_raw_rad'])
 def test_vertical_lift_and_orientation(self):
  for height in [.005,.05,.1,.15,.2]:
   target=self.planner.lift(height);pose=self.planner.g.transform(target)
   np.testing.assert_allclose(pose[:3,3],self.planner.base[:3,3]+[0,0,height],atol=2e-6)
   np.testing.assert_allclose(pose[:3,:3],self.planner.base[:3,:3],atol=2e-6)
 def test_scan_and_short_forward_preserve_bounds_and_orientation(self):
  for tilt in range(5,61,5):
   start=self.planner.lift(.2,tilt);a=self.planner.g.transform(start)
   target=self.planner.forward(start,.02);b=self.planner.g.transform(target)
   np.testing.assert_allclose(b[:3,3],a[:3,3]+a[:3,2]*.02,atol=1e-5)
   np.testing.assert_allclose(b[:3,:3],a[:3,:3],atol=1e-5)
 def test_limits(self):
  for args in [(.21,0),(.2,61),(-.01,0)]:
   with self.assertRaises(ValueError):self.planner.lift(*args)
  with self.assertRaises(ValueError):self.planner.forward(self.planner.start,.021)
 def test_observation_rejects_missing_close_unstable_off_axis(self):
  good={'translation_camera_m':[0,0,.4],'plane_distance_m':.35,'range_std_m':.001,'rms_px':.2}
  safe_observation(good)
  for changes in [None,{'plane_distance_m':.26},{'range_std_m':.01},{'rms_px':2}, {'plane_distance_m':float('nan')},{'translation_camera_m':[.4,0,.1]}]:
   with self.assertRaises(ValueError):safe_observation(None if changes is None else good|changes)
if __name__=='__main__':unittest.main()
