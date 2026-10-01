import json,unittest
import numpy as np
from continuous_approach import Route,moving_load_ready,smoothstep,PROFILE
class ContinuousTests(unittest.TestCase):
 def test_blend_stays_in_envelope_and_connects_endpoints(self):
  q=json.loads(PROFILE.read_text())['reference_raw_rad'];route=Route(q)
  np.testing.assert_allclose(route.q[0],q)
  self.assertTrue(np.all(route.q>=route.low));self.assertTrue(np.all(route.q<=route.high))
  self.assertTrue(np.all(np.diff(route.arc)>0));self.assertGreaterEqual(route.approach_m,.04)
  self.assertLess(np.max(abs(np.diff(route.q,axis=0))),.001)
  for index in [100,4000,7000,9000]:
   found,error=route.locate(route.q[index],index);self.assertEqual(found,index);self.assertEqual(error,0)
   self.assertGreaterEqual(route.preview(index,1,.05),index)
   self.assertLessEqual(route.preview(index,-1,.05),index)
 def test_load_gate_requires_height_time_and_tracking(self):
  rows=[(i*.05,.1) for i in range(45)]
  self.assertTrue(moving_load_ready(rows,.01))
  self.assertFalse(moving_load_ready(rows,.005))
  self.assertFalse(moving_load_ready(rows[-15:],.01))
  self.assertFalse(moving_load_ready(rows[:-1]+[(rows[-1][0],.4)],.01))
 def test_speed_blend_monotonic_and_clamped(self):
  values=[smoothstep(x) for x in np.linspace(-.1,1.1,100)]
  self.assertEqual(values[0],0);self.assertEqual(values[-1],1)
  self.assertTrue(np.all(np.diff(values)>=0))
