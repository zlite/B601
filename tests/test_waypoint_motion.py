import unittest
from unittest.mock import MagicMock
import numpy as np
from waypoint_motion import path_error,move_waypoint
class WaypointTests(unittest.TestCase):
 def test_path_allows_time_lag_but_detects_sideways_drift_and_overshoot(self):
  a=np.zeros(6);b=np.array([0,.1,.2,0,0,0])
  self.assertAlmostEqual(path_error(.3*b,a,b),0.)
  off=.3*b;off[3]=.02
  self.assertAlmostEqual(path_error(off,a,b),.02)
  self.assertGreater(path_error(1.2*b,a,b),.03)
 def test_rejects_invalid_and_unloaded_before_any_motion(self):
  arm=MagicMock();arm.load_taken=False
  with self.assertRaises(ValueError):move_waypoint(arm,np.zeros(6),np.ones(6)*.01,.1)
  arm.check.assert_not_called()
  arm.load_taken=True
  for speed in [.25,float('nan')]:
   with self.assertRaises(ValueError):move_waypoint(arm,np.zeros(6),np.ones(6)*.01,speed)
  with self.assertRaises(ValueError):move_waypoint(arm,np.zeros(6),np.ones(6),.1)
  with self.assertRaises(ValueError):path_error(np.ones(6)*float('nan'),np.zeros(6),np.ones(6))

 def test_slow_feedback_cycle_stops_motion(self):
  from unittest.mock import patch
  arm=MagicMock();arm.load_taken=True;arm.low=np.ones(6)*-1;arm.high=np.ones(6)
  arm.check.return_value=np.zeros(6);arm.read.return_value=np.zeros(6);arm.motors=[MagicMock() for _ in range(6)]
  with patch('waypoint_motion.time.monotonic',side_effect=[0.,.1,.8]):
   with self.assertRaisesRegex(RuntimeError,'Feedback cycle stalled'):
    move_waypoint(arm,np.zeros(6),np.ones(6)*.01,.1)
