import unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace
from pathlib import Path
import numpy as np
from arm_geometry import Geometry
from camera_visual_approach import route,RouteRunner,fast_indices,movement_duration,paced_progress

class VisualApproachTests(unittest.TestCase):
 def test_settle_checks_fresh_readback_without_minimum_move(self):
  arm=Mock();w=SimpleNamespace(camera_check=Mock())
  runner=RouteRunner(w,arm,[0.]*6,{i:(-10,10) for i in range(6)},Path('/tmp'),[[0.]*6,[5.]*6])
  runner.tick=Mock(return_value=[0.]*6);runner.vision_ready=lambda:True
  runner.settle([0.]*6)
  runner.tick.assert_called_once()
  with self.assertRaisesRegex(RuntimeError,'differs'):runner.settle([1.]*6)
  runner.tick.assert_called_once()
  runner.vision_ready=lambda:False
  with self.assertRaisesRegex(ValueError,'Camera lost'):runner.settle([0.]*6)

 def test_retrace_rejects_start_mismatch_before_any_command(self):
  arm=Mock();w=SimpleNamespace(camera_check=Mock())
  runner=RouteRunner(w,arm,[0.]*6,{i:(-10,10) for i in range(6)},Path('/tmp'),[[0.]*6,[5.]*6])
  with self.assertRaisesRegex(RuntimeError,'start mismatch'):runner.retrace([[1.]*6,[3.]*6])
  arm.command_group.assert_not_called();arm.read.assert_not_called()

 def test_plan_stays_in_existing_limits_and_only_arm_joints(self):
  q=np.degrees(Geometry().profile['reference_raw_rad']).tolist();poses,limits=route(q)
  self.assertEqual(poses[0],q)
  for point in poses:
   self.assertEqual(len(point),6)
   self.assertAlmostEqual(point[5],q[5])
   for i,x in enumerate(point):self.assertTrue(limits[i][0]-1e-8<=x<=limits[i][1]+1e-8)
  bad=q.copy();bad[1]-=10
  with self.assertRaises(ValueError):route(bad)
 def test_fault_or_unplanned_target_never_commands(self):
  for stall,position in [(1.,0.),(.05,5.)]:
   arm=Mock();arm.read.return_value=[position,0,0,0,0,0];w=SimpleNamespace(camera_check=Mock())
   with patch('camera_visual_approach.time.monotonic',return_value=100):runner=RouteRunner(w,arm,[0.]*6,{i:(-1,1) for i in range(6)},Path('/tmp'),[[0.]*6,[.5]*6])
   with patch('camera_visual_approach.time.monotonic',return_value=100+stall):
    with self.assertRaises(RuntimeError):runner.tick()
   with self.assertRaises(RuntimeError):runner.go([float('nan')]*6)
   arm.command_group.assert_not_called()

 def test_fast_route_keeps_corners_and_bounds_chord_error(self):
  q=np.degrees(Geometry().profile['reference_raw_rad']).tolist();poses,_=route(q,.20,6.)
  ids=fast_indices(poses);self.assertTrue({0,1,3,11,len(poses)-1}.issubset(ids));self.assertLess(len(ids),len(poses)/2)
  for a,b in zip(ids,ids[1:]):
   chord=np.array(poses[a])+np.linspace(0,1,b-a+1)[:,None]*(np.array(poses[b])-poses[a])
   self.assertLessEqual(np.max(abs(chord-np.array(poses[a:b+1]))),.200001)
  for distance in (.01,1,10,90):
   t=movement_duration(np.array([distance]),12.,36.)
   self.assertLessEqual(1.5*distance/t,12.000001);self.assertLessEqual(6*distance/t**2,36.000001)

 def test_double_speed_halves_unpaced_segment_duration(self):
  for distance in (1,10,90):
   slow=movement_duration(np.array([distance]),12.,36.)
   fast=movement_duration(np.array([distance]),24.,144.)
   self.assertAlmostEqual(fast,max(.25,slow/2))
   self.assertLessEqual(1.5*distance/fast,24.000001)
   self.assertLessEqual(6*distance/fast**2,144.000001)

 def test_high_speed_clock_reserves_tracking_margin_without_cutting_corner(self):
  origin=np.zeros(6);delta=np.array([40,20,-10,0,0,0.]);actual=np.zeros(6)
  duration=movement_duration(delta,48.,288.)
  p=paced_progress(origin,delta,0.,duration,duration,actual,7.5)
  u=p/duration;target=origin+delta*(u*u*(3-2*u))
  self.assertGreater(p,0);self.assertLess(p,duration)
  self.assertLessEqual(max(abs(target-actual)),7.5)
  self.assertAlmostEqual(target[1],target[0]/2)
  self.assertAlmostEqual(target[2],-target[0]/4)
