import unittest
import numpy as np
from blended_motion import BlendedPath,checked_blend_segments
from camera_visual_approach import route
from arm_geometry import Geometry

class BlendTests(unittest.TestCase):
 def test_retrace_partitions_sharp_turns_without_skipping_points(self):
  points=[[0.]*6,[4.,0,0,0,0,0],[8.,0,0,0,0,0],[8.,4.,0,0,0,0],[8.,8.,0,0,0,0]]
  checks=[];segments=checked_blend_segments(points,16.,144.,lambda:checks.append(True))
  joined=segments[0]+[q for segment in segments[1:] for q in segment[1:]]
  np.testing.assert_allclose(joined,points)
  for a,b in zip(segments,segments[1:]):np.testing.assert_allclose(a[-1],b[0])
  for segment in segments:self.assertLessEqual(BlendedPath(segment,16.,144.).max_deviation,.2)
  self.assertGreater(len(checks),0)

 def test_straight_retrace_eliminates_intermediate_stops(self):
  points=[[float(i)]*6 for i in range(20)]
  segments=checked_blend_segments(points,16.,144.)
  self.assertEqual(len(segments),1)
  path=BlendedPath(segments[0],16.,144.)
  mid=path.duration/2;eps=.001
  self.assertGreater(np.min((path.at(mid+eps)-path.at(mid-eps))/(2*eps)),1.)

 def test_dense_route_stays_bounded_with_continuous_velocity(self):
  q=np.degrees(Geometry().profile['reference_raw_rad']);poses,_=route(q,.20,6.)
  p=BlendedPath(poses[11:],24.,144.)
  t=np.linspace(0,p.duration,20001);a=np.array([p.at(x) for x in t]);dt=t[1]-t[0]
  self.assertLessEqual(np.max(abs(np.diff(a,axis=0)/dt)),24.001)
  self.assertLessEqual(np.max(abs(np.diff(a,n=2,axis=0)/dt**2)),144.01)
  np.testing.assert_allclose(a[0],poses[11],atol=1e-8);np.testing.assert_allclose(a[-1],poses[-1],atol=1e-8)
  self.assertLess(p.max_deviation,.02)
 def test_blend_reserves_feedback_margin(self):
  p=BlendedPath([[0]*6,[20]*6],24.,144.)
  x=p.advance(0,p.duration,np.zeros(6),3.)
  self.assertLessEqual(max(abs(p.at(x))),3.);self.assertGreater(x,0)
 def test_per_joint_margin_does_not_relax_other_axes(self):
  for joint,limit in ((0,1.),(5,1.25)):
   goal=np.zeros(6);goal[joint]=20
   p=BlendedPath([np.zeros(6),goal],24.,144.)
   t=p.advance(0,p.duration,np.zeros(6),np.array([1.]*5+[1.25]))
   self.assertLessEqual(p.at(t)[joint],limit)
   self.assertGreater(p.at(t)[joint],limit-.001)
 def test_bad_paths_and_limits_rejected(self):
  for q,s,a in [([[0]*6],24,144),([[0]*6,[0]*6],24,144),([[0]*6,[2]*6],100,144),([[0]*6,[float('nan')]*6],24,144)]:
   with self.assertRaises(ValueError):BlendedPath(q,s,a)
 def test_plate_approach_turns_and_reverse_paths_pass_blend_guard(self):
  from plate_hover import approach_segments
  g=Geometry();q=np.degrees(g.profile['reference_raw_rad']);q[4]=7.3
  lift,limits=route(q,.30,0);look=lift[11].copy();look[3]+=30
  endpoints=[]
  for approach in ('direct','left','right'):
   segments,clearance=approach_segments(g,look,limits,approach)
   self.assertGreater(clearance,.065)
   for a,b in zip(segments,segments[1:]):np.testing.assert_allclose(a[-1],b[0])
   for segment in segments:
    for points in (segment,list(reversed(segment))):
     path=BlendedPath(points,32.,288.)
     self.assertLessEqual(path.max_deviation,.2)
     np.testing.assert_allclose(path.at(path.duration),points[-1],atol=1e-8)
   endpoints.append(segments[-1][-1])
  np.testing.assert_allclose(endpoints[1:],np.tile(endpoints[0],(2,1)),atol=.001)
