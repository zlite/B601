import unittest
import numpy as np
from blended_motion import BlendedPath
from smooth_demo_path import JoinedPath,LocalTimedCurve
from replay_timing import ACCELERATION_LIMITS

class SmoothTests(unittest.TestCase):
    def test_local_timing_bounds_velocity_acceleration_and_keeps_continuous_path(self):
        groups=[[[0.]*6,[1.,2.,.5,3.,1.,0.],[2.,4.,1.,5.,2.,1.]],
                [[2.,4.,1.,5.,2.,1.],[3.,6.,1.5,7.,3.,2.],[4.,8.,2.,9.,4.,3.]]]
        p=JoinedPath(groups);t=LocalTimedCurve(p)
        self.assertLess(p.max_deviation,.08)
        self.assertEqual(t.v[0],0);self.assertEqual(t.v[-1],0)
        for i in range(len(t.a)):
            for f in np.linspace(0,1,15):
                dt=(t.times[i+1]-t.times[i])*f;s=t.s[i]+t.v[i]*dt+.5*t.a[i]*dt*dt
                v=t.v[i]+t.a[i]*dt
                velocity=p.spline(s,1)*v
                acceleration=p.spline(s,2)*v*v+p.spline(s,1)*t.a[i]
                self.assertTrue(np.all(abs(velocity)<=12.+1e-7))
                self.assertTrue(np.all(abs(acceleration)<=.9*ACCELERATION_LIMITS+1e-6))
                np.testing.assert_allclose(t.at(t.times[i]+dt),p.spline(s),atol=1e-9)
        # No commanded velocity discontinuity at any timing segment boundary.
        for i in range(1,len(t.a)):
            left=t.v[i-1]+t.a[i-1]*(t.times[i]-t.times[i-1])
            self.assertAlmostEqual(left,t.v[i],places=10)
        self.assertTrue(np.all(t.v[1:-1]>0))
        with self.assertRaises(ValueError):LocalTimedCurve(p,13.)

if __name__=='__main__':unittest.main()
