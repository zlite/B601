import unittest
import numpy as np
from arm_geometry import Geometry
from camera_visual_approach import route
from plate_transfer import cartesian_segment

class TransferPathTests(unittest.TestCase):
    def test_rejects_unbounded_displacement_before_planning(self):
        g=Geometry();q=np.degrees(g.profile['reference_raw_rad']);_,limits=route(q,.30,0.)
        for delta in ([.2,0,0],[float('nan'),0,0],[0,0]):
            with self.assertRaisesRegex(ValueError,'displacement'):
                cartesian_segment(g,q,delta,limits,np.zeros(3),np.array([0,0,1]))

    def test_rejects_path_below_support_plane(self):
        g=Geometry();q=np.degrees(g.profile['reference_raw_rad']);_,limits=route(q,.30,0.)
        with self.assertRaises(ValueError):
            cartesian_segment(g,q,[0,0,.001],limits,np.array([0,0,10.]),np.array([0,0,1]))


    def test_nonfinite_support_plane_cannot_bypass_clearance(self):
        g=Geometry();q=np.degrees(g.profile['reference_raw_rad']);_,limits=route(q,.30,0.)
        for normal in ([0,0,float('nan')],[0,0,0],[0,0,2]):
            with self.assertRaisesRegex(ValueError,'geometry'):
                cartesian_segment(g,q,[0,0,.001],limits,np.zeros(3),normal)
