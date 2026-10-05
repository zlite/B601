import unittest
import cv2
import numpy as np
from imu_camera_analysis import integrate, vector_angle


class ImuTests(unittest.TestCase):
    def test_known_rotation_with_linearly_drifting_bias(self):
        times=np.arange(0,2.01,.01)
        b0=np.array([.002,-.001,.003]);b1=np.array([.003,-.003,.002])
        bias=b0+(b1-b0)*times[:,None]/2
        rates=np.tile([0,0,np.pi/4],(len(times),1))+bias
        R=integrate(times,rates,0.,2.,b0,b1)
        np.testing.assert_allclose(R,cv2.Rodrigues(np.array([0.,0.,np.pi/2]))[0],atol=1e-10)
        self.assertAlmostEqual(vector_angle([0,1,0],R.T@[0,1,0]),90.)

    def test_missing_gyro_samples_or_unbracketed_times_rejected(self):
        times=np.r_[np.arange(0,1,.01),np.arange(1.2,2.01,.01)]
        rates=np.zeros((len(times),3))
        with self.assertRaisesRegex(ValueError,'gap'):integrate(times,rates,0.,2.,[0]*3,[0]*3)
        with self.assertRaises(ValueError):integrate(times,rates,-1.,1.,[0]*3,[0]*3)


if __name__=='__main__':unittest.main()
