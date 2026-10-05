import unittest
from types import SimpleNamespace
from plate_speed import motion_profile,apply_local_profile

class PlateSpeedTests(unittest.TestCase):
    def test_clear_travel_change_preserves_precision_profile(self):
        old=motion_profile(2.,1.);new=motion_profile(2.,2.)
        for key in ('approach_deg_s','approach_acceleration_deg_s2','gripper_ramp_rad_s','gripper_velocity_rad_s'):
            self.assertEqual(old[key],new[key])
        self.assertEqual(new['clear_approach_deg_s'],2*old['clear_approach_deg_s'])
        runner=SimpleNamespace()
        apply_local_profile(runner,new,True)
        self.assertEqual((runner.speed,runner.acceleration),(32.,288.))
        self.assertEqual(runner.pacing_lag,2.4)
        apply_local_profile(runner,new,False)
        self.assertEqual((runner.speed,runner.acceleration),(16.,144.))
        self.assertEqual(runner.pacing_lag,1.8)

    def test_all_clear_profiles_respect_controller_caps(self):
        for scale in (1.,2.,4.):
            for travel in (1.,2.):
                p=motion_profile(scale,travel)
                for prefix in ('transit','clear_approach'):
                    self.assertLessEqual(p[prefix+'_deg_s'],48.)
                    self.assertLessEqual(p[prefix+'_acceleration_deg_s2'],288.)
        for args in ((2,3),(8,2),(2,float('nan'))):
            with self.assertRaises(ValueError):motion_profile(*args)
