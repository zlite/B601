import json
import tempfile
import unittest
from pathlib import Path
from progress_dashboard import run_summary


class ProgressTests(unittest.TestCase):
    def summarize(self, report=None, pickup=None, recovery=None, fault=None, shutdown=None):
        with tempfile.TemporaryDirectory() as root:
            folder=Path(root)/'20261005T175841476303Z';folder.mkdir()
            for name,value in [('report',report),('pickup_report',pickup),('recovery',recovery),('fault_hold',fault),('shutdown_verification',shutdown)]:
                if value is not None:(folder/(name+'.json')).write_text(json.dumps(value))
            return run_summary(folder)

    def test_lift_alone_is_not_a_complete_cycle(self):
        r=self.summarize(report={},pickup={'lift_following_verified':True,'model_lift_height_m':.0197})
        self.assertFalse(r['cycle_passed'])
        self.assertEqual(r['status'],'Lift passed; cycle incomplete')
        self.assertAlmostEqual(r['lift_height_mm'],19.7)

    def test_complete_cycle_requires_release_return_and_power_off(self):
        pickup={'lift_following_verified':True,'released_verified':True}
        self.assertFalse(self.summarize(report={'returned_to_rest':True},pickup=pickup)['cycle_passed'])
        self.assertTrue(self.summarize(report={'returned_to_rest':True,'motors_disabled_verified':True},pickup=pickup)['cycle_passed'])

    def test_completed_report_supersedes_stale_fault_hold(self):
        r=self.summarize(report={'returned_to_rest':True,'motors_disabled_verified':True},
            recovery={'injected_grid_loss_after_clearance':True},fault={'holding':True})
        self.assertEqual(r['status'],'Recovery test passed')
        self.assertTrue(r['grid_loss_test_passed'])

    def test_active_fault_and_missing_power_verification_remain_explicit(self):
        self.assertEqual(self.summarize(fault={'holding':True})['status'],'Powered hold')
        r=self.summarize(report={})
        self.assertEqual(r['status'],'Stopped / power unverified')
        self.assertFalse(r['motors_off_verified'])

    def test_supported_shutdown_is_not_an_unassisted_cycle(self):
        r=self.summarize(report={'returned_to_rest':True},
            pickup={'lift_following_verified':True,'released_verified':True},
            shutdown={'motors_disabled_verified':True,'physically_supported_shutdown':True})
        self.assertEqual(r['status'],'Stopped; supported shutdown')
        self.assertTrue(r['motors_off_verified'])
        self.assertFalse(r['cycle_passed'])

    def test_carry_is_not_counted_as_holder_only_or_table_release(self):
        pickup={'lift_following_verified':True,'released_verified':True,
                'transfer':{'destination_reached':True,'returned_to_source':True}}
        r=self.summarize(report={'returned_to_rest':True,'motors_disabled_verified':True},pickup=pickup)
        self.assertTrue(r['carry_passed'])
        self.assertFalse(r['holder_cycle_passed'])
        self.assertFalse(r['table_release_verified'])
        pickup['transfer']['destination_reached']=False
        r=self.summarize(report={'returned_to_rest':True,'motors_disabled_verified':True},pickup=pickup)
        self.assertFalse(r['carry_passed'])
        self.assertEqual(r['status'],'Carry stopped; holder return passed')
