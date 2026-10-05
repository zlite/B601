import unittest
import numpy as np
from plate_touchdown import touch_and_backoff,remaining_contact_load


class TouchdownTests(unittest.TestCase):
    def test_recorded_reversed_torque_is_unloading(self):
        baseline=np.array([-.553846,9.114529,-6.488890,-1.540903,.070818,-.070818])
        contact=np.array([-.376068,6.365810,-4.396582,-1.443223,.085470,-.021978])
        backed=np.array([-.704273,12.246155,-7.651281,-1.619047,.080586,-.021978])
        self.assertLess(remaining_contact_load(baseline,contact,backed),0.)

    def test_persistent_contact_stays_positive(self):
        baseline=np.zeros(6);contact=np.array([0.,-3.,2.,0.,0.,0.])
        self.assertAlmostEqual(remaining_contact_load(baseline,contact,contact),1.)

    def run_search(self, contact=True, stuck=False, drift=False, excessive=False,backoff_scale=1.,friction=False,backoff_m=.001,contact_load=3.,contact_height=.006,max_descent_m=.008):
        center=np.array([.025,0.,.008]);moves=[];saved=[]
        def sample():
            load=np.zeros(6)
            if contact and center[2] <= contact_height+.00001:load[1]=-contact_load if not excessive else -5.
            if friction and moves:load[1]=-3.
            if stuck and moves and any(v[2]>0 for v in moves):load[1]=-3.
            return {'center_m':center.tolist(),'load_nm':load.tolist()}
        def move(delta):
            moves.append(delta.copy());center[:]+=delta*(backoff_scale if delta[2]>0 else 1.)
            if contact:center[2]=max(contact_height,center[2])
            if drift:center[1]+=.002
        try:
            result=touch_and_backoff(sample,move,lambda seconds:None,saved.append,[0,0,1],backoff_m=backoff_m,max_descent_m=max_descent_m)
            return result,moves
        finally:self.moves=moves

    def test_contact_then_one_mm_backoff(self):
        result,moves=self.run_search()
        self.assertAlmostEqual(result['observed_backoff_m'],.001)
        self.assertEqual(len(moves),7)
        self.assertTrue(all(v[2]==-.0005 for v in moves[:-1]))
        self.assertEqual(moves[-1][2],.001)

    def test_grip_at_contact_has_no_withdrawal_or_unloading_requirement(self):
        result,moves=self.run_search(stuck=True,backoff_m=0.)
        self.assertTrue(result['approach_complete'])
        self.assertTrue(result['grip_at_contact'])
        self.assertEqual(result['commanded_backoff_m'],0.)
        self.assertEqual(len(moves),6)
        self.assertTrue(all(v[2]==-.0005 for v in moves))
        self.assertAlmostEqual(result['contact_confirmed']['center_m'][2],.006)

    def test_no_backoff_policy_still_rejects_overload_and_free_motion(self):
        with self.assertRaisesRegex(ValueError,'Unexpected arm load'):
            self.run_search(excessive=True,backoff_m=0.)
        with self.assertRaisesRegex(RuntimeError,'No holder resistance'):
            self.run_search(contact=False,friction=True,backoff_m=0.)

    def test_smaller_load_needs_three_blocked_steps(self):
        result,moves=self.run_search(contact_load=1.5,backoff_m=0.)
        self.assertEqual(result['contact_blocked_steps'],3)
        self.assertEqual(result['contact_load_threshold'],.5)
        self.assertEqual(len(moves),7)
        self.assertTrue(all(v[2]<0 for v in moves))

    def test_extra_commands_confirm_late_contact_without_more_observed_depth(self):
        result,moves=self.run_search(contact_height=.0005,contact_load=1.5,backoff_m=0.,max_descent_m=.010)
        self.assertEqual(result['contact_blocked_steps'],3)
        self.assertAlmostEqual(result['contact_confirmed']['center_m'][2],.0005)
        self.assertLessEqual(len(moves),20)
        self.assertGreater(len(moves),16)

    def test_extended_command_budget_keeps_observed_travel_guard(self):
        with self.assertRaisesRegex(RuntimeError,'observed travel bound'):
            self.run_search(contact=False,max_descent_m=.010,backoff_m=0.)
        self.assertLessEqual(len(self.moves),20)

    def test_blocked_motion_without_load_does_not_qualify(self):
        with self.assertRaisesRegex(RuntimeError,'No holder resistance'):
            self.run_search(contact_load=0.,backoff_m=0.)

    def test_no_contact_stops_at_travel_limit(self):
        with self.assertRaisesRegex(RuntimeError,'No holder resistance'):self.run_search(contact=False)
        self.assertEqual(len(self.moves),16)

    def test_friction_without_observed_obstruction_never_triggers_backoff(self):
        with self.assertRaisesRegex(RuntimeError,'No holder resistance'):
            self.run_search(contact=False,friction=True)
        self.assertFalse(any(v[2]>0 for v in self.moves))

    def test_lateral_drift_stops_before_another_descent(self):
        with self.assertRaisesRegex(RuntimeError,'off center'):self.run_search(drift=True)
        self.assertEqual(len(self.moves),1)

    def test_resistance_must_ease_before_closure(self):
        with self.assertRaisesRegex(RuntimeError,'did not ease'):self.run_search(stuck=True)

    def test_excessive_load_is_fault_not_successful_touchdown(self):
        with self.assertRaisesRegex(ValueError,'Unexpected arm load'):self.run_search(excessive=True)
        self.assertFalse(any(v[2]>0 for v in self.moves))

    def test_undertravel_is_corrected_using_observed_clearance(self):
        result,moves=self.run_search(backoff_scale=.5)
        self.assertGreaterEqual(result['observed_backoff_m'],.00075)
        self.assertLessEqual(result['observed_backoff_m'],.00125)
        self.assertGreater(sum(v[2]>0 for v in moves),1)

    def test_no_backoff_motion_cannot_allow_closure(self):
        with self.assertRaisesRegex(RuntimeError,'backoff/alignment'):self.run_search(backoff_scale=0.)
        self.assertLessEqual(sum(v[2] for v in self.moves if v[2]>0),.003)

    def test_excessive_backoff_does_not_allow_closure(self):
        with self.assertRaisesRegex(RuntimeError,'backoff/alignment'):self.run_search(backoff_scale=2.)
