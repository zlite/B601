import copy
import json
from pathlib import Path
import unittest
import numpy as np
from scripts.build_standoff_demo import shorten

class StandoffTests(unittest.TestCase):
    def setUp(self):
        p=Path(__file__).resolve().parents[1]/'outputs/printer_replay/folded_recovery_rest_20261008T2000/simplified_plan.json'
        self.plan=json.loads(p.read_text())

    def test_contact_tail_removed_and_original_preserved(self):
        original=copy.deepcopy(self.plan)
        new,curves=shorten(self.plan,self.plan['start_raw_deg'],self.plan['gripper_raw_rad'])
        self.assertEqual(self.plan,original)
        self.assertEqual([s['name'] for s in new['segments']],['entry','standoff'])
        self.assertEqual(new['segments'][1]['joined_groups'],original['segments'][1]['joined_groups'][:2])
        np.testing.assert_allclose(curves[-1].at(1),original['segments'][1]['joined_groups'][1][-1])
        self.assertGreater(np.max(abs(curves[-1].at(1)-np.array(original['segments'][-1]['points'][-1]))),20)
        self.assertLessEqual(curves[-1].max_deviation,.08)

    def test_changed_gripper_or_nonfinite_capture_rejected(self):
        for grip in [float('nan'),float('inf'),self.plan['gripper_raw_rad']+.01]:
            with self.assertRaises(ValueError):shorten(self.plan,self.plan['start_raw_deg'],grip)
        for start in [[0]*5,[float('nan')]*6]:
            with self.assertRaises(ValueError):shorten(self.plan,start,self.plan['gripper_raw_rad'])

    def test_unrecognized_route_rejected(self):
        p=copy.deepcopy(self.plan);p['segments'][1]['joined_groups'].pop()
        with self.assertRaises(ValueError):shorten(p,p['start_raw_deg'],p['gripper_raw_rad'])

if __name__=='__main__':unittest.main()
