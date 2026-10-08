import tempfile
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np

from printer_demo_plan import prepare_entry, PreparedDemoPlan


class NewRestTests(unittest.TestCase):
    def test_offline_rebuild_requires_confirmation_and_still_rejects_invalid_pose_or_grip(self):
        cases=[([3.]*6,-1.,False,'resting-pose tolerance'),
               ([3.]*6,-1.,'yes','resting-pose tolerance'),
               ([float('nan')]*6,-1.,True,'Invalid resting pose'),
               ([3.]*6,-.9,True,'Gripper opening changed')]
        plan={'start_raw_deg':[0.]*6,'gripper_raw_rad':-1.}
        for start,grip,confirmed,message in cases:
            with self.subTest(message=message), tempfile.TemporaryDirectory() as folder:
                destination=Path(folder)/'entry'
                with patch('printer_demo_replay.load_plan',return_value=(plan,{},[],[])):
                    with self.assertRaisesRegex(ValueError,message):
                        prepare_entry('unused',destination,start,grip,confirmed_new_rest=confirmed)
                self.assertFalse((destination/'review.json').exists())

    def test_live_refresh_cannot_accept_a_new_rest_without_rebuilding_evidence(self):
        prepared=object.__new__(PreparedDemoPlan)
        prepared.plan={'start_raw_deg':[0.]*6,'gripper_raw_rad':-1.}
        for start in ([3.]*6,[0.]*5+[2.]):
            with self.assertRaisesRegex(ValueError,'outside the reviewed resting region'):
                prepared.refresh(np.array(start),-1.)


if __name__=='__main__':unittest.main()
