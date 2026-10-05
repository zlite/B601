"""A release failure must reach the owner hold without any outer retreat."""
import json,tempfile,unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
import numpy as np
import plate_lower
from plate_hover import TIPS,MID
from plate_grasp_geometry import grasp_rotation
from plate_recovery import PlateRecoveryNeeded

class NestedReleaseFailureTests(unittest.TestCase):
    def exercise(self,error):
        phase=[False]
        def observation():
            R=(grasp_rotation(TIPS,normal_in_grid=np.array([0.,0.,1.])) if phase[0] else grasp_rotation(TIPS)).T
            P=np.eye(4);P[:3,:3]=R;P[:3,3]=MID-R@np.array([.025,0,.008 if phase[0] else .025])
            return {'T_camera_b_plate':P.tolist(),'surface':{'normal_camera_b':R[:,2].tolist(),'T_camera_b_reference':P.tolist()}}
        observer=SimpleNamespace(thread=Mock(),stop=Mock(),get=observation)
        runner=SimpleNamespace(targets=dict(enumerate([0.]*6)),low=np.full(6,-100.),high=np.full(6,100.),speed=8.,acceleration=36.,limits=[(-180,180)]*6,vision_ready=lambda:True,rows=[{'raw_deg':[0.]*6}],tick=Mock(),go=Mock())
        def capture(label):
            if label=='lower':phase[0]=True
        with tempfile.TemporaryDirectory() as directory:
            folder=Path(directory);(folder/'grasp.txt').touch()
            with patch.object(plate_lower,'GridObserver',return_value=observer),patch('plate_pickup.grasp_and_test',side_effect=error):
                with self.assertRaisesRegex(RuntimeError,'hold without nested retreat'):
                    plate_lower.run(runner,SimpleNamespace(),folder,capture,pickup=True)
            self.assertTrue(json.loads((folder/'lower_report.json').read_text())['retreat_inhibited'])
        runner.go.assert_not_called()
        observer.stop.set.assert_called_once()
    def test_release_runtime_failure_does_not_retract(self):
        self.exercise(RuntimeError('Plate shifted during release'))
    def test_unhandled_pickup_value_error_does_not_retract(self):
        self.exercise(ValueError('Camera lost while verifying release'))

    def test_failed_automatic_recovery_still_inhibits_outer_retreat(self):
        with patch.object(plate_lower,'recover_to_standoff',side_effect=RuntimeError('Recovery failed')):
            # The generic failure contract above also remains valid when a
            # typed task fault reaches the recovery dispatcher.
            phase=[False]
            def observation():
                R=(grasp_rotation(TIPS,normal_in_grid=np.array([0.,0.,1.])) if phase[0] else grasp_rotation(TIPS)).T;P=np.eye(4);P[:3,:3]=R
                P[:3,3]=MID-R@np.array([.025,0,.008 if phase[0] else .025])
                return {'T_camera_b_plate':P.tolist(),'surface':{'normal_camera_b':R[:,2].tolist(),'T_camera_b_reference':P.tolist()}}
            observer=SimpleNamespace(thread=Mock(),stop=Mock(),get=observation)
            runner=SimpleNamespace(targets=dict(enumerate([0.]*6)),low=np.full(6,-100.),high=np.full(6,100.),speed=8.,acceleration=36.,limits=[(-180,180)]*6,vision_ready=lambda:True,rows=[{'raw_deg':[0.]*6}],tick=Mock(),go=Mock())
            with tempfile.TemporaryDirectory() as directory:
                folder=Path(directory);(folder/'grasp.txt').touch()
                with patch.object(plate_lower,'GridObserver',return_value=observer),patch('plate_pickup.grasp_and_test',side_effect=PlateRecoveryNeeded('Plate shifted')):
                    with self.assertRaisesRegex(RuntimeError,'Automatic plate recovery stopped') as caught:
                        plate_lower.run(runner,SimpleNamespace(),folder,lambda label:phase.__setitem__(0,True) if label=='lower' else None,True)
                    self.assertEqual(str(caught.exception.__cause__),'Recovery failed')
                self.assertTrue(json.loads((folder/'lower_report.json').read_text())['retreat_inhibited'])
            runner.go.assert_not_called()
