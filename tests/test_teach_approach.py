import math
import unittest
from unittest.mock import Mock,patch
import numpy as np
from teach_approach import validate,near,ReplayArm


class ApproachTests(unittest.TestCase):
    def path(self):
        return [[math.radians(i*.25),0,0,0,0,0] for i in range(9)]

    def test_short_path_accepted(self):
        self.assertEqual(validate(self.path()).shape,(9,6))

    def test_nan_jump_and_excessive_excursion_rejected(self):
        p=self.path();p[3][0]=float('nan')
        with self.assertRaises(ValueError):validate(p)
        p=self.path();p[3][0]=1
        with self.assertRaises(ValueError):validate(p)
        p=[[math.radians(i*.25),0,0,0,0,0] for i in range(50)]
        with self.assertRaises(ValueError):validate(p)

    def test_stationary_path_rejected(self):
        with self.assertRaises(ValueError):validate([[0]*6]*8)

    def test_start_mismatch_rejected(self):
        with self.assertRaises(ValueError):near([0]*6,[.1]*6,.5,'wrong start')

    def test_targets_outside_envelope_never_sent(self):
        arm=ReplayArm('unused');arm.low=np.zeros(6);arm.high=np.ones(6)*.1;arm.motors=[Mock() for _ in range(6)]
        with self.assertRaises(RuntimeError):arm.send(np.ones(6))
        for m in arm.motors:m.send_pos_vel.assert_not_called()

    def test_partial_enable_cleanup_only_disables_six_arm_motors(self):
        arm=ReplayArm('unused');arm.touched=True;arm.ctrl=Mock();arm.gripper=Mock();arm.motors=[Mock() for _ in range(6)]
        arm.motors[0].disable.side_effect=RuntimeError('lost reply')
        arm.__exit__(None,None,None)
        for m in arm.motors:m.disable.assert_called_once()
        arm.gripper.disable.assert_not_called();arm.ctrl.close.assert_called_once()

    def test_read_only_cleanup_leaves_torque_unchanged(self):
        arm=ReplayArm('unused');arm.touched=False;arm.ctrl=Mock();arm.motors=[Mock() for _ in range(6)]
        arm.__exit__(None,None,None)
        for m in arm.motors:m.disable.assert_not_called()

class FreshContactTests(unittest.TestCase):
    def test_teaching_accepts_new_contact_without_old_reference(self):
        import tempfile,json
        from pathlib import Path
        import teach_approach as app
        contact=np.array([.2,-.8,1.2,-.6,.1,.2])
        positions=[(contact+np.array([math.radians(.3*i),0,0,0,0,0])).tolist() for i in range(1,6)]
        reader=Mock();reader.__enter__=Mock(return_value=reader);reader.__exit__=Mock(return_value=False)
        reader.capture.side_effect=[(contact.tolist(),[0]*6),(positions[-1],[0]*6)]
        reader.read.side_effect=positions
        reader.add_motor.return_value.get_register_f32.return_value=-.5
        with tempfile.TemporaryDirectory() as tmp:
            path=Path(tmp)/'path.json'
            with patch.object(app,'FILE',path),patch.object(app,'Reader',return_value=reader),patch.object(app,'digest',return_value='reference'),patch('builtins.input',return_value=''),patch('builtins.print'),patch.object(app.select,'select',side_effect=[([],[],[])]*4+[([app.sys.stdin],[],[])]),patch.object(app.sys,'stdin',Mock(readline=Mock(return_value='\n'))):
                app.teach('unused')
                data,points=app.load_path()
                self.assertEqual(data['version'],2)
                self.assertEqual(data['contact_raw_joint_rad'],contact.tolist())
                self.assertNotIn('contact_file',data)
                data['contact_raw_joint_rad'][0]+=.1
                path.write_text(json.dumps(data))
                with self.assertRaises(ValueError):app.load_path()
