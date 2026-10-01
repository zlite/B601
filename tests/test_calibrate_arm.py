import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from calibrate_arm import Reader, summarize, direction


class CalibrationTests(unittest.TestCase):
    def test_capture_rejects_motion_and_nonfinite(self):
        rows = [[0.0]*6 for _ in range(10)]
        rows[-1][2] = math.radians(.6)
        with self.assertRaises(ValueError): summarize(rows)
        rows[-1][2] = float('nan')
        with self.assertRaises(ValueError): summarize(rows)

    def test_median_capture(self):
        median, span = summarize([[.2]*6]*10)
        self.assertEqual(median, [.2]*6)
        self.assertEqual(span, [0]*6)

    def test_direction_infers_both_signs(self):
        ref = [.2]*6
        q = ref.copy(); q[1] += math.radians(10)
        self.assertEqual(direction(ref,q,2,10),1)
        self.assertEqual(direction(ref,q,2,-10),-1)
        q[0] += math.radians(2)
        with self.assertRaises(ValueError): direction(ref,q,2,10)

    def test_wrong_magnitude_rejected(self):
        q=[0.0]*6; q[0]=math.radians(7)
        with self.assertRaises(ValueError): direction([0]*6,q,1,15)

    def test_read_has_only_read_methods_and_rejects_enabled(self):
        class Motor:
            def get_register_f32(self, register, timeout):
                assert register == 80
                return .2
            def request_feedback(self): pass
            def get_state(self): return SimpleNamespace(status_code=0)
        reader=Reader('unused')
        reader.ctrl=SimpleNamespace(poll_feedback_once=lambda:None)
        reader.motors=[Motor() for _ in range(6)]
        self.assertEqual(reader.read(), [.2]*6)
        reader.motors[0].get_state=lambda: SimpleNamespace(status_code=1)
        with self.assertRaises(RuntimeError): reader.read()

class GuidedDirectionTests(unittest.TestCase):
    def test_each_joint_and_motor_sign(self):
        from calibrate_arm import qualitative_direction, DIRECTION_STEPS
        for joint,(model_direction,_) in DIRECTION_STEPS.items():
            for sign in (-1,1):
                baseline=[.3]*6
                observed=baseline.copy()
                observed[joint-1] += sign*model_direction*math.radians(5)
                self.assertEqual(qualitative_direction(baseline,observed,joint,model_direction),sign)

    def test_rejects_coupled_small_large_and_nonfinite_changes(self):
        from calibrate_arm import qualitative_direction
        for selected, other in [(1,0),(16,0),(5,3),(float('nan'),0)]:
            observed=[math.radians(selected),math.radians(other),0,0,0,0]
            with self.assertRaises(ValueError):
                qualitative_direction([0]*6,observed,1,1)

class SerialLifetimeTests(unittest.TestCase):
    def test_closes_bus_and_all_handles_before_controller(self):
        events=[]
        from unittest.mock import Mock
        reader=Reader('unused')
        reader.ctrl=Mock()
        reader.ctrl.close_bus.side_effect=lambda:events.append('bus')
        reader.ctrl.close.side_effect=lambda:events.append('controller')
        arm_motor=Mock();gripper=Mock()
        arm_motor.close.side_effect=lambda:events.append('arm')
        gripper.close.side_effect=lambda:events.append('gripper')
        reader.ctrl.add_damiao_motor.side_effect=[arm_motor,gripper]
        reader.add_motor(1,17,'4340P');reader.add_motor(7,23,'4310')
        reader.__exit__(None,None,None)
        self.assertEqual(events,['bus','arm','gripper','controller'])
        reader.ctrl.disable_all.assert_not_called()
        arm_motor.disable.assert_not_called();gripper.disable.assert_not_called()

    def test_cleanup_continues_after_handle_close_error(self):
        from unittest.mock import Mock
        reader=Reader('unused');reader.ctrl=Mock()
        bad=Mock();good=Mock();bad.close.side_effect=RuntimeError('free failed')
        reader._handles=[bad,good]
        with self.assertRaises(RuntimeError):reader.__exit__(None,None,None)
        good.close.assert_called_once();reader.ctrl.close.assert_called_once()
