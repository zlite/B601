import math
import threading
import unittest
from concurrent.futures import ThreadPoolExecutor
from types import SimpleNamespace
from unittest.mock import Mock, patch

from fast_feedback import FreshMotorReader
from wrist_follow import FollowArm


class ConcurrentReadTests(unittest.TestCase):
    def test_position_timeout_retries_with_fresh_transaction(self):
        from motorbridge.errors import CallError
        reader=FreshMotorReader.__new__(FreshMotorReader);reader.retry_count=0
        motor=Mock()
        motor.get_register_f32.side_effect=[CallError('get_register_f32 failed: register 80 not received within 40ms'),.5]
        self.assertEqual(reader.position(motor),.5)
        self.assertEqual(motor.get_register_f32.call_count,2)
        self.assertEqual(reader.retry_count,1)
        motor.get_state.assert_not_called()

    def test_repeated_position_timeout_remains_a_fault(self):
        from motorbridge.errors import CallError
        reader=FreshMotorReader.__new__(FreshMotorReader);reader.retry_count=0
        motor=Mock()
        motor.get_register_f32.side_effect=CallError('register 80 not received within 40ms')
        with self.assertRaises(CallError):reader.position(motor)
        self.assertEqual(motor.get_register_f32.call_count,2)

    def test_position_transport_failure_is_not_retried(self):
        from motorbridge.errors import CallError
        reader=FreshMotorReader.__new__(FreshMotorReader);reader.retry_count=0
        motor=Mock();motor.get_register_f32.side_effect=CallError('bus closed')
        with self.assertRaises(CallError):reader.position(motor)
        self.assertEqual(motor.get_register_f32.call_count,1)

    def reader(self, operation):
        reader = FreshMotorReader.__new__(FreshMotorReader)
        reader.motors = list(range(6))
        reader.pool = ThreadPoolExecutor(max_workers=6)
        reader.one = operation
        self.addCleanup(reader.close)
        return reader

    def test_all_reads_finish_before_failure_can_close_bus(self):
        entered = threading.Barrier(6)
        completed = []
        def operation(index):
            entered.wait(timeout=2)
            if index == 0:
                raise RuntimeError('missing fresh reply')
            completed.append(index)
            return index
        reader = self.reader(operation)
        with self.assertRaisesRegex(RuntimeError, 'missing fresh reply'):
            reader.read()
        self.assertEqual(sorted(completed), [1, 2, 3, 4, 5])

    def test_results_preserve_joint_order(self):
        self.assertEqual(self.reader(lambda i: i).read(), list(range(6)))

    def test_deadline_rejects_slow_complete_batch(self):
        reader = self.reader(lambda i: i)
        with patch('fast_feedback.time.monotonic', side_effect=[1., 1.121]):
            with self.assertRaisesRegex(RuntimeError, 'deadline'):
                reader.read()

    def test_native_timeout_does_not_return_cached_feedback(self):
        from motorbridge.abi import CState
        reader = FreshMotorReader.__new__(FreshMotorReader)
        reader.CState = CState
        reader.call = Mock(return_value=-1)
        reader.MotorState = Mock()
        reader.retry_count = 0
        motor = Mock()
        with self.assertRaisesRegex(RuntimeError, 'timed out'):
            reader.one(motor)
        motor.get_state.assert_not_called()
        reader.MotorState.assert_not_called()
        self.assertEqual(reader.call.call_count, 2)

    def test_one_lost_reply_can_be_replaced_only_by_a_fresh_reply(self):
        from motorbridge.abi import CState
        import ctypes
        reader = FreshMotorReader.__new__(FreshMotorReader)
        reader.CState = CState
        reader.retry_count = 0
        reader.MotorState = SimpleNamespace
        def reply(handle, timeout, output):
            state = ctypes.cast(output, ctypes.POINTER(CState)).contents
            state.has_value = 1
            state.status_code = 0
            return 0
        reader.call = Mock(side_effect=[-1, 0])
        calls = []
        def first_lost(*args):
            calls.append(1)
            return -1 if len(calls) == 1 else reply(*args)
        reader.call.side_effect = first_lost
        motor = Mock()
        motor.get_register_f32.return_value = .25
        position, state = reader.one(motor)
        self.assertEqual(position, .25)
        self.assertEqual(state.status_code, 0)
        self.assertEqual(reader.retry_count, 1)
        motor.get_state.assert_not_called()


class FreshValidationTests(unittest.TestCase):
    def arm(self, **bad):
        arm = FollowArm()
        arm.motors = [Mock() for _ in range(6)]
        arm.enabled_indices = {0}
        arm.speed_limits = {0: 48.}
        readings = [(0., SimpleNamespace(status_code=1 if i == 0 else 0,
                     vel=0., torq=0., t_mos=30., t_rotor=30.)) for i in range(6)]
        for key, value in bad.items():
            setattr(readings[0][1], key, value)
        arm._fresh_reader = Mock(read=Mock(return_value=readings))
        return arm

    def test_status_thermal_and_velocity_guards_preserved(self):
        for bad in ({'status_code': 0}, {'t_mos': 61.}, {'vel': math.radians(49.)}, {'torq': math.nan}):
            with self.subTest(bad=bad), self.assertRaises(RuntimeError):
                self.arm(**bad).read()

    def test_fresh_failure_is_not_silently_replaced_with_old_reader(self):
        arm = self.arm()
        arm._fresh_reader.read.side_effect = RuntimeError('fresh timeout')
        with self.assertRaisesRegex(RuntimeError, 'fresh timeout'):
            arm.read()
        for motor in arm.motors:
            motor.request_feedback.assert_not_called()

    def test_valid_read_passes(self):
        self.assertEqual(self.arm().read(), [0.]*6)
