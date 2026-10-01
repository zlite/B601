import math
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from hello_world import Wrist, wave_offset


class WaveTests(unittest.TestCase):
    def test_bounds_and_endpoints(self):
        amplitude = math.radians(8)
        for step in range(12001):
            self.assertLessEqual(abs(wave_offset(step / 1000, amplitude, 6, 2)), amplitude)
        for t in (0, 3, 6, 9, 12, 20):
            self.assertAlmostEqual(wave_offset(t, amplitude, 6, 2), 0)
        self.assertAlmostEqual(wave_offset(1.5, amplitude, 6, 2), amplitude)
        self.assertAlmostEqual(wave_offset(4.5, amplitude, 6, 2), -amplitude)

    def wrist(self, position=0, status=0):
        wrist = Wrist.__new__(Wrist)
        wrist.motor, wrist.ctrl = Mock(), Mock()
        wrist.enabled = False
        wrist.state = Mock(return_value=SimpleNamespace(pos=position, status_code=status))
        return wrist

    def test_limit_rejected_before_enable(self):
        wrist = self.wrist(3)
        with self.assertRaises(RuntimeError):
            wrist.start(math.radians(8))
        wrist.motor.enable.assert_not_called()
        wrist.motor.ensure_mode.assert_not_called()

    def test_running_controller_not_taken_over(self):
        wrist = self.wrist(status=1)
        with self.assertRaises(RuntimeError):
            wrist.start(math.radians(8))
        wrist.close()
        wrist.motor.disable.assert_not_called()

    def test_enable_failure_still_disables(self):
        wrist = self.wrist()
        wrist.motor.enable.side_effect = RuntimeError('transport failure')
        with self.assertRaises(RuntimeError):
            wrist.start(math.radians(8))
        wrist.close()
        wrist.motor.disable.assert_called_once()
        wrist.ctrl.close.assert_called_once()

    def test_close_transport_even_when_disable_fails(self):
        wrist = self.wrist()
        wrist.enabled = True
        wrist.motor.disable.side_effect = RuntimeError('disconnected')
        with self.assertRaises(RuntimeError):
            wrist.close()
        wrist.ctrl.close.assert_called_once()


if __name__ == '__main__':
    unittest.main()
