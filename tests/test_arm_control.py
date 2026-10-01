import unittest
from unittest.mock import MagicMock,patch
from arm_control import ConfiguredArm,B601_GAINS,GAIN_REGISTERS

class GainSetupTests(unittest.TestCase):
 def arm(self):
  arm=ConfiguredArm('unused');arm.active=False;arm.touched=False;arm.gains_changed=False;arm.saved_gains=[];arm.read=MagicMock()
  arm.motors=[]
  for _ in range(6):
   values={25:.0038,26:.002,27:54.,28:0.}
   motor=MagicMock()
   motor.get_register_f32.side_effect=lambda rid,timeout,v=values:v[rid]
   motor.write_register_f32.side_effect=lambda rid,value,v=values:v.__setitem__(rid,value)
   arm.motors.append(motor)
  return arm
 def test_configure_readback_restore_no_flash_or_zero(self):
  arm=self.arm();arm.configure_gains()
  self.assertEqual(arm.saved_gains,[[.0038,.002,54.,0.]]*6)
  for motor,expected in zip(arm.motors,(*B601_GAINS[:3],*arm.saved_gains[3:])):
   self.assertEqual([motor.get_register_f32(r,500) for r in GAIN_REGISTERS],list(expected))
   motor.store_parameters.assert_not_called();motor.set_zero_position.assert_not_called()
  with patch('arm_control.ReplayArm.__exit__'):
   arm.touched=True;arm.active=True;arm.__exit__(None,None,None)
  self.assertFalse(arm.gains_changed)
  for motor in arm.motors:
   motor.disable.assert_called_once()
   self.assertEqual([motor.get_register_f32(r,500) for r in GAIN_REGISTERS],[.0038,.002,54.,0.])
 def test_partial_write_retains_backup_for_cleanup(self):
  arm=self.arm();original=arm.motors[2].write_register_f32.side_effect
  arm.motors[2].write_register_f32.side_effect=RuntimeError('write failed')
  with self.assertRaises(RuntimeError):arm.configure_gains()
  self.assertTrue(arm.gains_changed);self.assertEqual(len(arm.saved_gains),6)
  arm.motors[2].write_register_f32.side_effect=original
  with patch('arm_control.ReplayArm.__exit__'):arm.__exit__(RuntimeError,None,None)
  self.assertFalse(arm.gains_changed)
 def test_enabled_rejected(self):
  arm=self.arm();arm.active=True
  with self.assertRaises(RuntimeError):arm.configure_gains()
  for motor in arm.motors:motor.write_register_f32.assert_not_called()


class LoadTakeUpTests(unittest.TestCase):
 def test_fast_move_blocked_until_vertical_take_up_settles(self):
  import json
  from rise_approach import Planner,PROFILE
  from arm_control import LiftArm
  planner=Planner(json.loads(PROFILE.read_text())['reference_raw_rad'])
  arm=LiftArm('unused');arm.load_taken=False;arm.lift_start=planner.start
  arm.settle=MagicMock();target=planner.lift(.01);arm.read=MagicMock(return_value=target)
  with patch('arm_control.ReplayArm.segment') as segment:
   with self.assertRaises(ValueError):arm.segment(planner.start,target,.06)
   segment.assert_not_called()
   with self.assertRaises(ValueError):arm.finish_load_take_up(planner.lift(.02))
   arm.finish_load_take_up(target)
   self.assertTrue(arm.load_taken)
   arm.segment(target,planner.lift(.02),.06)
   segment.assert_called_once()
