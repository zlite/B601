import unittest
import json,tempfile,threading
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock,patch
from plate_motion_owner import PlateArm,read_object
from axis_follow import AxisArm

class PlateFaultTests(unittest.TestCase):
    def test_nonobject_json_cannot_escape_hold_parser(self):
        with tempfile.TemporaryDirectory() as directory:
            path=Path(directory)/'request.json'
            for value in ('[]','true','null','42','"text"','{'):
                path.write_text(value);self.assertEqual(read_object(path),{})

    def test_verified_resumed_recovery_cleans_up_and_suppresses_task_error(self):
        arm=object.__new__(PlateArm);arm.active=True;arm.read=Mock(return_value=[0.]*6)
        arm.folded=Mock(return_value=False);arm.hold_after_fault=Mock(return_value=True)
        with patch.object(AxisArm,'__exit__') as cleanup:
            self.assertTrue(arm.__exit__(RuntimeError,RuntimeError('task error'),None))
        cleanup.assert_called_once()
    def test_failed_recovery_holds_new_pose_then_accepts_second_request(self):
        arm=object.__new__(PlateArm);position=[0.]*6;commands=[];attempts=[]
        arm.read=lambda:list(position);arm.folded=lambda q:q==[0.]*6
        arm.workbench=SimpleNamespace(lock=threading.Lock(),survey_frames={})
        with tempfile.TemporaryDirectory() as directory:
            arm.output=Path(directory);request=arm.output/'recovery_request.json'
            request.write_text(json.dumps({'action':'recover_to_rest','request_id':'first'}))
            def command(target):
                commands.append(dict(target))
                if len(commands)==2:request.write_text(json.dumps({'action':'recover_to_rest','request_id':'second'}))
            arm.command_group=command
            def recover():
                attempts.append(True)
                if len(attempts)==1:
                    position[0]=1.
                    raise RuntimeError('Stopped part way through recovery')
                position[:]=[0.]*6
                return True
            arm.recovery_callback=recover
            self.assertTrue(arm.hold_after_fault([0.]*6,RuntimeError('Task error')))
            self.assertTrue(json.loads((arm.output/'fault_hold.json').read_text())['recovered_to_rest'])
        self.assertEqual(commands[1][0],1.)
        self.assertEqual(len(attempts),2)

    def test_same_failed_request_is_not_repeated(self):
        arm=object.__new__(PlateArm);arm.read=lambda:[0.]*6;arm.folded=lambda q:False
        arm.workbench=SimpleNamespace(lock=threading.Lock(),survey_frames={})
        arm.recovery_callback=Mock(side_effect=RuntimeError('Vision unavailable'))
        with tempfile.TemporaryDirectory() as directory:
            arm.output=Path(directory)
            (arm.output/'recovery_request.json').write_text(json.dumps({'action':'recover_to_rest','request_id':'once'}))
            ticks=[]
            def command(target):
                ticks.append(True)
                if len(ticks)==3:(arm.output/'supported_remove_power.json').write_text(json.dumps({'physically_supported_confirmed':True}))
            arm.command_group=command;arm.hold_after_fault([0.]*6,RuntimeError('Task error'))
        arm.recovery_callback.assert_called_once()

    def test_malformed_release_file_does_not_drop_hold(self):
        arm=object.__new__(PlateArm);arm.read=Mock(return_value=[0]*6)
        arm.workbench=SimpleNamespace(lock=threading.Lock(),survey_frames={})
        with tempfile.TemporaryDirectory() as directory:
            arm.output=Path(directory);release=arm.output/'supported_remove_power.json';release.write_text('{')
            calls=[]
            def command(targets):
                calls.append(targets)
                if len(calls)==2:release.write_text(json.dumps({'physically_supported_confirmed':True}))
            arm.command_group=command;arm.hold_after_fault([0]*6,ValueError('settle'))
        self.assertEqual(len(calls),2)
    def test_extended_healthy_arm_holds_before_driver_cleanup(self):
        arm=object.__new__(PlateArm);arm.active=True;arm.read=Mock(return_value=[0]*6);arm.folded=Mock(return_value=False)
        events=[];arm.hold_after_fault=lambda *a:events.append('hold')
        with patch.object(AxisArm,'__exit__',side_effect=lambda *a:events.append('cleanup')):
            arm.__exit__(ValueError,ValueError('settle'),None)
        self.assertEqual(events,['hold','cleanup'])
    def test_supported_folded_pose_can_cleanup(self):
        arm=object.__new__(PlateArm);arm.active=True;arm.read=Mock(return_value=[0]*6);arm.folded=Mock(return_value=True);arm.hold_after_fault=Mock()
        with patch.object(AxisArm,'__exit__') as cleanup:arm.__exit__(ValueError,ValueError('settle'),None)
        arm.hold_after_fault.assert_not_called();cleanup.assert_called_once()
    def test_drive_feedback_failure_uses_driver_shutdown(self):
        arm=object.__new__(PlateArm);arm.active=True;arm.read=Mock(side_effect=RuntimeError('feedback'));arm.hold_after_fault=Mock()
        with patch.object(AxisArm,'__exit__') as cleanup:arm.__exit__(RuntimeError,RuntimeError('feedback'),None)
        arm.hold_after_fault.assert_not_called();cleanup.assert_called_once()
