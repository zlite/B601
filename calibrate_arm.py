#!/usr/bin/env python3
"""Read-only reference recording for the second B601-DM. No motor writes."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import statistics
import sys
import time

from hello_world import PORT

ROOT = Path(__file__).resolve().parent
PROFILE = ROOT / 'calibration/arm2_reference.json'


def summarize(samples):
    if len(samples) < 5 or any(len(row) != 6 for row in samples):
        raise ValueError('Need at least five six-joint samples')
    if any(not math.isfinite(v) for row in samples for v in row):
        raise ValueError('Nonfinite joint reading')
    spans = [max(col) - min(col) for col in zip(*samples)]
    if max(spans) > math.radians(.5):
        raise ValueError('Arm moved more than 0.5 degrees during capture; support it and retry')
    return [statistics.median(col) for col in zip(*samples)], spans


def direction(reference, observed, joint, model_delta_deg):
    if not math.isfinite(model_delta_deg) or not 5 <= abs(model_delta_deg) <= 20:
        raise ValueError('Measured model-angle change must be 5–20 degrees')
    delta = [b-a for a,b in zip(reference, observed)]
    if any(abs(v) > math.radians(1) for i,v in enumerate(delta) if i != joint-1):
        raise ValueError('Other joints changed more than 1 degree; repeat the single-joint measurement')
    actual = delta[joint-1]
    expected = math.radians(model_delta_deg)
    if abs(abs(actual)-abs(expected)) > math.radians(1):
        raise ValueError('Measured model-angle change and motor change differ by more than 1 degree')
    return 1 if actual * expected > 0 else -1


# Directions in the DM URDF at the recorded zero/reference pose.
# Positive axes in the base frame: +Z, -Y, +Y, +Y, -Z, +X.
DIRECTION_STEPS = {
    1: (1, 'BASE: rotate the entire arm COUNTERCLOCKWISE as viewed from above; keep the base plate fixed.'),
    2: (-1, 'SHOULDER: raise the elbow by rotating the lower long link upward at the shoulder.'),
    3: (-1, 'ELBOW: hold the lower long link fixed; lift the wrist by rotating the upper long link upward at the elbow.'),
    4: (-1, 'WRIST BEND: keep both long links fixed; tilt the gripper nose upward at the wrist bend.'),
    5: (1, 'WRIST YAW: swivel only the gripper assembly CLOCKWISE as viewed from above.'),
    6: (1, 'WRIST ROLL: looking straight at the fingertips toward the wrist, roll the gripper COUNTERCLOCKWISE. Keep hands and face clear of the tips.'),
}


def qualitative_direction(reference, observed, joint, model_direction):
    if len(reference) != 6 or len(observed) != 6 or joint not in DIRECTION_STEPS or model_direction not in (-1, 1):
        raise ValueError('Invalid direction-check inputs')
    if not all(math.isfinite(v) for v in [*reference, *observed]):
        raise ValueError('Nonfinite joint reading')
    delta = [b-a for a,b in zip(reference, observed)]
    others = [str(i+1) for i,v in enumerate(delta) if i != joint-1 and abs(v) > math.radians(2)]
    if others:
        raise ValueError('Other joints moved more than 2 degrees: ' + ', '.join(others))
    actual = delta[joint-1]
    if not math.radians(3) <= abs(actual) <= math.radians(15):
        raise ValueError('Selected joint must move 3–15 degrees; aim for a small movement of about 5 degrees')
    return (1 if actual > 0 else -1) * model_direction


def guided_directions(port, selected=None):
    data = json.loads(PROFILE.read_text())
    if data.get('physical_arm') != 'second' or data.get('schema_version') != 1:
        raise ValueError('Expected second-arm reference profile')
    reference = data['reference_raw_rad']
    joints = [selected] if selected else [i for i in DIRECTION_STEPS if data['signs'][i-1] is None]
    if not joints:
        print('All directions recorded. Geometry validation and camera/fingertip calibration still pending.')
        return
    print('READ ONLY: support the disabled arm throughout. No motor or zero settings will change.')
    print('Each step starts from the SAME reference pose. Move only the named joint, about 5 degrees.')
    print('If the described movement is obstructed or unclear, stop with Ctrl+C; do not force it.')
    with Reader(port) as reader:
        for joint in joints:
            model_direction, instruction = DIRECTION_STEPS[joint]
            while True:
                print(f'\nJoint {joint}: return ALL joints to the recorded reference pose.')
                input('Press Enter when supported at reference (Ctrl+C stops): ')
                baseline, _ = reader.capture()
                differences = [math.degrees(a-b) for a,b in zip(baseline,reference)]
                if max(abs(v) for v in differences) > 1:
                    print('Not yet within 1 degree of reference. Differences by joint:',
                          ' '.join(f'J{i+1}:{v:+.1f}°' for i,v in enumerate(differences)))
                    continue
                print(instruction)
                print('Move by hand about 5 degrees, then hold steady. Keep other joint angles fixed.')
                if input('Type DONE after completing the described movement: ').strip().upper() != 'DONE':
                    print('Not recorded; return to reference to retry.')
                    continue
                observed, spans = reader.capture()
                try:
                    sign = qualitative_direction(baseline, observed, joint, model_direction)
                except ValueError as error:
                    print(f'Not recorded: {error}. Return to reference and retry.')
                    continue
                i = joint-1
                data['signs'][i] = sign
                data['software_offsets_rad'][i] = data['reference_model_rad'][i] - sign*reference[i]
                data['direction_checks'][str(joint)] = {
                    'method':'operator_observed_physical_direction', 'instruction':instruction,
                    'baseline_raw_rad':baseline, 'raw_rad':observed, 'sample_span_rad':spans,
                    'model_direction':model_direction,
                    'captured_utc':datetime.now(timezone.utc).isoformat(),
                    'angle_magnitude_independently_measured':False}
                data['motion_ready'] = False
                save(data)
                print(f'Saved joint {joint} direction ({sign:+d}). Progress is saved after every joint.')
                break
    print('Done recording directions. Gently rest the supported arm. No motor commands were sent.')
    print('Geometry validation and camera/fingertip calibration are still required before motion.')


class Reader:
    def __enter__(self):
        from motorbridge import Controller
        self.ctrl = Controller.from_dm_serial(self.port, 921600)
        self._handles = []
        self.motors = []
        try:
            for i in range(1,7):
                self.motors.append(self.add_motor(i, 16+i, '4340P' if i < 4 else '4310'))
        except BaseException:
            self.__exit__(*sys.exc_info())
            raise
        return self

    def __init__(self, port):
        self.port = port
        self._handles = []

    def add_motor(self, motor_id, feedback_id, model):
        motor = self.ctrl.add_damiao_motor(motor_id, feedback_id, model)
        self._handles.append(motor)
        return motor

    def read(self):
        values = []
        for i,motor in enumerate(self.motors, 1):
            # Register query waits for a response, unlike cached get_state().
            position = motor.get_register_f32(80, 500)
            motor.request_feedback()
            time.sleep(.01)
            self.ctrl.poll_feedback_once()
            state = motor.get_state()
            if state is None or state.status_code != 0:
                raise RuntimeError(f'Joint {i} is enabled, faulted, or has no feedback. No settings changed.')
            if not math.isfinite(position):
                raise RuntimeError(f'Invalid position for joint {i}')
            values.append(position)
        return values

    def capture(self):
        return summarize([self.read() for _ in range(10)])

    def __exit__(self, *args):
        # Motor handles retain the native controller/bus. Freeing only the
        # controller wrapper leaves the serial port exclusively open.
        errors = []
        try:
            self.ctrl.close_bus()  # transport closure only; no torque commands
        except Exception as error:
            errors.append(error)
        for motor in self._handles:
            try:
                motor.close()
            except Exception as error:
                errors.append(error)
        self._handles = []
        try:
            self.ctrl.close()
        except Exception as error:
            errors.append(error)
        if errors and (not args or args[0] is None):
            raise RuntimeError(f'Serial cleanup failed: {errors[0]}')



def save(data):
    PROFILE.parent.mkdir(exist_ok=True)
    if PROFILE.exists():
        stamp = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        PROFILE.with_name(f'arm2_reference.{stamp}.json').write_bytes(PROFILE.read_bytes())
    temp = PROFILE.with_suffix('.tmp')
    temp.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
    temp.replace(PROFILE)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('action', choices=['inspect', 'reference', 'direction', 'directions', 'show'])
    parser.add_argument('--port', default=PORT)
    parser.add_argument('--joint', type=int, choices=range(1,7))
    parser.add_argument('--model-delta-deg', type=float)
    args = parser.parse_args()
    if args.action == 'directions':
        guided_directions(args.port, args.joint)
        return
    if args.action == 'show':
        print(PROFILE.read_text() if PROFILE.exists() else 'No second-arm reference captured yet.')
        return
    if args.action == 'direction':
        if args.joint is None or args.model_delta_deg is None:
            parser.error('direction requires --joint and --model-delta-deg (independently measured, signed URDF angle)')
        if not math.isfinite(args.model_delta_deg) or not 5 <= abs(args.model_delta_deg) <= 20:
            parser.error('--model-delta-deg must have magnitude 5–20 degrees')
        data = json.loads(PROFILE.read_text())
    if args.action in ('reference','direction'):
        print('Support the disabled arm. Only position queries will be sent.')
        if args.action == 'reference':
            print('Match calibration/seeed_reference_pose.jpg; see calibration/README.md for alignment limits.')
            text = 'REFERENCE'
        else:
            print('Start from the recorded reference; move ONLY the selected joint by the measured signed MODEL angle.')
            text = 'MEASURED'
        if input(f'Type {text} when the physical pose is ready: ').strip() != text:
            print('Cancelled; nothing recorded.')
            return
    with Reader(args.port) as reader:
        q, spans = reader.capture()
    print('Raw motor degrees:', ' '.join(f'{math.degrees(x):+.2f}' for x in q))
    if args.action == 'inspect':
        return
    now = datetime.now(timezone.utc).isoformat()
    if args.action == 'reference':
        data = {'schema_version':1, 'physical_arm':'second', 'captured_utc':now,
                'reference_raw_rad':q, 'reference_model_rad':[0.0]*6,
                'sample_span_rad':spans, 'direction_checks':{},
                'reference_alignment':'operator_matched_photo_not_independently_verified',
                'signs':[None]*6, 'software_offsets_rad':[None]*6,
                'mapping_equation':'q_model[i] = signs[i] * q_raw[i] + software_offsets_rad[i]',
                'motion_ready':False, 'hardware_zero_changed':False}
    else:
        sign = direction(data['reference_raw_rad'], q, args.joint, args.model_delta_deg)
        i = args.joint-1
        data['signs'][i] = sign
        data['software_offsets_rad'][i] = data['reference_model_rad'][i] - sign * data['reference_raw_rad'][i]
        data['direction_checks'][str(args.joint)] = {'raw_rad':q, 'model_delta_deg':args.model_delta_deg,
                                                    'captured_utc':now, 'sample_span_rad':spans}
    save(data)
    print(f'Saved {PROFILE}. Motion remains disabled pending independent geometry validation.')


if __name__ == '__main__':
    try:
        main()
    except (KeyboardInterrupt, EOFError):
        raise SystemExit('Cancelled. Motor settings were not changed.')
    except Exception as error:
        raise SystemExit(f'Not captured: {error}')
