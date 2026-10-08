"""Staged observation from a checked resting pose; preview is the default.

Each stopped waypoint requires a fresh image review before the next segment.
A missing review times out to a controlled retrace. This is not a plate approach.
"""
import argparse
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import threading
import time

import numpy as np
from arm_geometry import Geometry
from axis_follow import AxisArm, axis_bounds
from camera_check import CameraCheck
from pairing_dashboard import Workbench
from printer_observation_lift import LiftRunner


def check_opening_recovery(source, result, supported_contact=False):
    initial = {(r['a'], r['b']): r['distance_m'] for r in result['rows'][0]['below_margin']}
    last, cleared = initial.copy(), set()
    start = np.array(source['rows'][0]['raw_rad'])
    for sample, row in zip(source['rows'], result['rows']):
        near = {(r['a'], r['b']): r for r in row['below_margin']}
        for pair, item in near.items():
            if (not sample['stage'].startswith('unfold') or pair not in initial or
                    pair in cleared or item['distance_m'] < last[pair]-1e-7):
                raise ValueError('Self-clearance does not monotonically recover from rest')
            if item['intersects']:
                delta = np.degrees(np.array(sample['raw_rad'])-start)
                if (not supported_contact or pair != ('link2', 'link5') or
                        not -.200001 <= delta[2] <= 0 or np.max(abs(delta[[0, 1, 3, 4, 5]])) > 1e-8):
                    raise ValueError('Intersection outside the verified supported-rest opening')
            last[pair] = item['distance_m']
        cleared.update(set(initial)-set(near))
    if set(initial) != cleared:
        raise ValueError('Initial margin never recovered')


def load_plan(folder):
    plan = json.loads((folder/'plan.json').read_text())
    g = Geometry()
    qs = np.array([p['raw_deg'] for p in plan['poses']], float)
    if (qs.ndim != 2 or qs.shape[1] != 6 or len(qs) < 2 or
            not np.isfinite(qs).all() or plan['geometry_fingerprint'] != g.fingerprint):
        raise ValueError('Invalid observation plan')
    if np.max(abs(qs[0, :3]-np.degrees(g.profile['reference_raw_rad'])[:3])) > 1:
        raise ValueError('Observation must begin at normal rest')
    limits = {i: axis_bounds(g, i, qs[0, i]) for i in range(6)}
    # Explicit local observation ranges, still intersected with URDF limits.
    compact = plan.get('observation_mode') == 'compact_wrist_pan'
    for i, radius in ([(0, 20.), (3, 70.)] if compact else [(0, 80.), (3, 55.)]):
        lim = g.joints[i].find('limit')
        a, b = sorted((np.degrees(float(lim.get(k)))-np.degrees(g.offsets[i]))/g.signs[i]
                      for k in ('lower', 'upper'))
        limits[i] = (max(qs[0, i]-radius, a+.25), min(qs[0, i]+radius, b-.25))
    if any(np.any(qs[:, i] < limits[i][0]) or np.any(qs[:, i] > limits[i][1])
           for i in range(6)):
        raise ValueError('Observation leaves joint bounds')
    if not compact and np.any(abs(qs[:, 4:]-qs[0, 4:]) > 1e-8):
        raise ValueError('Wrist yaw and roll must remain unchanged')
    if compact and (np.any(abs(qs[:, 5]-qs[0, 5]) > 1e-8) or
                    np.any(abs(qs[:, 0]-qs[0, 0]) > 10.) or np.any(qs[:, 4] < -40.000001)):
        raise ValueError('Compact observation leaves the checked base/yaw/roll scope')
    if np.any(np.sum(abs(np.diff(qs, axis=0)) > 1e-8, axis=1) != 1):
        raise ValueError('Only single-axis observation segments are supported')
    source = json.loads((folder/'stock_input.json').read_text())
    result = json.loads((folder/'stock_check.json').read_text())
    if (result['input_sha256'] != hashlib.sha256((folder/'stock_input.json').read_bytes()).hexdigest()
            or source['geometry_fingerprint'] != g.fingerprint
            or len(source['rows']) != len(result['rows'])):
        raise ValueError('Stock evidence mismatch')
    for mesh in source['meshes']:
        if result['mesh_sha256'][mesh['name']] != hashlib.sha256(Path(mesh['mesh']).read_bytes()).hexdigest():
            raise ValueError('Stock mesh changed after collision checking')
    # Bind the checked samples to this exact route, including all endpoints.
    for index, pose in enumerate(plan['poses'][1:]):
        rows = [r for r in source['rows'] if r['stage'] == pose['name']]
        expected_n = int(np.ceil(np.max(abs(qs[index+1]-qs[index]))/.1))+1
        if len(rows) != expected_n:
            raise ValueError('Missing dense segment evidence')
        expected = np.radians(np.linspace(qs[index], qs[index+1], expected_n))
        if not np.allclose([r['raw_rad'] for r in rows], expected, atol=1e-10, rtol=0):
            raise ValueError('Checked route does not match plan')
    supported_contact = False
    if plan.get('supported_rest_return_evidence'):
        evidence = json.loads(Path(plan['supported_rest_return_evidence']).read_text())
        supported_contact = (evidence.get('returned_to_start') is True and
                             evidence.get('motors_disabled_verified') is True and
                             np.max(abs(qs[0]-evidence['final_raw_deg'])) < 1.)
        if not supported_contact:
            raise ValueError('Supported resting-pose return evidence does not match')
    check_opening_recovery(source, result, supported_contact)
    for kind in ('tool', 'camera'):
        path = folder/f'{kind}_input.json'
        check = json.loads((folder/f'{kind}_check.json').read_text())
        if check['input_sha256'] != hashlib.sha256(path.read_bytes()).hexdigest() or check['near']:
            raise ValueError('Tool/camera evidence mismatch or proximity')
        if compact and kind == 'camera' and 'link4' not in [m['name'] for m in json.loads(path.read_text())['meshes']]:
            raise ValueError('Wrist pan requires camera clearance against link4')
    return plan, qs.tolist(), limits


def review_action(value, index):
    if not isinstance(value, dict) or value.get('completed_stage') != index:
        raise ValueError('Review is for a different stopped waypoint')
    if value.get('action') not in ('advance', 'return'):
        raise ValueError('Unsupported review action')
    if value['action'] == 'advance' and value.get('images_reviewed') is not True:
        raise ValueError('Fresh images must be reviewed before advancing')
    return value['action']


def capture_raw(runner, folder, index, powered=True):
    capture = folder/'raw_stereo'
    request = capture/'request.tmp'
    request.write_text(json.dumps({'stage': index, 'requested_at': time.monotonic()})+'\n')
    request.replace(capture/'request.json')
    deadline = time.monotonic()+6
    while time.monotonic() < deadline:
        if powered:
            runner.tick()
        else:
            if np.max(abs(np.array(runner.arm.read())-runner.start)) > .05:
                raise ValueError('Resting arm moved during raw capture')
            time.sleep(.01)
        if not runner.vision_ready():
            raise ValueError('Camera lost during raw capture')
        report = capture/f'stage_{index:02d}'/'report.json'
        if report.exists() and len(json.loads(report.read_text())['frames']) == 3:
            break
    else:
        raise ValueError('Raw held-pose capture timed out')


def wait_for_review(runner, folder, index, name):
    capture_raw(runner, folder, index)
    runner.save_view(f'stage_{index:02d}_{name}')
    status = {'completed_stage': index, 'name': name,
              'review_file': str(folder/f'review_{index:02d}.json'),
              'timeout_s': 90, 'timeout_action': 'retrace', 'motors_holding': True}
    (folder/'waiting.json').write_text(json.dumps(status, indent=2)+'\n')
    print('REVIEW', json.dumps(status), flush=True)
    deadline = time.monotonic()+90
    path = folder/f'review_{index:02d}.json'
    while time.monotonic() < deadline:
        runner.tick()
        if not runner.vision_ready():
            raise ValueError('Camera lost during waypoint review')
        if path.exists():
            return review_action(json.loads(path.read_text()), index)
    raise ValueError('Waypoint image review timed out')


def execute(folder):
    plan, poses, limits = load_plan(folder)  # All disk-heavy checks before motor enable.
    output = folder/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output.mkdir()
    print('OUTPUT', output, flush=True)
    w = Workbench()
    w.track_printer_target = True
    w.printer_capture_root = output/'raw_stereo'
    w.printer_capture_root.mkdir()
    w.camera_check = CameraCheck(w.geometry, output/'unused_tag_check')
    workers = [threading.Thread(target=w.camera_worker, args=(role,), daemon=True)
               for role in ('wrist', 'tripod')]
    for worker in workers:
        worker.start()
    report = {'scope': plan['scope'], 'gripper_commanded': False, 'rail_commanded': False,
              'returned_to_start': False, 'motors_disabled_verified': False,
              'completed_stages': [], 'workspace_revalidation_complete': False}
    runner = None
    try:
        with AxisArm() as arm:
            start = arm.read()
            if np.max(abs(np.array(start)-poses[0])) > .05:
                raise ValueError('Resting pose changed; replan before enabling motors')
            grip = arm.add_motor(7, 23, '4310')
            position = grip.get_register_f32(80, 300)
            grip.request_feedback()
            time.sleep(.01)
            arm.ctrl.poll_feedback_once()
            state = grip.get_state()
            if (state is None or state.status_code != 0 or not np.isfinite(position)
                    or abs(position-plan['gripper_raw_rad']) > .003):
                raise ValueError('Disabled gripper opening changed')
            runner = LiftRunner(w, arm, start, limits, output, poses)
            report['start_raw_deg'] = start
            deadline = time.monotonic()+25
            while not runner.vision_ready():
                if time.monotonic() > deadline:
                    raise ValueError('Fresh cameras unavailable')
                if np.max(abs(np.array(arm.read())-start)) > .05:
                    raise ValueError('Arm moved during camera startup')
                time.sleep(.01)
            runner.save_view('before')
            capture_raw(runner, output, 0, powered=False)
            arm.prepare_group(range(6))
            if np.max(abs(np.array(arm.read())-start)) > .05 or not runner.vision_ready():
                raise ValueError('Pose/camera changed before enable')
            arm.speed_limits = {i: 10. for i in range(6)}
            arm.command_speed_limits = {i: 2. for i in range(6)}
            arm.enable_group(dict(enumerate(start)))
            runner.last = time.monotonic()
            until = time.monotonic()+1
            while time.monotonic() < until:
                runner.tick(take_up=True)
            visited = [start]
            try:
                for index, goal in enumerate(poses[1:], 1):
                    # Append before travel so a partial segment reverses to its origin.
                    visited.append(goal)
                    runner.go(goal)
                    report['completed_stages'].append(index)
                    if wait_for_review(runner, output, index, plan['poses'][index]['name']) == 'return':
                        report['stop_reason'] = 'Image review requested return'
                        break
            except ValueError as error:
                report['stop_reason'] = str(error)
                print('RETRACE', str(error), flush=True)
            for goal in reversed(visited[:-1]):
                runner.go(goal, returning=True)
            runner.save_view('returned')
            report['before_disable_raw_deg'] = arm.read()
            report['returned_to_start'] = True
        with AxisArm() as reader:
            report['final_raw_deg'] = reader.read()
            report['motors_disabled_verified'] = True
    except BaseException as error:
        report['error'] = repr(error)
        raise
    finally:
        w.stop.set()
        for worker in workers:
            worker.join(timeout=16)
        if runner:
            report['motion_samples'] = runner.rows
        (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print('REPORT', output/'report.json', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('plan_folder', type=Path)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.execute:
        execute(args.plan_folder)
    else:
        plan, _, _ = load_plan(args.plan_folder)
        print(json.dumps(plan, indent=2))
