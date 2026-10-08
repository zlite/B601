"""Bounded eight-degree elbow lift from the freshly recorded normal rest.

Preview by default. This checks the first, upward part of the observation route
and returns along that same segment. No base turn, grasp or rail movement.
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
from camera_visual_approach import RouteRunner
from pairing_dashboard import Workbench

CAPTURE = Path('outputs/contact_camera/20261006T213644834808Z')


def checked_poses(geometry, start, saved):
    start, saved = np.asarray(start, float), np.asarray(saved, float)
    if (start.shape != (6,) or saved.shape != (6,) or
            not np.isfinite(start).all() or not np.isfinite(saved).all() or
            np.max(abs(start-saved)) > .05):
        raise ValueError('Current pose must match the stationary capture within 0.05 degrees')
    reference = np.degrees(geometry.profile['reference_raw_rad'])
    if np.max(abs(start[:3]-reference[:3])) > 1:
        raise ValueError('This diagnostic requires the normal folded resting pose')
    limits = {i: axis_bounds(geometry, i, start[i]) for i in range(6)}
    goal = start.copy()
    goal[2] -= 8.
    if any(not limits[i][0] <= goal[i] <= limits[i][1] for i in range(6)):
        raise ValueError('Lift leaves the existing joint bounds')
    return [start.tolist(), goal.tolist()], limits


def check_evidence():
    path = CAPTURE/'raised_search_mesh_input.json'
    route = json.loads(path.read_text())
    result = json.loads((CAPTURE/'raised_search_self_check.json').read_text())
    tool = json.loads((CAPTURE/'raised_search_tool_self_check.json').read_text())
    if (route['geometry_fingerprint'] != Geometry().fingerprint or
            result['input_sha256'] != hashlib.sha256(path.read_bytes()).hexdigest() or
            len(result['rows']) != len(route['rows']) or tool['near']):
        raise ValueError('Geometry evidence changed or finger clearance check failed')
    initial = {(x['a'], x['b']): x['distance_m'] for x in result['rows'][0]['below_margin']}
    last, cleared = initial.copy(), set()
    for sample, report in zip(route['rows'], result['rows']):
        if sample['stage'] != 'unfold':
            break
        near = {(x['a'], x['b']): x for x in report['below_margin']}
        for pair, value in near.items():
            if (pair not in initial or pair in cleared or value['intersects'] or
                    value['distance_m'] < last[pair]-1e-7):
                raise ValueError('Initial tight clearance does not recover monotonically')
            last[pair] = value['distance_m']
        cleared.update(set(initial)-set(near))
    if cleared != set(initial):
        raise ValueError('Folded clearance margin did not recover')
    capture = json.loads((CAPTURE/'joint_capture.json').read_text())
    if capture.get('stationary') is not True:
        raise ValueError('No verified stationary starting capture')
    return np.degrees(capture['arm_raw_rad']).tolist()


class LiftRunner(RouteRunner):
    def __init__(self, *args):
        super().__init__(*args)
        self.speed = 1.
        self.acceleration = 2.
        self.tracking_limit = 1.
        self.pacing_lag = .6
        self.settle_tolerance = np.full(6, .2)

    def vision_ready(self):
        # Tag absence is expected while acquiring a viewing pose. Both actual
        # image streams must remain fresh; this is not a plate-targeting gate.
        with self.w.lock:
            now = time.monotonic()
            for role in ('wrist', 'tripod'):
                source = self.w.sources.get(role, {})
                age = now-source.get('time', 0)
                if ('error' in source or age < 0 or
                        age+source.get('frame_age_s', 99) > .4 or
                        not self.w.images.get(role)):
                    return False
        return True

    def tick(self, moving=False, take_up=False):
        q = super().tick(moving=moving, take_up=take_up)
        self.w.publish('follower', angles=q)
        return q

    def save_view(self, name):
        with self.w.lock:
            for role, data in self.w.images.items():
                (self.output/f'{name}_{role}.jpg').write_bytes(data)
            target = dict(self.w.sources.get('printer_target', {}))
        (self.output/f'{name}_target.json').write_text(json.dumps(target, indent=2)+'\n')


def execute():
    expected = check_evidence()
    folder = Path('outputs/printer_observation_lift')/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    folder.mkdir(parents=True)
    w = Workbench()
    w.track_printer_target = True
    w.camera_check = CameraCheck(w.geometry, folder/'unused_tag_check')
    workers = [threading.Thread(target=w.camera_worker, args=(role,), daemon=True)
               for role in ('wrist', 'tripod')]
    for worker in workers:
        worker.start()
    report = {'scope': 'Eight-degree upward elbow observation and controlled return only',
              'gripper_commanded': False, 'rail_commanded': False,
              'returned_to_start': False, 'motors_disabled_verified': False,
              'global_workspace_revalidation_complete': False}
    runner = None
    try:
        with AxisArm() as arm:
            start = arm.read()
            grip = arm.add_motor(7, 23, '4310')
            gripper_position = grip.get_register_f32(80, 300)
            grip.request_feedback()
            time.sleep(.01)
            arm.ctrl.poll_feedback_once()
            gripper_state = grip.get_state()
            saved_grip = json.loads((CAPTURE/'joint_capture.json').read_text())['gripper_raw_rad']
            if (gripper_state is None or gripper_state.status_code != 0 or
                    not np.isfinite(gripper_position) or abs(gripper_position-saved_grip) > .003):
                raise RuntimeError('Gripper must remain disabled at the captured opening')
            report['gripper_start_raw_rad'] = gripper_position
            poses, limits = checked_poses(w.geometry, start, expected)
            runner = LiftRunner(w, arm, start, limits, folder, poses)
            report.update(start_raw_deg=start, goal_raw_deg=poses[-1])
            deadline = time.monotonic()+25
            while not runner.vision_ready():
                if time.monotonic() > deadline:
                    raise RuntimeError('Fresh overview and wrist images unavailable')
                if max(abs(a-b) for a, b in zip(arm.read(), start)) > .05:
                    raise RuntimeError('Resting pose changed during camera setup')
                time.sleep(.01)
            runner.save_view('before')
            arm.prepare_group(range(6))
            checked_poses(w.geometry, arm.read(), start)
            if not runner.vision_ready():
                raise RuntimeError('Cameras became stale before motor enable')
            arm.speed_limits = {i: 10. for i in range(6)}
            arm.command_speed_limits = {i: 2. for i in range(6)}
            arm.enable_group(dict(enumerate(start)))
            runner.last = time.monotonic()
            until = time.monotonic()+1
            while time.monotonic() < until:
                runner.tick(take_up=True)
            try:
                runner.go(poses[-1])
                until = time.monotonic()+3
                while time.monotonic() < until:
                    if not runner.vision_ready():
                        raise ValueError('Camera stream lost at observation pose')
                    runner.tick()
                runner.save_view('raised')
                report['reached_observation_pose'] = True
                print('Raised observation captured', folder, flush=True)
            except ValueError as error:
                report['observation_error'] = str(error)
            # This reverses only the same single-joint segment just traversed.
            runner.go(start, returning=True)
            report['returned_to_start'] = True
            report['before_disable_raw_deg'] = arm.read()
            runner.save_view('returned')
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
        (folder/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
        print('Observation lift report:', folder/'report.json', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute', action='store_true')
    args = parser.parse_args()
    if args.execute:
        execute()
    else:
        saved = check_evidence()
        poses, limits = checked_poses(Geometry(), saved, saved)
        print(json.dumps({'start_raw_deg': poses[0], 'goal_raw_deg': poses[-1],
                          'returns_along_same_segment': True, 'motion_commanded': False}, indent=2))
