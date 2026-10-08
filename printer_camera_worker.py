"""Parent-side read-only bridge to the proven DepthAI 2 stereo runtime."""
import base64
import json
from pathlib import Path
import select
import subprocess
import time

from printer_approach import rigid
from printer_target_tracker import compare_reference, stationary_joint_pose
from printer_target_plan import register_observation

ROOT = Path(__file__).resolve().parent


def camera_worker(workbench, config):
    process = None
    try:
        X = rigid(json.loads((ROOT/'calibration/stereo_sweep_handeye_20261004.json').read_text())['T_wrist_camera'])
        session = workbench.session+':'+str(time.monotonic_ns())
        baseline = None; revision = 0; last = time.monotonic(); last_stamp = -1.
        last_rail_motion = time.monotonic()
        rail_revision = None
        command = [str(ROOT/'.venv-depth-v2/bin/python'), '-u', str(ROOT/'printer_camera_stream_v2.py')]
        capture_root = getattr(workbench, 'printer_capture_root', None)
        if capture_root is not None:
            command += ['--capture-root', str(capture_root)]
        process = subprocess.Popen(command, cwd=ROOT, stdout=subprocess.PIPE,
            text=True, bufsize=1)
        while not workbench.stop.is_set():
            if process.poll() is not None:
                raise RuntimeError(f'Stereo camera subprocess exited ({process.returncode})')
            if time.monotonic()-last > 20:
                raise RuntimeError('Stereo camera stream timed out')
            if not select.select([process.stdout], [], [], .1)[0]:
                continue
            line = process.stdout.readline()
            if not line.startswith('{'):
                continue  # Native SDK informational logs are not camera records.
            row = json.loads(line)
            if row.get('kind') != 'printer_camera_frame':
                continue
            rail = getattr(workbench, 'rail', None)
            if rail is not None:
                with workbench.lock:
                    if rail.busy() or rail.state != 'Idle' or rail.commands != rail_revision:
                        last_rail_motion = time.monotonic()
                    rail_revision = rail.commands
                    rail_state = dict(connected=rail.connected, state=rail.state,
                        error=rail.error, x_mm=rail.x_mm, commands=rail.commands,
                        status_time=rail.status_time, last_motion_time=last_rail_motion)
                    history = list(workbench.follower_history)
                plate = dict(row.get('plate') or {})
                plate['session'] = session
                try:
                    registration = register_observation(plate, history, workbench.geometry,
                        X, rail_state, time.monotonic(), session)
                    plate['registration'] = registration
                except (ValueError, KeyError, TypeError) as error:
                    plate['registration_error'] = str(error)
                if not 0 <= time.monotonic()-plate.get('observation_time', -1.) <= .5:
                    plate.update(valid=False, reason='Plate observation expired; reacquire')
                    plate.pop('registration', None)
                workbench.publish('printer_plate', **plate)
                target_preview = getattr(workbench, 'target_preview', None)
                if target_preview is not None:
                    target_preview.observe(plate)
            rgb_age = time.monotonic()-row['rgb_observation_time']
            if not 0 <= rgb_age <= .5:
                # Drop a delayed frame, mark the feed unavailable, and drain
                # toward live frames. Motion gates still pause immediately;
                # a temporary scheduling delay must not kill the camera owner.
                workbench.error('wrist', RuntimeError('RGB overview is stale'))
                workbench.error('printer_target', RuntimeError('RGB overview is stale'))
                continue
            target = row['target']; target['session'] = session
            age = time.monotonic()-target['observation_time']
            if not 0 <= age <= .25 or target['observation_time'] <= last_stamp:
                target.update(valid=False, reason='Stale or nonadvancing stereo observation')
            last_stamp = max(last_stamp, target['observation_time'])
            if target['valid']:
                with workbench.lock:
                    history = list(workbench.follower_history)
                try:
                    q = stationary_joint_pose(history, target['observation_time'])
                    base_tag = workbench.geometry.transform(q)@X@rigid(target['T_camera_b_tag'])
                    target['T_base_tag_candidate'] = base_tag.tolist()
                    if baseline is None:
                        baseline = base_tag.copy()
                    change = compare_reference(baseline, base_tag)
                    target['reference_change'] = change
                    if change['requires_replan']:
                        revision += 1; baseline = base_tag.copy()
                    target['base_pose_scope'] = 'Stationary arm only; fixed current rail position'
                except ValueError as error:
                    target['base_pose_reason'] = str(error)
            target.update(scene_revision=revision, frame_age_s=age, motion_ready=False,
                          scope='Tag observation only; tag-to-plate and collision scene remain unvalidated')
            workbench.publish('printer_target', **target)
            rgb_age = time.monotonic()-row['rgb_observation_time']
            if not 0 <= rgb_age <= .5:
                workbench.error('wrist', RuntimeError('RGB overview is stale'))
                continue
            with workbench.lock:
                workbench.images['wrist'] = base64.b64decode(row['jpeg_base64'], validate=True)
            workbench.publish('wrist', device_id=config['device_id'], frame_age_s=rgb_age,
                visible_tags=row['visible_tags'], tag='Printer reference: stereo metric tracking',
                runtime='DepthAI 2.32 camera subprocess')
            last = time.monotonic()
    except Exception as error:
        workbench.error('wrist', error)
        workbench.error('printer_target', error)
    finally:
        if process is not None:
            if process.poll() is None:
                process.terminate()
                try:
                    process.wait(timeout=10)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait(timeout=3)
            if process.stdout:
                process.stdout.close()
