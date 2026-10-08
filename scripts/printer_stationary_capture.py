#!/usr/bin/env python3
"""Read-only stereo capture bracketed by disabled-arm and gripper readback."""
import json
import argparse
import math
from datetime import datetime, timezone
from pathlib import Path
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from calibrate_arm import Reader
from hello_world import PORT
from printer_target_tracker import validate_stationary_capture


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mono-exposure-us', type=int, default=2000)
    args = parser.parse_args()
    if not 100 <= args.mono_exposure_us <= 20000:
        parser.error('Exposure must be 100–20000 microseconds')
    folder = ROOT/'outputs/contact_camera'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    folder.mkdir(parents=True)
    record = {'motion_commanded': False, 'motion_ready': False, 'readings': []}
    child = None
    try:
        with Reader(PORT) as reader, (folder/'camera.log').open('w') as log:
            grip = reader.add_motor(7, 23, '4310')
            def sample():
                began = time.monotonic()
                q = reader.read()  # Rejects enabled/faulted/missing joint feedback.
                g = grip.get_register_f32(80, 300)
                if not math.isfinite(g):
                    raise RuntimeError('Nonfinite gripper readback')
                grip.request_feedback(); time.sleep(.01); reader.ctrl.poll_feedback_once()
                state = grip.get_state()
                if state is None or state.status_code != 0:
                    raise RuntimeError('Gripper must have disabled, valid feedback')
                record['readings'].append({'start_s': began, 'end_s': time.monotonic(),
                                          'arm_raw_rad': q, 'gripper_raw_rad': g})
            sample()
            child = subprocess.Popen([str(ROOT/'.venv-depth-v2/bin/python'),
                str(ROOT/'contact_camera_probe_v2.py'), '--tag-id', '18', '--tag-size', '.018',
                '--mono-exposure-us', str(args.mono_exposure_us), '--mono-iso', '100', '--output', str(folder)],
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
            deadline = time.monotonic()+40
            while child.poll() is None:
                sample()
                if time.monotonic() > deadline:
                    raise RuntimeError('Camera capture timed out')
                time.sleep(.04)
            sample()
            if child.returncode:
                raise RuntimeError('Camera capture failed; inspect camera.log')
        report = json.loads((folder/'report.json').read_text())
        times = [c['observation_monotonic_s'] for f in report['frames'] for c in f['cameras'].values()]
        record.update(validate_stationary_capture(record['readings'], times))
        record['valid_stereo_frames'] = sum('T_camera_b_tag' in f['stereo_tag_pose'] for f in report['frames'])
    except Exception as error:
        record.update(stationary=False, error=str(error))
        raise
    finally:
        if child is not None and child.poll() is None:
            child.terminate()
            try: child.wait(timeout=10)
            except subprocess.TimeoutExpired: child.kill(); child.wait(timeout=3)
        (folder/'joint_capture.json').write_text(json.dumps(record, indent=2, allow_nan=False)+'\n')
        print(json.dumps({'folder':str(folder), **{k:v for k,v in record.items() if k!='readings'}},indent=2))


if __name__ == '__main__': main()
