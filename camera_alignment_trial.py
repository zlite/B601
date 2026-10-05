#!/usr/bin/env python3
"""Bounded wrist-only camera check from the current supported folded pose.

Run only with the dashboard stopped. Preview is hardware-free; --execute owns
both cameras and the motor bus, returns to the starting pose, then removes torque.
No stored home route, calibration transform, gripper, or leader command is used.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time

from arm_geometry import Geometry
from axis_follow import AxisArm, axis_bounds
from camera_check import CameraCheck, BEND_STEPS
from pairing_dashboard import Workbench
from wrist_follow import WristTrajectory


def plan(geometry, start, bend_repeat=False):
    if len(start) != 6 or not all(math.isfinite(x) for x in start):
        raise ValueError('Expected six finite starting angles')
    reference = [math.degrees(x) for x in geometry.profile['reference_raw_rad']]
    if any(abs(start[i]-reference[i]) > 1 for i in (1,2)):
        raise ValueError('Shoulder and elbow must already be in the supported folded pose')
    limits = {i:axis_bounds(geometry,i,start[i]) for i in range(6)}
    poses = [list(start)]
    moves = ((3,-8.),(3,8.),(3,-8.)) if bend_repeat else ((5,8.),(4,8.),(3,-8.))
    for joint, delta in moves:
        q = poses[-1].copy()
        q[joint] += delta
        if not limits[joint][0] <= q[joint] <= limits[joint][1]:
            raise ValueError(f'Joint {joint+1} diagnostic would exceed its existing limits')
        poses.append(q)
    return poses


def recovery_target(geometry, current, report):
    """Recover only a small wrist displacement; never replay a large-arm route."""
    target = report.get('start_raw_deg',[])
    if len(target) != 6 or not all(math.isfinite(x) for x in target):
        raise ValueError('Invalid recovery target')
    plan(geometry,current)  # verifies the supported folded starting pose
    for i in range(6):
        if abs(current[i]-target[i]) > (.1 if i<3 else 10):
            raise ValueError('Recovery exceeds the wrist-only envelope')
        low,high = axis_bounds(geometry,i,current[i])
        if i>=3 and not low<=target[i]<=high:
            raise ValueError('Recovery target outside model limits')
    return list(current[:3])+list(target[3:])


class Trial:
    def __init__(self, workbench, arm, start):
        self.w, self.arm, self.start = workbench, arm, start
        self.targets = dict(enumerate(start))
        self.last = time.monotonic()
        self.rows = []
        self.images = {}
        self.max_error = 0.

    def tick(self, following=False, trajectories=None, goal=None, take_up=False):
        q = self.arm.read()
        now = time.monotonic()
        dt = now-self.last
        if not 0 < dt < .3:
            raise RuntimeError('Motor loop stalled')
        for i in range(6):
            error = abs(q[i]-self.targets[i])
            self.max_error = max(self.max_error,error)
            if error > 3:
                raise RuntimeError(f'Joint {i+1} tracking error {error:.2f} degrees')
            if abs(q[i]-self.start[i]) > (1 if i < 3 else 10):
                raise RuntimeError(f'Joint {i+1} left the diagnostic envelope')
        if trajectories is not None:
            self.targets = {i:trajectories[i].step(goal[i],dt) for i in range(6)}
        self.arm.command_group(self.targets,take_up=take_up)
        self.w.publish('follower',angles=q)
        self.w.camera_check.add_joints(now,q,powered=True,following=following,all_powered=True,fault=None)
        self.last = now
        self.rows.append({'time':now,'raw_deg':q,'target_deg':list(self.targets.values()),
                          'velocity_deg_s':list(self.arm.joint_velocities_deg_s),'moving':following})
        time.sleep(.005)
        return q

    def cameras_fresh(self):
        with self.w.lock:
            now = time.monotonic()
            return all('error' not in self.w.sources.get(role,{}) and
                now-self.w.sources.get(role,{}).get('time',0) < .5 and
                0 <= self.w.sources.get(role,{}).get('frame_age_s',99) < .5
                for role in ('wrist','tripod'))

    def move(self, goal, returning=False):
        trajectories = {i:WristTrajectory(self.start[i],self.targets[i],
            low=self.start[i]-(.01 if i<3 else 10), high=self.start[i]+(.01 if i<3 else 10),
            speed=4.,acceleration=12.) for i in range(6)}
        deadline = time.monotonic()+10
        while time.monotonic() < deadline:
            if not returning and not self.cameras_fresh():
                raise ValueError('Camera stream stale; returning along the recorded joint path')
            q = self.tick(True,trajectories,goal)
            if max(abs(self.targets[i]-goal[i]) for i in range(6)) < .03 and max(abs(q[i]-goal[i]) for i in range(6)) < .6:
                return
        raise ValueError('Joint movement failed to settle; return to start')

    def capture(self):
        self.w.camera_check.arm_capture()
        count = len(self.w.camera_check.samples)
        deadline = time.monotonic()+15
        while time.monotonic() < deadline:
            self.tick()
            status = self.w.camera_check.snapshot()
            if status['step'] > count:
                print(json.dumps({'saved':status['step'],'results':status['results']}),flush=True)
                with self.w.lock:
                    for role,data in self.w.images.items():
                        self.images[f'{role}_stage{count}.jpg'] = data
                return
            if not self.cameras_fresh():
                raise ValueError('Camera stream stale while capturing')
        raise ValueError('Capture timed out: '+status['message'])


def execute(return_report=None,wrist_width=1280,capture_imu=False,bend_repeat=False):
    w = Workbench()
    w.capture_imu = capture_imu
    w.imu_rows = []
    if wrist_width == 1920:
        w.camera_sizes = {'wrist':(1920,1200)}
        w.camera_fps = {'wrist':8}
    w.camera_check = CameraCheck(w.geometry,steps=BEND_STEPS if bend_repeat else None)
    workers = [threading.Thread(target=w.camera_worker,args=(role,),daemon=True) for role in ('wrist','tripod')]
    for worker in workers: worker.start()
    result = {'utc':datetime.now(timezone.utc).isoformat(),'session':w.camera_check.session,
              'wrist_width':wrist_width,
              'scope':'Wrist relative-rotation diagnostic; no calibration applied',
              'motors_disabled_verified':False,'returned_to_start':False}
    path = Path('outputs/camera_diagnosis')/f'autonomous_trial_{w.camera_check.session}.json'
    trial = None
    try:
        with AxisArm() as arm:
            start = arm.read()
            poses = plan(w.geometry,start,bend_repeat)
            home = recovery_target(w.geometry,start,json.loads(return_report.read_text())) if return_report else start
            if return_report:
                result['requested_recovery_raw_deg'] = home
            result.update(start_raw_deg=start,planned_raw_deg=poses)
            # Establish fresh stable camera/joint readings before any motor enable.
            deadline = time.monotonic()+25
            while time.monotonic() < deadline:
                q = arm.read()
                if max(abs(a-b) for a,b in zip(q,start)) > .1:
                    raise RuntimeError('Supported pose moved during camera setup')
                w.camera_check.add_joints(time.monotonic(),q,powered=False,following=False,all_powered=False,fault=None)
                if w.camera_check.snapshot()['ready']:
                    break
                time.sleep(.005)
            else:
                raise RuntimeError('Cameras did not become ready before enable')
            arm.prepare_group(range(6))
            q = arm.read()
            if max(abs(a-b) for a,b in zip(q,start)) > .1:
                raise RuntimeError('Supported pose changed during motor setup')
            if capture_imu:
                with w.lock:
                    if len(w.imu_rows)<100 or time.monotonic()-w.imu_rows[-1]['gyro_time']>.3:
                        raise RuntimeError('Independent IMU readings unavailable before enable')
            arm.speed_limits = {i:20. for i in range(6)}
            arm.enable_group(dict(enumerate(start)))
            trial = Trial(w,arm,start)
            deadline = time.monotonic()+.8
            while time.monotonic() < deadline:
                q = trial.tick(take_up=True)
            if max(abs(a-b) for a,b in zip(q,start)) > 1:
                raise RuntimeError('Arm did not settle during load take-up')
            visited = [poses[0]]
            try:
                if return_report:
                    trial.move(home,returning=True)
                else:
                    trial.capture()
                    for pose in poses[1:]:
                        visited.append(pose)  # covers a partially completed movement
                        trial.move(pose)
                        trial.capture()
            except ValueError as error:
                result['measurement_error'] = str(error)
            # Reverse each wrist movement, keeping all large joints at their start.
            print('Returning through the recorded wrist poses',flush=True)
            return_path = [home] if bend_repeat else list(reversed(visited[:-1]))
            for pose in return_path:
                trial.move(pose,returning=True)
            deadline = time.monotonic()+1.5
            while time.monotonic() < deadline:
                q = trial.tick()
            if max(abs(a-b) for a,b in zip(q,home)) > .6:
                raise RuntimeError('Return position not verified; cleanup required')
            result.update(returned_to_start=return_report is None,returned_to_requested_pose=True,pre_disable_raw_deg=q)
        # Context restores RAM settings and verifies off status. Read again after
        # torque release to record settling in the original supported folded pose.
        with AxisArm() as reader:
            time.sleep(.5)
            result['final_raw_deg'] = reader.read()
            result['motors_disabled_verified'] = True
    except Exception as error:
        result['error'] = str(error)
        raise
    finally:
        w.stop.set()
        for worker in workers:worker.join(timeout=8)
        result['camera_check'] = w.camera_check.snapshot()
        if capture_imu:
            result.update(imu_type=getattr(w,'imu_type',None),imu_samples=w.imu_rows)
        if trial:
            result.update(max_tracking_error_deg=trial.max_error,samples=trial.rows)
            image_dir = w.camera_check.output/w.camera_check.session
            image_dir.mkdir(parents=True,exist_ok=True)
            for name,data in trial.images.items():
                (image_dir/name).write_bytes(data)
        path.parent.mkdir(parents=True,exist_ok=True)
        path.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print(json.dumps({k:v for k,v in result.items() if k not in ('samples','imu_samples')}),flush=True)
        print('Report: '+str(path),flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--recover-trial',type=Path,help='Return only wrists to this trial start; requires --execute and a <=10 degree displacement')
    parser.add_argument('--wrist-width',type=int,choices=[1280,1920],default=1280,help='Diagnostic image resolution; 1920 uses 8 fps to fit USB 2')
    parser.add_argument('--imu',action='store_true',help='Record independent BMI270 gyro/accelerometer alongside images')
    parser.add_argument('--bend-repeat',action='store_true',help='Compare bend outward, return, and repeat while other joints stay fixed')
    args = parser.parse_args()
    if args.execute:
        execute(args.recover_trial,args.wrist_width,args.imu,args.bend_repeat)
    else:
        geometry = Geometry()
        start = [math.degrees(v) for v in geometry.profile['reference_raw_rad']]
        print(json.dumps({'preview_only':True,'example_raw_deg':plan(geometry,start)},indent=2))


if __name__ == '__main__':main()
