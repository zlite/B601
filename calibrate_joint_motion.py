#!/usr/bin/env python3
"""Autonomous camera-referenced local calibration of six arm joints.

Preview by default. --execute owns both cameras and the motor bus. Each joint
moves at most 8 degrees from the folded starting pose, then returns. No gripper,
firmware, zero, or active global kinematic changes. Camera measurements are truth.
"""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
from arm_geometry import Geometry
from axis_follow import AxisArm, axis_bounds
from camera_check import CameraCheck
from pairing_dashboard import Workbench
from wrist_follow import WristTrajectory
from joint_motion_calibration import NAMES, OFFSETS, TRAVEL_SIGNS, fit_joint, DENSE_OFFSETS, fit_dense_joint


def plan(geometry,start,joints,offsets=OFFSETS,travel_signs=TRAVEL_SIGNS):
    if len(start)!=6 or not all(math.isfinite(x) for x in start):
        raise ValueError('Invalid six-joint starting pose')
    reference=np.degrees(geometry.profile['reference_raw_rad'])
    if max(abs(start[i]-reference[i]) for i in (1,2))>1:
        raise ValueError('Begin in the supported folded pose')
    limits={i:axis_bounds(geometry,i,start[i]) for i in range(6)}
    result={}
    for j in joints:
        poses=[]
        for offset in offsets:
            q=list(start);q[j]+=travel_signs[j]*offset
            if not limits[j][0]<=q[j]<=limits[j][1]:
                raise ValueError(f'{NAMES[j]} target exceeds its existing limits')
            poses.append(q)
        result[j]=poses
    return result,limits


class Runner:
    def __init__(self,w,arm,start,limits,output):
        self.w,self.arm,self.start,self.limits,self.output=w,arm,start,limits,output
        self.targets=dict(enumerate(start));self.last=time.monotonic()
        self.joint=None;self.rows=[];self.max_error=0.

    def tick(self,moving=False,trajectories=None,goal=None,take_up=False):
        q=self.arm.read();now=time.monotonic();dt=now-self.last
        if not 0<dt<.3:raise RuntimeError('Motor loop stalled')
        for i in range(6):
            error=abs(q[i]-self.targets[i]);self.max_error=max(self.max_error,error)
            if error>3:raise RuntimeError(f'{NAMES[i]} tracking error {error:.2f} degrees')
            allowance=9. if i==self.joint else 1.
            if abs(q[i]-self.start[i])>allowance:raise RuntimeError(f'{NAMES[i]} left the local trial envelope')
            if not self.limits[i][0]-1<=q[i]<=self.limits[i][1]+1:raise RuntimeError('Measured position outside model envelope')
        if trajectories:
            self.targets={i:trajectories[i].step(goal[i],dt) for i in range(6)}
        self.arm.command_group(self.targets,take_up=take_up)
        self.w.camera_check.add_joints(now,q,powered=True,following=moving,all_powered=True,fault=None)
        self.rows.append({'time':now,'joint':self.joint,'moving':moving,'raw_deg':q,'target_deg':list(self.targets.values())})
        self.last=now;time.sleep(.005)
        return q

    def vision_ready(self):
        now=time.monotonic()
        with self.w.lock:
            if not all('error' not in self.w.sources.get(r,{}) and now-self.w.sources.get(r,{}).get('time',0)<.5
                       and self.w.sources.get(r,{}).get('frame_age_s',99)<.5 for r in ('wrist','tripod')):
                return False
        check=self.w.camera_check
        with check.lock:
            valid=[f for f in check.frames if f.get('valid') and now-f['time']<.5]
            if not valid:return False
            position=np.asarray(valid[-1]['T_camera_tag'])[:3,3]
            return float(np.linalg.norm(position))>.30

    def move(self,goal,returning=False):
        trajectories={}
        for i in range(6):
            radius=8.01 if i==self.joint else .01
            low=max(self.start[i]-radius,self.limits[i][0]);high=min(self.start[i]+radius,self.limits[i][1])
            if not low<=goal[i]<=high:raise RuntimeError('Unplanned target outside local bounds')
            trajectories[i]=WristTrajectory(self.start[i],self.targets[i],low=low,high=high,speed=3.,acceleration=9.)
        end=time.monotonic()+12
        while time.monotonic()<end:
            if not returning and not self.vision_ready():raise ValueError('Vision unavailable; return to starting pose')
            q=self.tick(True,trajectories,goal)
            if max(abs(self.targets[i]-goal[i]) for i in range(6))<.025 and max(abs(q[i]-goal[i]) for i in range(6))<.8:
                # Finish the last <0.025 degrees exactly before another joint's
                # tighter holding envelope is constructed.
                self.targets=dict(enumerate(goal));self.arm.command_group(self.targets)
                return
        raise ValueError('Movement did not settle; return to starting pose')

    def capture(self):
        self.w.camera_check=CameraCheck(self.w.geometry,self.output/'views')
        check=self.w.camera_check;check.arm_capture()
        end=time.monotonic()+18
        while time.monotonic()<end:
            self.tick()
            with check.lock:
                if check.samples:
                    sample=dict(check.samples[0])
                    sample['view_report']=str(check.output/check.session/'report.json')
                    return sample
            # An invalid tag can recover during a stationary hold; never move
            # onward until the same strict stationary gate has saved a view.
        raise ValueError('Steady capture unavailable: '+check.snapshot()['message'])


def combine_captures(captures):
    q=np.array([s['raw_joint_deg'] for s in captures])
    if np.ptp(q,axis=0).max()>.25:
        raise ValueError('Joint drift across repeated stationary captures')
    sample=dict(captures[-1])
    sample['raw_joint_deg']=q.mean(axis=0).tolist()
    sample['stationary_observations']=[o for s in captures for o in s['stationary_observations']]
    sample['repeat_view_reports']=[s['view_report'] for s in captures]
    sample['repeat_joint_span_deg']=np.ptp(q,axis=0).tolist()
    return sample


def execute(joints,dense=False,settle_seconds=0.,views_per_pose=1,reverse_travel=False):
    travel_signs=[-s if reverse_travel and j in joints else s for j,s in enumerate(TRAVEL_SIGNS)]
    offsets=DENSE_OFFSETS if dense else OFFSETS
    w=Workbench();w.capture_imu=True;w.imu_rows=[]
    batch=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    output=Path('outputs/joint_calibration')/batch
    w.camera_check=CameraCheck(w.geometry,output/'views')
    workers=[threading.Thread(target=w.camera_worker,args=(r,),daemon=True) for r in ('wrist','tripod')]
    for worker in workers:worker.start()
    result={'session':batch,'camera_is_reference':True,'geometry_fingerprint':w.geometry.fingerprint,
        'scope':'Local angular mappings only; existing raw motor limits and global kinematics unchanged.',
        'travel_signs':travel_signs,'sequence_raw_offsets_deg':offsets,'settle_seconds':settle_seconds,'views_per_pose':views_per_pose,
        'joints':{},'returned_to_start':False,'motors_disabled_verified':False}
    runner=None
    try:
        with AxisArm() as arm:
            start=arm.read();poses,limits=plan(w.geometry,start,joints,offsets,travel_signs);result['start_raw_deg']=start
            end=time.monotonic()+30
            while time.monotonic()<end:
                q=arm.read()
                if max(abs(a-b) for a,b in zip(q,start))>.1:raise RuntimeError('Starting pose changed during camera setup')
                w.camera_check.add_joints(time.monotonic(),q,powered=False,following=False,all_powered=False,fault=None)
                if w.camera_check.snapshot()['ready']:break
                time.sleep(.005)
            else:raise RuntimeError('Camera setup did not become ready')
            arm.prepare_group(range(6));q=arm.read()
            if max(abs(a-b) for a,b in zip(q,start))>.1:raise RuntimeError('Pose changed during motor configuration')
            arm.speed_limits={i:20. for i in range(6)}
            arm.enable_group(dict(enumerate(start)))
            runner=Runner(w,arm,start,limits,output)
            end=time.monotonic()+1.
            while time.monotonic()<end:q=runner.tick(take_up=True)
            if max(abs(a-b) for a,b in zip(q,start))>1:raise RuntimeError('Load take-up failed')
            for j in joints:
                runner.joint=j
                entry={'joint':j+1,'name':NAMES[j],'samples':[]};result['joints'][str(j+1)]=entry
                print('JOINT '+NAMES[j],flush=True)
                try:
                    for index,goal in enumerate(poses[j]):
                        if index:runner.move(goal)
                        until=time.monotonic()+settle_seconds
                        while time.monotonic()<until:runner.tick()
                        captures=[runner.capture() for _ in range(views_per_pose)]
                        sample=combine_captures(captures)
                        sample.update(sequence_index=index,requested_offset_deg=offsets[index])
                        entry['samples'].append(sample)
                        print(f'{NAMES[j]}: saved {index+1}/{len(offsets)} ({offsets[index]:g} degrees)',flush=True)
                except ValueError as error:
                    entry['capture_error']=str(error)
                    print(f'{NAMES[j]}: {error}',flush=True)
                # Only this joint ever left home. Always return it before testing
                # another joint, even when a camera gate rejected a measurement.
                runner.move(start,returning=True)
                end=time.monotonic()+1.
                while time.monotonic()<end:q=runner.tick()
                if max(abs(a-b) for a,b in zip(q,start))>.8:raise RuntimeError('Return could not be verified')
            result.update(returned_to_start=True,pre_disable_raw_deg=q)
        with AxisArm() as reader:
            time.sleep(.5);result['final_raw_deg']=reader.read();result['motors_disabled_verified']=True
    except Exception as error:
        result['error']=str(error)
        raise
    finally:
        w.stop.set()
        for worker in workers:worker.join(timeout=8)
        result['imu_samples']=w.imu_rows
        if runner:result.update(motion_samples=runner.rows,max_tracking_error_deg=runner.max_error)
        output.mkdir(parents=True,exist_ok=True)
        (output/'measurements.json').write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print('Measurements: '+str(output/'measurements.json'),flush=True)
    profile={'session':batch,'geometry_fingerprint':w.geometry.fingerprint,'camera_is_reference':True,
             'scope':result['scope'],'joints':[],'global_mapping_changed':False}
    for key,entry in result['joints'].items():
        try:model=(fit_dense_joint if dense else fit_joint)(int(key)-1,entry['samples'],travel_sign=travel_signs[int(key)-1])
        except ValueError as error:model={'joint':int(key),'name':entry['name'],'validation_passed':False,'reason':str(error)}
        profile['joints'].append(model)
        print(json.dumps({k:v for k,v in model.items() if k in ('name','endpoint_scale','max_validation_error_deg','validation_passed','reason')}),flush=True)
    path=Path('calibration')/f'camera_joint_motion_{batch}.json'
    path.write_text(json.dumps(profile,indent=2,allow_nan=False)+'\n')
    print('Profile: '+str(path),flush=True)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute',action='store_true')
    parser.add_argument('--dense',action='store_true',help='Fit full camera rotation with 2-degree knots and separate 1-degree validation positions')
    parser.add_argument('--reverse-travel',action='store_true',help='Measure the other direction within existing joint limits')
    parser.add_argument('--settle-seconds',type=float,default=0.,choices=[0.,1.,2.,3.])
    parser.add_argument('--views-per-pose',type=int,default=1,choices=[1,2,3])
    parser.add_argument('--joints',nargs='+',type=int,choices=range(1,7),default=[1,2,3,4,5,6])
    args=parser.parse_args();joints=list(dict.fromkeys(j-1 for j in args.joints))
    if args.execute:execute(joints,args.dense,args.settle_seconds,args.views_per_pose,args.reverse_travel)
    else:
        g=Geometry();start=np.degrees(g.profile['reference_raw_rad']).tolist()
        offsets=DENSE_OFFSETS if args.dense else OFFSETS
        signs=[-s if args.reverse_travel and j in joints else s for j,s in enumerate(TRAVEL_SIGNS)]
        poses,_=plan(g,start,joints,offsets,signs)
        print(json.dumps({'preview_only':True,'joint_names':[NAMES[j] for j in joints],
            'offsets':offsets,'poses':poses},indent=2))


if __name__=='__main__':main()
