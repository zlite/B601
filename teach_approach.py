#!/usr/bin/env python3
"""Teach a short retreat by hand; supervise its reverse replay. Never recalibrates."""
import argparse
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import select
import signal
import sys
import time

import numpy as np
from hello_world import PORT
from calibrate_arm import Reader, PROFILE

ROOT=Path(__file__).resolve().parent
FILE=ROOT/'calibration/arm2_short_approach.json'
SPEED=.01  # rad/s. Final segment uses .005 rad/s.


def digest(path):return hashlib.sha256(path.read_bytes()).hexdigest()


def near(a,b,degrees,message):
    if max(abs(x-y) for x,y in zip(a,b))>math.radians(degrees):raise ValueError(message)


def validate(points):
    p=np.array(points,dtype=float)
    if p.ndim!=2 or p.shape[1]!=6 or len(p)<5 or not np.isfinite(p).all():
        raise ValueError('Need at least five finite six-joint samples')
    if np.max(np.abs(np.diff(p,axis=0)))>math.radians(1):
        raise ValueError('Recording contains a jump above 1 degree; teach the path more slowly')
    if np.max(np.abs(p-p[0]))>math.radians(10):
        raise ValueError('Path exceeds 10 degrees from contact; record only a short retreat')
    if np.max(np.abs(p[-1]-p[0]))<math.radians(1):
        raise ValueError('Retreat too small to distinguish from contact')
    return p


def teach(port, keep_supported=False):
    print('READ-ONLY TEACHING. Keep tag, base, camera, jaws and motor zeros unchanged.')
    print('Support the disabled arm. Gently place one padded fingertip at the desired light-contact point.')
    print('This records a NEW contact pose and jaw opening. No match to the previous contact is required.')
    print('After capture, wait for RECORDING before starting your slow retreat.')
    input('Press Enter when lightly touching and holding steady: ')
    with Reader(port) as reader:
        g=reader.add_motor(7,23,'4310')
        q,_=reader.capture()
        grip=g.get_register_f32(80,500)
        print('RECORDING: slowly retreat a few centimeters; press Enter to finish while steady.',flush=True)
        points=[q];times=[0.];start=time.monotonic()
        while True:
            current=reader.read();elapsed=time.monotonic()-start
            if elapsed>45:raise ValueError('45-second recording limit reached; retry a short retreat')
            near(current,points[-1],1,'Moved too quickly between samples; retry more slowly')
            near(current,points[0],10,'Retreat exceeds this short-path limit')
            points.append(current);times.append(elapsed)
            if select.select([sys.stdin],[],[],.02)[0]:
                if not sys.stdin.readline():raise EOFError
                break
        settled,_=reader.capture();near(settled,points[-1],.3,'Hold the retreat endpoint steady and retry')
        points.append(settled);times.append(time.monotonic()-start)
        near([g.get_register_f32(80,500)],[grip],.5,'Gripper opening changed during teaching')
        validate(points)
    data={'version':2,'physical_arm':'second','created_utc':datetime.now(timezone.utc).isoformat(),
          'reference_sha256':digest(PROFILE),'contact_raw_joint_rad':points[0],
          'gripper_raw_rad':grip,'retreat_points':points,'times_s':times,
          'motion_commanded':False,'path_clearance_basis':'operator manually traversed path; no collision model',
          'contact_basis':'fresh operator-taught contact in this recording; no force sensing','autonomous_motion_ready':False}
    if FILE.exists():FILE.with_name('arm2_short_approach.'+datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%f')+'.json').write_bytes(FILE.read_bytes())
    temp=FILE.with_suffix('.tmp');temp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n');temp.replace(FILE)
    print(f'Saved {len(points)} points to {FILE}. Motors unchanged.')
    if keep_supported:
        print('Keep holding the RETREAT endpoint; supervised replay setup follows.')
    else:
        print('Gently rest/support the arm. Next: uv run teach_approach.py preview')


def load_path():
    if not FILE.exists():raise ValueError('No short path recorded yet. Run: uv run teach_approach.py teach')
    d=json.loads(FILE.read_text())
    if d.get('physical_arm')!='second' or d.get('version') not in (1,2):raise ValueError('Wrong path profile')
    if digest(PROFILE)!=d['reference_sha256']:
        raise ValueError('Reference changed; re-teach the short path')
    if d['version']==1 and digest(Path(d['contact_file']))!=d['contact_sha256']:
        raise ValueError('Legacy contact changed; re-teach the short path')
    p=validate(d['retreat_points'])
    if d['version']==2 and not np.array_equal(np.array(d['contact_raw_joint_rad']),p[0]):
        raise ValueError('Contact endpoint does not match the recorded path')
    return d,p


def preview():
    d,p=load_path()
    arc=float(np.sum(np.max(np.abs(np.diff(p,axis=0)),axis=1)))
    print('Recorded retreat points:',len(p))
    print('Contact (raw degrees):',np.rad2deg(p[0]).round(2).tolist())
    print('Replay start (raw degrees):',np.rad2deg(p[-1]).round(2).tolist())
    print('Maximum excursion per joint (degrees):',np.rad2deg(np.max(abs(p-p[0]),axis=0)).round(2).tolist())
    print(f'Approximate one-way minimum streaming duration: {arc/SPEED:.1f}s, plus slower final segment.')
    print('Replay starts ONLY at the taught retreat endpoint. It pauses before the final contact segment.')
    print('No hardware accessed.')


class ReplayArm(Reader):
    def __enter__(self):
        super().__enter__();self.touched=False;self.active=False
        try:
            self.gripper=self.add_motor(7,23,'4310')
        except BaseException:
            super().__exit__(*sys.exc_info())
            raise
        return self

    def read(self):
        values=[]
        for i,m in enumerate(self.motors,1):
            q=m.get_register_f32(80,200)
            m.request_feedback();time.sleep(.003);self.ctrl.poll_feedback_once();s=m.get_state()
            if s is None or s.status_code!=(1 if self.active else 0):raise RuntimeError(f'Joint {i} unexpected status')
            if max(s.t_mos,s.t_rotor)>60 or not math.isfinite(q):raise RuntimeError(f'Joint {i} invalid/hot')
            values.append(q)
        return values

    def send(self,q,speed=SPEED):
        if np.any(q<self.low) or np.any(q>self.high):raise RuntimeError('Target outside taught envelope')
        for m,x in zip(self.motors,q):m.send_pos_vel(float(x),speed)

    def check(self,target):
        q=self.read();near(q,target,1,'Tracking error above 1 degree; stopping')
        near([self.gripper.get_register_f32(80,200)],[self.grip],.5,'Gripper opening changed')
        return q

    def segment(self,a,b,speed):
        duration=max(.10,1.6*float(np.max(abs(np.array(b)-a)))/speed)
        start=time.monotonic();last=start;nextcheck=start
        while True:
            now=time.monotonic()
            if now-last>.5:raise RuntimeError('Control loop stalled; stopping')
            last=now;u=min(1.,(now-start)/duration);f=u*u*(3-2*u)
            target=np.array(a)+(np.array(b)-a)*f
            self.send(target,speed)
            if now>=nextcheck:self.check(target);nextcheck=time.monotonic()+.15
            if u>=1:return
            time.sleep(.02)

    def hold(self,target,prompt):
        print(prompt,flush=True)
        while True:
            self.send(target);self.check(target)
            if select.select([sys.stdin],[],[],.03)[0]:
                text=sys.stdin.readline()
                if not text:raise EOFError
                return text.strip().upper()

    def __exit__(self,*args):
        try:
            if self.touched:
                print('Disabling arm joints: keep the arm supported.',flush=True)
                for i,m in enumerate(self.motors,1):
                    try:m.disable()
                    except Exception as e:print(f'Joint {i} disable failed: use hardware cutoff. {e}',flush=True)
        finally:super().__exit__(*args)


def replay(port):
    from motorbridge import Mode
    d,p=load_path();start=p[-1]
    print('SUPERVISED REPLAY. This drives all six arm joints; jaws are not commanded.')
    print('Keep the tag/base fixed and the taught route clear. Support the arm during enable/disable.')
    print('Ctrl+C/fault disables torque and the arm can drop. Keep hardware cutoff accessible.')
    with ReplayArm(port) as arm:
        arm.grip=d['gripper_raw_rad'];arm.low=p.min(axis=0)-math.radians(.5);arm.high=p.max(axis=0)+math.radians(.5)
        q,_=arm.capture();near(q,start,.5,'Place the arm manually at the taught RETREAT endpoint first')
        near([arm.gripper.get_register_f32(80,200)],[arm.grip],.5,'Gripper opening changed')
        if input('Type REPLAY after checking the saved path, fixed tag and clear route: ').strip()!='REPLAY':return
        q=arm.read();near(q,start,.5,'Arm moved before enable; restart')
        arm.touched=True
        for m,x in zip(arm.motors,q):
            m.ensure_mode(Mode.POS_VEL,1500);m.send_pos_vel(x,SPEED)
        fresh=arm.read();near(fresh,q,.3,'Arm moved during mode setup; keep it supported')
        for m,x in zip(arm.motors,fresh):m.send_pos_vel(x,SPEED);m.enable();m.send_pos_vel(x,SPEED)
        arm.active=True;arm.segment(fresh,start,SPEED)
        while True:
            answer=arm.hold(start,'Holding retreat pose. Move supporting hands clear and type GO; or support the arm and type SUPPORT to disable: ')
            if answer=='SUPPORT':return
            if answer=='GO':break
        approach=p[::-1]
        lengths=np.max(abs(np.diff(approach,axis=0)),axis=1);threshold=.8*float(lengths.sum());travel=0;paused=False
        previous=start;reached=0
        for index,(target,length) in enumerate(zip(approach[1:],lengths),1):
            if not paused and travel+length>=threshold:
                answer=arm.hold(previous,'Paused before final recorded segment (not a measured clearance). Type TOUCH to continue slowly, or BACK to retreat: ')
                if answer!='TOUCH':break
                paused=True
            arm.segment(previous,target,.005 if paused else SPEED)
            previous=target;travel+=length;reached=index
        else:
            print('Reached taught endpoint; contact is not force-verified. Retreating immediately.',flush=True)
        # Return along exactly the portion of recorded path that was traversed.
        for target in approach[:reached][::-1]:arm.segment(previous,target,SPEED);previous=target
        while arm.hold(start,'Back at retreat pose, holding. Support the arm and type SUPPORT to disable: ')!='SUPPORT':pass


def main():
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('action',choices=['teach','teach-replay','preview','replay']);p.add_argument('--port',default=PORT);a=p.parse_args()
    if a.action!='preview' and not sys.stdin.isatty():p.error('Teach/replay require an attended interactive terminal')
    signal.signal(signal.SIGTERM,lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    if a.action=='teach-replay':
        teach(a.port,keep_supported=True)
        preview()
        replay(a.port)
    elif a.action=='teach':teach(a.port)
    elif a.action=='preview':preview()
    else:replay(a.port)

if __name__=='__main__':
    try:main()
    except (KeyboardInterrupt,EOFError):raise SystemExit('Stopped. Support the arm; use hardware cutoff if required.')
    except Exception as e:raise SystemExit(f'Stopped: {e}')
