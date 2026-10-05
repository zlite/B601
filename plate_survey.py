#!/usr/bin/env python3
"""Raise along the checked lift, look down at the plate holder, then return."""
import json,math,time,threading,subprocess,argparse
from pathlib import Path
from datetime import datetime,timezone
import cv2
import numpy as np
from axis_follow import AxisArm
from pairing_dashboard import Workbench
from camera_check import CameraCheck
from camera_visual_approach import route,RouteRunner
from tag_view import detect,annotate_view


def execute(max_bend=36.):
    folder=Path('outputs/plate_survey')/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');folder.mkdir(parents=True)
    w=Workbench();w.survey_mode=True;w.camera_sizes={'tripod':(1280,800)};w.camera_fps={'tripod':5};w.camera_check=CameraCheck(w.geometry,folder/'views')
    workers=[threading.Thread(target=w.camera_worker,args=(r,),daemon=True) for r in ('wrist','tripod')]
    for x in workers:x.start()
    runner=None;record={'contact_attempted':False,'samples':[],'returned_to_rest':False,'motors_disabled_verified':False}
    try:
        with AxisArm() as arm:
            start=arm.read();poses,limits=route(start,.06,0.);lift=poses[11];planned=poses[:12]
            for offset in sorted(set([x for x in (12.,24.,36.) if x<=max_bend]+[max_bend])):
                goal=lift.copy();goal[3]+=offset;planned.append(goal)
            # Known 5 cm lift, then wrist-only look-down, all inside existing joint limits.
            if any(not limits[i][0]<=q[i]<=limits[i][1] for q in planned for i in range(6)):raise ValueError('Plate survey leaves joint bounds')
            deadline=time.monotonic()+25
            while time.monotonic()<deadline:
                arm.read()
                if all(time.monotonic()-w.sources.get(r,{}).get('time',0)<.5 for r in ('wrist','tripod')):break
            else:raise RuntimeError('Cameras not ready')
            # Gripper is read but never enabled in the survey.
            grip=arm.add_motor(7,23,'4310');record['gripper_raw_rad']=grip.get_register_f32(80,300)
            arm.prepare_group(range(6));arm.speed_limits={i:30. for i in range(6)};arm.enable_group(dict(enumerate(start)))
            runner=RouteRunner(w,arm,start,limits,folder,planned);runner.speed=10.;runner.acceleration=36.;runner.tracking_limit=3.;runner.pacing_lag=1.8
            def camera_ready():
                with w.lock:return all('error' not in w.sources.get(r,{}) and time.monotonic()-w.sources.get(r,{}).get('time',0)<.5 and w.sources.get(r,{}).get('frame_age_s',99)<.5 for r in ('wrist','tripod'))
            runner.vision_ready=camera_ready;until=time.monotonic()+1
            while time.monotonic()<until:runner.tick(take_up=True)
            visited=[start]
            try:
                for index,goal in enumerate([poses[1],poses[3],lift,*planned[12:]]):
                    visited.append(goal);runner.go(goal)
                    until=time.monotonic()+.7
                    while time.monotonic()<until:runner.tick()
                    with w.lock:data=dict(w.survey_frame)
                    image=data.pop('image');tags=detect(image,'auto',None);name=f'view_{index}.jpg'
                    cv2.imwrite(str(folder/name),annotate_view(image,tags));cv2.imwrite(str(folder/f'raw_{index}.png'),image)
                    sample={**data,'image':name,'raw_deg':runner.rows[-1]['raw_deg'],'tags':tags};record['samples'].append(sample)
                    with w.lock:tripod=dict(w.survey_frames['tripod'])
                    trip_image=tripod.pop('image');trip_tags=detect(trip_image,'auto',None)
                    cv2.imwrite(str(folder/f'tripod_{index}.png'),trip_image)
                    sample['tripod']={**tripod,'tags':trip_tags,'image':f'tripod_{index}.png'}
                    (folder/'progress.json').write_text(json.dumps(record,indent=2)+'\n')
                    print('Survey',index,'tags',[(t['family'],t['id']) for t in tags],flush=True)
                # Capture a calibrated stereo triplet in the final look-down pose.
                w.stop.set();deadline=time.monotonic()+15
                while any(x.is_alive() for x in workers):
                    runner.tick()
                    if time.monotonic()>deadline:raise ValueError('Camera release timeout')
                with (folder/'stereo.log').open('w') as log:
                    child=subprocess.Popen(['.venv-depth-v2/bin/python','contact_camera_probe_v2.py','--all-tags'],stdout=log,stderr=subprocess.STDOUT)
                    try:
                        deadline=time.monotonic()+50
                        while child.poll() is None:
                            runner.tick()
                            if time.monotonic()>deadline:raise ValueError('Stereo survey timeout')
                        record['stereo_exit_code']=child.returncode
                    finally:
                        if child.poll() is None:child.terminate()
            except ValueError as error:record['stop_reason']=str(error)
            for goal in reversed(visited[:-1]):runner.go(goal,returning=True)
            runner.go(start,returning=True);until=time.monotonic()+2
            while time.monotonic()<until:q=runner.tick()
            record['return_error_deg']=(np.array(q)-start).tolist()
            if max(abs(a-b) for a,b in zip(q[:5],start[:5]))>.2 or abs(q[5]-start[5])>.5:raise RuntimeError('Folded return not verified')
            record['returned_to_rest']=True
        with AxisArm() as arm:time.sleep(.5);record['final_raw_deg']=arm.read();record['motors_disabled_verified']=True
    finally:
        w.stop.set()
        for x in workers:x.join(timeout=8)
        if runner:record['motion_samples']=runner.rows
        (folder/'report.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n');print('Plate survey:',folder,flush=True)


def recover(report):
    old=json.loads(Path(report).read_text());goal=old['motion_samples'][0]['raw_deg']
    with AxisArm() as arm:
        start=arm.read()
        if any(abs(start[i]-goal[i])>(3. if i<3 else 8.) for i in range(6)):raise ValueError('Recovery is outside nearby folded pose')
        w=Workbench();w.camera_check=CameraCheck(w.geometry,Path(report).parent/'recovery_views')
        workers=[threading.Thread(target=w.camera_worker,args=(r,),daemon=True) for r in ('wrist','tripod')]
        for x in workers:x.start()
        try:
            deadline=time.monotonic()+25
            def ready():
                with w.lock:return all('error' not in w.sources.get(r,{}) and time.monotonic()-w.sources.get(r,{}).get('time',0)<.5 for r in ('wrist','tripod'))
            while not ready() and time.monotonic()<deadline:arm.read()
            if not ready():raise RuntimeError('Recovery cameras not ready')
            limits={i:(min(start[i],goal[i])-1.,max(start[i],goal[i])+1.) for i in range(6)}
            arm.prepare_group(range(6));arm.enable_group(dict(enumerate(start)))
            runner=RouteRunner(w,arm,start,limits,Path(report).parent,[start,goal]);runner.vision_ready=ready
            until=time.monotonic()+1
            while time.monotonic()<until:runner.tick(take_up=True)
            runner.go(goal);until=time.monotonic()+2
            while time.monotonic()<until:q=runner.tick()
            print('Rest recovery error degrees:',(np.array(q)-goal).tolist(),flush=True)
        finally:
            arm.disable();w.stop.set()
            for x in workers:x.join(timeout=8)
    with AxisArm() as arm:print('Motors off verified:',arm.read(),flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--execute',action='store_true');p.add_argument('--hover',action='store_true');p.add_argument('--hover-stereo',action='store_true');p.add_argument('--lower',action='store_true');p.add_argument('--pickup',action='store_true');p.add_argument('--table-survey',action='store_true');p.add_argument('--table-carry',action='store_true');p.add_argument('--grid-reference',type=Path);p.add_argument('--review',action='store_true');p.add_argument('--approach',choices=['direct','left','right'],default='direct');p.add_argument('--travel-speed-scale',type=float,choices=[1.,2.],default=2.);p.add_argument('--speed-scale',type=float,choices=[1.,2.,4.],default=2.);p.add_argument('--alignment-review',action='store_true');p.add_argument('--recovery-test',action='store_true');p.add_argument('--recovery-inject-failure',action='store_true');p.add_argument('--recovery-inject-grid-loss',action='store_true');p.add_argument('--recovery-height-mm',type=int,choices=[8,18,25],default=25);p.add_argument('--max-bend',type=float,choices=[24.,30.,36.],default=36.);p.add_argument('--recover',type=Path);a=p.parse_args()
    if a.recover:recover(a.recover)
    elif a.execute and a.hover:
        from plate_hover import execute as hover
        hover(a.hover_stereo,a.lower or a.pickup or a.table_carry or a.recovery_test or a.alignment_review or a.table_survey,a.pickup or a.table_carry,a.recovery_test,a.recovery_height_mm,a.recovery_inject_failure,a.alignment_review,a.speed_scale,a.review,a.recovery_inject_grid_loss,a.approach,a.travel_speed_scale,a.table_survey,a.grid_reference,a.table_carry)
    elif a.execute:execute(a.max_bend)
    else:print('Preview: 5 cm checked lift, wrist bend +12/+24/+36 degrees to survey plate; gripper unpowered; return.')
