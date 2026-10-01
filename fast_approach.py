#!/usr/bin/env python3
"""Four-times-speed waypoint trial and closer noncontact AprilTag approach.
Preview is read-only. Execute requires a terminal and the previously cleared route.
"""
from contextlib import ExitStack
from pathlib import Path
import argparse,json,math,select,signal,sys,time
import numpy as np
from hello_world import PORT
from arm_control import LiftArm
from rise_approach import Planner,MetricCamera,observe,safe_observation,ROOT,PROFILE,PLANE_STANDOFF
from tag_view import detect
from waypoint_motion import move_waypoint

HEIGHTS=[.005,.01,.03,.05,.075,.1,.125,.15,.175,.2]
SPEEDS=[.005,.005,.09,.12,.18,.24,.24,.24,.24,.24]

def execute():
 arm=None
 log={'contact_attempted':False,'returned_to_rest':False,'approach_m':0.,'speed_target_rad_s':.24}
 try:
  with ExitStack() as stack:
   camera=MetricCamera(stack,False,ROOT/'outputs/fast_last.jpg');detect(camera.frame,'36h11',0)
   with LiftArm(PORT) as arm:
    arm.camera=camera;arm.waypoint_log=[];start=np.array(arm.read());planner=Planner(start)
    arm.low=planner.low;arm.high=planner.high;arm.grip=arm.gripper.get_register_f32(80,200)
    lifts=[planner.lift(h) for h in HEIGHTS];scans=[planner.lift(.2,t) for t in range(5,61,5)]
    # Validate maximum endpoint size offline before enabling.
    for a,b in zip([lifts[1]]+lifts[2:-1],lifts[2:]):
     if max(abs(b-a))>math.radians(8):raise ValueError('Oversized lift waypoint')
    arm.start(start);current=start.copy();history=[];speed_cap=.24
    def move(target,speed,kind):
     nonlocal current
     old=current.copy()
     if kind=='stream':arm.segment(current,target,speed);result=None
     else:result=move_waypoint(arm,current,target,speed)
     current=target;history.append((old,current.copy(),speed,kind));return result
    def restore():
     nonlocal current
     print('Returning along executed path.',flush=True)
     for old,end,speed,kind in history[::-1]:
      if kind=='stream':arm.segment(current,old,speed)
      else:move_waypoint(arm,current,old,min(speed,speed_cap))
      current=old
     arm.check(start);log['returned_to_rest']=True
    for index,(target,speed,height) in enumerate(zip(lifts,SPEEDS,HEIGHTS)):
     if index<2:
      move(target,speed,'stream')
      if index==1:arm.finish_load_take_up(current);print('Load held; beginning speed ramp.',flush=True)
     else:
      result=move(target,min(speed,speed_cap),'waypoint')
      print(json.dumps({'lift_m':height,'speed_rad_s':result['speed_limit_rad_s'],'max_path_error_deg':result['max_path_error_deg'],'duration_s':result['duration_s']}),flush=True)
      if result['max_path_error_deg']>.75:speed_cap=min(speed_cap,speed)
    found=None;last_good=None;last_good_pose=None
    for index,target in enumerate(scans,1):
     result=move(target,min(.24,speed_cap),'waypoint')
     if result['max_path_error_deg']>.75:speed_cap=min(speed_cap,result['speed_limit_rad_s'])
     if not detect(camera.frame,'36h11',0):continue
     found=observe(arm,current,camera,f'fast_tilt_{index*5:02d}')
     try:safe_observation(found)
     except ValueError:continue
     last_good=found;last_good_pose=current.copy()
     vector=np.array(found['translation_camera_m'])
     if abs(vector[0]/vector[2])<.15:break
    if last_good is None:
     log['stop_reason']='No suitable full-tag observation';restore();return
    if not np.array_equal(current,last_good_pose):
     log['stop_reason']='Tag lost after scan; returning without approach';restore();return
    print('READY: centered tag. Type APPROACH or REST; 60s timeout returns to rest.',flush=True)
    deadline=time.monotonic()+60;answer='REST'
    while time.monotonic()<deadline:
     arm.send(current);arm.check(current)
     if select.select([sys.stdin],[],[],.02)[0]:answer=sys.stdin.readline().strip();break
    if answer=='APPROACH':
     measured=observe(arm,current,camera,'fast_before');log['before']=measured
     for index in range(12):
      try:safe_observation(measured,reserve_m=.013)
      except ValueError as error:log['stop_reason']=str(error);break
      step=min(.005,measured['plane_distance_m']-PLANE_STANDOFF-.008)
      if step<.002:log['stop_reason']='Reached noncontact standoff';break
      try:target=planner.forward(current,step)
      except ValueError as error:log['stop_reason']=str(error);break
      before=measured
      move(target,.03,'stream');log['approach_m']+=step
      measured=observe(arm,current,camera,f'fast_close_{index+1:02d}');log['after']=measured
      if measured is None or measured['range_m']>before['range_m']+.003 or measured['plane_distance_m']<PLANE_STANDOFF:
       log['stop_reason']='Observation or distance check failed';break
     else:log['stop_reason']='Completed 6 cm approach budget'
     print(json.dumps({'approach_m':log['approach_m'],'stop_reason':log['stop_reason'],'after':log.get('after')}),flush=True)
    restore()
 finally:
  log['telemetry']=getattr(arm,'telemetry',[]);log['waypoints']=getattr(arm,'waypoint_log',[])
  (ROOT/'outputs/fast_approach_result.json').write_text(json.dumps(log,indent=2)+'\n')

def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');a=p.parse_args()
 if a.execute:
  if not sys.stdin.isatty():p.error('Execute requires an attended terminal')
  execute()
 else:
  planner=Planner(json.loads(PROFILE.read_text())['reference_raw_rad'])
  for height,speed in zip(HEIGHTS,SPEEDS):
   planner.lift(height);print(f'{height*100:.1f} cm lift; {speed:.3f} rad/s limit')
  print('Preview only; no hardware accessed. Maximum approach 6 cm; camera-plane standoff 25 cm.')
if __name__=='__main__':
 signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
 main()
