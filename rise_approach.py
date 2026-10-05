#!/usr/bin/env python3
"""Bounded noncontact lift/look/2-cm approach demo. --execute moves the arm.

This does not use the unvalidated fingertip or hand-eye fit for contact.
The wrist +Z axis approximates the forward camera direction for a short
stand-off move; actual range reduction is checked using fresh tag observations.
"""
from contextlib import ExitStack
from pathlib import Path
import argparse
import json
import math
import select
import sys
import time
import signal
import cv2
import numpy as np
from scipy.optimize import least_squares
from motorbridge import Mode
from arm_geometry import Geometry
from calibrate_arm import PROFILE
from hello_world import Camera, PORT
from teach_approach import near
from arm_control import LiftArm
from tag_view import detect
from tag_pose import estimate
from camera_view import upright

ROOT=Path(__file__).resolve().parent
SPEED=.06
APPROACH_SPEED=.015
STEP=.02
PLANE_STANDOFF=.25  # camera-to-tag plane, deliberately far from physical contact

class Planner:
 def __init__(self,start):
  self.g=Geometry();self.start=np.array(start,dtype=float);self.base=self.g.transform(start)
  if not np.isfinite(self.start).all() or max(abs(self.start*self.g.signs+self.g.offsets))>math.radians(5):
   raise ValueError('Start must be near the folded resting reference')
  self.low=self.start+np.deg2rad([-5,-35,-55,-3,-5,-5])
  self.high=self.start+np.deg2rad([5,3,3,85,5,5])
 def lift(self,height,tilt=0):
  if not 0<=height<=.2 or not 0<=tilt<=60:raise ValueError('Lift/tilt outside demo limits')
  def candidate(x):
   t=self.start.copy();t[1:4]+=x;return t
  def residual(x):return np.r_[(self.g.transform(candidate(x))[:3,3]-self.base[:3,3]-[0,0,height])*10,-x[0]+x[1]+x[2]]
  fit=least_squares(residual,np.deg2rad([-5,-10,5]),bounds=(np.deg2rad([-30,-50,0]),np.deg2rad([0,0,25])))
  if np.linalg.norm(residual(fit.x))>1e-4:raise ValueError('Lift solver residual too large')
  target=candidate(fit.x);target[3]+=math.radians(tilt);self.validate(target);return target
 def validate(self,target):
  if not np.isfinite(target).all() or np.any(target<self.low) or np.any(target>self.high):raise ValueError('Target outside bounded joint envelope')
 def forward(self,current,distance):
  if not 0<distance<=STEP:raise ValueError('Approach is limited to 2 cm')
  initial=self.g.transform(current);goal=initial.copy();goal[:3,3]+=initial[:3,2]*distance
  def residual(q):
   t=self.g.transform(q)
   return np.r_[(t[:3,3]-goal[:3,3])*10,cv2.Rodrigues(goal[:3,:3].T@t[:3,:3])[0].ravel()]
  fit=least_squares(residual,current,bounds=(self.low,self.high),max_nfev=200)
  if np.linalg.norm(residual(fit.x))>1e-4:raise ValueError('Approach solver residual too large')
  near(fit.x,current,8,'Approach requires too much joint travel')
  self.validate(fit.x);return fit.x

def safe_observation(observation,reserve_m=STEP):
 if not math.isfinite(reserve_m) or not 0<=reserve_m<=STEP:raise ValueError('Invalid observation reserve')
 if observation is None:raise ValueError('No stable full-tag observation')
 if not all(math.isfinite(observation[k]) for k in ('plane_distance_m','range_std_m','rms_px')):raise ValueError('Nonfinite tag quality')
 v=np.array(observation['translation_camera_m'])
 if v.shape!=(3,) or not np.isfinite(v).all() or v[2]<=0:raise ValueError('Invalid tag translation')
 if observation['plane_distance_m']<PLANE_STANDOFF+reserve_m:raise ValueError('Too close for this noncontact demo')
 if observation['range_std_m']>.004 or observation['rms_px']>1.5:raise ValueError('Tag measurement not stable enough')
 if v[2]/np.linalg.norm(v)<.85:raise ValueError('Tag too far off the camera forward axis')

class MetricCamera(Camera):
 def poll(self,timeout=2):
  packet=self.queue.tryGet()
  if packet is not None and packet.getLensPosition()!=self.focus:packet=None
  if packet is not None:
   self.frame=packet.getCvFrame();self.count+=1;self.last=time.monotonic()
   trans=packet.getTransformation()
   if not trans.isValid() or 'Perspective' not in str(trans.getDistortionModel()):raise RuntimeError('Invalid camera calibration')
   self.matrix=np.array(trans.getIntrinsicMatrix(),dtype=float)
   self.distortion=np.array(trans.getDistortionCoefficients(),dtype=float)
  if time.monotonic()-self.last>timeout:raise RuntimeError('Camera frames stopped')

class ObservedArm(LiftArm):
 pass

def observe(arm,current,camera,label):
 samples=[];last=-1;deadline=time.monotonic()+2
 while time.monotonic()<deadline:
  arm.send(current);arm.check(current)
  if camera.count==last:continue
  last=camera.count;tags=detect(camera.frame,'36h11',0)
  if len(tags)!=1:continue
  corners=np.array(tags[0]['corners_px'])
  if corners[:,0].min()<8 or corners[:,0].max()>camera.frame.shape[1]-8 or corners[:,1].min()<8 or corners[:,1].max()>camera.frame.shape[0]-8:continue
  poses=estimate(corners,camera.matrix,camera.distortion,.06)
  if not poses or poses[0]['reprojection_rms_px']>1.5:continue
  pose=poses[0];v=np.array(pose['translation_camera_m'])
  normal=cv2.Rodrigues(np.array(pose['rotation_vector']))[0][:,2]
  samples.append((v,abs(float(normal@v)),pose['reprojection_rms_px']))
  if len(samples)>=8:break
 path=ROOT/'outputs'/f'rise_{label}.jpg';camera.cv2.imwrite(str(path),upright(camera.frame))
 result=None
 if len(samples)>=8:
  vectors=np.array([s[0] for s in samples])
  result={'translation_camera_m':np.median(vectors,axis=0).tolist(),'range_m':float(np.median(np.linalg.norm(vectors,axis=1))),
   'range_std_m':float(np.max(vectors.std(axis=0))),'plane_distance_m':float(min(s[1] for s in samples)),'rms_px':float(max(s[2] for s in samples))}
 print(json.dumps({'image':str(path),'observation':result}),flush=True)
 return result

def execute():
 arm=None
 log={'contact_attempted':False,'approach_completed':False,'returned_to_rest':False,'observations':[]}
 try:
  with ExitStack() as stack:
   camera=MetricCamera(stack,False,ROOT/'outputs/rise_last.jpg')
   detect(camera.frame,'36h11',0)
   with ObservedArm(PORT) as arm:
    arm.camera=camera;start=np.array(arm.read());planner=Planner(start);arm.low=planner.low;arm.high=planner.high
    arm.grip=arm.gripper.get_register_f32(80,200)
    liftpath=[planner.lift(float(h)) for h in np.linspace(.005,.2,40)]
    scanpath=[planner.lift(.2,float(t)) for t in range(5,61,5)]
    log['start_rad']=start.tolist();log['speed_rad_s']=SPEED
    arm.start(start);current=start.copy();history=[start.copy()];history_speeds=[SPEED]
    def move(target,speed=SPEED):
     nonlocal current
     arm.segment(current,target,speed);current=target;history.append(current.copy());history_speeds.append(speed)
    def restore():
     nonlocal current
     print('Returning along executed path to rest.',flush=True)
     for index in range(len(history)-1,0,-1):
      target=history[index-1];arm.segment(current,target,history_speeds[index]);current=target
     arm.check(start);log['returned_to_rest']=True
    for index,target in enumerate(liftpath,1):
     move(target,.005 if index<=2 else SPEED)
     if index==2:
      arm.finish_load_take_up(current)
      print('Load taken up; accelerating to .06 rad/s trajectory speed.',flush=True)
     if index%10==0:
      print(f'LIFT: {index*.005:.2f} m commanded',flush=True)
      log['observations'].append(observe(arm,current,camera,f'lift_{index:02d}'))
    found=None
    for index,target in enumerate(scanpath,1):
     move(target);found=observe(arm,current,camera,f'tilt_{index*5:02d}');log['observations'].append(found)
     if found is not None:
      try:safe_observation(found)
      except ValueError as error:print('Observe only:',error,flush=True)
      else:break
    try:safe_observation(found)
    except ValueError as error:print('No approach:',error,flush=True);restore();return
    print('READY FOR 2 CM NONCONTACT APPROACH. Type APPROACH or REST; timeout 60s returns to rest.',flush=True)
    deadline=time.monotonic()+60;answer='REST'
    while time.monotonic()<deadline:
     arm.send(current);arm.check(current)
     if select.select([sys.stdin],[],[],.02)[0]:answer=sys.stdin.readline().strip();break
    if answer=='APPROACH':
     before=observe(arm,current,camera,'before_approach')
     try:safe_observation(before)
     except ValueError as error:print('No approach:',error,flush=True);restore();return
     for _ in range(4):
      move(planner.forward(current,.005),APPROACH_SPEED)
      measured=observe(arm,current,camera,f'approach_{_+1}')
      if measured is None or measured['plane_distance_m']<PLANE_STANDOFF or measured['range_m']>before['range_m']+.003:
       print('Stopping approach: observation lost or standoff reached.',flush=True);break
     else:
      log['approach_completed']=True;log['before_approach']=before;log['after_approach']=measured
      print('2 CM APPROACH COMPLETE; measured camera range change:',before['range_m']-measured['range_m'],flush=True)
    restore()
 finally:
  log['telemetry']=getattr(arm,'telemetry',[])
  (ROOT/'outputs/rise_approach_result.json').write_text(json.dumps(log,indent=2)+'\n')

def main():
 parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true');args=parser.parse_args()
 if args.execute:
  if not sys.stdin.isatty():parser.error('Execute requires an attended terminal')
  execute()
 else:
  profile=json.loads(PROFILE.read_text());planner=Planner(profile['reference_raw_rad'])
  for h in [.05,.10,.15,.20]:print('Lift',h,'joint delta degrees',np.rad2deg(planner.lift(h)-planner.start).round(2).tolist())
  print('Preview only. No hardware accessed. Contact is excluded.')
if __name__=='__main__':
 signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
 main()
