#!/usr/bin/env python3
"""Continuously blended lift, camera tilt, noncontact approach and return.
The initial load transfer stays gentle; vision and path checks run during motion.
"""
from contextlib import ExitStack
from collections import deque
from pathlib import Path
import argparse,json,math,signal,time
import numpy as np
from scipy.interpolate import BSpline
from arm_control import LiftArm
from hello_world import PORT
from rise_approach import Planner,MetricCamera,ROOT,PROFILE,PLANE_STANDOFF
from tag_view import detect
from tag_pose import estimate
from camera_view import upright


def smoothstep(x):
 x=float(np.clip(x,0,1));return x*x*x*(10+x*(-15+6*x))


def moving_load_ready(samples,height):
 if height<.009 or len(samples)<20:return False
 end=samples[-1][0];recent=[r for r in samples if r[0]>=end-2.1]
 return len(recent)>=20 and end-recent[0][0]>=2. and max(r[1] for r in recent)<=.3


class Route:
 def __init__(self,start):
  self.planner=Planner(start);self.start=np.array(start,dtype=float)
  controls=[self.start]+[self.planner.lift(float(h)) for h in np.linspace(.005,.2,40)]
  self.lift_control=len(controls)-1
  controls += [self.planner.lift(.2,float(a)) for a in np.arange(2.5,40.1,2.5)]
  self.approach_control=len(controls)-1
  q=controls[-1];self.approach_m=0.
  for _ in range(24):
   try:q=self.planner.forward(q,.0025)
   except ValueError:break
   controls.append(q);self.approach_m+=.0025
  if self.approach_m<.04:raise ValueError('Insufficient reach for the verified closer route')
  controls=np.array(controls);n=len(controls)
  knots=np.r_[np.zeros(4),np.linspace(0,1,n-2)[1:-1],np.ones(4)]
  spline=BSpline(knots,controls,3)
  self.parameter=np.linspace(0,1,10001);self.q=spline(self.parameter)
  if np.any(self.q<self.planner.low) or np.any(self.q>self.planner.high):raise ValueError('Blended route outside joint bounds')
  self.arc=np.r_[0.,np.cumsum(np.max(abs(np.diff(self.q,axis=0)),axis=1))]
  # Gate before the lift/tilt curve begins blending into forward translation.
  gate_u=(self.approach_control-3)/(n-3)
  self.gate_index=int(np.searchsorted(self.parameter,gate_u))
  self.gate_arc=float(self.arc[self.gate_index])
  self.low=self.planner.low;self.high=self.planner.high
 def locate(self,q,index):
  low=max(0,index-600);high=min(len(self.q),index+601)
  errors=np.max(abs(self.q[low:high]-q),axis=1);offset=int(np.argmin(errors))
  return low+offset,float(errors[offset])
 def preview(self,index,direction,lookahead):
  goal=float(np.clip(self.arc[index]+direction*lookahead,0,self.arc[-1]))
  other=int(np.searchsorted(self.arc,goal));other=min(other,len(self.q)-1)
  # Limit corner cutting by reducing preview until its chord follows the curve.
  for _ in range(8):
   points=np.linspace(index,other,9).astype(int)
   line=np.linspace(self.q[index],self.q[other],9)
   if np.max(abs(self.q[points]-line))<=math.radians(.2):break
   other=(other+index)//2
  return other


class MovingVision:
 def __init__(self,camera):self.camera=camera;self.last_count=-1;self.samples=deque(maxlen=8)
 def poll(self):
  c=self.camera
  if c.count==self.last_count:return
  self.last_count=c.count;tags=detect(c.frame,'36h11',0)
  if len(tags)!=1:return
  corners=np.array(tags[0]['corners_px']);h,w=c.frame.shape[:2]
  if corners[:,0].min()<8 or corners[:,0].max()>w-8 or corners[:,1].min()<8 or corners[:,1].max()>h-8:return
  poses=estimate(corners,c.matrix,c.distortion,.06)
  if not poses or poses[0]['reprojection_rms_px']>1.5:return
  best=poses[0];vector=np.array(best['translation_camera_m'])
  import cv2
  normal=cv2.Rodrigues(np.array(best['rotation_vector']))[0][:,2]
  self.samples.append({'t':time.monotonic(),'vector':vector.tolist(),'plane_m':abs(float(normal@vector)),'rms_px':best['reprojection_rms_px']})
 def latest(self):
  if len(self.samples)<8 or time.monotonic()-self.samples[-1]['t']>.25:return None
  rows=list(self.samples);times=np.array([r['t'] for r in rows]);times-=times[0]
  if times[-1]>.9 or times[-1]<.25:return None
  vectors=np.array([r['vector'] for r in rows])
  model=np.c_[np.ones(len(times)),times]
  residual=vectors-model@np.linalg.lstsq(model,vectors,rcond=None)[0]
  if np.max(np.linalg.norm(residual,axis=1))>.004:return None
  last=rows[-1]
  return {'range_m':float(np.linalg.norm(last['vector'])),'plane_m':last['plane_m'],'vector':last['vector'],'rms_px':last['rms_px']}


def execute():
 arm=None;log={'returned_to_rest':False,'contact_attempted':False,'forward_complete':False,'motion':[]}
 try:
  with ExitStack() as stack:
   camera=MetricCamera(stack,False,ROOT/'outputs/continuous_last.jpg');vision=MovingVision(camera)
   with LiftArm(PORT) as arm:
    arm.camera=camera;start=np.array(arm.read());route=Route(start);g=route.planner.g
    arm.low=route.low;arm.high=route.high;arm.grip=arm.gripper.get_register_f32(80,200)
    arm.start(start);actual=start.copy();index=0;direction=1;reference_arc=0.;load_arc=None
    samples=deque(maxlen=150);began=last=time.monotonic();cycle_last=began;previous_speed=.0;reported=False;approach_ok=False
    log['planned_approach_m']=route.approach_m
    print('Starting continuous route: gentle load take-up, blended lift/tilt, checked approach, then return.',flush=True)
    while True:
     now=time.monotonic();dt=now-cycle_last;cycle_last=now
     if dt>.5:raise RuntimeError('Motion loop stalled')
     index,error=route.locate(actual,index)
     if error>math.radians(1):raise RuntimeError('Deviation exceeds 1 degree from the blended route')
     s=float(route.arc[index]);height=float(g.transform(actual)[2,3]-route.planner.base[2,3])
     if not arm.load_taken:
      reference_arc=min(reference_arc+.005*max(dt,.01),s+math.radians(.5))
      target_index=min(int(np.searchsorted(route.arc,reference_arc)),len(route.q)-1)
      target=route.q[target_index];arm.send(target)
      requested_speed=.005
     else:
      if direction==1 and s>=route.gate_arc-.025 and not approach_ok:
       seen=vision.latest()
       if seen is None or seen['plane_m']<PLANE_STANDOFF+route.approach_m+.01:
        log['stop_reason']='Vision gate not satisfied; reversing';log['gate_observation']=seen;log['gate_samples']=list(vision.samples);camera.cv2.imwrite(str(ROOT/'outputs/continuous_gate.jpg'),upright(camera.frame));direction=-1
       else:
        approach_ok=True;log['before_approach']=seen
        print('Tag verified while moving; blending into approach.',flush=True)
      if direction==1 and approach_ok:
       seen=vision.latest()
       if seen is None or seen['plane_m']<PLANE_STANDOFF+.01:
        log['stop_reason']='Vision/clearance guard; reversing';direction=-1
      # Smooth acceleration after take-up, and deceleration into the approach.
      speed=.005+(.48-.005)*smoothstep((s-load_arc)/.14)
      if direction==1:
       factor=smoothstep((route.gate_arc+.02-s)/.20)
       speed=.03+(speed-.03)*factor
       # Brake early enough to obtain eight fresh frames during the final tilt.
       speed=min(speed,math.sqrt(.03**2+2*.12*max(0.,route.gate_arc-.12-s)))
      if direction==-1:
       speed=.005+(.48-.005)*smoothstep((s-load_arc)/.14)
      remaining=route.arc[-1]-s if direction==1 else s
      speed=min(speed,math.sqrt(max(0.,2*.12*remaining)))
      if error>math.radians(.6):speed=min(speed,.03)
      # Motor limits ramp continuously; targets look ahead along the same curve.
      requested_speed=float(np.clip(speed,max(.003,previous_speed-.3*dt),previous_speed+.30*dt))
      requested_speed=max(.003,requested_speed)
      lookahead=min(.06,max(.008,requested_speed*.35))
      if direction==1 and s>route.gate_arc-.03:lookahead=min(lookahead,.015)
      target_index=route.preview(index,direction,lookahead);target=route.q[target_index]
      delta=abs(target-actual);limits=np.maximum(.01,requested_speed*delta/max(float(delta.max()),1e-9))
      for motor,x,limit in zip(arm.motors,target,limits):motor.send_pos_vel(float(x),float(limit))
      camera.poll()
     measured=np.array(arm.read());stamp=time.monotonic()
     if stamp-last>.5:raise RuntimeError('Feedback cycle stalled')
     velocity=float(np.max(abs(measured-actual)))/max(stamp-last,.001)
     if velocity>.74:raise RuntimeError('Measured speed exceeds continuous-motion bound')
     from teach_approach import near
     near([arm.gripper.get_register_f32(80,200)],[arm.grip],.5,'Gripper opening changed')
     target_error=float(np.rad2deg(np.max(abs(measured-target))))
     if not arm.load_taken:
      if target_error>1:raise RuntimeError('Initial load take-up tracking error')
      samples.append((stamp,target_error))
      measured_height=float(g.transform(measured)[2,3]-route.planner.base[2,3])
      if moving_load_ready(samples,measured_height):
       arm.load_taken=True;load_arc=s;previous_speed=.005
       print('Load stable while moving; smoothly accelerating.',flush=True)
      elif measured_height>.015:raise RuntimeError('Load did not stabilize in initial lift')
     actual=measured;last=stamp;previous_speed=requested_speed
     vision.poll()
     index,error=route.locate(actual,index)
     log['motion'].append({'t':stamp,'direction':direction,'arc':float(route.arc[index]),'path_error_deg':math.degrees(error),'speed_limit_rad_s':requested_speed,'measured_speed_rad_s':velocity,'raw_joint_rad':actual.tolist()})
     if error>math.radians(1):raise RuntimeError('Deviation exceeds 1 degree from blended route')
     if direction==1 and not reported and height>.18:print('Lift blended into camera tilt; motion continuing.',flush=True);reported=True
     endpoint=route.q[-1] if direction==1 else start
     if arm.load_taken and np.max(abs(actual-endpoint))<math.radians(.15):
      if direction==1:
       log['forward_complete']=True;log['closest_observation']=vision.latest()
       camera.cv2.imwrite(str(ROOT/'outputs/continuous_closest.jpg'),upright(camera.frame))
       print('Approach endpoint reached; continuous turn-around.',flush=True);direction=-1;previous_speed=.003
      else:
       log['returned_to_rest']=True;print('Returned to rest.',flush=True);return
     if stamp-began>150 and direction==1:log['stop_reason']='Duration guard; reversing';direction=-1
     if stamp-began>240:raise RuntimeError('Return timed out')
     time.sleep(.01)
 finally:
  (ROOT/'outputs/continuous_result.json').write_text(json.dumps(log,indent=2)+'\n')


def main():
 p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');a=p.parse_args()
 if a.execute:execute()
 else:
  route=Route(json.loads(PROFILE.read_text())['reference_raw_rad'])
  print(f'Preview: {len(route.q)} samples, {route.approach_m*100:.1f} cm approach, continuous lift/tilt blends. No hardware accessed.')
if __name__=='__main__':
 signal.signal(signal.SIGTERM,lambda *_:(_ for _ in ()).throw(KeyboardInterrupt()))
 main()
