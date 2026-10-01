"""Experimental endpoint controller; failed hardware validation and is not used.

Use rise_approach.py and arm_control.py for the verified load-take-up sequence.
Short synchronized position moves, monitored against the geometric path.

Damiao POS_VEL handles the velocity-limited motion to each endpoint. The
monitor checks measured path deviation, speed, endpoint settling and timeout;
it does not mistake uniform lag along the route for a collision-path deviation.
"""
import math
import time
import numpy as np
from teach_approach import ReplayArm, near

def path_error(q,a,b):
 q=np.asarray(q);a=np.asarray(a);b=np.asarray(b);d=b-a
 denominator=float(d@d)
 fraction=0. if denominator<1e-15 else float((q-a)@d/denominator)
 closest=a+np.clip(fraction,0,1)*d
 return float(np.max(abs(q-closest)))

class BoundedPositionArm(ReplayArm):
 def segment(self,a,b,speed):
  a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float);delta=abs(b-a);distance=float(max(delta))
  if not 0<speed<=.06 or distance>math.radians(8):raise ValueError('Segment speed or excursion too large')
  if np.any(b<self.low) or np.any(b>self.high):raise ValueError('Target outside joint envelope')
  measured=np.array(self.check(a));previous=measured;last=time.monotonic();start=last
  deadline=start+max(4.,4*distance/speed+2)
  limits=np.maximum(.01,speed*delta/max(distance,1e-12))
  stable=0;max_error=0.
  while True:
   now=time.monotonic()
   if now-last>.5:raise RuntimeError('Control loop stalled')
   if now>deadline:raise RuntimeError('Waypoint did not settle in time')
   for motor,target,limit in zip(self.motors,b,limits):motor.send_pos_vel(float(target),float(limit))
   if hasattr(self,'camera'):self.camera.poll()
   measured=np.array(self.read());timestamp=time.monotonic()
   error=path_error(measured,a,b);max_error=max(max_error,error)
   if error>math.radians(1):
    print('PATH FAULT: actual degrees',np.rad2deg(measured).tolist(),'start',np.rad2deg(a).tolist(),'target',np.rad2deg(b).tolist(),flush=True)
    raise RuntimeError('Measured deviation exceeds 1 degree from the planned path')
   if np.max(abs(measured-previous))/max(timestamp-last,.001)>1.5*speed+.02:
    raise RuntimeError('Measured joint speed exceeds bound')
   near([self.gripper.get_register_f32(80,200)],[self.grip],.5,'Gripper changed')
   if np.max(abs(measured-b))<math.radians(.15):stable+=1
   else:stable=0
   if stable>=3:
    if hasattr(self,'motion_log'):self.motion_log.append({'duration_s':time.monotonic()-start,'max_path_error_deg':math.degrees(max_error),'speed_rad_s':speed,'start':a.tolist(),'end':b.tolist()})
    return
   previous=measured;last=timestamp;time.sleep(.025)
