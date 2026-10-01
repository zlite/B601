"""Velocity-limited waypoint motion after the arm has taken its load.

Monitor measured deviation from the joint-space segment (<=1 degree), measured
speed, feedback health and endpoint settling. Native position mode supplies the
ramp; a waypoint may lag in time but may not leave the bounded geometric path.
"""
import math
import time
import numpy as np
from teach_approach import near


def path_error(q,a,b):
 q=np.asarray(q,dtype=float);a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float)
 if any(v.shape!=(6,) or not np.isfinite(v).all() for v in [q,a,b]):raise ValueError('Invalid joint vector')
 d=b-a;denominator=float(d@d)
 fraction=0. if denominator<1e-15 else float((q-a)@d/denominator)
 return float(np.max(abs(q-(a+np.clip(fraction,0,1)*d))))


def move_waypoint(arm,a,b,speed):
 a=np.asarray(a,dtype=float);b=np.asarray(b,dtype=float)
 path_error(a,a,b)
 if not arm.load_taken:raise ValueError('Load take-up must complete before waypoint motion')
 if not math.isfinite(speed) or not .01<=speed<=.24:raise ValueError('Waypoint speed outside 0.01..0.24 rad/s')
 delta=abs(b-a);distance=float(max(delta))
 if distance>math.radians(8):raise ValueError('Waypoint exceeds 8 degrees')
 if np.any(b<arm.low) or np.any(b>arm.high):raise ValueError('Target outside joint envelope')
 previous=np.array(arm.check(a));start=last=time.monotonic()
 deadline=start+max(4.,4*distance/speed+2)
 limits=np.maximum(.01,speed*delta/max(distance,1e-12));stable=0;max_error=0.;max_velocity=0.
 samples=[]
 result={'completed':False,'speed_limit_rad_s':speed,'samples':samples}
 if hasattr(arm,'waypoint_log'):arm.waypoint_log.append(result)
 while True:
  now=time.monotonic()
  if now-last>.5:raise RuntimeError('Control loop stalled')
  if now>deadline:raise RuntimeError('Waypoint did not settle in time')
  for motor,target,limit in zip(arm.motors,b,limits):motor.send_pos_vel(float(target),float(limit))
  if hasattr(arm,'camera'):arm.camera.poll()
  measured=np.array(arm.read());timestamp=time.monotonic()
  if timestamp-now>.5:raise RuntimeError('Feedback cycle stalled')
  error=path_error(measured,a,b);velocity=float(np.max(abs(measured-previous)))/max(timestamp-last,.001)
  max_error=max(max_error,error);max_velocity=max(max_velocity,velocity)
  samples.append({'t':timestamp,'actual_rad':measured.tolist(),'path_error_deg':math.degrees(error),'joint_speed_rad_s':velocity})
  if error>math.radians(1):raise RuntimeError('Measured deviation exceeds 1 degree from the planned path')
  if velocity>1.5*speed+.02:raise RuntimeError('Measured joint speed exceeds bound')
  near([arm.gripper.get_register_f32(80,200)],[arm.grip],.5,'Gripper opening changed')
  stable=stable+1 if np.max(abs(measured-b))<math.radians(.2) else 0
  if stable>=3:
   result.update(completed=True,duration_s=time.monotonic()-start,max_path_error_deg=math.degrees(max_error),peak_joint_speed_rad_s=max_velocity)
   return result
  previous=measured;last=timestamp;time.sleep(.01)
