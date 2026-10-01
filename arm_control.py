"""B601-DM position control with vendor arm gains and retained wrist gains.

Values are from vendor/reBotArm_control_py/config/rebotarm_dm.yaml. Never
stores parameters to flash or changes zeros, protection or mapping registers.
Original gains are restored after disabling, including on setup/motion faults.
"""
import math
import time
from teach_approach import ReplayArm

GAIN_REGISTERS=(25,26,27,28)
B601_GAINS=((.0125,.004,150.,.5),)*3+((.0008,.002,70.,1.),(.0008,.002,60.,1.),(.0008,.002,60.,1.))

class ConfiguredArm(ReplayArm):
 def __enter__(self):
  self.saved_gains=[];self.gains_changed=False
  return super().__enter__()
 def configure_gains(self):
  if self.active:raise RuntimeError('Configure gains only while disabled')
  self.read()  # checks all six disabled and fresh finite positions
  self.saved_gains=[[m.get_register_f32(rid,500) for rid in GAIN_REGISTERS] for m in self.motors]
  if not all(math.isfinite(v) for row in self.saved_gains for v in row):raise RuntimeError('Invalid original gains')
  self.gains_changed=True  # rollback also covers a partial write failure
  self._write_gains((*B601_GAINS[:3],*self.saved_gains[3:]))
 def _write_gains(self,rows):
  for motor,row in zip(self.motors,rows):
   for rid,value in zip(GAIN_REGISTERS,row):
    motor.write_register_f32(rid,value)
    actual=motor.get_register_f32(rid,500)
    if not math.isclose(actual,value,rel_tol=1e-5,abs_tol=1e-7):raise RuntimeError(f'Gain register {rid} readback mismatch')
 def __exit__(self,*args):
  restore_error=None
  try:
   if self.gains_changed:
    if self.touched:
     for motor in self.motors:motor.disable()
     self.active=False
     time.sleep(.03)
     self.read()  # refuse to retune an arm that did not disable
     self.touched=False
    self._write_gains(self.saved_gains)
    self.gains_changed=False
  except Exception as error:
   restore_error=error
   print(f'Controller cleanup failed: {error}',flush=True)
  finally:
   super().__exit__(*args)
  if restore_error and (not args or args[0] is None):raise restore_error


class LiftArm(ConfiguredArm):
 """Verified position controller. Take the first centimeter at .005 rad/s
 and settle for two seconds before .06 rad/s travel. Do not skip load take-up.
 """
 def __enter__(self):
  self.telemetry=[];self.load_taken=False
  return super().__enter__()
 def send(self,q,speed=.01):
  # Trajectory speed comes from segment timing. This is correction headroom.
  super().send(q,.25)
  if hasattr(self,'camera'):self.camera.poll()
 def check(self,target):
  import numpy as np
  from teach_approach import near
  q=self.read();error=np.rad2deg(np.asarray(q)-target)
  self.telemetry.append({'t':time.monotonic(),'target_rad':np.asarray(target).tolist(),'actual_rad':q,'error_deg':error.tolist()})
  near(q,target,1,'Tracking error above 1 degree')
  near([self.gripper.get_register_f32(80,200)],[self.grip],.5,'Gripper opening changed')
  return q
 def start(self,q):
  import numpy as np
  self.lift_start=np.array(q,dtype=float)
  from motorbridge import Mode
  from teach_approach import near
  self.configure_gains();near(self.read(),q,.3,'Start moved during gain setup');self.touched=True
  for motor,x in zip(self.motors,q):motor.ensure_mode(Mode.POS_VEL,1500);motor.send_pos_vel(float(x),.01)
  near(self.read(),q,.3,'Start moved during mode setup')
  for motor,x in zip(self.motors,q):motor.send_pos_vel(float(x),.01);motor.enable();motor.send_pos_vel(float(x),.01)
  self.active=True;self.settle(q,1.)
 def settle(self,q,seconds=2.):
  until=time.monotonic()+seconds
  while time.monotonic()<until:self.send(q);self.check(q);time.sleep(.02)

 def finish_load_take_up(self,target):
  import numpy as np
  from arm_geometry import Geometry
  from teach_approach import near
  geometry=Geometry();delta=geometry.transform(target)[:3,3]-geometry.transform(self.lift_start)[:3,3]
  if not .009<=delta[2]<=.011 or np.linalg.norm(delta[:2])>.002:
   raise ValueError('Load take-up requires the first 1 cm vertical lift')
  self.settle(target,2.)
  near(self.read(),target,.3,'Arm has not settled after taking the load')
  self.load_taken=True
 def segment(self,a,b,speed):
  if not self.load_taken and speed>.005:raise ValueError('Take the load slowly before faster motion')
  if not 0<speed<=.06:raise ValueError('Speed exceeds the tested range')
  return super().segment(a,b,speed)
