#!/usr/bin/env python3
"""Read-only gripper inspection. No motors enabled or settings changed."""
import argparse,json,math,time,subprocess,shutil
from pathlib import Path
from datetime import datetime,timezone
from calibrate_arm import Reader
from hello_world import PORT

def inspect():
    with Reader(PORT) as arm:
        before=arm.read();grip=arm.add_motor(7,23,'4310')
        q=grip.get_register_f32(80,300);grip.request_feedback();time.sleep(.01);arm.ctrl.poll_feedback_once();s=grip.get_state()
        if s is None or s.status_code!=0:raise RuntimeError('Gripper is not disabled with valid feedback')
        result={'motion_commanded':False,'arm_raw_rad':before,'gripper_raw_rad':q,'gripper_raw_deg':math.degrees(q),'mode':grip.get_register_u32(10,300),'timeout_ticks':grip.get_register_u32(9,300),'status':s.status_code,'torque_nm':s.torq,'velocity_rad_s':s.vel,'temperatures_c':[s.t_mos,s.t_rotor]}
    folder=Path('outputs/gripper');folder.mkdir(exist_ok=True)
    p=folder/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'_inspect.json');p.write_text(json.dumps(result,indent=2)+'\n');print(json.dumps(result,indent=2));print(p)

def trial(stiffness=2.,native=False,opening=False,open_by=.8,force=False):
    if force and (not native or opening):raise ValueError('Force-position trial requires native small-step mode')
    if opening and not native:raise ValueError('Opening survey requires native position mode')
    if open_by not in (.2,.4,.6,.8):raise ValueError('Invalid bounded opening extent')
    if stiffness not in (2.,8.):raise ValueError('Unsupported gripper stiffness')
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');folder=Path('outputs/gripper')/stamp;folder.mkdir(parents=True)
    sensor=Path('outputs/stereo_contact')/('gripper_'+stamp);sensor.mkdir(parents=True)
    record={'native_position_mode':native,'force_position_mode':force,'force_ratio':.02 if force else None,'opening_survey':opening,'stiffness':stiffness,'arm_motors_commanded':False,'samples':[],'gripper_disabled_verified':False};child=None
    try:
        with (folder/'camera.log').open('w') as log:
            child=subprocess.Popen(['.venv-depth-v2/bin/python','stereo_contact_observer.py','--output',str(sensor)],stdout=log,stderr=subprocess.STDOUT)
            deadline=time.monotonic()+20
            while time.monotonic()<deadline:
                if (sensor/'C.jpg').exists() and (sensor/'latest.json').exists():break
                time.sleep(.05)
            else:raise RuntimeError('Gripper cameras unavailable')
            with Reader(PORT) as arm:
                start_arm=arm.read();grip=arm.add_motor(7,23,'4310');start=grip.get_register_f32(80,300)
                mode=grip.get_register_u32(10,300);timeout=grip.get_register_u32(9,300);enabled=False
                def read(bounds=True):
                    q=grip.get_register_f32(80,100);grip.request_feedback();time.sleep(.005);arm.ctrl.poll_feedback_once();state=grip.get_state()
                    if state is None or state.status_code!=(1 if enabled else 0):raise RuntimeError('Unexpected gripper status')
                    if not all(math.isfinite(x) for x in (q,state.torq,state.vel)) or max(state.t_mos,state.t_rotor)>50:raise RuntimeError('Invalid/hot gripper')
                    if bounds and (q<start-(open_by+.05 if opening else .12) or q>start+.12 or abs(state.torq)>(.35 if stiffness==2 and not native else .8) or abs(state.vel)>.3):
                        record['bound_stop']={'q':q,'start':start,'torque':state.torq,'velocity':state.vel}
                        raise RuntimeError('Gripper trial bound exceeded: '+str(record['bound_stop']))
                    return q,state
                read()
                try:
                    desired_mode=4 if force else (2 if native else 1)
                    grip.ensure_mode(desired_mode,1000)
                    if grip.get_register_u32(10,300)!=desired_mode:raise RuntimeError('Gripper mode readback mismatch')
                    grip.set_can_timeout_ms(500)
                    if grip.get_register_u32(9,300)!=10000:raise RuntimeError('Gripper watchdog readback mismatch')
                    def command(target):
                        if force:grip.send_force_pos(target,.03,.02)
                        elif native:grip.send_pos_vel(target,.03)
                        else:grip.send_mit(target,0.,stiffness,.2,0.)
                    if native:
                        gains=[grip.get_register_f32(i,300) for i in (25,26,27,28)]
                        if not all(math.isfinite(v) and v>=0 for v in gains) or gains[0]<=0 or gains[2]<=0:raise RuntimeError('Gripper gains invalid')
                        record['native_gains']=gains
                    command(start);enabled=True;grip.enable()
                    origin=start
                    opening_steps=[-.2*i for i in range(round(open_by/.2)+1)]
                    deltas=opening_steps+opening_steps[-2::-1] if opening else ((0.,.08,0.) if native else (0.,-.08,0.,.08,0.))
                    for index,delta in enumerate(deltas):
                        goal=start+delta;began=time.monotonic();rows=[]
                        duration=max(2.,abs(goal-origin)*1.5/.025)
                        while time.monotonic()-began<duration+1.5:
                            sensor_state=json.loads((sensor/'latest.json').read_text())
                            if time.monotonic()-sensor_state['frame_time']>.5:raise RuntimeError('Gripper camera is stale')
                            q,state=read();u=min(1.,(time.monotonic()-began)/duration);target=origin+(goal-origin)*(u*u*(3-2*u))
                            if abs(target-q)>.09:raise RuntimeError('Gripper tracking limit')
                            command(target);rows.append({'q':q,'torque':state.torq});time.sleep(.01)
                        if abs(q-goal)>.02:raise RuntimeError('Gripper did not settle at survey position')
                        for role in ('B','C'):shutil.copyfile(sensor/f'{role}.jpg',folder/f'{index}_{role}.jpg')
                        record['samples'].append({'index':index,'target_rad':goal,'actual_rad':q,'rows':rows})
                        print('Gripper sample',index,'target',goal,'actual',q,flush=True);origin=goal
                    record['returned_error_rad']=q-start
                finally:
                    if enabled:grip.disable();enabled=False
                    read(bounds=False);record['gripper_disabled_verified']=True
                    grip.ensure_mode(mode,1000);grip.write_register_u32(9,timeout)
                    if grip.get_register_u32(10,300)!=mode or grip.get_register_u32(9,300)!=timeout:raise RuntimeError('Gripper settings not restored')
                    record['settings_restored']=True;record['arm_position_change_rad']=[b-a for a,b in zip(start_arm,arm.read())]
    finally:
        if child:
            (sensor/'stop').touch()
            try:child.wait(timeout=8)
            except subprocess.TimeoutExpired:child.terminate()
        if (sensor/'calibration.json').exists():shutil.copyfile(sensor/'calibration.json',folder/'camera_calibration.json')
        (folder/'report.json').write_text(json.dumps(record,indent=2)+'\n');print('Gripper trial:',folder,flush=True)

if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--trial',action='store_true');p.add_argument('--native',action='store_true');p.add_argument('--force',action='store_true');p.add_argument('--opening-trial',action='store_true');p.add_argument('--open-by',type=float,choices=[.2,.4,.6,.8],default=.8);p.add_argument('--stiffness',type=float,choices=[2.,8.],default=2.);a=p.parse_args()
    if a.trial or a.opening_trial:trial(a.stiffness,a.native,a.opening_trial,a.open_by,a.force)
    else:inspect()
