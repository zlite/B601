"""Read raw motor state and stationary cameras without enabling or clearing motors."""
import json,sys,time,threading,math,argparse
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
import cv2
from calibrate_arm import Reader
from hello_world import PORT
from pairing_dashboard import Workbench

parser=argparse.ArgumentParser();parser.add_argument('--clear-communication-at-rest',action='store_true');args=parser.parse_args()
folder=Path('outputs/plate_status');folder.mkdir(exist_ok=True)
rows=[]
with Reader(PORT) as arm:
    for index,motor in enumerate(arm.motors):
        q=motor.get_register_f32(80,200);motor.request_feedback();time.sleep(.02);arm.ctrl.poll_feedback_once();s=motor.get_state()
        rows.append({'joint':index+1,'q_rad':q,'status':s.status_code,'velocity':s.vel,'torque':s.torq,'temperature':[s.t_mos,s.t_rotor]})
    if args.clear_communication_at_rest:
        reference=json.loads(Path('calibration/arm2_reference.json').read_text())['reference_raw_rad']
        if any(r['status'] not in (0,13) or max(r['temperature'])>50 or abs(r['velocity'])>.02 for r in rows):
            raise RuntimeError('Fault reset preconditions failed')
        if any(abs(math.degrees(rows[i]['q_rad']-reference[i]))>1.5 for i in (1,2)):
            raise RuntimeError('Timeout reset requires folded resting shoulder and elbow')
        time.sleep(.3)
        for row,motor in zip(rows,arm.motors):
            if abs(math.degrees(motor.get_register_f32(80,200)-row['q_rad']))>.1:
                raise RuntimeError('Arm is moving; reset refused')
        for row,motor in zip(rows,arm.motors):
            if row['status']==13:motor.clear_error()
            motor.disable();motor.request_feedback();time.sleep(.02);arm.ctrl.poll_feedback_once()
            row['status_after_clear']=motor.get_state().status_code
            if row['status_after_clear']!=0:raise RuntimeError('Disabled status not verified after timeout reset')
print(json.dumps(rows,indent=2),flush=True)
(folder/'motors.json').write_text(json.dumps(rows,indent=2)+'\n')
w=Workbench();w.survey_mode=True;w.camera_sizes={'tripod':(1280,800)}
workers=[threading.Thread(target=w.camera_worker,args=(role,),daemon=True) for role in ('wrist','tripod')]
for worker in workers:worker.start()
try:
    deadline=time.monotonic()+25
    while time.monotonic()<deadline:
        with w.lock:frames=dict(getattr(w,'survey_frames',{}))
        if all(role in frames for role in ('wrist','tripod')) and time.monotonic()>deadline-20:
            for role,frame in frames.items():cv2.imwrite(str(folder/(role+'.png')),frame['image'])
            print('Fresh stationary images:',folder,flush=True);break
        time.sleep(.05)
finally:
    w.stop.set()
    for worker in workers:worker.join(timeout=30)
