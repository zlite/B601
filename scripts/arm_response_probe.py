"""Read-only motor settings and transaction timing; refuses enabled motors."""
import json,sys,time,math,argparse,os
from pathlib import Path
from datetime import datetime,timezone
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from calibrate_arm import Reader
from hello_world import PORT
from motorbridge.damiao_registers import DAMIAO_RW_REGISTERS

def main():
    parser=argparse.ArgumentParser();parser.add_argument('--fresh',action='store_true')
    parser.add_argument('--status-only',action='store_true',help='Read raw statuses, including faults; never clears faults or enables motors')
    args=parser.parse_args()
    output=ROOT/'outputs/arm_response'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');output.mkdir(parents=True)
    report={'motion_commanded':False,'settings_changed':False,'joints':[]}
    if args.status_only:
        with Reader(PORT) as arm:
            for i,m in enumerate(arm.motors):
                q=m.get_register_f32(80,200)
                m.request_feedback();time.sleep(.02);arm.ctrl.poll_feedback_once();s=m.get_state()
                report['joints'].append({'joint':i+1,'raw_deg':math.degrees(q),
                    'status_code':None if s is None else s.status_code,
                    'velocity_rad_s':None if s is None else s.vel,
                    'temperature_c':None if s is None else [s.t_mos,s.t_rotor],
                    'timeout_register':m.get_register_u32(9,200)})
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print(json.dumps({'report':str(output/'report.json'),**report},indent=2));return
    from axis_follow import AxisArm
    if args.fresh:os.environ['B601_FRESH_FEEDBACK']='1'
    Arm=AxisArm if args.fresh else lambda:Reader(PORT)
    with Arm() as arm:
        report['start_raw_deg' if args.fresh else 'start_raw_rad']=arm.read()
        for i,m in enumerate(arm.motors):
            regs={str(rid):{'name':DAMIAO_RW_REGISTERS[rid].variable,'value':m.get_register_f32(rid,300)} for rid in (4,5,6,24,25,26,27,28,30,31,32,33,34)}
            regs['10']={'name':'CTRL_MODE','value':m.get_register_u32(10,300)}
            report['joints'].append({'joint':i+1,'registers':regs})
        samples=[]
        for _ in range(20):
            row=[]
            for m in arm.motors:
                began=time.monotonic();q=m.get_register_f32(80,100);queried=time.monotonic()
                m.request_feedback();time.sleep(.005);arm.ctrl.poll_feedback_once();s=m.get_state()
                if s is None or s.status_code!=0:raise RuntimeError('Motor is not disabled')
                row.append({'query_s':queried-began,'feedback_s':time.monotonic()-queried,'raw_rad':q,'feedback_raw_rad':s.pos})
            samples.append(row)
        report['timing_samples']=samples
        if args.fresh:
            fast=[]
            for _ in range(100):
                began=time.monotonic();q=arm.read();fast.append({'elapsed_s':time.monotonic()-began,'raw_deg':q})
            report['fresh_samples']=fast
            report['fresh_median_s']=float(np.median([x['elapsed_s'] for x in fast]));report['fresh_max_s']=max(x['elapsed_s'] for x in fast)
        report['final_raw_rad' if not args.fresh else 'final_raw_deg']=arm.read();report['motors_disabled_verified']=True
    (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
    print(json.dumps({'report':str(output/'report.json'),'joints':report['joints'],'fresh_median_s':report.get('fresh_median_s'),'fresh_max_s':report.get('fresh_max_s'),'median_query_s':float(np.median([[x['query_s'] for x in row] for row in samples]))},indent=2))
if __name__=='__main__':main()
