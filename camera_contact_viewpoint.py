#!/usr/bin/env python3
"""Shift only the base 6 degrees to clear stereo tag occlusion at folded rest."""
import argparse,json,threading,time,subprocess
from pathlib import Path
from datetime import datetime,timezone
from axis_follow import AxisArm
from pairing_dashboard import Workbench
from camera_check import CameraCheck
from calibrate_joint_motion import Runner,plan


def execute(stereo_elevated=False):
    folder=Path('outputs/contact_viewpoint')/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    folder.mkdir(parents=True);w=Workbench();w.camera_check=CameraCheck(w.geometry,folder/'views')
    workers=[threading.Thread(target=w.camera_worker,args=(r,),daemon=True) for r in ('wrist','tripod')]
    for worker in workers:worker.start()
    report={'purpose':'Unocclude tag in both stereo cameras; folded shoulder/elbow unchanged','motors_disabled_verified':False}
    runner=None
    try:
        with AxisArm() as arm:
            start=arm.read();_,limits=plan(w.geometry,start,[1] if stereo_elevated else [0]);report['start_raw_deg']=start
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                q=arm.read();w.camera_check.add_joints(time.monotonic(),q,powered=False,following=False,all_powered=False,fault=None)
                if max(abs(a-b) for a,b in zip(q,start))>.1:raise RuntimeError('Starting pose moved')
                if w.camera_check.snapshot()['ready']:break
                time.sleep(.005)
            else:raise RuntimeError('Vision did not become ready')
            arm.prepare_group(range(6));arm.speed_limits={i:20. for i in range(6)};arm.enable_group(dict(enumerate(start)))
            runner=Runner(w,arm,start,limits,folder);runner.joint=1 if stereo_elevated else 0
            deadline=time.monotonic()+1
            while time.monotonic()<deadline:runner.tick(take_up=True)
            goal=list(start);goal[runner.joint]+=-8. if stereo_elevated else 6.
            try:
                runner.move(goal);report['view']=runner.capture()
                report['reached_viewpoint']=True
                if stereo_elevated:
                    report['purpose']='Raise shoulder 8 degrees, hold while measuring stereo fingertips, return to folded start'
                    w.stop.set()
                    deadline=time.monotonic()+15
                    while any(worker.is_alive() for worker in workers):
                        runner.tick()
                        if time.monotonic()>deadline:raise ValueError('Camera workers did not release devices')
                    with (folder/'stereo_probe.log').open('w') as log:
                        child=subprocess.Popen(['.venv-depth-v2/bin/python','contact_camera_probe_v2.py'],stdout=log,stderr=subprocess.STDOUT)
                        try:
                            deadline=time.monotonic()+50
                            while child.poll() is None:
                                runner.tick()
                                if time.monotonic()>deadline:raise ValueError('Stereo probe timed out')
                            report['stereo_probe_exit_code']=child.returncode
                        finally:
                            if child.poll() is None:child.terminate()
                    runner.move(start,returning=True)
                    report['returned_to_start']=True
            except ValueError:
                runner.move(start,returning=True);raise
            # Only base changed; the shoulder and elbow remain at folded rest.
            report['before_disable_raw_deg']=arm.read()
        with AxisArm() as reader:time.sleep(.5);report['final_raw_deg']=reader.read();report['motors_disabled_verified']=True
    finally:
        w.stop.set()
        for worker in workers:worker.join(timeout=8)
        if runner:report['motion_samples']=runner.rows
        (folder/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print('Viewpoint report:',folder/'report.json',flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--execute',action='store_true');parser.add_argument('--stereo-elevated',action='store_true');args=parser.parse_args()
    if args.execute:execute(args.stereo_elevated)
    else:print('Preview: base +6 degrees only, at folded rest; stops powered off at the new view. No approach/contact.')
