"""Camera-observed six-axis angular coverage at two noncontact distances."""
import json,math,subprocess,threading,time,shutil,sys
from pathlib import Path
import numpy as np
from stereo_servo import Observer,geometry_error,step


def run_sweep(runner,w,folder,workers):
    folder=Path(folder);sensor=Path('outputs/stereo_contact')/(folder.name+'_sweep');sensor.mkdir(parents=True)
    record={'contact_attempted':False,'samples':[],'attempts':[],'returned_to_entry':False}
    entry=list(runner.targets.values());transit=[entry];child=None;log=None
    saved=(runner.vision_ready,runner.low.copy(),runner.high.copy(),runner.speed,runner.acceleration,runner.tracking_limit,runner.pacing_lag)
    limits=dict(runner.limits);runner.low=np.array([limits[i][0] for i in range(6)]);runner.high=np.array([limits[i][1] for i in range(6)])
    runner.speed=8.;runner.acceleration=36.;runner.tracking_limit=3.;runner.pacing_lag=1.8
    observer=Observer(sensor)
    X_a=np.array(json.loads(Path('calibration/arm2_handeye_fixed_focus_refined_candidate.json').read_text())['T_wrist_camera'])
    rig=json.loads(Path('outputs/contact_camera/20261004T194430628210Z/report.json').read_text());X_b=X_a@np.array(rig['extrinsics']['B_A'])
    def ready():
        try:
            T=np.array(observer.latest()['T_camera_b_tag']);_,gap,other,_=geometry_error(T)
            return min(gap,other)>.07
        except ValueError:return False
    def capture(label):
        T,rows=observer.stable(runner,frames=7);_,gap,other,lateral=geometry_error(T)
        sample={'label':label,'time':time.monotonic(),'raw_deg':runner.rows[-1]['raw_deg'],'T_camera_b_tag':T.tolist(),'gap_m':gap,'other_gap_m':other,'lateral_m':lateral,'position_std_m':np.std([np.array(r['T_camera_b_tag'])[:3,3] for r in rows],axis=0).tolist()}
        record['samples'].append(sample)
        for role in ('B','C'):shutil.copyfile(sensor/f'{role}.jpg',folder/f'sweep_{len(record["samples"])-1:03d}_{role}.jpg')
        (folder/'sweep_progress.json').write_text(json.dumps(record,indent=2)+'\n')
        return T
    try:
        w.stop.set();deadline=time.monotonic()+15
        while any(x.is_alive() for x in workers):
            runner.tick()
            if time.monotonic()>deadline:raise ValueError('RGB camera release timed out')
        log=(folder/'angle_sweep_observer.log').open('w')
        child=subprocess.Popen(['.venv-depth-v2/bin/python','stereo_contact_observer.py','--output',str(sensor)],stdout=log,stderr=subprocess.STDOUT)
        runner.vision_ready=ready;deadline=time.monotonic()+20
        while time.monotonic()<deadline and not ready():runner.tick()
        if not ready():raise ValueError('Stereo sweep view unavailable')
        for station in ('far','middle'):
            if station=='middle':
                for _ in range(30):
                    T=capture('transit');delta,gap,other,lateral=geometry_error(T,.15)
                    if np.linalg.norm(delta)<.004:break
                    goal,_=step(w.geometry,runner.rows[-1]['raw_deg'],T,X_b,limits,standoff=.15,travel_m=.01,joint_deg=2.)
                    transit.append(goal);runner.go(goal)
                else:raise ValueError('Middle sweep station did not converge')
            centre=list(runner.targets.values());capture(station+'_centre')
            for joint in range(6):
                for offset in (4.,8.,-4.,-8.):
                    goal=centre.copy();goal[joint]+=offset
                    attempt={'station':station,'joint':joint+1,'offset_deg':offset,'completed':False}
                    record['attempts'].append(attempt)
                    if not limits[joint][0]<=goal[joint]<=limits[joint][1]:attempt['reason']='joint bound';continue
                    try:
                        runner.go(goal);capture(f'{station}_joint{joint+1}_{offset:+g}');attempt['completed']=True
                    except ValueError as error:attempt['reason']=str(error)
                    finally:runner.go(centre,returning=True)
                    # Visibility must recover at the proven centre before any new angle.
                    capture(station+'_return')
                print(f'Sweep {station}: joint {joint+1}/6 complete',flush=True)
        record['completed']=True
    except ValueError as error:
        record['stop_reason']=str(error);print('Sweep return: '+str(error),flush=True)
    finally:
        healthy=sys.exc_info()[0] is None
        try:
            if healthy:
                from camera_visual_approach import fast_indices
                for i in list(reversed(fast_indices(transit)))[1:]:runner.go(transit[i],returning=True)
                runner.go(entry,returning=True);record['returned_to_entry']=True
        finally:
            if child:
                (sensor/'stop').touch()
                try:
                    if healthy:
                        deadline=time.monotonic()+10
                        while child.poll() is None and time.monotonic()<deadline:runner.tick()
                finally:
                    if child.poll() is None:child.terminate()
                if log:log.close()
            runner.vision_ready,runner.low,runner.high,runner.speed,runner.acceleration,runner.tracking_limit,runner.pacing_lag=saved
            (folder/'angle_sweep_report.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
        if healthy:
            w.stop.clear();workers=[threading.Thread(target=w.camera_worker,args=(role,),daemon=True) for role in ('wrist','tripod')]
            for worker in workers:worker.start()
            deadline=time.monotonic()+20
            while time.monotonic()<deadline and not runner.vision_ready():runner.tick()
    return record,workers
