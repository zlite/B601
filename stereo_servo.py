"""Incremental stereo position servo. Optional bounded final approach to the estimated paper plane."""
import json,math,time
from pathlib import Path
import cv2
import numpy as np
from arm_geometry import Geometry
from rotation_math import rotation_vector

TIP_B=np.array([-.0028641976884,.0008496377972,.1551784683549])
OTHER_TIP_B=np.array([.0822,.0008,.1527])
TAG_POINT=np.array([0.,-.035,0.])  # white border, leaving the black pattern visible


def mean_pose(rows):
    matrices=np.array([r['T_camera_b_tag'] for r in rows])
    t=matrices[:,:3,3]
    if np.max(t.std(0))>.001:raise ValueError('Stereo position not stationary to 1 mm')
    u,_,v=np.linalg.svd(matrices[:,:3,:3].mean(0));fix=np.eye(3);fix[2,2]=np.linalg.det(u@v)
    R=u@fix@v
    span=max(float(np.linalg.norm(cv2.Rodrigues(R.T@x)[0])) for x in matrices[:,:3,:3])
    if span>math.radians(1.5):raise ValueError('Stereo orientation still changing')
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t.mean(0)
    return T


def geometry_error(T,standoff=.03):
    normal=T[:3,2];contact=T[:3,:3]@TAG_POINT+T[:3,3]
    delta=contact+normal*standoff-TIP_B
    gap=float(normal@(TIP_B-contact))
    other_gap=float(normal@(OTHER_TIP_B-contact))
    lateral=float(np.linalg.norm((TIP_B-contact)-normal*gap))
    return delta,gap,other_gap,lateral


def step(geometry,q_deg,T,X_b,limits,standoff=.03,travel_m=.002,joint_deg=.5,contact=False):
    """Small nominal-Jacobian step; its actual effect must be measured again."""
    if not .00025<=travel_m<=.020 or not .1<=joint_deg<=4.:raise ValueError('Invalid incremental movement bound')
    if standoff<.03 and not contact:raise ValueError('Contact is not enabled in this controller')
    if standoff<0:raise ValueError('No commanded penetration beyond the estimated paper plane')
    q=np.radians(q_deg);C=geometry.transform(q)@X_b;tip=(C@np.r_[TIP_B,1])[:3]
    delta,gap,other,lateral=geometry_error(T,standoff)
    if not contact and min(gap,other)<.015:raise ValueError('Reached fingertip noncontact clearance guard')
    if contact and (gap<-.001 or other<.002):raise ValueError('Contact clearance guard reached')
    if contact and (travel_m>.002 or joint_deg>.5 or (gap<.010 and travel_m>.0005)):
        raise ValueError('Final contact movement exceeds its small-step bound')
    desired=C[:3,:3]@delta
    desired*=min(1.,travel_m/max(np.linalg.norm(desired),1e-12))
    target_R=C[:3,:3]@T[:3,:3]@np.diag([-1.,1.,-1.])
    rotate=cv2.Rodrigues(C[:3,:3].T@target_R)[0].ravel()
    rotate*=min(1.,math.radians(.25)/max(np.linalg.norm(rotate),1e-12))
    J=np.zeros((6,6));eps=1e-5
    for i in range(5):
        qq=q.copy();qq[i]+=eps;N=geometry.transform(qq)@X_b
        J[:3,i]=100*((N@np.r_[TIP_B,1])[:3]-tip)/eps
        J[3:,i]=rotation_vector(C[:3,:3].T@N[:3,:3])/eps
    if contact or gap<.06:
        J[3:,:]=0.;rotate[:]=0.
    target=np.r_[100*desired,rotate]
    dq=np.linalg.solve(J.T@J+.02**2*np.eye(6),J.T@target)
    dq*=min(1.,math.radians(joint_deg)/max(np.max(abs(dq)),1e-12))
    candidate=np.degrees(q+dq)
    for i,(lo,hi) in limits.items():candidate[i]=np.clip(candidate[i],lo,hi)
    predicted=J@np.radians(candidate-np.array(q_deg))
    if np.linalg.norm(candidate-np.array(q_deg))<.005 or desired@predicted[:3]<=0:
        raise ValueError('No bounded progress toward the stereo target')
    return candidate.tolist(),{'gap_m':gap,'other_finger_gap_m':other,'lateral_error_m':lateral,'error_norm_m':float(np.linalg.norm(delta))}


class Observer:
    def __init__(self,folder):self.folder=Path(folder)
    def latest(self):
        try:r=json.loads((self.folder/'latest.json').read_text())
        except (OSError,ValueError):raise ValueError('Waiting for stereo camera')
        now=time.monotonic()
        if not r.get('valid'):raise ValueError(r.get('reason','Stereo view unavailable'))
        if now-r['received']>.3 or not 0<=now-r['frame_time']<.4:raise ValueError('Stereo camera data is stale')
        return r
    def stable(self,runner,timeout=8.,frames=7):
        rows=[];last=-1;started=time.monotonic();reason='No stereo frames'
        while time.monotonic()-started<timeout:
            runner.tick()
            try:r=self.latest()
            except ValueError as error:rows=[];reason=str(error);continue
            if r['frame_time']<started or r['sequence']==last:continue
            last=r['sequence'];rows.append(r);rows=rows[-frames:]
            if len(rows)==frames:
                try:return mean_pose(rows),rows
                except ValueError as error:reason=str(error)
        raise ValueError('Stable stereo view unavailable: '+reason)


def run_phase(runner,w,folder,workers,touch=False,speed_scale=1.):
    """Approach to 30 mm with stereo feedback, then retrace to the entry pose."""
    import subprocess,threading
    folder=Path(folder);observation_folder=Path('outputs/stereo_contact')/folder.name
    observation_folder.mkdir(parents=True,exist_ok=True)
    record={'contact_attempted':False,'target_standoff_m':0. if touch else .03,'contact_confirmed':False,'steps':[],'reached_standoff':False,'returned_to_entry':False}
    entry=list(runner.targets.values());visited=[entry];child=None;old_vision=runner.vision_ready
    old_low,old_high=runner.low.copy(),runner.high.copy()
    old_speed,old_acceleration=runner.speed,runner.acceleration
    old_tracking,old_pacing=runner.tracking_limit,runner.pacing_lag
    runner.tracking_limit=2. if speed_scale==1 else 3.5;runner.pacing_lag=1.1*speed_scale
    contact_phase=False;load_baseline=None;load_changes=0
    runner.speed=6.*speed_scale;runner.acceleration=24.*speed_scale**2
    g=w.geometry
    limits=dict(runner.limits)
    for i in (1,2):
        joint=g.joints[i];lo,hi=sorted((float(joint.find('limit').get(k))-g.offsets[i])/g.signs[i] for k in ('lower','upper'))
        limits[i]=(max(runner.start[i]-175.,math.degrees(lo)+2.),min(runner.start[i],math.degrees(hi)-2.))
    runner.low=np.array([limits[i][0] for i in range(6)]);runner.high=np.array([limits[i][1] for i in range(6)])
    # The camera mount is only an initial local Jacobian model. Every small
    # movement is checked again against stereo; it is not globally activated.
    X_a=np.array(json.loads(Path('calibration/arm2_handeye_fixed_focus_refined_candidate.json').read_text())['T_wrist_camera'])
    rig=json.loads(Path('outputs/contact_camera/20261004T194430628210Z/report.json').read_text())
    X_b=X_a@np.array(rig['extrinsics']['B_A'])
    observer=Observer(observation_folder)
    def stereo_ready():
        try:
            T=np.array(observer.latest()['T_camera_b_tag']);_,gap,other,_=geometry_error(T)
            return (gap>-.001 and other>.002) if contact_phase else min(gap,other)>.015
        except ValueError:return False
    try:
        w.stop.set();deadline=time.monotonic()+15
        while any(worker.is_alive() for worker in workers):
            runner.tick()
            if time.monotonic()>deadline:raise ValueError('RGB workers did not stop')
        log=(folder/'stereo_servo_observer.log').open('w')
        child=subprocess.Popen(['.venv-depth-v2/bin/python','stereo_contact_observer.py','--output',str(observation_folder)],stdout=log,stderr=subprocess.STDOUT)
        runner.vision_ready=stereo_ready
        deadline=time.monotonic()+20
        while time.monotonic()<deadline and not stereo_ready():runner.tick()
        if not stereo_ready():raise ValueError('Stereo could not acquire the tag')
        best=1e9;began=time.monotonic()
        for index in range(300):
            T,rows=observer.stable(runner,frames=3 if speed_scale==2 and not contact_phase else 7);q=runner.rows[-1]['raw_deg']
            target_gap=0. if contact_phase else .03
            delta,gap,other,lateral=geometry_error(T,target_gap);error=float(np.linalg.norm(delta))
            motors=[m.get_state().torq for m in runner.arm.motors]
            record['steps'].append({'index':index,'raw_deg':q,'T_camera_b_tag':T.tolist(),'contact_phase':contact_phase,
                'gap_m':gap,'other_finger_gap_m':other,'lateral_error_m':lateral,'error_norm_m':error,'motor_torques_nm':motors,
                'stereo_position_std_m':np.std([np.array(r['T_camera_b_tag'])[:3,3] for r in rows],axis=0).tolist()})
            if gap<record.get('closest_gap_m',1e9):
                record['closest_gap_m']=gap
                import shutil
                for role in ('B','C'):shutil.copyfile(observation_folder/f'{role}.jpg',folder/f'closest_{role}.jpg')
            if index%5==0:print(f'Stereo step {index}: fingertip gap {gap*1000:.1f} mm, lateral error {lateral*1000:.1f} mm',flush=True)
            if (not contact_phase and min(gap,other)<.015) or (contact_phase and (gap<-.001 or other<.002)):
                raise ValueError('Fingertip clearance guard reached')
            if contact_phase:
                if gap<.015 and len(record['steps'])>=5:
                    recent=record['steps'][-5:]
                    if all(row['contact_phase'] for row in recent) and recent[0]['gap_m']-gap<.0004:
                        record['visual_advance_stalled']=True
                        raise ValueError('Visual advance stalled near the paper; possible contact, retracting')
                if gap<.010 and load_baseline is None:load_baseline=np.array(motors)
                if load_baseline is not None:
                    changed=np.any(abs(np.array(motors)-load_baseline)>np.array([.6,.6,.6,.25,.15,.15]))
                    load_changes=load_changes+1 if changed else 0
                    if load_changes>=3:
                        record['contact_load_change']=True
                        raise ValueError('Motor load changed during final approach; retracting')
                if gap<=.0008 and lateral<.002:
                    record['reached_estimated_paper_plane']=True
                    record['contact_confirmed']=False
                    print('Reached estimated paper plane (within 0.8 mm); retracting without pushing.',flush=True)
                    for role in ('B','C'):
                        import shutil
                        shutil.copyfile(observation_folder/f'{role}.jpg',folder/f'touch_{role}.jpg')
                    break
            if contact_phase and load_changes:continue
            normal_angle=math.degrees(math.acos(np.clip(-T[2,2],-1,1)))
            if not contact_phase and error<.003:
                record['reached_standoff']=True
                if not touch:
                    print('Reached 30 mm stereo standoff; returning.',flush=True);break
                contact_phase=True;record['contact_attempted']=True;best=1e9
                print('Aligned at 30 mm. Beginning bounded final approach to the paper.',flush=True)
                continue
            if error>best+.015:raise ValueError('Camera feedback shows divergence from the target')
            best=min(best,error)
            if time.monotonic()-began>600:raise ValueError('Stereo approach time limit reached')
            travel=.002 if index<3 or gap<.060 else (.005 if gap<.10 else .010)
            if index>=3:travel*=speed_scale
            if contact_phase:
                travel=.0005 if gap<.01 else .002
                runner.speed=1. if gap<.01 else 3.;runner.acceleration=6.;runner.tracking_limit=2.;runner.pacing_lag=1.1
            goal,metrics=step(g,q,T,X_b,limits,standoff=target_gap,travel_m=travel,joint_deg=.5 if index<3 or contact_phase else 2.*speed_scale,contact=contact_phase)
            visited.append(goal);runner.go(goal)
        else:raise ValueError('Stereo approach step limit reached')
    except ValueError as error:
        record['stop_reason']=str(error);print('Stereo return: '+str(error),flush=True)
    finally:
        # A healthy camera/geometry stop retraces recorded joint waypoints.
        # Motor/transport faults propagate through the existing torque-off cleanup.
        import sys
        healthy=sys.exc_info()[0] is None
        try:
            if healthy:
                from camera_visual_approach import fast_indices
                runner.speed=6.*speed_scale;runner.acceleration=24.*speed_scale**2
                for i in list(reversed(fast_indices(visited))) [1:]:runner.go(visited[i],returning=True)
                runner.go(entry,returning=True);record['returned_to_entry']=True
        finally:
            if child is not None:
                (observation_folder/'stop').touch()
                if not healthy or not record['returned_to_entry']:
                    if child.poll() is None:child.terminate()
                else:
                    deadline=time.monotonic()+10
                    try:
                        while child.poll() is None and time.monotonic()<deadline:runner.tick()
                    finally:
                        if child.poll() is None:child.terminate()
                log.close()
            runner.speed,runner.acceleration=old_speed,old_acceleration
            runner.tracking_limit,runner.pacing_lag=old_tracking,old_pacing
        runner.vision_ready=old_vision;runner.low=old_low;runner.high=old_high
        (folder/'stereo_servo_report.json').write_text(json.dumps(record,indent=2,allow_nan=False)+'\n')
        if healthy:
            w.stop.clear();workers=[threading.Thread(target=w.camera_worker,args=(r,),daemon=True) for r in ('wrist','tripod')]
            for worker in workers:worker.start()
            deadline=time.monotonic()+20
            while time.monotonic()<deadline and not runner.vision_ready():runner.tick()
    return record,workers
