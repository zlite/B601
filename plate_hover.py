"""Camera-observed, noncontact hover above the plate; retrace to rest."""
import json,time,threading,subprocess
from pathlib import Path
from datetime import datetime,timezone
import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation,Slerp
from arm_geometry import Geometry
from axis_follow import AxisArm
from pairing_dashboard import Workbench
from camera_check import CameraCheck
from camera_visual_approach import route,RouteRunner
from tag_view import detect,annotate_view
from plate_motion_owner import PlateArm

X=np.array(json.loads(Path('calibration/stereo_sweep_handeye_20261004.json').read_text())['T_wrist_camera'])
TIPS=np.array(json.loads(Path('outputs/gripper/20261004T204827125655Z/jaw_geometry.json').read_text())[-1]['points_B_m'])
MID=TIPS.mean(0)
R_TARGET=np.array([[0,2**-.5,2**-.5],[1,0,0],[0,2**-.5,-2**-.5]])

def plan(g,q_deg,target,limits):
    q=np.radians(q_deg);initial=g.transform(q)@X;point=(initial@np.r_[MID,1])[:3]
    if not .49<=target[0]<=.58 or not -.03<=target[1]<=.06 or not .075<=target[2]<=.18:raise ValueError('Outside noncontact plate hover workspace')
    rotations=Slerp([0,1],Rotation.from_matrix([initial[:3,:3],R_TARGET]))
    lo=np.radians([limits[i][0] for i in range(6)]);hi=np.radians([limits[i][1] for i in range(6)])
    poses=[list(q_deg)];clearance=[]
    for u in np.linspace(0,1,max(3,int(np.ceil(np.linalg.norm(target-point)/.0075))+1))[1:]:
        p=point+(target-point)*u;R=rotations(u).as_matrix()
        def residual(a):
            C=g.transform(a)@X
            return np.r_[10*((C@np.r_[MID,1])[:3]-p),cv2.Rodrigues(R.T@C[:3,:3])[0].ravel()]
        fit=least_squares(residual,q,bounds=(lo,hi),max_nfev=100)
        if np.linalg.norm(residual(fit.x))>1e-4 or max(abs(np.degrees(fit.x-q)))>5:raise ValueError('Hover IK/path check failed')
        q=fit.x;C=g.transform(q)@X
        # Full nominal 61 mm pad extent plus 8 mm height, measured distal landmarks.
        corners=np.array([tip-d*np.array([0,2**-.5,2**-.5])+h*np.array([0,-2**-.5,2**-.5]) for tip in TIPS for d in (0,.061) for h in (-.004,.004)])
        world=corners@C[:3,:3].T+C[:3,3];clearance.append(float(world[:,2].min()))
        if world[:,2].min()<.065 or C[2,3]<.11:raise ValueError('Hover/table clearance guard')
        poses.append(np.degrees(q).tolist())
    return poses,min(clearance)

def approach_segments(g,look,limits,approach='direct'):
    target=np.array([.547,.017,.135])
    if approach=='direct':
        poses,clearance=plan(g,look,target,limits)
        return [poses],clearance
    if approach not in ('left','right'):raise ValueError('Unsupported approach trajectory')
    # Vary the clear overhead entry while keeping the same final visual
    # acquisition pose and aligned descent. Every segment uses the existing
    # IK, joint-step and full-pad table-clearance checks.
    via=np.array([.530,.017+(-.020 if approach=='left' else .020),.155])
    first,c1=plan(g,look,via,limits)
    second,c2=plan(g,first[-1],target,limits)
    return [first,second],min(c1,c2)

def approach_plan(g,look,limits,approach='direct'):
    segments,clearance=approach_segments(g,look,limits,approach)
    return segments[0]+[q for segment in segments[1:] for q in segment[1:]],clearance


def execute(stereo=False,lower=False,pickup=False,recovery_test=False,recovery_height_mm=25,recovery_inject_failure=False,alignment_review=False,speed_scale=2.,pause_for_review=False,recovery_inject_grid_loss=False,approach='direct',travel_speed_scale=2.,table_survey=False,grid_reference=None,table_carry=False):
    if speed_scale not in (1.,2.,4.):raise ValueError('Unsupported plate motion speed scale')
    from plate_training import require_current_workspace
    require_current_workspace()
    holder_evidence=[]
    if table_survey or table_carry:
        from plate_training import require_ten_holder_cycles
        holder_evidence=require_ten_holder_cycles()
        if table_survey and pickup:raise ValueError('Adjacent survey requires empty jaws')
        if table_carry and (not pickup or table_survey):raise ValueError('Table carry requires pickup without survey')
    from plate_speed import motion_profile
    profile=motion_profile(speed_scale,travel_speed_scale)
    transit_speed=profile['transit_deg_s']
    transit_acceleration=profile['transit_acceleration_deg_s2']
    folder=Path('outputs/plate_hover')/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');folder.mkdir(parents=True)
    w=Workbench();w.survey_mode=True;w.camera_sizes={'tripod':(1280,800)};w.camera_fps={'tripod':5};w.camera_check=CameraCheck(w.geometry,folder/'views')
    w.recovery_test=recovery_test
    w.table_survey=table_survey
    w.table_carry=table_carry
    w.grid_reference=grid_reference
    w.holder_evidence=holder_evidence
    w.alignment_review=alignment_review
    w.plate_speed_scale=speed_scale
    w.plate_motion_profile=profile
    w.pause_for_plate_review=pause_for_review
    if recovery_height_mm not in (8,18,25):raise ValueError('Unsupported recovery rehearsal height')
    w.recovery_height_mm=recovery_height_mm
    w.recovery_inject_failure=recovery_inject_failure
    if recovery_inject_grid_loss and not recovery_test:raise ValueError('Grid-loss injection requires a noncontact recovery test')
    w.recovery_inject_grid_loss=recovery_inject_grid_loss
    workers=[threading.Thread(target=w.camera_worker,args=(r,),daemon=True) for r in ('wrist','tripod')]
    for worker in workers:worker.start()
    runner=None;record={'contact_attempted':False,'gripper_commanded':False,'returned_to_rest':False,'motors_disabled_verified':False,'views':[],'approach':approach}
    recording_stop=threading.Event();recording_folder=folder/'recording';recording_folder.mkdir()
    def record_views():
        last={};count=0
        while not recording_stop.wait(.5) and count<600:
            for role in ('wrist','tripod'):
                with w.lock:data=dict(getattr(w,'survey_frames',{}).get(role,{}))
                received=data.get('received',0)
                if 'image' in data and received>last.get(role,0):
                    last[role]=received
                    cv2.imwrite(str(recording_folder/f'{received:.6f}_{role}.jpg'),data['image'],[cv2.IMWRITE_JPEG_QUALITY,90])
            count+=1
    recorder=threading.Thread(target=record_views,daemon=True);recorder.start()
    try:
        with PlateArm(folder,w) as arm:
            start=arm.read();fold=list(start);fold[4]=7.3;fold[5]=float(np.degrees(w.geometry.profile['reference_raw_rad'][5]))
            if max(abs(fold[i]-start[i]) for i in (4,5))>8:raise ValueError('Folded wrist adjustment outside checked recovery range')
            lift,limits=route(fold,.30,0);look=lift[11].copy();look[3]+=30
            segments,clearance=approach_segments(w.geometry,look,limits,approach)
            poses=segments[0]+[q for segment in segments[1:] for q in segment[1:]]
            # Preflight the exact forward/return curves before enabling torque.
            # Settle at the overhead turn rather than smoothing across its corner.
            from blended_motion import BlendedPath
            for segment in segments:
                BlendedPath(segment,transit_speed,transit_acceleration)
                BlendedPath(list(reversed(segment)),transit_speed,transit_acceleration)
            record['planned_min_pad_base_z_m']=clearance;record['planned_hover_deg']=poses
            grip=arm.add_motor(7,23,'4310');grip_q=grip.get_register_f32(80,300);record['gripper_raw_rad']=grip_q
            def ready():
                with w.lock:return all('error' not in w.sources.get(r,{}) and time.monotonic()-w.sources.get(r,{}).get('time',0)<.5 and w.sources.get(r,{}).get('frame_age_s',99)<.5 for r in ('wrist','tripod'))
            deadline=time.monotonic()+25
            while time.monotonic()<deadline and not ready():arm.read()
            if not ready():raise RuntimeError('Hover cameras unavailable')
            from plate_gripper import restore_open_at_rest
            record['jaw_reset']=restore_open_at_rest(arm,grip,w.geometry,ready)
            grip_q=record['jaw_reset']['q_rad']
            if not -1.22<grip_q<-1.1:raise ValueError('Gripper not at the measured open configuration')
            planned=[start]+lift[:12]+poses
            arm.prepare_group(range(6))
            arm.command_speed_limits={i:min(60.,(27. if i<3 else 36.)*speed_scale) for i in range(5)}
            arm.enable_group(dict(enumerate(start)))
            runner=RouteRunner(w,arm,start,limits,folder,planned);runner.vision_ready=ready
            runner.gripper=grip
            runner.settle_tolerance=np.array([.5]*5+[.8])
            runner.speed=transit_speed;runner.acceleration=transit_acceleration;runner.minimum_duration=.25/speed_scale
            runner.tracking_limit=3.;runner.pacing_lag=profile['clear_pacing_lag_deg']
            record['speed_scale']=speed_scale
            record['travel_speed_scale']=travel_speed_scale
            record['motion_profile']=profile
            def pregrasp_stereo():
                w.stop.set();deadline=time.monotonic()+15
                while any(worker.is_alive() for worker in workers):
                    runner.tick()
                    if time.monotonic()>deadline:raise ValueError('Camera release timeout')
                with (folder/'pregrasp_stereo.log').open('w') as log:
                    child=subprocess.Popen(['.venv-depth-v2/bin/python','contact_camera_probe_v2.py','--all-tags'],stdout=log,stderr=subprocess.STDOUT)
                    try:
                        deadline=time.monotonic()+50
                        while child.poll() is None:
                            runner.tick()
                            if time.monotonic()>deadline:raise ValueError('Pregrasp stereo timeout')
                        record['pregrasp_stereo_exit_code']=child.returncode
                    finally:
                        if child.poll() is None:child.terminate()
            w.pregrasp_stereo=pregrasp_stereo
            until=time.monotonic()+1
            while time.monotonic()<until:runner.tick(take_up=True)
            visited=[start];hover_complete=False
            def capture(label):
                until=time.monotonic()+.8
                while time.monotonic()<until:runner.tick()
                row={'label':label,'raw_deg':runner.rows[-1]['raw_deg'],'cameras':{}}
                for role in ('wrist','tripod'):
                    with w.lock:data=dict(w.survey_frames[role])
                    im=data.pop('image');tags=detect(im,'auto',None);name=f'{label}_{role}.png';cv2.imwrite(str(folder/name),im)
                    if role=='wrist':cv2.imwrite(str(folder/f'{label}_view.jpg'),annotate_view(im,tags))
                    row['cameras'][role]={**data,'image':name,'tags':tags}
                record['views'].append(row);(folder/'progress.json').write_text(json.dumps(record,indent=2)+'\n')
                print('Hover view',label,folder,flush=True)
            def return_home():
                # Retrace dense recorded commands, retaining the already checked envelope.
                if hover_complete:
                    # A nested phase may stop during its own retreat. Return to
                    # the checked hover entry before beginning that reverse blend.
                    if np.max(abs(np.array(list(runner.targets.values()))-poses[-1]))>.01:
                        raise RuntimeError('Nested plate phase did not return to hover; holding for recovery')
                    for segment in reversed(segments):
                        runner.blend(list(reversed(segment)),[],returning=True)
                    runner.go(lift[11],returning=True)
                    runner.blend(list(reversed(lift[3:12])),[],returning=True)
                    runner.go(lift[1],returning=True)
                else:
                    for goal in reversed(visited[:-1]):runner.go(goal,returning=True)
                runner.go(fold,returning=True);until=time.monotonic()+2
                while time.monotonic()<until:q=runner.tick()
                record['return_error_deg']=(np.array(q)-fold).tolist()
                if max(abs(np.array(q[:5])-fold[:5]))>.2 or abs(q[5]-fold[5])>.5:raise RuntimeError('Hover folded return not verified')
                record['returned_to_rest']=True
            standard_tick=runner.tick
            def recover_from_hold():
                from plate_recovery import recover_to_standoff,open_jaws_at_placement,at_open_recovery_anchor
                context=getattr(w,'plate_recovery_context',None)
                if not context:raise RuntimeError('No checked plate recovery context')
                if not ready():raise RuntimeError('Recovery cameras not fresh')
                observer=context['observer']
                runner.tick=standard_tick
                current=arm.read()
                if np.any(np.array(current)<context['low']) or np.any(np.array(current)>context['high']):
                    raise RuntimeError('Stopped arm outside recovery envelope')
                runner.targets=dict(enumerate(current));runner.last=time.monotonic()
                runner.low=context['low'];runner.high=context['high'];runner.speed=5.*speed_scale;runner.pacing_lag=profile['approach_pacing_lag_deg']
                def observed_ready():
                    try:observer.get();return ready()
                    except ValueError:return False
                try:
                    runner.vision_ready=ready;runner.tick()
                    gripper_q=runner.gripper.get_register_f32(80,100);runner.tick()
                    if at_open_recovery_anchor(runner.rows[-1]['raw_deg'],context['anchor'],gripper_q):
                        # The arm already reached the recorded clear waypoint.
                        # Grid reacquisition is unnecessary for retracing this
                        # checked prefix; both camera streams remain required.
                        record['recovery_from_verified_anchor']=True
                    else:
                        observer.resume();deadline=time.monotonic()+5
                        while not observed_ready() and time.monotonic()<deadline:runner.tick()
                        if not observed_ready():raise RuntimeError('Plate tracking not reacquired')
                        runner.vision_ready=observed_ready
                        if context.get('grip_release'):
                            record['recovery_jaw_opening']=open_jaws_at_placement(runner,observer,context['grip_release'])
                        record['resumed_recovery']=recover_to_standoff(runner,w,observer,folder,context['anchor'],capture,'Powered hold recovery request')
                    runner.vision_ready=ready
                    runner.speed=profile['clear_approach_deg_s'];runner.acceleration=profile['clear_approach_acceleration_deg_s2'];runner.pacing_lag=profile['clear_pacing_lag_deg']
                    record['recovery_clear_return']=runner.retrace(list(reversed(context['visited'])))
                    runner.low=context['route_low'];runner.high=context['route_high'];runner.speed=transit_speed;runner.acceleration=transit_acceleration
                    return_home()
                    w.plate_recovery_context=None
                    record['recovered_from_powered_hold']=True
                    return True
                finally:
                    observer.stop.set();observer.thread.join(timeout=1)
                    runner.tick=standard_tick;runner.vision_ready=ready
            arm.recovery_callback=recover_from_hold
            arm.recovery_target=np.array(fold)
            try:
                for goal in [fold,lift[1],lift[3],lift[11],look]:visited.append(goal);runner.go(goal)
                capture('entry')
                for segment in segments:runner.blend(segment,visited)
                hover_complete=True;capture('hover')
                if lower:
                    from plate_lower import run
                    record['lower']=run(runner,w,folder,capture,pickup)
                    record['contact_attempted']=record['lower']['contact_attempted']
                    record['gripper_commanded']=record['lower'].get('pickup',{}).get('gripper_commanded',False)
                if stereo:
                    w.stop.set();deadline=time.monotonic()+15
                    while any(worker.is_alive() for worker in workers):
                        runner.tick()
                        if time.monotonic()>deadline:raise ValueError('Camera release timeout')
                    with (folder/'stereo.log').open('w') as log:
                        child=subprocess.Popen(['.venv-depth-v2/bin/python','contact_camera_probe_v2.py','--all-tags'],stdout=log,stderr=subprocess.STDOUT)
                        try:
                            deadline=time.monotonic()+50
                            while child.poll() is None:
                                runner.tick()
                                if time.monotonic()>deadline:raise ValueError('Hover stereo timeout')
                            record['stereo_exit_code']=child.returncode
                        finally:
                            if child.poll() is None:child.terminate()
                # Allow offline inspection while the existing guarded motor loop holds.
                print('Holding noncontact hover. Write return to return.txt to retrace.',flush=True)
                until=time.monotonic()+(0 if stereo or lower else 120)
                while time.monotonic()<until and not (folder/'return.txt').exists():
                    if not ready():raise ValueError('Hover cameras stale')
                    runner.tick()
            except ValueError as error:record['stop_reason']=str(error)
            return_home()
        with AxisArm() as arm:record['final_raw_deg']=arm.read();record['motors_disabled_verified']=True
    finally:
        recording_stop.set();recorder.join(timeout=2)
        w.stop.set()
        # Let camera startup/teardown finish before interpreter shutdown;
        # abandoning a DepthAI worker during early preflight failure can crash.
        for worker in workers:worker.join(timeout=30)
        if runner:record['motion_samples']=runner.rows
        (folder/'report.json').write_text(json.dumps(record,indent=2)+'\n');print('Plate hover:',folder,flush=True)
