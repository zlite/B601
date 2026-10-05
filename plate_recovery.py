"""One bounded, powered recovery for explicitly recoverable plate errors."""
import json,time
import numpy as np
from plate_contact_guard import check_descent_load
from rotation_math import rotation_vector


class PlateRecoveryNeeded(RuntimeError):
    """A task-level stop; not a motor, feedback, or watchdog fault."""


def check_jaw_recovery_placement(current,placed,reference,released):
    from plate_contact_guard import check_plate_release
    current=np.asarray(current,float);placed=np.asarray(placed,float)
    if current.shape!=(6,) or placed.shape!=(6,) or not np.isfinite(np.r_[current,placed]).all():
        raise RuntimeError('Jaw recovery joint pose invalid')
    if np.any(abs(current-placed)>np.array([.5]*5+[.8])):
        raise RuntimeError('Jaw recovery requires the recorded placement pose')
    return check_plate_release(reference,released)


def at_open_recovery_anchor(current,anchor,gripper_q):
    """A known clear waypoint can use its recorded return without grid PnP."""
    current=np.asarray(current,float);anchor=np.asarray(anchor,float)
    if current.shape!=(6,) or anchor.shape!=(6,) or not np.isfinite(np.r_[current,anchor,gripper_q]).all():return False
    return bool(-1.22<gripper_q<-1.10 and np.all(abs(current-anchor)<=np.array([.3]*5+[.8])))


def open_jaws_at_placement(runner,observer,context):
    """Complete interrupted opening only at a visually verified placement.

    This stays in the motor-owning process. It never clears a motor fault or
    opens above the holder. All other conditions remain a powered hold.
    """
    from plate_surface import plate_in_surface_frame
    grip=runner.gripper;opening=float(context['opening_rad']);runner.tick()
    q=grip.get_register_f32(80,100);runner.tick()
    if -1.22<q<-1.10:return {'already_open':True}
    if not -1.22<opening<-1.10 or not np.isfinite(q) or not opening-.04<=q<=opening+1.25:
        raise RuntimeError('Jaw recovery position outside measured travel')
    def placement():
        if not runner.vision_ready():raise RuntimeError('Jaw recovery cameras not fresh')
        observation=observer.get()
        return check_jaw_recovery_placement(runner.rows[-1]['raw_deg'],context['placed'],
            context['reference_surface'],plate_in_surface_frame(observation))
    placement()
    def feedback(expected):
        grip.request_feedback();runner.tick();runner.arm.ctrl.poll_feedback_once();s=grip.get_state()
        if s is None or s.status_code!=expected:
            raise RuntimeError('Jaw recovery motor status invalid; no fault reset attempted')
        state={'velocity':s.vel,'torque':s.torq,'temperature_mos_c':s.t_mos,'temperature_rotor_c':s.t_rotor}
        if not np.isfinite(list(state.values())).all() or abs(s.vel)>.35 or abs(s.torq)>.5 or max(s.t_mos,s.t_rotor)>50:
            raise RuntimeError('Jaw recovery feedback guard: '+json.dumps(state))
        return state
    feedback(0)
    mode=grip.get_register_u32(10,200);runner.tick();timeout=grip.get_register_u32(9,200);runner.tick()
    enabled=False;rows=[]
    try:
        grip.ensure_mode(4,1000);runner.tick()
        if grip.get_register_u32(10,200)!=4:raise RuntimeError('Jaw recovery mode readback failed')
        runner.tick();grip.write_register_u32(9,10000);runner.tick()
        if grip.get_register_u32(9,200)!=10000:raise RuntimeError('Jaw recovery watchdog readback failed')
        runner.tick();grip.send_force_pos(q,.05,.02);grip.enable();enabled=True
        began=time.monotonic();next_check=began
        while time.monotonic()-began<35:
            target=max(opening,q-.05*(time.monotonic()-began))
            grip.send_force_pos(target,.05,.02);runner.tick();state=feedback(1)
            actual=grip.get_register_f32(80,100);runner.tick()
            if not opening-.04<=actual<=q+.04:raise RuntimeError('Jaw recovery moved outside opening corridor')
            rows.append(dict(state,q_rad=actual,target_rad=target))
            if time.monotonic()>=next_check:placement();next_check=time.monotonic()+.2
            if abs(actual-opening)<.015:
                placement();return {'opened_at_verified_placement':True,'samples':rows}
        raise RuntimeError('Jaw recovery opening timed out')
    finally:
        if enabled:grip.disable()
        runner.tick();grip.ensure_mode(mode,1000);runner.tick();grip.write_register_u32(9,timeout);runner.tick()
        if grip.get_register_u32(10,200)!=mode or grip.get_register_u32(9,200)!=timeout:
            raise RuntimeError('Jaw recovery settings restore failed')


def check_withdrawal_load(baseline,current,clear,plate_stationary=False):
    if not clear and not plate_stationary:return check_descent_load(baseline,current)
    baseline=np.asarray(baseline,float);current=np.asarray(current,float)
    if baseline.shape!=(6,) or current.shape!=(6,) or not np.isfinite(np.r_[baseline,current]).all():
        raise RuntimeError('Invalid recovery load feedback')
    delta=current-baseline
    # Only in the >=15 mm observed clearance region: the recorded upward
    # reversal added 4.17 Nm at the shoulder without contact. Keep other-axis
    # bounds and bound the upward gravity/friction change at 6 Nm.
    # Near the holder, the shoulder allowance additionally requires an
    # independently verified stationary plate and an absolute 13.5 Nm cap.
    # Successful cycles reached 12.53 Nm; failed reversals stayed below 12.28.
    # The extra elbow allowance remains exclusive to the clear region.
    lower=np.array([-1.5,-4.,-6. if clear else -4.,-1.5,-.8,-.5])
    upper=np.array([1.5,6.,4.,1.5,.8,.5])
    if not clear and abs(current[1])>13.5:raise RuntimeError('Absolute shoulder withdrawal load exceeded')
    if np.any(delta<lower) or np.any(delta>upper):raise RuntimeError('Unexpected withdrawal load')


def withdraw_open_gripper(sample, move_up, save):
    """Verify open jaws, visible separation and clearance before any retrace."""
    start=sample();origin=np.asarray(start['center_m'],float)
    baseline=np.asarray(start['load_nm'],float);rows=[];commanded=0.
    previous=float(start['min_height_m']);separated=False
    for index in range(21):
        state=sample();center=np.asarray(state['center_m'],float)
        if center.shape!=(3,) or not np.isfinite(center).all():
            raise RuntimeError('Recovery position invalid')
        if not state['jaws_open'] or not state['vision_fresh']:
            raise RuntimeError('Recovery requires open jaws and both fresh cameras')
        height=float(state['min_height_m'])
        if not np.isfinite(height) or height<-.002 or height> .09:
            raise RuntimeError('Recovery height outside local corridor')
        if np.linalg.norm(center[:2]-origin[:2])>.004 or abs(center[1])>.006:
            raise RuntimeError('Recovery left vertical corridor')
        if height<previous-.0005:
            raise RuntimeError('Recovery moved downward unexpectedly')
        previous=height
        if not separated:check_withdrawal_load(baseline,state['load_nm'],start['min_height_m']>=.015,state.get('plate_stationary',False))
        rise=height-float(start['min_height_m'])
        if commanded>=.005:
            if rise<.002:raise RuntimeError('Open gripper separation not observed; possible retained plate')
            separated=True
        rows.append(dict(state,index=index,commanded_rise_m=commanded,observed_rise_m=rise))
        save({'phase':'vertical_withdrawal','samples':rows,'separation_verified':separated})
        if separated and height>=.030:
            return {'phase':'clear','samples':rows,'separation_verified':True}
        if index==20:raise RuntimeError('Recovery travel budget exhausted')
        move_up(.0025);commanded+=.0025


def recover_to_standoff(runner,w,observer,folder,anchor,capture,reason):
    from plate_hover import X,MID,TIPS
    original_tick=runner.tick;original_speed=runner.speed;original_vision=runner.vision_ready
    original_acceleration=runner.acceleration;original_minimum_duration=runner.minimum_duration
    data={'reason':str(reason),'automatic':True,'returned_to_standoff':False}
    reference=getattr(w,'plate_recovery_context',{}).get('seated_plate_surface')
    def placement(observation):
        if reference is None:return {}
        from plate_surface import plate_in_surface_frame
        from plate_contact_guard import check_plate_release
        return dict(check_plate_release(reference,plate_in_surface_frame(observation)),plate_stationary=True)
    def save(state):
        data.update(state);(folder/'recovery.json').write_text(json.dumps(data,indent=2)+'\n')
    def load():
        now=runner.rows[-1]['time']
        return np.median([r['motor_torques_nm'] for r in runner.rows if now-r['time']<.25],axis=0)
    def hold(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:runner.tick()
    def sample():
        runner.tick();qg=runner.gripper.get_register_f32(80,100);runner.tick()
        observation=observer.get();P=np.array(observation['T_camera_b_plate'])
        pads=(np.linalg.inv(P)@np.c_[TIPS,np.ones(2)].T).T[:,:3]
        return {'center_m':pads.mean(0).tolist(),'min_height_m':float(pads[:,2].min()),
                'load_nm':load().tolist(),'jaws_open':bool(-1.22<qg<-1.10),
                'gripper_q_rad':qg,'vision_fresh':bool(runner.vision_ready()),
                'time':runner.rows[-1]['time'],'raw_deg':runner.rows[-1]['raw_deg'],**placement(observation)}
    def move_up(distance):
        P=np.array(observer.get()['T_camera_b_plate']);q=np.radians(runner.rows[-1]['raw_deg'])
        C=w.geometry.transform(q)@X;point=(C@np.r_[MID,1])[:3]
        desired=C[:3,:3]@P[:3,:3]@np.array([0.,0.,distance]);J=np.zeros((6,6));eps=1e-5
        for i in range(6):
            a=q.copy();a[i]+=eps;N=w.geometry.transform(a)@X
            J[:3,i]=100*((N@np.r_[MID,1])[:3]-point)/eps
            J[3:,i]=3*rotation_vector(C[:3,:3].T@N[:3,:3])/eps
        dq=np.linalg.solve(J.T@J+.03**2*np.eye(6),J.T@np.r_[100*desired,np.zeros(3)])
        if max(abs(np.degrees(dq)))>1.5:raise RuntimeError('Recovery step exceeds local joint bound')
        runner.go(np.degrees(q+dq).tolist());hold(1.2)
    try:
        runner.speed=min(original_speed,5.);runner.acceleration=min(original_acceleration,36.)
        runner.minimum_duration=max(original_minimum_duration,.25);hold(.5)
        # Same owner, same powered motor loop. Never re-enable or reset faults.
        initial=sample()
        if not initial['jaws_open'] or not initial['vision_fresh']:
            raise RuntimeError('Recovery preconditions unavailable')
        baseline=load()
        def guarded_tick(*args,**kwargs):
            q=original_tick(*args,**kwargs)
            if not data.get('separation_verified'):
                stationary=placement(observer.get()).get('plate_stationary',False) if reference is not None else False
                current=load()
                try:check_withdrawal_load(baseline,current,initial['min_height_m']>=.015,stationary)
                except (ValueError,RuntimeError):
                    save({'rejected_load':{'baseline_nm':baseline.tolist(),'current_nm':current.tolist(),'plate_stationary':stationary}})
                    raise
            return q
        runner.tick=guarded_tick
        def tested_move(distance):
            move_up(distance)
            if getattr(w,'recovery_inject_failure',False):
                w.recovery_inject_failure=False
                raise RuntimeError('Injected failure during upward recovery')
        save(withdraw_open_gripper(sample,tested_move,save))
        # Withdrawal already established separation and >=30 mm clearance.
        # The remaining segment rejoins a recorded clear joint waypoint. Do
        # not make that checked segment depend on tracking wells that can
        # disappear from view; continue requiring fresh camera streams.
        def camera_streams_ready():
            with w.lock:
                return all('error' not in w.sources.get(role,{})
                    and time.monotonic()-w.sources.get(role,{}).get('time',0)<.5
                    and w.sources.get(role,{}).get('frame_age_s',99)<.5
                    for role in ('wrist','tripod'))
        runner.vision_ready=camera_streams_ready
        if getattr(w,'recovery_inject_grid_loss',False):
            w.recovery_inject_grid_loss=False
            observer.stop.set();deadline=time.monotonic()+1
            while observer.thread.is_alive() and time.monotonic()<deadline:runner.tick()
            if observer.thread.is_alive():raise RuntimeError('Recovery test observer failed to stop')
            observer.latest={'valid':False,'reason':'Injected grid loss after verified clearance'}
            data['injected_grid_loss_after_clearance']=True;save({})
        capture('recovery_clear')
        # The anchor was recorded before descending into the contact region.
        # Only join that route after observed upward separation and 30 mm gap.
        runner.go(anchor);hold(.8)
        runner.tick();qg=runner.gripper.get_register_f32(80,100);runner.tick()
        if not camera_streams_ready() or not at_open_recovery_anchor(runner.rows[-1]['raw_deg'],anchor,qg):
            raise RuntimeError('Recorded recovery standoff not verified')
        data['standoff_verification']='recorded joint waypoint after observed separation'
        data['returned_to_standoff']=True;data['phase']='standoff_verified';save({})
        capture('recovery_standoff')
        return data
    except Exception as error:
        data['stop_reason']=str(error);data['phase']='hold_required';save({})
        raise
    finally:
        runner.tick=original_tick;runner.speed=original_speed;runner.vision_ready=original_vision
        runner.acceleration=original_acceleration;runner.minimum_duration=original_minimum_duration
