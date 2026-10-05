"""One force-limited plate grasp, 20 mm lift, placement and release."""
import json,math,time
import cv2
import numpy as np
from rotation_math import rotation_vector
from plate_contact_guard import check_descent_load,check_touchdown_load,check_plate_release,check_release_separation
from plate_grasp_geometry import grasp_rotation as measured_grasp_rotation,TABLE_NORMAL_IN_GRID,CONFIRMED_CONTACT_Q
from plate_touchdown import touch_and_backoff
from plate_recovery import PlateRecoveryNeeded
from plate_surface import plate_in_surface_frame

def grasp_and_test(runner,w,observer,folder,capture):
    from plate_hover import X,MID,TIPS
    grip=runner.gripper;original_tick=runner.tick;enabled=False;lifted=False;placed=None;release_reference=None;withdrawn=False;pre=list(runner.targets.values())
    touchdown_started=False;touchdown_complete=False
    # The 4x profile tripped a guard during opening. Keep the gripper at its
    # demonstrated 2x profile while faster arm transit is evaluated.
    speed_scale=min(2.,getattr(w,'plate_speed_scale',1.))
    gripper_velocity=.05*speed_scale
    record={'test_lift_m':.020,'force_ratio':.02,'gripper_commanded':False,'lift_following_verified':False,'released_verified':False,'jaw_open_verified':False,'gripper_rows':[]}
    original_tick();mode=grip.get_register_u32(10,200);original_tick();timeout=grip.get_register_u32(9,200);original_tick()
    opening=grip.get_register_f32(80,150);target=opening
    def tick(*args,**kwargs):
        if enabled:grip.send_force_pos(target,gripper_velocity,.02)
        q=original_tick(*args,**kwargs)
        if enabled:
            gq=grip.get_register_f32(80,100);grip.request_feedback();runner.arm.ctrl.poll_feedback_once();s=grip.get_state()
            if s is None or s.status_code!=1 or not all(math.isfinite(v) for v in (gq,s.vel,s.torq)):raise ValueError('Gripper feedback invalid')
            feedback={'time':time.monotonic(),'q':gq,'target':target,'torque':s.torq,'velocity':s.vel,
                      'temperature_mos_c':s.t_mos,'temperature_rotor_c':s.t_rotor,'status':s.status_code}
            record['gripper_rows'].append(feedback)
            if not opening-.04<=gq<=opening+1.25 or abs(s.vel)>.35 or abs(s.torq)>.5 or max(s.t_mos,s.t_rotor)>50:
                record['gripper_guard_feedback']=feedback
                raise ValueError('Gripper grasp guard: '+json.dumps(feedback))
        return q
    def hold(seconds):
        until=time.monotonic()+seconds
        while time.monotonic()<until:runner.tick()
    def shift(distance,correction=None,rotate=None):
        P=np.array(observer.get()['T_camera_b_plate']);q=np.radians(runner.rows[-1]['raw_deg']);C=w.geometry.transform(q)@X;tip=(C@np.r_[MID,1])[:3]
        desired=C[:3,:3]@P[:3,:3]@(np.array([0.,0.,distance]) if correction is None else correction);J=np.zeros((6,6));eps=1e-5
        for i in range(6):
            a=q.copy();a[i]+=eps;N=w.geometry.transform(a)@X;J[:3,i]=100*((N@np.r_[MID,1])[:3]-tip)/eps;J[3:,i]=rotation_vector(C[:3,:3].T@N[:3,:3])/eps
        J[3:]*=3
        dq=np.linalg.solve(J.T@J+.03**2*np.eye(6),J.T@np.r_[100*desired,np.zeros(3) if rotate is None else 3*rotate])
        if max(abs(np.degrees(dq)))>1.5:raise ValueError('Test lift step too large')
        runner.go(np.degrees(q+dq).tolist());hold(.5)
    try:
        from plate_pregrasp import wait_pregrasp
        wait_pregrasp(observer.get,runner.tick,TIPS,lambda sample:record.setdefault('pregrasp_samples',[]).append(sample))
        # Establish alignment above contact, then find height from resistance.
        record['alignment_grid_z_m']=.008
        record['descent_geometry']=[]
        def settled_load():
            now=runner.rows[-1]['time']
            return np.median([r['motor_torques_nm'] for r in runner.rows if now-r['time']<.25],axis=0)
        baseline_load=settled_load();record['baseline_arm_torque_nm']=baseline_load.tolist()
        # The transparent well-grid pose has a viewpoint-dependent tilt bias.
        # Use independently tracked wood landmarks for each alignment sample.
        table_normal=TABLE_NORMAL_IN_GRID.copy()
        for _ in range(45):
            data=observer.get();P=np.array(data['T_camera_b_plate']);pin=(np.linalg.inv(P)@np.c_[TIPS,np.ones(2)].T).T[:,:3]
            if 'surface' not in data:raise ValueError('Independent table alignment unavailable')
            table_normal=P[:3,:3].T@np.array(data['surface']['normal_camera_b'])
            grasp_rotation=measured_grasp_rotation(TIPS,normal_in_grid=table_normal)
            record['table_alignment_normal_in_grid']=table_normal.tolist()
            record['descent_geometry'].append(pin.tolist())
            record.setdefault('descent_joint_states',[]).append({'time':runner.rows[-1]['time'],'raw_deg':runner.rows[-1]['raw_deg'],'torques_nm':settled_load().tolist()})
            (folder/'pickup_progress.json').write_text(json.dumps(record,indent=2)+'\n')
            check_descent_load(baseline_load,settled_load())
            if (folder/'observe_only.txt').exists():
                capture(f'descent_{len(record["descent_geometry"])-1:02d}')
            center=pin.mean(0);error=np.array([.025,0.,.008])-center
            rotate=rotation_vector(P[:3,:3]@grasp_rotation)
            if pin[:,2].min()<.0035 or abs(center[1])>.004 or np.ptp(pin@table_normal)>.002:
                raise ValueError('Final descent clearance/alignment guard')
            if np.linalg.norm(error)<.001 and np.linalg.norm(rotate)<np.deg2rad(1.):break
            correction=error.copy();correction[1]*=1.5
            if abs(center[1])>.0007 or abs(error[0])>.0015 or np.linalg.norm(rotate)>np.deg2rad(1.):correction[2]=0
            if min(pin[:,2])<.005 and (abs(center[1])>.0012 or min(abs(pin[:,1]))<.045):correction=np.array([0.,0.,.008-min(pin[:,2])])
            correction*=min(1.,.002/max(np.linalg.norm(correction),1e-9))
            if np.linalg.norm(rotate)<np.deg2rad(.5):rotate[:]=0
            rotate*=min(1.,np.deg2rad(.8)/max(np.linalg.norm(rotate),1e-9))
            shift(0.,correction,rotate)
        else:raise ValueError('Grasp hover alignment did not converge')
        if (folder/'observe_only.txt').exists():
            raise ValueError('Observation repeat complete; returning without closing the gripper')
        def contact_sample():
            data=contact_observation()
            # A rejected frame does not authorize another movement. Hold the
            # current target for up to 0.3 s to obtain a new accepted sample.
            deadline=time.monotonic()+.3
            while 'surface' not in data and time.monotonic()<deadline:
                record.setdefault('surface_rejections',[]).append({k:data[k] for k in ('received','surface_error') if k in data})
                runner.tick();data=observer.get()
            if 'surface' not in data:raise RuntimeError('Table tracking lost during touchdown: '+str(data.get('surface_error')))
            pose=np.array(data['T_camera_b_plate'])
            axis=pose[:3,:3]@measured_grasp_rotation(TIPS,normal_in_grid=pose[:3,:3].T@np.array(data['surface']['normal_camera_b']))
            if np.linalg.norm(rotation_vector(axis))>np.deg2rad(1.5):raise RuntimeError('Physical table alignment lost during touchdown; hold')
            pads=(np.linalg.inv(pose)@np.c_[TIPS,np.ones(2)].T).T[:,:3]
            if np.ptp(pads@table_normal)>.002 or min(abs(pads[:,1]))<.0445:
                raise RuntimeError('Touchdown pad alignment lost; hold')
            return {'center_m':pads.mean(0).tolist(),'load_nm':settled_load().tolist(),
                    'raw_deg':runner.rows[-1]['raw_deg'],'time':runner.rows[-1]['time']}
        def save_touchdown(data):
            record['touchdown']=data
            (folder/'touchdown.json').write_text(json.dumps(data,indent=2)+'\n')
        def contact_move(delta):
            shift(0.,delta)
            # Recorded direction reversal continued for ~1 s after a tiny
            # command. Let the mechanics and camera settle before measuring
            # the backoff or deciding that the next downward step is needed.
            hold(.7)
        touchdown_started=True
        contact_baseline=settled_load()
        def contact_observation():
            from plate_observation import observe_or_hold
            paused={}
            def freeze():
                paused['q']=np.array(runner.rows[-1]['raw_deg'])
                paused['load']=settled_load()
                runner.targets=dict(enumerate(paused['q'].tolist()))
            def hold_tick():
                q=np.array(original_tick())
                if np.any(abs(q-paused['q'])>np.array([.3]*5+[.8])):
                    raise RuntimeError('Arm moved during stationary vision pause')
                current=settled_load();check_descent_load(paused['load'],current)
                if abs(current[1])>13.5:raise RuntimeError('Shoulder load exceeded during vision pause')
            return observe_or_hold(observer.get,freeze,hold_tick,
                lambda row:record.setdefault('vision_pauses',[]).append(row))
        def contact_load_check(baseline,current):
            reference=w.plate_recovery_context['seated_plate_surface']
            check_plate_release(reference,plate_in_surface_frame(contact_observation()))
            check_touchdown_load(baseline,current,True)
            check_touchdown_load(baseline,settled_load(),True)
        def contact_tick(*args,**kwargs):
            q=original_tick(*args,**kwargs)
            # Check during movement as well as at the settled endpoints.
            contact_load_check(contact_baseline,settled_load())
            return runner.rows[-1]['raw_deg']
        runner.tick=contact_tick
        contact_profile=(runner.speed,runner.acceleration,runner.minimum_duration)
        runner.speed=min(runner.speed,4.);runner.acceleration=min(runner.acceleration,36.)
        runner.minimum_duration=max(runner.minimum_duration,.25)
        try:
            # Operator-requested policy: the open pads may slide on the holder
            # during closure. Sustained resistance completes the approach;
            # maintain this height instead of reversing upward before gripping.
            touchdown=touch_and_backoff(contact_sample,contact_move,hold,
                                       save_touchdown,table_normal.copy(),backoff_m=0.,max_descent_m=.010,load_check=contact_load_check)
            # Remove accumulated downward command error at a blocked contact.
            # Hold the measured joint pose; do not command a vertical backoff.
            runner.go(touchdown['contact_confirmed']['raw_deg'])
            touchdown_complete=True
        except Exception as error:
            record['touchdown_stop_reason']=str(error)
            if isinstance(error,PlateRecoveryNeeded):raise
            if isinstance(error,ValueError) and str(error).startswith('Unexpected arm load during plate descent:'):
                raise PlateRecoveryNeeded('Touchdown load guard; withdraw with open jaws') from error
            raise RuntimeError('Touchdown not verified; hold before closing') from error
        finally:
            runner.tick=original_tick
            runner.speed,runner.acceleration,runner.minimum_duration=contact_profile
        capture('grasp_height');placed=list(runner.targets.values())
        release_observation=observer.get()
        release_reference=np.array(release_observation['T_camera_b_plate'])
        release_reference_surface=plate_in_surface_frame(release_observation)
        w.plate_recovery_context['grip_release']={'placed':placed,'opening_rad':opening,
            'reference_surface':release_reference_surface.tolist()}
        original_tick();grip.ensure_mode(4,1000);original_tick()
        if grip.get_register_u32(10,200)!=4:raise ValueError('Force-position mode readback failed')
        original_tick();grip.set_can_timeout_ms(500);original_tick()
        if grip.get_register_u32(9,200)!=10000:raise ValueError('Gripper watchdog readback failed')
        original_tick();grip.send_force_pos(opening,.05*speed_scale,.02);grip.enable();enabled=True;runner.tick=tick;hold(.5)
        record['gripper_commanded']=True
        began=time.monotonic();stall=0;relief_steps=0;closing_start=opening
        confirmed_contact_q=CONFIRMED_CONTACT_Q
        closing_goal=confirmed_contact_q+.04
        record['operator_confirmed_contact_q_rad']=confirmed_contact_q
        record['operator_confirmation_source']='20261004T220104487958Z: user observed plate contact'
        while time.monotonic()-began<30:
            if not runner.vision_ready():raise ValueError('Vision lost while closing')
            target=min(closing_goal,closing_start+(time.monotonic()-began)*.04*speed_scale)
            runner.tick();r=record['gripper_rows'][-1]
            stall=stall+1 if target-r['q']>.035 and abs(r['velocity'])<.03 else 0
            if stall>=10:
                from plate_closure import closure_action
                if closure_action(r['q'],relief_steps)=='relieve_half_mm':
                    # Wide-jaw rubbing is not a grasp. Keep the existing force
                    # limit and lift only 0.5 mm to relieve it, at most 3 times.
                    # The plate must remain seated in the independent wood frame.
                    check_plate_release(release_reference_surface,plate_in_surface_frame(observer.get()))
                    target=r['q'];hold(.2)
                    shift(.0005);relief_steps+=1
                    check_plate_release(release_reference_surface,plate_in_surface_frame(observer.get()))
                    record.setdefault('early_stall_relief',[]).append({'stall_q_rad':r['q'],'commanded_rise_m':.0005})
                    placed=list(runner.targets.values())
                    w.plate_recovery_context['grip_release']['placed']=placed
                    closing_start=record['gripper_rows'][-1]['q'];began=time.monotonic();stall=0
                    continue
                record['closing_stall_q']=r['q'];target=min(target,r['q']+.045);break
            if target==closing_goal and abs(r['q']-closing_goal)<.015:
                record['reached_operator_confirmed_contact_position']=True;break
        else:raise ValueError('Grasp did not reach a bounded contact condition')
        record['lift_trials']=[]
        for attempt in range(1):
            hold(.5);capture(f'closed_{attempt}')
            before=np.array(observer.get()['T_camera_b_plate']);qbefore=np.radians(runner.rows[-1]['raw_deg']);model_before=(w.geometry.transform(qbefore)@X@np.r_[MID,1])[:3]
            world_normal=(w.geometry.transform(qbefore)@X)[:3,:3]@before[:3,:3]@table_normal
            lifted=True
            # Joint settling error makes summed tiny commands overstate the
            # achieved rise. Use readback geometry, bounded to 30 mm commanded.
            for lift_step in range(12):
                shift(.0025)
                actual_q=np.radians(runner.rows[-1]['raw_deg'])
                actual_tip=(w.geometry.transform(actual_q)@X@np.r_[MID,1])[:3]
                rise=float((actual_tip-model_before)@world_normal)
                record['model_lift_height_m']=rise
                record['commanded_lift_m']=(lift_step+1)*.0025
                if rise>=record['test_lift_m']-.001:break
            else:raise ValueError('Requested 20 mm lift not reached within bounded travel')
            capture(f'test_lift_{attempt}')
            after=np.array(observer.get()['T_camera_b_plate']);qafter=np.radians(runner.rows[-1]['raw_deg']);model_after=(w.geometry.transform(qafter)@X@np.r_[MID,1])[:3]
            record['plate_motion_relative_camera_m']=float(np.linalg.norm(after[:3,3]-before[:3,3]));record['model_tip_travel_m']=float(np.linalg.norm(model_after-model_before))
            record['lift_following_verified']=record['plate_motion_relative_camera_m']<.002 and .019<=record['model_lift_height_m']<=.023
            record['lift_trials'].append({'attempt':attempt,'closing_target_rad':target,'gripper_feedback':record['gripper_rows'][-1],**{k:record[k] for k in ('plate_motion_relative_camera_m','model_tip_travel_m','lift_following_verified')}})
            print('TEST LIFT',attempt,':',record['lift_following_verified'],'plate motion relative camera mm',round(record['plate_motion_relative_camera_m']*1000,2),flush=True)
            if record['lift_following_verified']:
                if getattr(w,'table_carry',False):
                    from plate_carry import carry_and_return
                    record['transfer']=carry_and_return(runner,w,observer,folder,capture)
                break
            runner.go(placed,returning=True);hold(.6);lifted=False
    except ValueError as e:
        record['stop_reason']=str(e)
        capture('pickup_stop')
    finally:
        try:
            if lifted and placed is not None:runner.go(placed,returning=True);hold(.6);lifted=False
            if enabled:
                gripper_velocity=.05
                # Ramp opening as well as closure. A full-position step caused
                # measured velocity to exceed .35 rad/s despite the requested
                # force-position velocity limit on the physical gripper.
                release_start=grip.get_register_f32(80,100);original_tick()
                release_began=time.monotonic();until=release_began+40
                while time.monotonic()<until:
                    target=max(opening,release_start-.04*(time.monotonic()-release_began))
                    runner.tick()
                    if target==opening and abs(record['gripper_rows'][-1]['q']-opening)<.004:
                        hold(.3)
                        if abs(record['gripper_rows'][-1]['q']-opening)<.004:
                            record['jaw_open_verified']=True;break
                if not record['jaw_open_verified']:raise RuntimeError('Plate release not verified; do not retract with a possible retained plate')
                capture('released')
                try:
                    released_observation=observer.get()
                    released_pose=np.array(released_observation['T_camera_b_plate'])
                    released_surface=plate_in_surface_frame(released_observation)
                except ValueError as error:raise RuntimeError('Plate pose unavailable after opening; hold before retreat') from error
                record['release_reference_pose']=release_reference.tolist();record['released_plate_pose']=released_pose.tolist()
                record['release_reference_surface_pose']=release_reference_surface.tolist();record['released_surface_pose']=released_surface.tolist()
                record.update(check_plate_release(release_reference_surface,released_surface))
                # A visually seated plate can still adhere to a finger. Verify
                # Verify observed separation, allowing one additional 2.5 mm
                # command for measured undertravel (7.5 mm total maximum).
                shift(.0025)
                for withdrawal_step in (2,3):
                    shift(.0025);capture('release_withdrawal' if withdrawal_step==2 else 'release_withdrawal_retry')
                    separated_observation=observer.get()
                    check_plate_release(release_reference_surface,plate_in_surface_frame(separated_observation))
                    separated_pose=np.array(separated_observation['T_camera_b_plate'])
                    record['release_withdrawal_pose']=separated_pose.tolist()
                    record['release_withdrawal_commanded_m']=withdrawal_step*.0025
                    try:
                        record.update(check_release_separation(released_pose,separated_pose,table_normal))
                        break
                    except PlateRecoveryNeeded:
                        if withdrawal_step==3:raise
                withdrawn=True
                record['released_verified']=True
            if not withdrawn and (not touchdown_started or touchdown_complete):runner.go(pre,returning=True)
        finally:
            if enabled:grip.disable();enabled=False
            runner.tick=original_tick;original_tick();grip.ensure_mode(mode,1000);original_tick();grip.write_register_u32(9,timeout);original_tick()
            record['gripper_settings_restored']=grip.get_register_u32(10,200)==mode and grip.get_register_u32(9,200)==timeout
            (folder/'pickup_report.json').write_text(json.dumps(record,indent=2)+'\n')
    return record
