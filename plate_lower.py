"""Bounded camera-relative noncontact descent using the tracked plate grid."""
import json,time
import cv2
import numpy as np
from plate_grid import GridObserver
from rotation_math import rotation_vector
from plate_grasp_geometry import grasp_rotation as measured_grasp_rotation
from plate_recovery import PlateRecoveryNeeded,recover_to_standoff

def run(runner,w,folder,capture,pickup=False):
    from plate_hover import X,TIPS,MID,R_TARGET
    observer=GridObserver(w);observer.thread.start();entry=list(runner.targets.values());visited=[entry]
    low,high=runner.low.copy(),runner.high.copy();speed,acc=runner.speed,runner.acceleration
    pacing=getattr(runner,'pacing_lag',1.8)
    # The checked 5 mm pregrasp solution needs ~24 degrees of wrist bend
    # from hover after leveling the pads; retain margin for visual correction.
    local=np.array([10,30,30,40,15,12]);runner.low=np.maximum(np.array(entry)-local,[runner.limits[i][0] for i in range(6)]);runner.high=np.minimum(np.array(entry)+local,[runner.limits[i][1] for i in range(6)])
    scale=getattr(w,'plate_speed_scale',1.)
    from plate_speed import motion_profile,apply_local_profile
    profile=getattr(w,'plate_motion_profile',motion_profile(scale,1.))
    apply_local_profile(runner,profile,clear=True)
    rows=[];result={'contact_attempted':False,'reached_25mm_standoff':False,'rows':rows};oldvision=runner.vision_ready
    span=TIPS[1]-TIPS[0];span/=np.linalg.norm(span);long=np.array([0,2**-.5,2**-.5]);long-=span*(long@span);long/=np.linalg.norm(long)
    grasp_rotation=np.array([long,span,np.cross(long,span)])
    standoff=.025;unsafe_to_retrace=False;clear_anchor=None;clear_visited=None
    def ready():
        try:observer.get();return oldvision()
        except ValueError:return False
    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and not ready():runner.tick()
        if not ready():raise ValueError('Plate grid acquisition failed')
        runner.vision_ready=ready
        for index in range(90 if pickup or getattr(w,'recovery_test',False) else 50):
            until=time.monotonic()+.35
            while time.monotonic()<until:runner.tick()
            data=observer.get();P=np.array(data['T_camera_b_plate']);pin=(np.linalg.inv(P)@np.c_[TIPS,np.ones(2)].T).T[:,:3];center=pin.mean(0)
            if standoff<.025:
                if 'surface' not in data:raise ValueError('Independent table tracking unavailable: '+str(data.get('surface_error')))
                normal=P[:3,:3].T@np.array(data['surface']['normal_camera_b'])
                grasp_rotation=measured_grasp_rotation(TIPS,normal_in_grid=normal)
            error=np.array([.025,0.,standoff])-center
            orientation=cv2.Rodrigues(P[:3,:3]@grasp_rotation)[0].ravel()
            row={'index':index,**data,'tips_in_plate_m':pin.tolist(),'midpoint_error_m':error.tolist(),'orientation_error_deg':float(np.degrees(np.linalg.norm(orientation))),'raw_deg':runner.rows[-1]['raw_deg']};rows.append(row)
            (folder/'lower_progress.json').write_text(json.dumps(result,indent=2)+'\n')
            print('Plate descent',index,'gap mm',round(center[2]*1000,1),'lateral mm',round(center[1]*1000,1),flush=True)
            if min(pin[:,2])<(.016 if standoff==.025 else .001):raise ValueError('Reached plate clearance guard')
            raise_for_alignment=standoff<.025 and min(pin[:,2])<.012 and (abs(center[1])>.0015 or min(abs(pin[:,1]))<.0445)
            if standoff<.025 and abs(center[1])>.006:raise ValueError('Plate alignment left the approach corridor')
            # Match pickup's final alignment criterion before taking its load
            # baseline; handing off early caused another tiny direction reversal.
            fine_grasp=pickup and standoff<.025
            if np.linalg.norm(error)<(.001 if fine_grasp else .0012) and np.linalg.norm(orientation)<np.deg2rad(1. if fine_grasp else 1.5):
                if standoff==.025:
                    result['reached_25mm_standoff']=True;capture('lower')
                    clear_anchor=list(runner.targets.values());clear_visited=list(visited)
                    w.plate_recovery_context={'observer':observer,'anchor':clear_anchor,'visited':clear_visited,
                        'entry':entry,'low':runner.low.copy(),'high':runner.high.copy(),
                        'route_low':low,'route_high':high}
                    if getattr(w,'recovery_test',False) and getattr(w,'recovery_height_mm',25)==25:
                        raise PlateRecoveryNeeded('Injected open-jaw stop at the 25 mm standoff')
                    if not pickup and not getattr(w,'recovery_test',False) and not getattr(w,'alignment_review',False) and not getattr(w,'table_survey',False):break
                    observer.surface_enabled=True
                    deadline=time.monotonic()+3
                    while time.monotonic()<deadline and 'surface' not in observer.get():runner.tick()
                    if 'surface' not in observer.get():raise ValueError('Independent table tracking acquisition failed')
                    from plate_surface import plate_in_surface_frame
                    w.plate_recovery_context['seated_plate_surface']=plate_in_surface_frame(observer.get()).tolist()
                    if getattr(w,'table_survey',False):
                        from plate_transfer import survey
                        result['table_survey']=survey(runner,w,observer,folder,capture)
                        break
                    standoff=getattr(w,'recovery_height_mm',18)/1000 if getattr(w,'recovery_test',False) else .018 if getattr(w,'alignment_review',False) else .008
                    apply_local_profile(runner,profile,clear=False)
                    continue
                if getattr(w,'recovery_test',False):
                    capture('recovery_test_start')
                    raise PlateRecoveryNeeded(f'Injected open-jaw stop at the {standoff*1000:.0f} mm standoff')
                if getattr(w,'alignment_review',False):
                    capture('alignment_review')
                    observer.stop.set();observer.thread.join(timeout=1)
                    w.pregrasp_stereo();result['alignment_review_only']=True;break
                capture('pregrasp');result['pregrasp_standoff_m']=standoff
                review=getattr(w,'pause_for_plate_review',False)
                if review:
                    print('PREGRASP REVIEW:',folder,flush=True)
                    deadline=time.monotonic()+180
                    while time.monotonic()<deadline and not (folder/'grasp.txt').exists() and not (folder/'stereo.txt').exists():
                        if not ready():raise ValueError('Camera lost during pregrasp review')
                        runner.tick()
                if (folder/'stereo.txt').exists():
                    observer.stop.set();observer.thread.join(timeout=1)
                    w.pregrasp_stereo();result['stereo_review_only']=True;break
                if review and not (folder/'grasp.txt').exists():raise ValueError('Pregrasp review elapsed; returning without closing')
                if not ready():raise ValueError('Camera lost before automatic grasp')
                result['automatic_grasp_after_alignment']=not review
                from plate_pickup import grasp_and_test
                result['contact_attempted']=True
                try:result['pickup']=grasp_and_test(runner,w,observer,folder,capture)
                except PlateRecoveryNeeded:raise
                except Exception as error:
                    raise RuntimeError('Pickup/release did not complete; hold without nested retreat') from error
                break
            q=np.radians(row['raw_deg']);C=w.geometry.transform(q)@X;tip=(C@np.r_[MID,1])[:3]
            correction=error.copy()
            if standoff<.025:
                # Resolve lateral drift before spending the small side
                # clearance at the rim. Keep the current height while aligning.
                if abs(center[1])>.0007 or abs(error[0])>.0015 or np.linalg.norm(orientation)>np.deg2rad(1.):correction[2]=0.
                correction[1]*=1.5
                if raise_for_alignment:correction=np.array([0.,0.,.014-min(pin[:,2])])
            desired=C[:3,:3]@P[:3,:3]@correction;travel=.004 if standoff==.025 else .002
            desired*=min(1.,travel/max(np.linalg.norm(desired),1e-9))
            rotate=orientation.copy()
            if np.linalg.norm(rotate)<np.deg2rad(.5):rotate[:]=0.
            rotate*=min(1.,np.deg2rad(.8)/max(np.linalg.norm(rotate),1e-9))
            J=np.zeros((6,6));eps=1e-5
            for i in range(6):
                qq=q.copy();qq[i]+=eps;N=w.geometry.transform(qq)@X
                J[:3,i]=100*((N@np.r_[MID,1])[:3]-tip)/eps;J[3:,i]=rotation_vector(C[:3,:3].T@N[:3,:3])/eps
            J[3:]*=3;rotate*=3
            dq=np.linalg.solve(J.T@J+.03**2*np.eye(6),J.T@np.r_[100*desired,rotate]);dq*=min(1.,np.deg2rad(1.5)/max(abs(dq)))
            goal=np.degrees(q+dq)
            if np.any(goal<runner.low) or np.any(goal>runner.high):raise ValueError('Plate descent leaves local joint bounds')
            visited.append(goal.tolist());runner.go(goal.tolist())
        else:raise ValueError('Plate descent did not converge within bounded steps')
    except PlateRecoveryNeeded as error:
        result['stop_reason']=str(error)
        try:
            if clear_anchor is None:raise RuntimeError('No checked recovery anchor')
            apply_local_profile(runner,profile,clear=False)
            result['recovery']=recover_to_standoff(runner,w,observer,folder,clear_anchor,capture,error)
            visited=clear_visited
        except BaseException as recovery_error:
            unsafe_to_retrace=True;result['retreat_inhibited']=True
            raise RuntimeError('Automatic plate recovery stopped; powered hold required') from recovery_error
    except ValueError as e:result['stop_reason']=str(e)
    except BaseException:
        unsafe_to_retrace=True
        result['retreat_inhibited']=True
        raise
    finally:
        observer.stop.set();observer.thread.join(timeout=1)
        runner.vision_ready=oldvision
        if not unsafe_to_retrace:
            if clear_visited is not None:
                # Retain measured local withdrawal below the 25 mm anchor.
                # Above it, retrace the checked route without waypoint dwells.
                apply_local_profile(runner,profile,clear=False)
                for goal in reversed(visited[len(clear_visited)-1:-1]):runner.go(goal,returning=True)
                apply_local_profile(runner,profile,clear=True)
                result['clear_return']=runner.retrace(list(reversed(clear_visited)))
            else:
                for goal in reversed(visited[:-1]):runner.go(goal,returning=True)
                runner.go(entry,returning=True)
            w.plate_recovery_context=None
        runner.low,runner.high=low,high;runner.speed,runner.acceleration=speed,acc;runner.pacing_lag=pacing
        (folder/'lower_report.json').write_text(json.dumps(result,indent=2)+'\n')
    return result
