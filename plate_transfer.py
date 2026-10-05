"""Checked adjacent-table route, initially rehearsed with empty open jaws."""
import json,time,threading
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from rotation_math import rotation_vector
from plate_surface import SurfaceTracker,CALIBRATION

def table_geometry(geometry,q,observation):
    from plate_hover import X
    calibration=json.loads(Path(CALIBRATION).read_text())
    C=geometry.transform(np.radians(q))@X
    B=C@np.array(observation['surface']['T_camera_b_reference'])
    normal=B[:3,:3]@np.array(calibration['normal_reference_b'])
    normal/=np.linalg.norm(normal)
    if normal[2]<0:normal=-normal
    origin=(B@np.r_[np.mean(calibration['points_reference_b_m'],axis=0),1])[:3]
    P=C@np.array(observation['T_camera_b_plate'])
    side=P[:3,1]-normal*(normal@P[:3,1]);side/=np.linalg.norm(side)
    return origin,normal,side

def cartesian_segment(geometry,start,displacement,limits,origin,normal,min_clearance=.020,between_checks=lambda:None):
    from plate_hover import X,TIPS,MID
    start=np.asarray(start,float);displacement=np.asarray(displacement,float)
    origin=np.asarray(origin,float);normal=np.asarray(normal,float)
    if start.shape!=(6,) or origin.shape!=(3,) or normal.shape!=(3,) or not np.isfinite(np.r_[start,origin,normal]).all() or abs(np.linalg.norm(normal)-1.)>1e-6:
        raise ValueError('Invalid table-transfer geometry')
    if displacement.shape!=(3,) or not np.isfinite(displacement).all() or np.linalg.norm(displacement)>.16:
        raise ValueError('Invalid table-transfer displacement')
    q=np.radians(start);initial=geometry.transform(q)@X
    tip=(initial@np.r_[MID,1])[:3];R=initial[:3,:3]
    lo=np.radians([limits[i][0] for i in range(6)]);hi=np.radians([limits[i][1] for i in range(6)])
    poses=[start.tolist()];clearance=[]
    corners=np.array([t-d*np.array([0,2**-.5,2**-.5])+h*np.array([0,-2**-.5,2**-.5]) for t in TIPS for d in (0,.061) for h in (-.004,.004)])
    for u in np.linspace(0,1,max(2,int(np.ceil(np.linalg.norm(displacement)/.0025))+1))[1:]:
        between_checks()
        goal=tip+u*displacement
        def residual(a):
            C=geometry.transform(a)@X
            return np.r_[10*((C@np.r_[MID,1])[:3]-goal),rotation_vector(R.T@C[:3,:3])]
        fit=least_squares(residual,q,bounds=(lo,hi),max_nfev=80)
        if np.linalg.norm(residual(fit.x))>1e-4 or np.max(abs(np.degrees(fit.x-q)))>2.:
            raise ValueError(f'Adjacent-table IK/step check failed at {u:.3f}: residual {np.linalg.norm(residual(fit.x)):.5f}, step {np.max(abs(np.degrees(fit.x-q))):.3f}, q {np.degrees(fit.x).tolist()}')
        q=fit.x;C=geometry.transform(q)@X
        height=float(np.min((corners@C[:3,:3].T+C[:3,3]-origin)@normal))
        if height<min_clearance:raise ValueError('Adjacent-table full-pad clearance failed')
        clearance.append(height);poses.append(np.degrees(q).tolist())
    return poses,min(clearance)

class TableObserver:
    def __init__(self,w):
        self.w=w;self.tracker=SurfaceTracker();self.latest=None;self.error=None
        self.stop=threading.Event();self.thread=threading.Thread(target=self.run,daemon=True)
    def run(self):
        last=0
        while not self.stop.is_set():
            with self.w.lock:frame=dict(self.w.survey_frames.get('wrist',{}))
            if frame.get('received',0)<=last:time.sleep(.005);continue
            last=frame['received']
            try:self.latest={**self.tracker.update(frame['image']),'received':last};self.error=None
            except (ValueError,cv2.error) as error:self.error=str(error)
    def get(self):
        if not self.latest or time.monotonic()-self.latest['received']>.5:
            raise ValueError('Independent transfer table tracking unavailable: '+str(self.error))
        return self.latest

def stop_observer(observer,tick):
    observer.stop.set();deadline=time.monotonic()+2
    while observer.thread.is_alive():
        tick()
        if time.monotonic()>deadline:raise RuntimeError('Transfer observer did not stop')

def survey(runner,w,observer,folder,capture,offset_m=-.115):
    from blended_motion import checked_blend_segments
    evidence=getattr(w,'holder_evidence',[])
    if len(evidence)<10:raise ValueError('Table survey requires preflight holder evidence')
    if offset_m not in (.115,.125,-.115,-.125):raise ValueError('Unsupported adjacent-table offset')
    observation=observer.get();q=list(runner.targets.values())
    origin,normal,side=table_geometry(w.geometry,runner.rows[-1]['raw_deg'],observation)
    first,c1=cartesian_segment(w.geometry,q,.015*normal,runner.limits,origin,normal,between_checks=runner.tick)
    second,c2=cartesian_segment(w.geometry,first[-1],offset_m*side,runner.limits,origin,normal,between_checks=runner.tick)
    paths=[]
    for segment in (first,second):
        paths.extend(checked_blend_segments(segment,16.,144.,runner.tick))
    for segment in paths:checked_blend_segments(list(reversed(segment)),16.,144.,runner.tick)
    report={'holder_success_count':len(evidence),'offset_m':offset_m,'table_origin_base_m':origin.tolist(),
            'normal_base':normal.tolist(),'side_base':side.tolist(),'planned_paths_deg':paths,
            'min_pad_table_clearance_m':min(c1,c2),'returned_to_anchor':False,'destination_reached':False}
    saved=(runner.low.copy(),runner.high.copy(),runner.speed,runner.acceleration,runner.pacing_lag,runner.vision_ready)
    stop_observer(observer,runner.tick)
    table=TableObserver(w);table.thread.start();visited=[q]
    try:
        deadline=time.monotonic()+5
        # Require current camera streams and independently fitted wood pose;
        # the plate may leave the wrist image during this empty-jaw survey.
        def ready():
            try:table.get()
            except ValueError:return False
            with w.lock:return all(time.monotonic()-w.sources.get(r,{}).get('time',0)<.5 and w.sources.get(r,{}).get('frame_age_s',99)<.5 for r in ('wrist','tripod'))
        while not ready() and time.monotonic()<deadline:runner.tick()
        if not ready():raise ValueError('Table survey could not acquire independent tracking')
        allpoints=np.array([p for segment in paths for p in segment])
        runner.low=allpoints.min(0)-.05;runner.high=allpoints.max(0)+.05
        runner.speed=16.;runner.acceleration=144.;runner.pacing_lag=1.8;runner.vision_ready=ready
        for segment in paths:runner.blend(segment,visited)
        report['destination_reached']=True
        report['destination_raw_deg']=runner.rows[-1]['raw_deg'];report['destination_table_pose']=table.get()
        capture('adjacent_table_survey')
    except ValueError as error:
        report['stop_reason']=str(error)
        report['table_tracking_rejection']=table.error
        with w.lock:report['camera_sources_at_stop']={role:{k:v for k,v in w.sources.get(role,{}).items() if k in ('time','frame_age_s','error')} for role in ('wrist','tripod')}
    finally:
        # Empty open jaws, retrace only the dense commands actually visited.
        # Keep the current motor owner and bounds until the anchor is reached.
        try:
            current=list(runner.targets.values())
            if np.max(abs(np.array(current)-visited[-1]))>1e-8:visited.append(current)
            if report['destination_reached']:
                # The full checked route is available. Replay its original
                # reverse curves; refitting hundreds of time-sampled commands
                # can create tiny spline intervals and excessive duration.
                report['return_segments']=[]
                for segment in reversed(paths):
                    report['return_segments'].append(runner.blend(list(reversed(segment)),[],returning=True))
            else:
                report['partial_return']=runner.retrace(list(reversed(visited)))
            report['returned_to_anchor']=True
        finally:
            stop_observer(table,runner.tick)
            runner.low,runner.high,runner.speed,runner.acceleration,runner.pacing_lag,runner.vision_ready=saved
            (folder/'table_survey.json').write_text(json.dumps(report,indent=2)+'\n')
        observer.resume()
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            runner.tick()
            try:observer.get();break
            except ValueError:pass
        else:raise RuntimeError('Plate tracker did not reacquire after adjacent survey')
    return report
