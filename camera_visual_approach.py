#!/usr/bin/env python3
"""Camera-checked noncontact approach using current AxisArm control; preview default."""
import argparse,json,math,threading,time,subprocess
from datetime import datetime,timezone
from pathlib import Path
import numpy as np
from axis_follow import AxisArm,axis_bounds
from arm_geometry import Geometry
from pairing_dashboard import Workbench
from camera_check import CameraCheck
from calibrate_joint_motion import Runner
from rise_approach import Planner


def route(start,forward_m=.06,bend_view=0.):
    if forward_m not in (.06,.12,.20,.30) or bend_view not in (0.,6.):raise ValueError('Unsupported bounded route')
    p=Planner(np.radians(start));poses=[list(start)]
    view=list(start);view[4]+=6.;view[3]+=bend_view;poses.append(view)
    p.start=np.radians(view);p.base=p.g.transform(p.start)
    existing={i:axis_bounds(p.g,i,start[i]) for i in range(6)}
    if forward_m>.06:
        radii=[20.,140.,140.,90.,30.,20.]
        for i in range(6):
            limit=p.g.joints[i].find('limit');sign=p.g.signs[i];offset=math.degrees(p.g.offsets[i])
            a,b=sorted((math.degrees(float(limit.get(k)))-offset)/sign for k in ('lower','upper'))
            existing[i]=(max(start[i]-radii[i],min(start[i],a+1.)),min(start[i]+radii[i],max(start[i],b-1.)))
    p.low=np.radians([existing[i][0] for i in range(6)]);p.high=np.radians([existing[i][1] for i in range(6)])
    for h in np.arange(.005,.051,.005):poses.append(np.degrees(p.lift(float(h))).tolist())
    q=np.radians(poses[-1])
    for _ in range(round(forward_m/.005)):
        q=p.forward(q,.005)
        # This position approach does not need roll changes. Preserve the
        # measured starting roll rather than chasing small IK roll adjustments.
        q[5]=p.start[5]
        poses.append(np.degrees(q).tolist())
    g=p.g;limits=existing
    for point in poses:
        if any(not limits[i][0]<=point[i]<=limits[i][1] for i in range(6)):
            raise ValueError('Route leaves existing raw joint limits')
    return poses,limits


def fast_indices(poses, max_error=.20):
    """Merge verified waypoints only where joint-space chord error is bounded."""
    qs=np.asarray(poses);result=[0]
    # Preserve viewing offset, gentle initial lift and the lift/forward corner.
    for end in sorted(set([1,3,11,len(qs)-1])):
        if end>=len(qs):continue
        while result[-1]<end:
            start=result[-1];chosen=start+1
            for candidate in range(start+2,end+1):
                u=np.linspace(0,1,candidate-start+1)[:,None]
                chord=qs[start]+u*(qs[candidate]-qs[start])
                if np.max(abs(chord-qs[start:candidate+1]))>max_error:break
                chosen=candidate
            result.append(chosen)
    return result


def movement_duration(delta,speed,acceleration,minimum=.25):
    distance=float(np.max(abs(delta)))
    return max(minimum,1.5*distance/speed,math.sqrt(6*distance/acceleration))


def paced_progress(origin,delta,progress,advance,duration,actual,max_lag):
    """Advance along the same segment, reserving tracking margin at high speed."""
    def target(p):
        u=min(1.,p/duration)
        return origin+delta*(u*u*(3-2*u))
    proposed=min(duration,progress+advance)
    if np.max(abs(target(proposed)-actual))<=max_lag:return proposed
    low,high=progress,proposed
    for _ in range(16):
        middle=(low+high)/2
        if np.max(abs(target(middle)-actual))<=max_lag:low=middle
        else:high=middle
    return low


class RouteRunner(Runner):
    def __init__(self,w,arm,start,limits,output,poses):
        super().__init__(w,arm,start,limits,output)
        qs=np.array(poses);self.low=qs.min(0)-.05;self.high=qs.max(0)+.05
        self.speed=2.25;self.acceleration=9.;self.clearance=.315;self.tracking_limit=2.;self.pacing_lag=1.1
        self.settle_tolerance=np.full(6,.5)
    def tick(self,moving=False,take_up=False):
        q=self.arm.read();now=time.monotonic();dt=now-self.last
        if not 0<dt<.3:raise RuntimeError('Motor loop stalled')
        if max(abs(q[i]-self.targets[i]) for i in range(6))>self.tracking_limit:raise RuntimeError(f'Tracking error exceeds {self.tracking_limit} degrees')
        allowance=np.where(self.high-self.low>.2,self.tracking_limit,.5)
        outside=np.where((np.array(q)<self.low-allowance)|(np.array(q)>self.high+allowance))[0]
        if len(outside):raise RuntimeError(f'Measured joints {(outside+1).tolist()} left route envelope: {q}')
        if any(not self.limits[i][0]-.5<=q[i]<=self.limits[i][1]+.5 for i in range(6)):
            raise RuntimeError('Measured joint left absolute joint bounds')
        self.arm.command_group(self.targets,take_up=take_up)
        self.w.camera_check.add_joints(now,q,powered=True,following=moving,all_powered=True,fault=None)
        self.rows.append({'time':now,'raw_deg':q,'target_deg':list(self.targets.values()),'moving':moving,'cruise_cap_deg_s':self.speed,'motor_torques_nm':[m.get_state().torq for m in self.arm.motors],'motor_temperatures_c':[[m.get_state().t_mos,m.get_state().t_rotor] for m in self.arm.motors]})
        self.last=now;time.sleep(.005);return q
    def vision_ready(self):
        if not super().vision_ready():return False
        with self.w.camera_check.lock:
            T=np.array([f for f in self.w.camera_check.frames if f.get('valid')][-1]['T_camera_tag'])
            return abs(float(T[:3,2]@T[:3,3]))>self.clearance
    def go(self,goal,returning=False):
        if not np.isfinite(goal).all() or np.any(np.array(goal)<self.low) or np.any(np.array(goal)>self.high):raise RuntimeError('Target leaves route')
        origin=np.array(list(self.targets.values()));delta=np.array(goal)-origin
        duration=movement_duration(delta,self.speed,self.acceleration,getattr(self,'minimum_duration',.25))
        began=time.monotonic();previous=began;progress=0.
        while True:
            if not returning and not self.vision_ready():raise ValueError('Camera freshness or tag visibility lost')
            now=time.monotonic();dt=now-previous;previous=now
            if now-began>max(12.,duration*4):raise ValueError('Movement could not keep up with its paced target')
            # Slow the trajectory clock before lag reaches the configured tracking
            # fault. Keep commanding/reading all motors throughout each pause.
            actual=np.array(self.rows[-1]['raw_deg']) if self.rows else origin
            lag=float(np.max(abs(actual-np.array(list(self.targets.values())))))
            advance=min(dt,.06)*max(0.,min(1.,(self.pacing_lag-lag)/.5))
            progress=paced_progress(origin,delta,progress,advance,duration,actual,self.tracking_limit-.5)
            u=min(1.,progress/duration);target=origin+delta*(u*u*(3-2*u))
            self.targets=dict(enumerate(target.tolist()));q=self.tick(moving=True)
            if u>=1:break
        self.settle(goal,returning=returning)

    def settle(self,goal,returning=False):
        # The trajectory already reached this command. Check measured settling
        # directly instead of adding a zero-distance minimum-duration move.
        if np.max(abs(np.array(goal)-list(self.targets.values())))>.01:
            raise RuntimeError('Settle target differs from commanded endpoint')
        until=time.monotonic()+3
        while time.monotonic()<until:
            if not returning and not self.vision_ready():raise ValueError('Camera lost while settling')
            q=self.tick()
            if np.all(abs(np.array(q)-goal)<self.settle_tolerance):return
        raise ValueError('Route endpoint did not settle: '+str([round(q[i]-goal[i],3) for i in range(6)]))

    def blend(self,points,visited,returning=False):
        from blended_motion import BlendedPath
        path=BlendedPath(points,self.speed,self.acceleration)
        if np.max(abs(np.array(points[0])-list(self.targets.values())))>.01:raise RuntimeError('Blend start mismatch')
        if np.any(path.points<self.low) or np.any(path.points>self.high):raise RuntimeError('Blend leaves route')
        began=previous=time.monotonic();elapsed=0.
        while True:
            if not returning and not self.vision_ready():raise ValueError('Camera freshness or tag visibility lost during blend')
            now=time.monotonic();dt=now-previous;previous=now
            if now-began>max(20.,path.duration*4):raise ValueError('Blended path could not keep pace')
            actual=np.array(self.rows[-1]['raw_deg']);lag=np.max(abs(actual-list(self.targets.values())))
            advance=min(dt,.06)*max(0.,min(1.,(self.pacing_lag-lag)/.5))
            elapsed=path.advance(elapsed,advance,actual,self.tracking_limit-.5)
            target=path.at(elapsed).tolist();self.targets=dict(enumerate(target));self.tick(moving=True)
            # A partial camera stop can retrace every commanded point.
            if not returning and np.max(abs(np.array(target)-visited[-1]))>.1:visited.append(target)
            if elapsed>=path.duration:break
        self.settle(points[-1],returning=returning)
        if not returning:visited.append(list(points[-1]))
        return {'planned_duration_s':path.duration,'actual_duration_s':time.monotonic()-began,'max_curve_deviation_deg':path.max_deviation}

    def retrace(self,points):
        from blended_motion import checked_blend_segments
        points=np.asarray(points,float)
        if points.ndim!=2 or points.shape[1]!=6 or len(points)<1 or not np.isfinite(points).all():
            raise RuntimeError('Invalid recorded retrace')
        if np.max(abs(points[0]-list(self.targets.values())))>.01:
            raise RuntimeError('Recorded retrace start mismatch')
        if np.any(points<self.low) or np.any(points>self.high):
            raise RuntimeError('Recorded retrace leaves route')
        segments=checked_blend_segments(points,self.speed,self.acceleration,self.tick)
        result={'recorded_waypoints':len(points),'blended_segments':[]}
        for segment in segments:
            result['blended_segments'].append(self.blend(segment,[],returning=True))
        return result


def observation(sample):
    poses=np.array([s['T_camera_tag'] for s in sample['stationary_observations']]);T=np.mean(poses,axis=0)
    t=T[:3,3];normal=T[:3,2];normal/=np.linalg.norm(normal)
    return {'camera_range_m':float(np.linalg.norm(t)),'camera_plane_m':abs(float(normal@t)),
            'translation_camera_m':t.tolist(),'translation_std_m':poses[:,:3,3].std(0).tolist()}


def execute(forward_m=.06,bend_view=0.,stereo_end=False,stereo_servo=False,fast=False,touch=False,speed_scale=1.,blend=False,sweep=False):
    if speed_scale not in (1.,2.,4.) or (speed_scale!=1 and not fast):raise ValueError('Unsupported speed profile')
    if speed_scale==4 and (stereo_servo or stereo_end or touch):raise ValueError('48 degree/s profile is a noncontact route trial only')
    folder=Path('outputs/visual_approach')/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');folder.mkdir(parents=True)
    w=Workbench();w.camera_check=CameraCheck(w.geometry,folder/'views')
    workers=[threading.Thread(target=w.camera_worker,args=(r,),daemon=True) for r in ('wrist','tripod')]
    for worker in workers:worker.start()
    report={'blend':blend,'sweep':sweep,'speed_scale':speed_scale,'fast':fast,'nominal_forward_m':forward_m,'bend_view_deg':bend_view,'contact_attempted':False,'returned_to_start':False,'motors_disabled_verified':False,'samples':[]};runner=None
    try:
        with AxisArm() as arm:
            start=arm.read();poses,limits=route(start,forward_m,bend_view);report['start_raw_deg']=start;report['planned_raw_deg']=poses
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                q=arm.read();w.camera_check.add_joints(time.monotonic(),q,powered=False,following=False,all_powered=False,fault=None)
                if max(abs(a-b) for a,b in zip(q,start))>.1:raise RuntimeError('Starting pose moved')
                if w.camera_check.snapshot()['ready']:break
                time.sleep(.005)
            else:raise RuntimeError('Vision did not become ready')
            arm.prepare_group(range(6));arm.speed_limits={i:min(60.,20.*speed_scale) for i in range(6)}
            if speed_scale==4:arm.command_speed_limits={i:60. for i in range(5)}
            arm.enable_group(dict(enumerate(start)))
            runner=RouteRunner(w,arm,start,limits,folder,poses)
            if fast:
                runner.speed=12.*speed_scale;runner.acceleration=36.*speed_scale**2;runner.clearance=.340
                runner.tracking_limit={1.:4.,2.:6.,4.:8.}[speed_scale];runner.pacing_lag={1.:2.5,2.:4.5,4.:6.}[speed_scale]
                if speed_scale==4:runner.speed=36.;runner.acceleration=288.
            indices=([0,1,3,11,len(poses)-1] if blend else fast_indices(poses)) if fast else list(range(len(poses)))
            report['executed_indices']=indices
            end=time.monotonic()+1.
            while time.monotonic()<end:runner.tick(take_up=True)
            visited=[start]
            try:
                first=runner.capture();first['metric']=observation(first);report['samples'].append(first)
                previous_index=0
                for index in indices[1:]:
                    goal=poses[index]
                    if speed_scale==4 and index>11:runner.speed=48.
                    if blend and index>3:
                        report.setdefault('blended_segments',[]).append(runner.blend(poses[previous_index:index+1],visited))
                    else:visited.append(goal);runner.go(goal)
                    previous_index=index
                    if fast and index not in (1,11,len(poses)-1):continue
                    sample=runner.capture();sample['metric']=observation(sample);report['samples'].append(sample)
                    print(f'{index}/{len(poses)-1}: camera-to-tag plane {sample["metric"]["camera_plane_m"]*1000:.1f} mm',flush=True)
                    if sample['metric']['camera_plane_m']<.30:raise ValueError('Reached the noncontact clearance boundary')
                    if index==11:forward_plane=sample['metric']['camera_plane_m']
                    if index>11 and sample['metric']['camera_plane_m']>forward_plane+.010:
                        raise ValueError('Observed approach moved away from the tag by more than 10 mm')
                report['forward_complete']=True
                if sweep:
                    from angle_sweep import run_sweep
                    report['angle_sweep'],workers=run_sweep(runner,w,folder,workers)
                if stereo_servo:
                    from stereo_servo import run_phase
                    report['stereo_servo'],workers=run_phase(runner,w,folder,workers,touch=touch,speed_scale=speed_scale)
                    report['contact_attempted']=report['stereo_servo']['contact_attempted']
                elif stereo_end:
                    w.stop.set();deadline=time.monotonic()+15
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
                    w.stop.clear()
                    workers=[threading.Thread(target=w.camera_worker,args=(r,),daemon=True) for r in ('wrist','tripod')]
                    for worker in workers:worker.start()
                    deadline=time.monotonic()+20
                    while time.monotonic()<deadline and not runner.vision_ready():runner.tick()
                    report['cameras_restarted']=runner.vision_ready()

            except ValueError as error:
                report['stop_reason']=str(error);print('Returning: '+str(error),flush=True)
                with w.lock:report['stop_camera_sources']={r:dict(w.sources.get(r,{})) for r in ('wrist','tripod')}
            if blend and report.get('forward_complete'):
                runner.blend(list(reversed(poses[11:])),[],returning=True)
                runner.blend(list(reversed(poses[3:12])),[],returning=True)
                runner.go(poses[1],returning=True);runner.go(start,returning=True)
            else:
                for goal in reversed(visited[:-1]):runner.go(goal,returning=True)
            runner.go(start,returning=True)
            until=time.monotonic()+2
            while time.monotonic()<until:q=runner.tick()
            if max(abs(a-b) for a,b in zip(q,start))>.2:raise RuntimeError('Folded return not verified')
            report['returned_to_start']=True;report['before_disable_raw_deg']=q
        with AxisArm() as arm:time.sleep(.5);report['final_raw_deg']=arm.read();report['motors_disabled_verified']=True
    finally:
        w.stop.set()
        for worker in workers:worker.join(timeout=8)
        if runner:report['motion_samples']=runner.rows
        (folder/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
        print('Approach report:',folder/'report.json',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--execute',action='store_true');p.add_argument('--blend',action='store_true');p.add_argument('--sweep',action='store_true');p.add_argument('--fast',action='store_true');p.add_argument('--touch',action='store_true');p.add_argument('--speed-scale',type=float,choices=[1.,2.,4.],default=1.);p.add_argument('--stereo-end',action='store_true');p.add_argument('--stereo-servo',action='store_true');p.add_argument('--forward-m',type=float,choices=[.06,.12,.20,.30],default=.06);p.add_argument('--bend-view',type=float,choices=[0.,6.],default=0.);a=p.parse_args()
    if a.speed_scale!=1 and not a.fast:p.error('--speed-scale requires --fast')
    if a.touch and not a.stereo_servo:p.error('--touch requires --stereo-servo')
    if a.execute:execute(a.forward_m,a.bend_view,a.stereo_end,a.stereo_servo,a.fast,a.touch,a.speed_scale,a.blend,a.sweep)
    else:
        q=np.degrees(Geometry().profile['reference_raw_rad']).tolist();poses,_=route(q,a.forward_m,a.bend_view)
        print(json.dumps({'preview':True,'speed_scale':a.speed_scale,'cruise_cap_deg_s':12.*a.speed_scale,'nominal_lift_m':.05,'nominal_forward_m':a.forward_m,'max_raw_joint_excursion_deg':np.max(abs(np.array(poses)-q),axis=0).tolist()}))
