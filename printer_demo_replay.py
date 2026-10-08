"""Staged noncontact replay of a reviewed, unchanged-scene demonstration.

No gripper or rail movement. Preview is the default. Uses the existing paced
arm controller; each stage stops for fresh camera review before continuing.
"""
import argparse
from datetime import datetime, timezone
import hashlib
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import secrets
import threading
import time

import numpy as np

from arm_geometry import Geometry
from axis_follow import AxisArm
from blended_motion import BlendedPath
from camera_check import CameraCheck
from pairing_dashboard import Workbench
from printer_observation_lift import LiftRunner
from rail_jog import RailJog
from replay_timing import TimedCurve
from smooth_demo_path import JoinedPath, LocalTimedCurve


class ReturnRequested(ValueError):
    pass


class ReversedCurve:
    """Retrace the identical checked curve, without fitting another spline."""
    advance = BlendedPath.advance

    def __init__(self, path, elapsed):
        self.path = path
        self.elapsed = float(elapsed)
        if not 0 < self.elapsed <= path.duration:
            raise ValueError('Invalid executed curve extent')
        self.partial = self.elapsed < path.duration-1e-9
        # For an interrupted curve, ease from rest. Quintic remapping has
        # |ds/dt| <= 1.875. Fourfold timing also bounds added acceleration,
        # using |v(t)| <= acceleration*t from the original zero-speed start.
        self.duration = self.elapsed*4 if self.partial else path.duration
        self.max_deviation = path.max_deviation
        self.points = np.array([self.at(0), self.at(self.duration)])

    def at(self, elapsed):
        u = float(np.clip(elapsed/self.duration, 0, 1))
        fraction = u**3*(10-15*u+6*u*u) if self.partial else u
        return self.path.at(self.elapsed*(1-fraction))


def collision_times(path):
    if isinstance(path, JoinedPath):
        maximum=0.
        for k,h in enumerate(np.diff(path.knots)):
            a,b,c,_=path.spline.c[:,k,:]
            for j in range(6):
                xs=[0.,h]
                if abs(a[j])>1e-12:
                    x=-b[j]/(3*a[j])
                    if 0<x<h:xs.append(x)
                maximum=max(maximum,max(abs(3*a[j]*x*x+2*b[j]*x+c[j]) for x in xs))
        count=max(20,int(np.ceil(maximum/.15))+1)
    else:count=max(20,int(np.ceil(path.duration*3/.25))+1)
    return np.linspace(0,path.duration,count)

def timed_path(path, scale=4.):
    return LocalTimedCurve(path,3.*scale) if isinstance(path,JoinedPath) else TimedCurve(path,scale)

def load_plan(folder):
    plan_path = folder/'simplified_plan.json'
    plan = json.loads(plan_path.read_text())
    approval = json.loads((folder/'review.json').read_text())
    if (approval['plan_sha256'] != hashlib.sha256(plan_path.read_bytes()).hexdigest()
            or not approval['scene_unchanged_operator_confirmed']
            or not approval['noncontact_route_reviewed']):
        raise ValueError('Missing exact-plan review')
    geometry = Geometry()
    if plan['geometry_fingerprint'] != geometry.fingerprint:
        raise ValueError('Geometry changed')
    source = json.loads((folder/'curve_stock_input.json').read_text())
    check = json.loads((folder/'curve_stock_check.json').read_text())
    if (source.get('plan_sha256') != approval['plan_sha256'] or
            len(source['rows']) != len(check['rows']) or
            check['input_sha256'] != hashlib.sha256((folder/'curve_stock_input.json').read_bytes()).hexdigest()):
        raise ValueError('Collision evidence mismatch')
    for mesh in source['meshes']:
        if check['mesh_sha256'][mesh['name']] != hashlib.sha256(Path(mesh['mesh']).read_bytes()).hexdigest():
            raise ValueError('Collision mesh changed')
    # The tightly folded supported start must open monotonically. Every later
    # sample must meet the 5 mm stock-mesh diagnostic margin.
    previous, cleared = {}, set()
    for sample, result in zip(source['rows'], check['rows']):
        near = {}
        for item in result['below_margin']:
            key = (item['a'], item['b'])
            if item['intersects'] or sample['stage'] != 'entry' or key in cleared:
                raise ValueError('Route fails stock clearance')
            if key in previous and item['distance_m'] < previous[key]-1e-7:
                raise ValueError('Folded clearance worsens')
            near[key] = item['distance_m']
        cleared.update(set(previous)-set(near))
        previous.update(near)
    for kind in ('tool', 'camera'):
        custom_check=json.loads((folder/f'curve_{kind}_check.json').read_text())
        if (custom_check['near'] or custom_check.get('input_sha256') !=
                hashlib.sha256((folder/f'curve_{kind}_input.json').read_bytes()).hexdigest()):
            raise ValueError(f'{kind} proximity')
    start = np.asarray(plan['start_raw_deg'], float)
    limits = {}
    for i, joint in enumerate(geometry.joints):
        lim = joint.find('limit')
        lo, hi = sorted((np.degrees(float(lim.get(k)))-np.degrees(geometry.offsets[i]))/geometry.signs[i]
                        for k in ('lower', 'upper'))
        limits[i] = (min(start[i], lo+.25), max(start[i], hi-.25))
    paths, poses = [], [start.tolist()]
    for stage in plan['segments']:
        chunk_paths = []
        for chunk_index, points in enumerate(stage['chunks']):
            if np.max(abs(np.asarray(points[0])-poses[-1])) > 1e-7:
                raise ValueError('Discontinuous stage')
            path = JoinedPath(stage['joined_groups']) if stage.get('curve_type')=='joined_corridor_v1' else BlendedPath(points,3.,9.)
            if not np.allclose(path.points,np.asarray(points),atol=1e-10,rtol=0):raise ValueError('Joined path points differ from reviewed plan')
            checked = [r for r in source['rows'] if r['stage']==stage['name'] and r['chunk']==chunk_index]
            times=collision_times(path);count=len(times)
            if len(checked)!=count or not np.allclose(
                    [r['raw_rad'] for r in checked],
                    [np.radians(path.at(t)) for t in times],
                    atol=1e-10,rtol=0):
                raise ValueError('Checked curve differs from executable path')
            if any(np.any(path.points[:,i] < limits[i][0]-.001) or
                   np.any(path.points[:,i] > limits[i][1]+.001) for i in range(6)):
                raise ValueError('Path exceeds calibrated model limits')
            chunk_paths.append(path)
            poses.extend(path.points[1:].tolist())
        paths.append((stage['name'], chunk_paths))
    return plan, limits, poses, paths


class ReplayRunner(LiftRunner):
    def __init__(self, *args, rail, **kwargs):
        super().__init__(*args, **kwargs)
        self.rail = rail
        self.speed = 3.
        self.acceleration = 9.
        self.tracking_limit = 1.5
        self.pacing_lag = 1.
        self.settle_tolerance = np.full(6, .2)
        self.settle_tolerance[5] = .75
        self.pause_requested = False
        self.phase = 'Preparing'
        self.returning = False
        self.return_pauses = 0
        self.dynamic_reviews = 300
        self.executed_curves = []

    def vision_ready(self):
        return (not self.pause_requested and super().vision_ready()
                and self.rail.connected and self.rail.state == 'Idle'
                and not self.rail.error and time.monotonic()-self.rail.status_time < .4
                and abs(self.rail.x_mm) <= .001)

    def tick(self, moving=False, take_up=False):
        if moving and not self.vision_ready():
            if not self.returning:
                raise ValueError('Replay paused: camera, rail or operator stop')
            self.return_pauses += 1
            review(self,self.output,200+self.return_pauses,'return_paused')
        return super().tick(moving=moving,take_up=take_up)

    def blend(self, points, visited, returning=False):
        """Preserve exact curve progress through pacing/camera/operator pauses."""
        path=BlendedPath(points,self.speed,self.acceleration)
        return self.follow_curve(path, visited, returning)

    def follow_curve(self, path, visited, returning=False):
        if np.max(abs(path.at(0)-list(self.targets.values())))>.01:
            raise RuntimeError('Blend start mismatch')
        if np.any(path.points<self.low) or np.any(path.points>self.high):
            raise RuntimeError('Blend leaves reviewed route')
        record = None
        if not returning:
            record = {'path': path, 'elapsed': 0.}
            self.executed_curves.append(record)
        began=previous=time.monotonic();elapsed=0.;phase=self.phase
        while elapsed<path.duration:
            now=time.monotonic()
            if not self.vision_ready() or now-began>max(30.,path.duration*6):
                self.dynamic_reviews+=1
                cause='readiness_or_operator_pause' if not self.vision_ready() else 'pacing_timeout'
                self.pause_cause=cause
                action=review(self,self.output,self.dynamic_reviews,cause)
                if action=='return' and not returning:raise ReturnRequested(cause)
                self.phase=phase;began=previous=time.monotonic()
            now=time.monotonic();dt=now-previous;previous=now
            actual=np.asarray(self.rows[-1]['raw_deg'])
            lag=float(np.max(abs(actual-list(self.targets.values()))))
            advance=min(dt,.06)*max(0.,min(1.,(self.pacing_lag-lag)/.5))
            elapsed=path.advance(elapsed,advance,actual,self.tracking_limit-.5)
            self.targets=dict(enumerate(path.at(elapsed).tolist()))
            if record is not None:record['elapsed']=elapsed
            self.tick(moving=True)
            if not returning and np.max(abs(np.asarray(list(self.targets.values()))-visited[-1]))>.1:
                visited.append(list(self.targets.values()))
        until=time.monotonic()+4
        while True:
            q=self.tick()
            if np.all(abs(np.asarray(q)-path.at(path.duration))<self.settle_tolerance):break
            if time.monotonic()>until:
                self.dynamic_reviews+=1;self.pause_cause='settling_timeout'
                if review(self,self.output,self.dynamic_reviews,'settling_timeout')=='return' and not returning:
                    raise ReturnRequested('settling_timeout')
                until=time.monotonic()+4
        if not returning:visited.append(path.at(path.duration).tolist())
        return {'planned_duration_s':path.duration,'actual_duration_s':time.monotonic()-began,
                'max_curve_deviation_deg':path.max_deviation}

    def return_executed_curves(self, home_entry=None):
        results=[]
        for index in reversed(range(len(self.executed_curves))):
            item=self.executed_curves[index]
            if item['elapsed'] > 1e-9:
                original,extent=item['path'],item['elapsed']
                if index==0 and home_entry is not None:
                    if extent < original.duration-1e-9 or np.max(abs(original.at(original.duration)-home_entry.at(home_entry.duration)))>1e-8:
                        raise RuntimeError('Fixed-home entry does not join the completed route')
                    original,extent=home_entry,home_entry.duration
                path=ReversedCurve(original,extent)
                results.append(self.follow_curve(path,[],returning=True))
        return results


def server_for(runner):
    token = secrets.token_urlsafe(24)
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_): pass
        def do_GET(self):
            route = self.path.split('?', 1)[0]
            if route == '/':
                data = ("<!doctype html><title>Printer approach replay</title>"
                        "<h1>Printer approach replay</h1><p id='state'></p>"
                        "<button onclick=\"fetch('/pause',{method:'POST',headers:{'X-Replay-Token':'"+token+"'}})\">Pause and hold</button>"
                        "<p>Gripper and rail remain stationary. Agent reviews each stopped stage.</p>"
                        "<img id='wrist' width='640'><img id='tripod' width='800'>"
                        "<script>setInterval(async()=>{let s=await(await fetch('/state')).json();"
                        "document.getElementById('state').textContent=s.phase;"
                        "for(let r of ['wrist','tripod'])document.getElementById(r).src='/'+r+'.jpg?t='+Date.now()},300)</script>").encode()
                kind = 'text/html'
            elif route == '/state':
                data = json.dumps({'phase':runner.phase,'pause_requested':runner.pause_requested,
                                   'samples':len(runner.rows),'pause_cause':getattr(runner,'pause_cause',None),
                                   'last_sample':runner.rows[-1] if runner.rows else None}).encode(); kind='application/json'
            elif route in ('/wrist.jpg','/tripod.jpg'):
                with runner.w.lock: data=runner.w.images.get(route[1:-4],b'')
                kind='image/jpeg'
            else:
                self.send_error(404); return
            self.send_response(200);self.send_header('Content-Type',kind)
            self.send_header('Cache-Control','no-store');self.end_headers();self.wfile.write(data)
        def do_POST(self):
            if self.path != '/pause' or self.headers.get('X-Replay-Token') != token:
                self.send_error(403); return
            runner.pause_requested=True
            self.send_response(200);self.end_headers();self.wfile.write(b'ok')
    return ThreadingHTTPServer(('127.0.0.1',8765),Handler)


def review(runner, output, index, label):
    if getattr(runner, 'review_callback', None):
        return runner.review_callback(runner, output, index, label)
    runner.phase = f'Holding for camera review: {label}'
    runner.save_view(f'stage_{index:02d}_{label}')
    (output/'waiting.json').write_text(json.dumps({'stage':index,'label':label,
        'action_file':str(output/f'review_{index:02d}.json'),'motors_holding':True,
        'last_sample':runner.rows[-1] if runner.rows else None})+'\n')
    print('REVIEW',index,label,flush=True)
    while True:
        runner.tick()
        path=output/f'review_{index:02d}.json'
        if path.exists():
            decision=json.loads(path.read_text())
            if decision.get('stage')!=index or decision.get('action') not in ('advance','return'):
                raise ValueError('Invalid stage decision')
            runner.pause_requested=False
            if not runner.vision_ready():
                runner.pause_requested=True
                continue
            return decision['action']


def execute(folder, speed_scale=1.):
    plan,limits,poses,paths=load_plan(folder)
    paths=[(name,[timed_path(p,speed_scale) for p in chunks]) for name,chunks in paths]
    output=folder/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');output.mkdir()
    print('OUTPUT',output,flush=True)
    w=Workbench();w.track_printer_target=True;w.recording_root=output
    w.camera_check=CameraCheck(w.geometry,output/'views')
    rail=RailJog(w,control_enabled=False)
    workers=[threading.Thread(target=w.camera_worker,args=(r,),daemon=True) for r in ('wrist','tripod')]
    workers.append(threading.Thread(target=rail.worker))
    for worker in workers:worker.start()
    report={'gripper_commanded':False,'rail_commanded':False,'returned_to_start':False,
            'motors_disabled_verified':False,'completed_stages':[], 'requested_speed_scale':speed_scale,
            'planned_outbound_s':sum(p.duration for _,cs in paths for p in cs)}
    runner=None;server=None
    try:
        with AxisArm() as arm:
            start=arm.read()
            if np.max(abs(np.asarray(start)-plan['start_raw_deg']))>.1:
                raise ValueError('Start moved; refresh the entry plan before enabling')
            grip=arm.add_motor(7,23,'4310');position=grip.get_register_f32(80,300)
            grip.request_feedback();time.sleep(.01);arm.ctrl.poll_feedback_once()
            if grip.get_state().status_code!=0 or abs(position-plan['gripper_raw_rad'])>.003:
                raise ValueError('Gripper opening or motor state changed')
            runner=ReplayRunner(w,arm,start,limits,output,poses,rail=rail)
            runner.speed=3.*speed_scale
            server=server_for(runner);threading.Thread(target=server.serve_forever,daemon=True).start()
            until=time.monotonic()+30
            while not runner.vision_ready():
                if time.monotonic()>until:raise ValueError('Live cameras/rail not ready')
                if np.max(abs(np.asarray(arm.read())-start))>.05:raise ValueError('Arm moved during startup')
                time.sleep(.02)
            runner.save_view('before')
            (output/'ready.json').write_text(json.dumps({'start_raw_deg':start,'motors_off':True})+'\n')
            print('READY_FOR_START_REVIEW',flush=True)
            while not (output/'start.json').exists():
                if not runner.vision_ready():raise ValueError('Readiness lost before start')
                if np.max(abs(np.asarray(arm.read())-start))>.05:raise ValueError('Arm moved before start')
                time.sleep(.05)
            if json.loads((output/'start.json').read_text()) != {'action':'start','images_reviewed':True}:
                raise ValueError('Invalid start request')
            arm.prepare_group(range(6))
            if np.max(abs(np.asarray(arm.read())-start))>.05 or not runner.vision_ready():
                raise ValueError('Start changed during setup')
            arm.speed_limits={i:12.*speed_scale for i in range(6)}
            arm.command_speed_limits={i:4.5*speed_scale for i in range(6)}
            arm.enable_group(dict(enumerate(start)));runner.last=time.monotonic()
            until=time.monotonic()+1
            while time.monotonic()<until:runner.tick(take_up=True)
            visited=[start];return_requested=False
            for index,(name,chunks) in enumerate(paths):
                runner.phase=f'Moving: {name}'
                for path in chunks:
                    try:
                        runner.follow_curve(path,visited)
                    except ReturnRequested as error:
                        report['pause_reason']=str(error);return_requested=True;break
                    except ValueError as error:
                        report['pause_reason']=str(error)
                        if review(runner,output,100+index,'interrupted')=='return':return_requested=True;break
                        # Resume from last commanded sample by retracing first;
                        # never jump ahead into a partly executed curve.
                        return_requested=True;break
                if return_requested:break
                report['completed_stages'].append(name)
                if review(runner,output,index,name)=='return':break
            runner.phase='Returning along executed path'
            if np.max(abs(np.asarray(list(runner.targets.values()))-visited[-1]))>1e-8:
                visited.append(list(runner.targets.values()))
            runner.returning=True
            report['return_segments']=runner.return_executed_curves()
            report['returned_to_start']=True
            runner.save_view('returned')
            runner.phase='Returned; removing motor power'
        with AxisArm() as reader:
            report['final_raw_deg']=reader.read();report['motors_disabled_verified']=True
        runner.phase='Complete; motors off'
    except BaseException as error:
        report['error']=repr(error)
        raise
    finally:
        w.stop.set()
        if server:server.shutdown();server.server_close()
        for worker in workers:worker.join(timeout=16)
        if runner:report['samples']=runner.rows
        (output/'report.json').write_text(json.dumps(report,indent=2)+'\n')
        print('RESULT',json.dumps({k:v for k,v in report.items() if k!='samples'}),flush=True)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('folder',type=Path);parser.add_argument('--execute',action='store_true')
    parser.add_argument('--speed-scale',type=float,default=1.)
    args=parser.parse_args()
    if args.execute:execute(args.folder,args.speed_scale)
    else:
        _,_,_,paths=load_plan(args.folder)
        paths=[(n,[timed_path(p,args.speed_scale) for p in ps]) for n,ps in paths]
        print(json.dumps({'stages':[{'name':n,'seconds':sum(p.duration for p in ps)} for n,ps in paths],
                          'full_return_seconds':sum(p.duration for _,ps in paths for p in ps),
                          'motion_commanded':False},indent=2))
