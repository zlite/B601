"""Refresh only a stationary demo entry; reuse hash-bound unchanged route evidence."""
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import numpy as np
from arm_geometry import Geometry
from printer_approach import link_transforms
from blended_motion import BlendedPath

ROOT=Path(__file__).resolve().parent
REST_TOLERANCE_DEG=np.array([.75,.75,.75,.75,.75,1.5])

def read(path):return json.loads(Path(path).read_text())
def sha(path):return hashlib.sha256(Path(path).read_bytes()).hexdigest()
def write(path,value):Path(path).write_text(json.dumps(value,indent=2)+'\n')


def prepare_entry(source,destination,start_deg,gripper_rad,*,confirmed_new_rest=False):
    """Offline entry rebuild; an explicitly confirmed new rest needs fresh CAD checks.

    This never enables motors. Live admission in PreparedDemoPlan.refresh and
    DemoController.reason continues to use the unchanged REST_TOLERANCE_DEG.
    """
    from printer_demo_replay import load_plan
    source=Path(source);destination=Path(destination);destination.mkdir(parents=True,exist_ok=False)
    plan,_,_,_=load_plan(source)
    start=np.asarray(start_deg,float)
    if start.shape!=(6,) or not np.isfinite(start).all():
        raise ValueError('Invalid resting pose')
    if np.any(abs(start-plan['start_raw_deg'])>REST_TOLERANCE_DEG) and confirmed_new_rest is not True:
        raise ValueError('Arm is outside the recorded resting-pose tolerance (0.75 degrees; roll 1.5 degrees)')
    if not np.isfinite(gripper_rad) or abs(gripper_rad-plan['gripper_raw_rad'])>.003:
        raise ValueError('Gripper opening changed')
    plan.update(start_raw_deg=start.tolist(),gripper_raw_rad=float(gripper_rad),start_capture='Fresh disabled motor readback in demo report')
    entry=plan['segments'][0]
    entry['points'][0]=start.tolist();entry['chunks'][0][0]=start.tolist()
    path=BlendedPath(entry['chunks'][0],3.,9.);entry['planned_duration_s']=path.duration
    write(destination/'simplified_plan.json',plan)
    old=read(source/'curve_stock_input.json');n=sum(r['stage']=='entry' for r in old['rows'])
    if any(r['stage']=='entry' for r in old['rows'][n:]):raise ValueError('Unexpected source entry ordering')
    end=np.linalg.inv(np.array(old['rows'][0]['transforms']['link6']))@old['rows'][0]['transforms']['end_link']
    geometry=Geometry();rows=[]
    for t in np.linspace(0,path.duration,max(20,int(np.ceil(path.duration*3/.25))+1)):
        q=np.radians(path.at(t));tf=link_transforms(geometry,q);tf['end_link']=tf['link6']@end
        rows.append(dict(stage='entry',chunk=0,curve_time_s=float(t),fraction=float(t/path.duration),raw_rad=q.tolist(),transforms={k:v.tolist() for k,v in tf.items()}))
    meta={k:v for k,v in old.items() if k!='rows'};meta['plan_sha256']=sha(destination/'simplified_plan.json')
    write(destination/'entry_stock_input.json',{**meta,'rows':rows})
    write(destination/'curve_stock_input.json',{**meta,'rows':rows+old['rows'][n:]})
    # Reconstruct the same registered custom boxes from the hash-bound source row.
    for kind in ('tool','camera'):
        prior=read(source/f'curve_{kind}_input.json');oldrow=prior['rows'][0]
        inv=np.linalg.inv(np.asarray(oldrow['transforms']['link6']));local=[]
        for b in oldrow['custom_boxes']:
            local.append({**b,'center':inv[:3,:3]@b['center']+inv[:3,3],'axes':inv[:3,:3]@b['axes']})
        custom=[]
        for row in rows:
            tf=np.asarray(row['transforms']['link6']);boxes=[]
            for b in local:boxes.append({**b,'center':(tf[:3,:3]@b['center']+tf[:3,3]).tolist(),'axes':(tf[:3,:3]@b['axes']).tolist()})
            custom.append({**row,'custom_boxes':boxes})
        write(destination/f'entry_{kind}_input.json',{'meshes':prior['meshes'],'rows':custom})
        write(destination/f'curve_{kind}_input.json',{'meshes':prior['meshes'],'rows':custom+prior['rows'][n:]})
    proof={'source':str(source),'old_entry_samples':n,'new_entry_samples':len(rows),
           'operator_confirmed_new_rest':confirmed_new_rest is True,
           'previous_start_raw_deg':read(source/'simplified_plan.json')['start_raw_deg'],
           'previous_input_sha256':sha(source/'curve_stock_input.json'),
           'unchanged_tail_sha256':hashlib.sha256(json.dumps(old['rows'][n:],sort_keys=True).encode()).hexdigest()}
    write(destination/'entry_refresh_proof.json',proof)
    commands=[['scripts/stock_mesh_self_check.py','stock'],['scripts/custom_tool_mesh_check.py','tool'],['scripts/custom_tool_mesh_check.py','camera']]
    jobs=[]
    with (destination/'entry_checks.log').open('w') as log:
        for script,kind in commands:
            jobs.append(subprocess.Popen([str(ROOT/'.venv-cad/bin/python'),str(ROOT/script),str(destination/f'entry_{kind}_input.json'),str(destination/f'entry_{kind}_check.json')],cwd=ROOT,stdout=log,stderr=log))
        try:
            for job in jobs:
                if job.wait(timeout=45):raise ValueError('Entry collision check failed; inspect entry_checks.log')
        finally:
            for job in jobs:
                if job.poll() is None:job.terminate();job.wait(timeout=3)
    previous=read(source/'curve_stock_check.json');check=read(destination/'entry_stock_check.json')
    if previous['input_sha256']!=proof['previous_input_sha256'] or previous['mesh_sha256']!=check['mesh_sha256']:
        raise ValueError('Source collision evidence changed')
    checked=check['rows']+previous['rows'][n:]
    for i,r in enumerate(checked):r['sample']=i
    check.update(rows=checked,samples=len(checked),samples_below_margin=sum(bool(r['below_margin']) for r in checked),input_sha256=sha(destination/'curve_stock_input.json'),entry_refresh_proof=proof)
    write(destination/'curve_stock_check.json',check)
    for kind in ('tool','camera'):
        previous=read(source/f'curve_{kind}_check.json');check=read(destination/f'entry_{kind}_check.json')
        if previous['near'] or check['near']:raise ValueError(f'{kind} entry clearance failed')
        previous.update(samples=len(checked),input_sha256=sha(destination/f'curve_{kind}_input.json'),entry_refresh_proof=proof)
        write(destination/f'curve_{kind}_check.json',previous)
    review=read(source/'review.json');review.update(plan_sha256=sha(destination/'simplified_plan.json'),fresh_entry_checked=True)
    write(destination/'review.json',review)
    return load_plan(destination)

class PreparedDemoPlan:
    """Validate/cache the fixed route before rail travel; check only fresh entry after."""
    def __init__(self,source):
        import select
        from printer_demo_replay import load_plan
        self.source=Path(source);self.plan,self.limits,self.poses,self.paths=load_plan(self.source)
        self.geometry=Geometry();self.source_sha=sha(self.source/'simplified_plan.json')
        stock=read(self.source/'curve_stock_input.json')
        self.mesh_sha=read(self.source/'curve_stock_check.json')['mesh_sha256']
        self.end=np.linalg.inv(np.array(stock['rows'][0]['transforms']['link6']))@stock['rows'][0]['transforms']['end_link']
        self.worker=subprocess.Popen([str(ROOT/'.venv-cad/bin/python'),str(ROOT/'scripts/demo_entry_worker.py'),str(self.source)],cwd=ROOT,stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,bufsize=1)
        self.counter=0
        if not select.select([self.worker.stdout],[],[],15)[0] or json.loads(self.worker.stdout.readline())!={'ready':True}:
            self.close();raise ValueError('Entry checker did not initialize')
    def close(self):
        if self.worker.poll() is None:
            self.worker.terminate()
            try:self.worker.wait(timeout=3)
            except subprocess.TimeoutExpired:self.worker.kill();self.worker.wait(timeout=3)
        self.worker.stdin.close();self.worker.stdout.close()
    def refresh(self,start_deg,gripper_rad,output=None):
        import select,time
        from printer_demo_replay import collision_times
        began=time.monotonic();start=np.asarray(start_deg,float)
        if start.shape!=(6,) or not np.isfinite(start).all() or np.any(abs(start-self.plan['start_raw_deg'])>REST_TOLERANCE_DEG):raise ValueError('Arm is outside the reviewed resting region')
        if not np.isfinite(gripper_rad) or abs(gripper_rad-self.plan['gripper_raw_rad'])>.003:raise ValueError('Gripper opening changed')
        points=self.paths[0][1][0].points.copy();points[0]=start
        entry=BlendedPath(points,3.,9.);rows=[]
        for t in collision_times(entry):
            q=np.radians(entry.at(t));tf=link_transforms(self.geometry,q);tf['end_link']=tf['link6']@self.end
            rows.append(dict(stage='entry',chunk=0,curve_time_s=float(t),fraction=float(t/entry.duration),raw_rad=q.tolist(),transforms={k:v.tolist() for k,v in tf.items()}))
        self.counter+=1;self.worker.stdin.write(json.dumps({'id':self.counter,'rows':rows})+'\n');self.worker.stdin.flush()
        if not select.select([self.worker.stdout],[],[],10)[0]:raise ValueError('Fresh entry collision check timed out')
        result=json.loads(self.worker.stdout.readline())
        if 'error' in result or result.get('id')!=self.counter:raise ValueError('Fresh entry checker failed: '+str(result.get('error')))
        if result['stock']['mesh_sha256']!=self.mesh_sha or len(result['stock']['rows'])!=len(rows):raise ValueError('Entry mesh evidence changed')
        initial={(item['a'],item['b']) for item in result['stock']['rows'][0]['below_margin']}
        previous={};cleared=set()
        for r in result['stock']['rows']:
            near={}
            for item in r['below_margin']:
                key=(item['a'],item['b'])
                if key not in initial or item['intersects'] or key in cleared or (key in previous and item['distance_m']<previous[key]-1e-7):raise ValueError('Fresh folded entry clearance fails')
                near[key]=item['distance_m']
            cleared.update(set(previous)-set(near));previous.update(near)
        if result['stock']['rows'][-1]['below_margin'] or any(result[k]['near'] for k in ('tool','camera')):raise ValueError('Fresh entry does not recover clearance')
        paths=[(self.paths[0][0],[entry]),*self.paths[1:]]
        limits={i:(min(self.limits[i][0],start[i]),max(self.limits[i][1],start[i])) for i in range(6)}
        poses=[start.tolist(),*self.poses[1:]]
        evidence={'source':str(self.source),'source_plan_sha256':self.source_sha,'geometry_fingerprint':self.geometry.fingerprint,'start_raw_deg':start.tolist(),'gripper_raw_rad':gripper_rad,'entry_points':points.tolist(),'rows':rows,'checks':result,'check_seconds':time.monotonic()-began}
        if output is not None:
            Path(output).mkdir(parents=True,exist_ok=False);write(Path(output)/'fresh_entry.json',evidence)
        return {**self.plan,'start_raw_deg':start.tolist(),'gripper_raw_rad':gripper_rad},limits,poses,paths,evidence
