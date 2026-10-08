"""Offline rebuild ending before the plate-level descent; never commands hardware."""
import argparse
import copy
import json
import sys
from pathlib import Path
import subprocess
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(ROOT))
from arm_geometry import Geometry
from blended_motion import BlendedPath
from printer_approach import link_transforms
from printer_demo_plan import read,write,sha
from printer_demo_replay import load_plan,collision_times
from smooth_demo_path import JoinedPath

def shorten(plan,start,gripper):
    plan=copy.deepcopy(plan)
    if not np.isfinite(start).all() or np.asarray(start).shape!=(6,):
        raise ValueError('Invalid disabled start')
    if not np.isfinite(gripper) or abs(gripper-plan['gripper_raw_rad'])>.003:
        raise ValueError('Gripper opening changed')
    if len(plan['segments'])!=3 or len(plan['segments'][1].get('joined_groups',[]))!=5:
        raise ValueError('Expected the reviewed five-group approach and plate-view tail')
    groups=plan['segments'][1]['joined_groups'][:2]
    joined=JoinedPath(groups)
    entry=plan['segments'][0]
    entry['points'][0]=list(start);entry['chunks'][0][0]=list(start)
    first=BlendedPath(entry['chunks'][0],3.,9.)
    entry['planned_duration_s']=first.duration
    plan.update(start_raw_deg=list(start),gripper_raw_rad=gripper)
    plan['segments']=[entry,dict(name='standoff',curve_type='joined_corridor_v1',
        joined_groups=groups,points=joined.points.tolist(),chunks=[joined.points.tolist()],planned_duration_s=None)]
    plan['scope']='Rail and open-jaw standoff/return only. Final descent and plate-level approach removed after observed plate contact.'
    return plan,[first,joined]

def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('source',type=Path);p.add_argument('capture',type=Path);p.add_argument('destination',type=Path)
    args=p.parse_args();source=args.source;dest=args.destination
    old_plan,_,_,_=load_plan(source)
    capture=read(args.capture)
    if capture.get('motion_commanded') is not False or capture.get('status')!=0:
        raise ValueError('Requires disabled read-only gripper/arm capture')
    plan,curves=shorten(old_plan,np.degrees(capture['arm_raw_rad']).tolist(),capture['gripper_raw_rad'])
    dest.mkdir(exist_ok=False,parents=True);write(dest/'simplified_plan.json',plan)
    old=read(source/'curve_stock_input.json')
    end=np.linalg.inv(np.asarray(old['rows'][0]['transforms']['link6']))@old['rows'][0]['transforms']['end_link']
    g=Geometry();rows=[]
    for stage,curve in zip(plan['segments'],curves):
        for t in collision_times(curve):
            q=np.radians(curve.at(t));tf=link_transforms(g,q);tf['end_link']=tf['link6']@end
            rows.append(dict(stage=stage['name'],chunk=0,curve_time_s=float(t),fraction=float(t/curve.duration),raw_rad=q.tolist(),transforms={k:v.tolist() for k,v in tf.items()}))
    write(dest/'curve_stock_input.json',{**{k:v for k,v in old.items() if k!='rows'},'plan_sha256':sha(dest/'simplified_plan.json'),'rows':rows})
    for kind in ('tool','camera'):
        prior=read(source/f'curve_{kind}_input.json');row=prior['rows'][0]
        inv=np.linalg.inv(np.asarray(row['transforms']['link6']))
        local=[{**b,'center':inv[:3,:3]@b['center']+inv[:3,3],'axes':inv[:3,:3]@b['axes']} for b in row['custom_boxes']]
        custom=[]
        for row in rows:
            tf=np.asarray(row['transforms']['link6'])
            boxes=[{**b,'center':(tf[:3,:3]@b['center']+tf[:3,3]).tolist(),'axes':(tf[:3,:3]@b['axes']).tolist()} for b in local]
            custom.append({**row,'custom_boxes':boxes})
        write(dest/f'curve_{kind}_input.json',{'meshes':prior['meshes'],'rows':custom})
    jobs=[]
    with (dest/'checks.log').open('w') as log:
        for kind,script in [('stock','stock_mesh_self_check.py'),('tool','custom_tool_mesh_check.py'),('camera','custom_tool_mesh_check.py')]:
            jobs.append(subprocess.Popen([str(ROOT/'.venv-cad/bin/python'),str(ROOT/'scripts'/script),str(dest/f'curve_{kind}_input.json'),str(dest/f'curve_{kind}_check.json')],stdout=log,stderr=log))
        try:
            for job in jobs:
                if job.wait(timeout=60):raise RuntimeError('Geometry check failed')
        finally:
            for job in jobs:
                if job.poll() is None:job.terminate();job.wait(timeout=3)
    for kind in ('tool','camera'):
        check=read(dest/f'curve_{kind}_check.json');check['input_sha256']=sha(dest/f'curve_{kind}_input.json');write(dest/f'curve_{kind}_check.json',check)
    # Review is intentionally not inherited from the contact-causing route.
    write(dest/'review.json',{'plan_sha256':sha(dest/'simplified_plan.json'),
        'scene_unchanged_operator_confirmed':False,'noncontact_route_reviewed':False,
        'scope':plan['scope'],'source':str(source),'disabled_start_capture':str(args.capture),
        'removed_joined_groups':[2,3,4],'removed_stage':'plate_view',
        'geometry_checks_complete':True,'live_motion_validated':False})
    print('Geometry artifacts ready; explicit scene/route review still required:',dest,flush=True)

if __name__=='__main__':main()
