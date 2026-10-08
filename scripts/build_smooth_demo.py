"""Offline geometry evidence for a joined version of the successful demo."""
from pathlib import Path
import sys,json,hashlib,subprocess
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from printer_demo_replay import load_plan,collision_times
from smooth_demo_path import JoinedPath
from printer_approach import link_transforms
from printer_demo_plan import read,write,sha
from arm_geometry import Geometry
source=Path('outputs/printer_replay/20261007_demo/corrected_return_route')
dest=Path('outputs/printer_replay/smooth_demo_v1');dest.mkdir(exist_ok=False)
plan,_,_,paths=load_plan(source);curves=[c for _,cs in paths for c in cs]
groups=[c.points.tolist() for c in curves[1:-1]];joined=JoinedPath(groups)
plan['segments']=[plan['segments'][0],dict(name='smooth_approach',curve_type='joined_corridor_v1',joined_groups=groups,points=joined.points.tolist(),chunks=[joined.points.tolist()],planned_duration_s=None),dict(name='plate_view',points=curves[-1].points.tolist(),chunks=[curves[-1].points.tolist()],planned_duration_s=curves[-1].duration)]
plan['smoothing']={'source':str(source),'max_joint_path_deviation_deg':joined.max_deviation,'original_motion_chunks':len(curves),'new_motion_chunks':3,'rail_commanded':False}
write(dest/'simplified_plan.json',plan)
old=read(source/'curve_stock_input.json');end=np.linalg.inv(np.array(old['rows'][0]['transforms']['link6']))@old['rows'][0]['transforms']['end_link'];g=Geometry();rows=[]
for stage,curve in zip(plan['segments'],[curves[0],joined,curves[-1]]):
 for t in collision_times(curve):
  q=np.radians(curve.at(t));tf=link_transforms(g,q);tf['end_link']=tf['link6']@end
  rows.append(dict(stage=stage['name'],chunk=0,curve_time_s=float(t),fraction=float(t/curve.duration),raw_rad=q.tolist(),transforms={k:v.tolist() for k,v in tf.items()}))
write(dest/'curve_stock_input.json',{**{k:v for k,v in old.items() if k!='rows'},'plan_sha256':sha(dest/'simplified_plan.json'),'rows':rows})
for kind in ('tool','camera'):
 prior=read(source/f'curve_{kind}_input.json');row=prior['rows'][0];inv=np.linalg.inv(np.asarray(row['transforms']['link6']));local=[]
 for b in row['custom_boxes']:local.append({**b,'center':inv[:3,:3]@b['center']+inv[:3,3],'axes':inv[:3,:3]@b['axes']})
 custom=[]
 for row in rows:
  tf=np.asarray(row['transforms']['link6']);boxes=[{**b,'center':(tf[:3,:3]@b['center']+tf[:3,3]).tolist(),'axes':(tf[:3,:3]@b['axes']).tolist()} for b in local]
  custom.append({**row,'custom_boxes':boxes})
 write(dest/f'curve_{kind}_input.json',{'meshes':prior['meshes'],'rows':custom})
print('Checking',len(rows),'curve samples',flush=True)
jobs=[]
with (dest/'checks.log').open('w') as log:
 for kind,script in [('stock','stock_mesh_self_check.py'),('tool','custom_tool_mesh_check.py'),('camera','custom_tool_mesh_check.py')]:
  jobs.append(subprocess.Popen([str(ROOT/'.venv-cad/bin/python'),str(ROOT/'scripts'/script),str(dest/f'curve_{kind}_input.json'),str(dest/f'curve_{kind}_check.json')],stdout=log,stderr=log))
 for job in jobs:
  if job.wait():raise RuntimeError('Geometry check process failed')
for kind in ('tool','camera'):
 check=read(dest/f'curve_{kind}_check.json');check['input_sha256']=sha(dest/f'curve_{kind}_input.json');write(dest/f'curve_{kind}_check.json',check)
review=read(source/'review.json');review.update(plan_sha256=sha(dest/'simplified_plan.json'),smoothed_from_verified_route=True,max_joint_path_deviation_deg=joined.max_deviation)
write(dest/'review.json',review);load_plan(dest)
print('All smoothed route checks passed',flush=True)
