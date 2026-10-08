import json,sys
from pathlib import Path
import numpy as np
import fcl,trimesh
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from scripts.stock_mesh_self_check import prepared_meshes

def check(data):
 meshes,_,bounds=prepared_meshes(json.dumps(data['meshes'],sort_keys=True))
 near=[];minimum=None
 for i,row in enumerate(data['rows']):
  for b in row['custom_boxes']:
   center=np.array(b['center']);axes=np.array(b['axes']);half=np.array(b['half_size']);aabbhalf=np.abs(axes)@half;blo,bhi=center-aabbhalf,center+aabbhalf
   body=fcl.CollisionObject(fcl.Box(*(2*half)),fcl.Transform(axes,center))
   for name,mesh in meshes.items():
    if name=='link6':continue
    t=np.array(row['transforms'][name]);lo,hi=bounds[name];mc=t[:3,:3]@((lo+hi)/2)+t[:3,3];mh=np.abs(t[:3,:3])@((hi-lo)/2)
    if np.linalg.norm(np.maximum(0,np.maximum(blo-(mc+mh),(mc-mh)-bhi)))>.005:continue
    obj=fcl.CollisionObject(mesh,fcl.Transform(t[:3,:3],t[:3,3]));hit=fcl.collide(body,obj,fcl.CollisionRequest(),fcl.CollisionResult());d=0 if hit else float(fcl.distance(body,obj,fcl.DistanceRequest(),fcl.DistanceResult()))
    if d<.005:near.append({'sample':i,'stage':row['stage'],'fraction':row['fraction'],'body':b['name'],'link':name,'distance_m':d,'intersects':bool(hit)})
 result={'samples':len(data['rows']),'near':near,'motion_ready':False,'scope':'Coarsely sampled hypothetical installed finger/pad boxes versus stock links 0-5; camera, gripper motor, cables and external scene excluded'}
 return result

if __name__=='__main__':
 p=Path(sys.argv[1]);result=check(json.loads(p.read_text()))
 Path(sys.argv[2]).write_text(json.dumps(result,indent=2)+'\n');print('near pairs',len(result['near']))
