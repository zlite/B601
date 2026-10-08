"""Persistent offline entry checker; never accesses motors or rail hardware."""
from pathlib import Path
import sys,json,time
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.stock_mesh_self_check import check as stock_check, prepared_meshes
from scripts.custom_tool_mesh_check import check as custom_check
source=Path(sys.argv[1])
stock=json.loads((source/'curve_stock_input.json').read_text())
custom={}
for kind in ('tool','camera'):
 prior=json.loads((source/f'curve_{kind}_input.json').read_text());row=prior['rows'][0]
 inv=np.linalg.inv(np.asarray(row['transforms']['link6']));local=[]
 for b in row['custom_boxes']:local.append({**b,'center':inv[:3,:3]@b['center']+inv[:3,3],'axes':inv[:3,:3]@b['axes']})
 custom[kind]=(prior['meshes'],local)
for meshes in [stock['meshes'],*[v[0] for v in custom.values()]]:prepared_meshes(json.dumps(meshes,sort_keys=True))
print(json.dumps({'ready':True}),flush=True)
for line in sys.stdin:
 try:
  request=json.loads(line);rows=request['rows'];began=time.monotonic();result={'id':request['id'],'stock':stock_check({'meshes':stock['meshes'],'rows':rows})}
  result['timing']={'stock':time.monotonic()-began}
  for kind,(meshes,local) in custom.items():
   began=time.monotonic()
   transformed=[]
   for row in rows:
    tf=np.asarray(row['transforms']['link6']);boxes=[{**b,'center':(tf[:3,:3]@b['center']+tf[:3,3]).tolist(),'axes':(tf[:3,:3]@b['axes']).tolist()} for b in local]
    transformed.append({**row,'custom_boxes':boxes})
   result[kind]=custom_check({'meshes':meshes,'rows':transformed});result['timing'][kind]=time.monotonic()-began
  print(json.dumps(result),flush=True)
 except Exception as error:print(json.dumps({'error':repr(error)}),flush=True)
