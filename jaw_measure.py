"""Offline stereo tracking of the two distal pad landmarks."""
import json
from pathlib import Path
import cv2
import numpy as np
from contact_geometry import triangulate

BASE=Path('outputs/gripper/20261004T204041141803Z')
PIXELS={'B':[(308.5,225.5),(564.,225.)],'C':[(97.5,268.5),(349.,268.)]}

def measure(folder,index):
    folder=Path(folder);cal=json.loads((folder/'camera_calibration.json').read_text());pixels={};scores={}
    for role,coords in PIXELS.items():
        before=cv2.imread(str(BASE/f'0_{role}.jpg'),0);after=cv2.imread(str(folder/f'{index}_{role}.jpg'),0)
        pixels[role]=[];scores[role]=[]
        for x,y in coords:
            xi,yi=round(x),round(y);template=before[yi-12:yi+10,xi-10:xi+11]
            radius=40;search=after[yi-radius:yi+radius,xi-radius:xi+radius]
            match=cv2.matchTemplate(search,template,cv2.TM_CCOEFF_NORMED)
            # Parallel jaw translation stays horizontal in this fixed rig;
            # exclude neighboring pad texture two rows above the tip edge.
            candidates=match.copy();candidates[:radius-12,:]=-1;candidates[radius-11:,:]=-1
            _,score,_,(dx,dy)=cv2.minMaxLoc(candidates)
            if score<.7 or dx<1 or dy<1 or dx>=match.shape[1]-1 or dy>=match.shape[0]-1:raise ValueError('Pad landmark match ambiguous/outside search')
            def peak(a,b,c):return float(.5*(a-c)/(a-2*b+c))
            sx=peak(match[dy,dx-1],match[dy,dx],match[dy,dx+1]);sy=0.
            pixels[role].append([x+dx-radius+10+sx,y+dy-radius+12+sy]);scores[role].append(score)
    points,errors=triangulate(pixels['B'],pixels['C'],cal['cameras']['B'],cal['cameras']['C'],cal['extrinsics']['B_C'])
    if max(errors)>.6:raise ValueError('Pad stereo reprojection check failed')
    distance=float(np.linalg.norm(points[1]-points[0]))
    return {'index':index,'pixels':pixels,'scores':scores,'points_B_m':points.tolist(),'edge_distance_m':distance,'estimated_inner_gap_m':distance-.00511625,'inner_gap_reference':'80 mm user measurement at baseline, not an independent absolute measurement','reprojection_px':errors.tolist()}

if __name__=='__main__':
    import argparse
    p=argparse.ArgumentParser();p.add_argument('folder',type=Path);a=p.parse_args()
    report=json.loads((a.folder/'report.json').read_text());rows=[]
    for sample in report['samples']:
        row=measure(a.folder,sample['index']);row['q_rad']=sample['actual_rad'];rows.append(row)
        print(row['index'],'q',round(row['q_rad'],4),'estimated inner mm',round(row['estimated_inner_gap_m']*1000,2),'scores',row['scores'])
    (a.folder/'jaw_geometry.json').write_text(json.dumps(rows,indent=2)+'\n')
