"""Offline whole-grid registration from a known hover; never accesses motors.

The seed must preserve well-row identity using the visible end of the plate.
This makes a candidate reference, not permission to replay an old workspace.
"""
import argparse,json
from pathlib import Path
import cv2
import numpy as np
from plate_grid import GridTracker

def register(reference_run,current_image,shift):
    folder=Path(reference_run)
    rows=json.loads((folder/'lower_progress.json').read_text())['rows']
    tracker=GridTracker();P=np.array(rows[0]['T_camera_b_plate']);A=tracker.BA@P
    r=cv2.Rodrigues(A[:3,:3])[0];t=A[:3,3].reshape(3,1)
    uv=cv2.projectPoints(tracker.model,r,t,tracker.K,tracker.D)[0].astype(np.float32)
    old=cv2.imread(str(folder/'hover_wrist.png'),0);new=cv2.imread(str(current_image),0)
    if old is None or new is None or old.shape!=new.shape:raise ValueError('Matching calibrated images required')
    shift=np.asarray(shift,float)
    if shift.shape!=(2,) or not np.isfinite(shift).all() or max(abs(shift))>30:
        raise ValueError('Seed must be a small, row-identity-checked shift')
    mask=np.zeros(old.shape,np.uint8)
    hull=cv2.convexHull(uv.reshape(-1,2)).astype(np.int32)
    cv2.fillConvexPoly(mask,hull,255)
    mask=cv2.dilate(mask,np.ones((15,15),np.uint8))
    H=np.array([[1,0,shift[0]],[0,1,shift[1]],[0,0,1]],np.float32)
    correlation,H=cv2.findTransformECC(old,new,H,cv2.MOTION_HOMOGRAPHY,
        (cv2.TERM_CRITERIA_EPS|cv2.TERM_CRITERIA_COUNT,100,1e-6),mask,5)
    if correlation<.9 or not np.isfinite(H).all():raise ValueError('Whole-grid photometric alignment failed')
    xy=cv2.perspectiveTransform(uv,H).reshape(-1,2)
    if np.max(np.linalg.norm(xy-uv.reshape(-1,2),axis=1))>40:
        raise ValueError('Grid registration exceeded local displacement bound')
    okay,r,t=cv2.solvePnP(tracker.model,xy,tracker.K,tracker.D,r,t,True)
    projected=cv2.projectPoints(tracker.model,r,t,tracker.K,tracker.D)[0].reshape(-1,2)
    rms=float(np.sqrt(np.mean(np.sum((projected-xy)**2,axis=1))))
    if not okay or rms>.5 or t[2,0]<.08:raise ValueError('Registered grid geometry inconsistent')
    A=np.eye(4);A[:3,:3]=cv2.Rodrigues(r)[0];A[:3,3]=t.ravel();B=np.linalg.solve(tracker.BA,A)
    return {'reference_image':str(current_image),'T_camera_b_plate':B.tolist(),
            'camera_matrix':tracker.K.tolist(),'distortion':tracker.D.tolist(),
            'T_camera_a_b':tracker.BA.tolist(),'model':tracker.model.tolist(),'pixels':xy.tolist(),
            'rms_px':rms,'ecc':float(correlation),'homography':H.tolist(),
            'source_run':str(folder),'seed_shift_px':shift.tolist(),
            'live_validated':False,'scope':'Local reacquisition candidate; well-row identity and live tracking must be checked'}

if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--reference-run',required=True,type=Path)
    p.add_argument('--image',required=True,type=Path)
    p.add_argument('--seed-shift-px',required=True,nargs=2,type=float)
    p.add_argument('--output',required=True,type=Path)
    a=p.parse_args();result=register(a.reference_run,a.image,a.seed_shift_px)
    a.output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:result[k] for k in ('ecc','rms_px','live_validated','scope')},indent=2))
