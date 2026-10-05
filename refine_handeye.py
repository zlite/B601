#!/usr/bin/env python3
"""Offline reprojection refinement; saves a candidate, never changes active calibration."""
import json
from pathlib import Path
import cv2
import numpy as np
from scipy.optimize import least_squares
from arm_geometry import Geometry
from calibrate_camera import load_data, solve_samples, residuals

ROOT=Path(__file__).resolve().parent


def pack(T):
    T=np.array(T,dtype=float)
    return np.r_[cv2.Rodrigues(T[:3,:3])[0].ravel(), T[:3,3]]


def unpack(v):
    T=np.eye(4);T[:3,:3]=cv2.Rodrigues(v[:3])[0];T[:3,3]=v[3:]
    return T


def refine(data):
    samples=data['samples']
    initial=solve_samples(samples)
    train=[i for i in range(len(samples)) if i%5!=4]
    held=[i for i in range(len(samples)) if i%5==4]
    half=data['tag']['size_m']/2
    obj=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]])
    if any('corners_px' not in s for s in samples):
        raise ValueError('Raw corner observations required')
    def projected(v,i):
        X,Y=unpack(v[:6]),unpack(v[6:])
        s=samples[i]
        C=np.linalg.inv(X)@np.linalg.inv(np.array(s['T_base_wrist']))@Y
        r=cv2.Rodrigues(C[:3,:3])[0]
        p,_=cv2.projectPoints(obj,r,C[:3,3],np.array(s['camera_matrix']),np.array(s['distortion']))
        return p.reshape(4,2),C
    def residual(v):
        return np.concatenate([(projected(v,i)[0]-np.array(samples[i]['corners_px'])).ravel() for i in train])
    start=np.r_[pack(initial['T_wrist_camera']),pack(initial['T_base_tag_candidate'])]
    fit=least_squares(residual,start,loss='linear',x_scale='jac',max_nfev=300)
    if not fit.success or not np.isfinite(fit.x).all():raise ValueError('Refinement did not converge')
    X,Y=unpack(fit.x[:6]),unpack(fit.x[6:])
    boards=[np.array(s['T_base_wrist'])@X@np.array(s['T_camera_tag']) for s in samples]
    pe,re=residuals([boards[i] for i in held],Y)
    rms=[]
    for i in held:
        p,C=projected(fit.x,i)
        if np.any((obj@C[:3,:3].T+C[:3,3])[:,2]<=0):raise ValueError('Predicted tag is behind camera')
        rms.append(float(np.sqrt(np.mean(np.sum((p-np.array(samples[i]['corners_px']))**2,axis=1)))))
    return {'method':'training-only least-squares corner reprojection refinement',
            'T_wrist_camera':X.tolist(),'T_base_tag_candidate':Y.tolist(),
            'heldout_indices':held,'heldout_translation_errors_m':pe,'heldout_rotation_errors_deg':re,
            'heldout_corner_rms_px':rms,'training_corner_rms_px':float(np.sqrt(np.mean(residual(fit.x)**2)*2)),
            'validation_passed':bool(max(pe)<=.005 and max(re)<=2 and max(rms)<=2),
            'motion_ready':False,'geometry_fingerprint':data['geometry_fingerprint'],
            'note':'Diagnostic refinement; validation views previously inspected. Fresh physical verification and fingertip calibration are still required.'}


def main():
    data=load_data(Geometry());result=refine(data)
    result['wrist_camera']=data['wrist_camera']
    result['session_id']=data.get('session_id')
    output=ROOT/'calibration/arm2_handeye_refined_candidate.json'
    output.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
    print(json.dumps(result,indent=2))

if __name__=='__main__':main()
