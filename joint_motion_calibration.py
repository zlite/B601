"""Camera-referenced local joint-motion models; never commands hardware."""
import json
import math
from pathlib import Path

import cv2
import numpy as np

NAMES = ['Base','Shoulder','Elbow','Wrist bend','Wrist yaw','Wrist roll']
OFFSETS = [0.,4.,8.,4.,0.,6.,8.,6.,0.]
BRANCHES = ['outward','outward','outward','return','return','outward','outward','return','return']
TRAVEL_SIGNS = [1,-1,-1,-1,-1,1]
DENSE_OFFSETS = [0.,2.,4.,6.,8.,6.,4.,2.,0.,1.,3.,5.,7.,8.,7.,5.,3.,1.,0.]


def load_profile(path, geometry_fingerprint):
    """Load a display-only profile; never alters a control or geometry model."""
    profile=json.loads(Path(path).read_text())
    if profile.get('geometry_fingerprint') != geometry_fingerprint:
        raise ValueError('Joint calibration belongs to a different geometry')
    if profile.get('camera_is_reference') is not True or profile.get('global_mapping_changed') is not False:
        raise ValueError('Expected a camera-referenced local calibration report')
    def finite(value):
        if isinstance(value,float) and not math.isfinite(value):
            raise ValueError('Nonfinite calibration value')
        if isinstance(value,dict):
            for item in value.values():finite(item)
        elif isinstance(value,list):
            for item in value:finite(item)
    finite(profile)
    if not isinstance(profile.get('joints'),list) or not profile['joints']:
        raise ValueError('Calibration report has no joints')
    seen=set()
    for model in profile['joints']:
        joint=model.get('joint')
        if type(joint) is not int or joint not in range(1,7) or joint in seen:
            raise ValueError('Invalid or duplicate joint')
        seen.add(joint)
        for attempt in [model]+model.get('repeat_checks',[]):
            if type(attempt.get('validation_passed')) is not bool:
                raise ValueError('Missing calibration acceptance result')
            if 'curves' not in attempt:
                if attempt['validation_passed'] or not attempt.get('reason'):
                    raise ValueError('Missing calibration measurements')
                continue
            for branch in ('outward','return'):
                curve=attempt['curves'][branch]
                x,y=curve['motor_travel_deg'],curve['camera_travel_deg']
                if len(x)<3 or len(y)!=len(x) or not all(x[i]<x[i+1] for i in range(len(x)-1)):
                    raise ValueError('Invalid measured calibration curve')
            for key in ('endpoint_scale','max_validation_error_deg','max_off_axis_deg',
                        'max_other_joint_drift_deg','midpoint_direction_difference_deg'):
                if type(attempt.get(key)) not in (int,float):
                    raise ValueError('Missing calibration metric: '+key)
            if len(attempt.get('validation',[])) < 4 or len(attempt.get('raw_range_deg',[])) != 2:
                raise ValueError('Incomplete calibration validation')
    return profile


def mean_camera_rotation(sample):
    observations=sample.get('stationary_observations') or [sample]
    rotations=np.array([r['T_camera_tag'] for r in observations])[:,:3,:3]
    u,_,v=np.linalg.svd(rotations.mean(axis=0))
    correction=np.eye(3);correction[2,2]=np.linalg.det(u@v)
    return u@correction@v


def predict(model, raw_deg, branch):
    if branch not in ('outward','return'):
        raise ValueError('Choose outward or return')
    curve=model['curves'][branch]
    x=model['travel_sign']*(raw_deg-model['origin_raw_deg'])
    if not curve['motor_travel_deg'][0] <= x <= curve['motor_travel_deg'][-1]:
        raise ValueError('Outside measured calibration range; no extrapolation')
    return float(np.interp(x,curve['motor_travel_deg'],curve['camera_travel_deg']))


def predict_rotation(model, raw_deg, branch):
    """Return measured relative camera rotation, rejecting extrapolation."""
    if branch not in ('outward','return'):
        raise ValueError('Choose outward or return')
    if model.get('model_type') != 'local_full_rotation':
        raise ValueError('A full-rotation calibration is required')
    curve=model['curves'][branch];xx=np.array(curve['motor_travel_deg'])
    x=model['travel_sign']*(raw_deg-model['origin_raw_deg'])
    if not xx[0]<=x<=xx[-1]:
        raise ValueError('Outside measured calibration range; no extrapolation')
    k=int(np.clip(np.searchsorted(xx,x)-1,0,len(xx)-2))
    t=float((x-xx[k])/(xx[k+1]-xx[k]))
    a,b=np.array(curve['camera_rotation_matrices'])[k:k+2]
    return a@cv2.Rodrigues(t*cv2.Rodrigues(a.T@b)[0])[0]


def fit_joint(joint, samples, travel_sign=None):
    if len(samples)!=9:
        raise ValueError('Need the complete nine-view training and validation sequence')
    q=np.array([s['raw_joint_deg'] for s in samples])
    rotations=[mean_camera_rotation(s) for s in samples]
    vectors=np.array([cv2.Rodrigues(rotations[0]@R.T)[0].ravel() for R in rotations])
    sign=TRAVEL_SIGNS[joint] if travel_sign is None else travel_sign
    x=sign*(q[:,joint]-q[0,joint])
    # Axis and curves are estimated only from the first five training views.
    axis=(vectors[1]+vectors[2])
    if np.linalg.norm(axis)<np.radians(2):
        raise ValueError('Insufficient visible angular motion')
    axis/=np.linalg.norm(axis)
    y=np.degrees(vectors@axis)
    off_axis=np.degrees(np.linalg.norm(vectors-(vectors@axis)[:,None]*axis,axis=1))
    curves={}
    for branch,indices in [('outward',[0,1,2]),('return',[4,3,2])]:
        xx,yy=x[indices],y[indices]
        if np.min(np.diff(xx))<1.5 or np.min(np.diff(yy))<=.2:
            raise ValueError('Measured mapping is not sufficiently monotonic')
        curves[branch]={'motor_travel_deg':xx.tolist(),'camera_travel_deg':yy.tolist()}
    model={'joint':joint+1,'name':NAMES[joint],'origin_raw_deg':float(q[0,joint]),
        'reference_raw_deg':q[0].tolist(),'travel_sign':sign,'camera_axis_in_reference_frame':axis.tolist(),
        'curves':curves,'endpoint_scale':float((y[2]-y[0])/(x[2]-x[0])),
        'scope':'Local angular mapping with other joints held at the recorded reference; outward starts near zero, return starts near the endpoint. No extrapolation.',
        'training_indices':[0,1,2,3,4],'validation_indices':[5,6,7,8]}
    validation=[]
    for i in model['validation_indices']:
        curve=curves[BRANCHES[i]];low,high=curve['motor_travel_deg'][0],curve['motor_travel_deg'][-1]
        outside=max(low-x[i],x[i]-high,0.)
        # Repeated endpoint readings can differ slightly; report this explicitly.
        prediction=float(np.interp(x[i],curve['motor_travel_deg'],curve['camera_travel_deg']))
        validation.append({'index':i,'branch':BRANCHES[i],'motor_travel_deg':float(x[i]),
            'observed_camera_deg':float(y[i]),'predicted_camera_deg':prediction,
            'error_deg':float(y[i]-prediction),'outside_training_range_deg':float(outside)})
    other=np.delete(q-q[0],joint,axis=1)
    midpoint=float((curves['outward']['camera_travel_deg'][1]-curves['return']['camera_travel_deg'][1]))
    worst=max(abs(r['error_deg']) for r in validation)
    coupled=float(np.max(abs(other)))
    off_axis_max=float(np.max(off_axis))
    passed=worst<=.75 and coupled<=.35 and off_axis_max<=1. and max(r['outside_training_range_deg'] for r in validation)<=.35
    model.update(validation=validation,max_validation_error_deg=worst,
        max_other_joint_drift_deg=coupled,max_off_axis_deg=off_axis_max,
        midpoint_direction_difference_deg=midpoint,validation_passed=bool(passed),
        camera_travel_deg=y.tolist(),motor_travel_deg=x.tolist(),
        raw_range_deg=sorted([float(q[0,joint]),float(q[2,joint])]),
        acceptance={'max_validation_error_deg':.75,'max_other_joint_drift_deg':.35,'max_off_axis_deg':1.},
        globally_applied=False)
    return model


def fit_dense_joint(joint, samples, travel_sign=None):
    """Fit a local SO(3) response with 2-degree knots and unseen 1-degree checks.

    Camera rotation need not follow a perfectly fixed axis. Interpolation uses
    geodesics between measured orientations. No validation view enters the fit.
    """
    if len(samples)!=len(DENSE_OFFSETS):
        raise ValueError('Need the complete nineteen-view dense sequence')
    q=np.array([s['raw_joint_deg'] for s in samples])
    rotations=[mean_camera_rotation(s) for s in samples]
    relative=[rotations[0]@R.T for R in rotations]
    vectors=np.array([cv2.Rodrigues(R)[0].ravel() for R in relative])
    sign=TRAVEL_SIGNS[joint] if travel_sign is None else travel_sign;x=sign*(q[:,joint]-q[0,joint])
    axis=vectors[4].copy()
    if np.linalg.norm(axis)<np.radians(2):raise ValueError('Insufficient visible angular motion')
    axis/=np.linalg.norm(axis)
    y=np.degrees(vectors@axis)
    curves={}
    for branch,ids in [('outward',[0,1,2,3,4]),('return',[8,7,6,5,4])]:
        xx,yy=x[ids],y[ids]
        if np.min(np.diff(xx))<.8 or np.min(np.diff(yy))<=.05:
            raise ValueError('Dense angular mapping is not monotonic')
        curves[branch]={'motor_travel_deg':xx.tolist(),'camera_travel_deg':yy.tolist(),
                        'camera_rotation_matrices':[relative[i].tolist() for i in ids]}
    validation=[]
    for i in range(9,len(samples)):
        branch='outward' if i<=13 else 'return';curve=curves[branch]
        xx=np.array(curve['motor_travel_deg']);outside=max(xx[0]-x[i],x[i]-xx[-1],0.)
        k=int(np.clip(np.searchsorted(xx,x[i])-1,0,len(xx)-2))
        t=float(np.clip((x[i]-xx[k])/(xx[k+1]-xx[k]),0,1))
        a,b=np.array(curve['camera_rotation_matrices'])[k:k+2]
        prediction=a@cv2.Rodrigues(t*cv2.Rodrigues(a.T@b)[0])[0]
        error=float(np.degrees(np.linalg.norm(cv2.Rodrigues(prediction.T@relative[i])[0])))
        projection=float(np.degrees(cv2.Rodrigues(prediction)[0].ravel()@axis))
        validation.append({'index':i,'branch':branch,'motor_travel_deg':float(x[i]),
            'observed_camera_deg':float(y[i]),'predicted_camera_deg':projection,
            'error_deg':error,'outside_training_range_deg':float(outside)})
    worst=max(v['error_deg'] for v in validation)
    coupled=float(np.max(abs(np.delete(q-q[0],joint,axis=1))))
    outside=max(v['outside_training_range_deg'] for v in validation)
    return {'joint':joint+1,'name':NAMES[joint],'model_type':'local_full_rotation',
        'origin_raw_deg':float(q[0,joint]),'reference_raw_deg':q[0].tolist(),'travel_sign':sign,
        'camera_axis_in_reference_frame':axis.tolist(),'curves':curves,
        'endpoint_scale':float(y[4]/x[4]),'training_indices':list(range(9)),
        'validation_indices':list(range(9,len(samples))),'validation':validation,
        'max_validation_error_deg':worst,'max_other_joint_drift_deg':coupled,
        'max_off_axis_deg':float(np.max(np.degrees(np.linalg.norm(vectors-(vectors@axis)[:,None]*axis,axis=1)))),
        'midpoint_direction_difference_deg':float(y[2]-y[6]),
        'validation_passed':bool(worst<=.75 and coupled<=.35 and outside<=.35),
        'camera_travel_deg':y.tolist(),'motor_travel_deg':x.tolist(),
        'raw_range_deg':sorted([float(q[0,joint]),float(q[4,joint])]),
        'scope':'Local full camera rotation versus one raw joint, with other joints fixed. Separate outward and return curves; no extrapolation or global application.',
        'acceptance':{'max_validation_error_deg':.75,'max_other_joint_drift_deg':.35,
                      'max_outside_training_range_deg':.35,'error_metric':'SO(3) geodesic rotation error'},
        'globally_applied':False}
