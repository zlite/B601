"""Detect unexpected settled arm loading during the short plate descent."""
import numpy as np
from rotation_math import rotation_vector

def recovery_error(message):
    # Lazy import avoids a module cycle with recovery's independent load guard.
    from plate_recovery import PlateRecoveryNeeded
    return PlateRecoveryNeeded(message)

def check_release_separation(before,after,normal):
    before=np.asarray(before,float);after=np.asarray(after,float);normal=np.asarray(normal,float)
    if before.shape!=(4,4) or after.shape!=(4,4) or normal.shape!=(3,) or not np.isfinite(np.r_[before.ravel(),after.ravel(),normal]).all():
        raise RuntimeError('Invalid release separation feedback; hold before retreat')
    # Camera origins expressed in the observed plate frame. A retained plate
    # follows the camera, leaving nearly zero relative separation.
    delta=np.linalg.inv(after)[:3,3]-np.linalg.inv(before)[:3,3]
    separation=float(delta@(normal/np.linalg.norm(normal)))
    if separation<.002:
        raise recovery_error('Plate did not separate from the open gripper; hold before retreat')
    return {'release_observed_separation_m':separation}

def check_plate_release(reference,released):
    reference=np.asarray(reference,float);released=np.asarray(released,float)
    if reference.shape!=(4,4) or released.shape!=(4,4) or not np.isfinite(np.r_[reference.ravel(),released.ravel()]).all():
        raise RuntimeError('Invalid plate release pose; hold before retreat')
    position=float(np.linalg.norm(released[:3,3]-reference[:3,3]))
    angle=float(np.degrees(np.linalg.norm(rotation_vector(reference[:3,:3].T@released[:3,:3]))))
    if position>.002 or angle>3.:
        raise recovery_error('Plate shifted during grasp/release; hold before retreat')
    return {'release_plate_translation_m':position,'release_plate_rotation_deg':angle}

def check_descent_load(baseline, current):
    baseline=np.asarray(baseline,float);current=np.asarray(current,float)
    if baseline.shape!=(6,) or current.shape!=(6,) or not np.isfinite(np.r_[baseline,current]).all():
        raise ValueError('Invalid plate descent load feedback')
    delta=current-baseline
    # These are motor torques, not calibrated tool contact forces. Compare
    # settled readings over the short descent; do not apply to route transit.
    thresholds=np.array([1.5,4.,4.,1.5,.8,.5])
    if np.any(abs(delta)>thresholds):
        raise ValueError('Unexpected arm load during plate descent: '+str(np.round(delta,3).tolist()))


def check_touchdown_load(baseline,current,plate_stationary):
    """Direction-specific holder search; caller must verify seated plate."""
    if not plate_stationary:return check_descent_load(baseline,current)
    baseline=np.asarray(baseline,float);current=np.asarray(current,float)
    if baseline.shape!=(6,) or current.shape!=(6,) or not np.isfinite(np.r_[baseline,current]).all():
        raise ValueError('Invalid plate descent load feedback')
    # During downward motion holder support reduces shoulder torque and
    # increases elbow torque. The old symmetric 4 Nm guard fired before the
    # blocked-motion detector could establish contact. Cap only these two
    # directions at 6 Nm; all other relative limits remain unchanged.
    delta=current-baseline
    lower=np.array([-1.5,-6.,-4.,-1.5,-.8,-.5])
    upper=np.array([1.5,4.,6.,1.5,.8,.5])
    if abs(current[1])>13.5 or np.any(delta<lower) or np.any(delta>upper):
        raise ValueError('Unexpected arm load during plate descent: '+str(np.round(delta,3).tolist()))
