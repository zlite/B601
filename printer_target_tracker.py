"""Read-only stereo tracking of the movable printer-bed reference.

A measured tag pose is not a validated plate pose or a motion authorization.
The printer's fixed gantry must not be translated with the moving bed.
"""
import cv2
import numpy as np

from printer_approach import rigid


def validate_stationary_capture(readings, frame_times):
    """Require one unchanged arm/jaw configuration bracketing every image."""
    if len(readings) < 3 or not frame_times:
        raise ValueError('Need bracketed joint readings and camera timestamps')
    bounds = np.array([[r['start_s'], r['end_s']] for r in readings], float)
    poses = np.array([r['arm_raw_rad']+[r['gripper_raw_rad']] for r in readings], float)
    stamps = np.asarray(frame_times, float)
    if (poses.shape != (len(readings), 7) or not np.isfinite(poses).all() or
            not np.isfinite(bounds).all() or not np.isfinite(stamps).all() or
            np.any(bounds[:, 1] < bounds[:, 0]) or np.any(bounds[1:, 0] < bounds[:-1, 1])):
        raise ValueError('Invalid or unordered capture telemetry')
    if bounds[0, 1] > min(stamps) or bounds[-1, 0] < max(stamps):
        raise ValueError('Camera exposures are not bracketed by joint readback')
    if np.max(bounds[1:, 1]-bounds[:-1, 0]) > .6:
        raise ValueError('Joint readback gap too long for stationary capture')
    span = np.ptp(poses, axis=0)
    if np.max(span[:6]) > np.deg2rad(.05) or span[6] > .003:
        raise ValueError('Arm or jaw moved during capture; discard the pose association')
    median = np.median(poses, axis=0)
    return {'stationary': True, 'arm_raw_rad': median[:6].tolist(),
            'gripper_raw_rad': float(median[6]), 'joint_span_deg': np.degrees(span[:6]).tolist(),
            'gripper_span_rad': float(span[6]), 'motion_ready': False}


def stationary_joint_pose(history, stamp, max_age=.2):
    """Allow base-frame reporting only with fresh, stationary joint readback.

    This deliberately does not attempt moving-arm timestamp interpolation.
    """
    rows = [(t, q) for t, q in history if abs(t-stamp) <= .3]
    if len(rows) < 3 or abs(rows[-1][0]-stamp) > max_age or rows[-1][0]-rows[0][0] < .15:
        raise ValueError('Need fresh stationary joint history')
    q = np.asarray([x[1] for x in rows], float)
    if q.shape[1:] != (6,) or not np.isfinite(q).all() or np.max(np.ptp(q, axis=0)) > .05:
        raise ValueError('Arm moving or joint history invalid; no base-frame estimate')
    return np.radians(np.median(q, axis=0))


def compare_reference(reference, current, max_translation=.003, max_rotation_deg=3.):
    """Compare poses in the SAME fixed arm-base/rail-position frame."""
    a, b = rigid(reference), rigid(current)
    translation = float(np.linalg.norm(a[:3, 3]-b[:3, 3]))
    rotation = float(np.degrees(np.linalg.norm(cv2.Rodrigues(a[:3, :3].T@b[:3, :3])[0])))
    return {'translation_change_m': translation, 'rotation_change_deg': rotation,
            'requires_replan': translation > max_translation or rotation > max_rotation_deg}


def require_current_reference(reference, observation, now, session, max_age=.25):
    """Reject stale, occluded, restarted or displaced references before planning.

    This gate alone never permits execution. It also cannot establish the
    still-unverified tag-to-plate/fixture transform.
    """
    if (not observation.get('valid') or observation.get('session') != session or
            not np.isfinite(now) or not np.isfinite(observation.get('observation_time', float('nan'))) or
            not 0 <= now-observation['observation_time'] <= max_age):
        raise ValueError('Need a fresh valid reference from this camera session')
    if 'T_base_tag_candidate' not in observation:
        raise ValueError('No stationary base-frame target estimate')
    result = compare_reference(reference, observation['T_base_tag_candidate'])
    if result['requires_replan']:
        raise ValueError('Bed/reference moved: discard the old path and replan')
    return result
