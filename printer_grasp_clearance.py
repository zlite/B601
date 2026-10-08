"""Offline pad/rim diagnostics. Never authorizes or commands robot motion."""
import numpy as np

from plate_grasp_geometry import MEASURED_LONG_AXIS
from printer_approach import box_gap, rigid, transform_box


def pad_boxes(tips, *, length, height, width):
    """Simplified contact-pad boxes in camera B; excludes finger necks.

    Tips must be distal face-center landmarks, not backing-seam corners.
    Dimensions must be explicit so old nominal pads cannot be used by default.
    For the installed tool/backing-seam hypothesis use gripper_cad.registered_boxes.
    Opening must remain fixed.
    """
    tips = np.asarray(tips, float)
    if tips.shape != (2, 3) or not np.isfinite(tips).all():
        raise ValueError('Expected two finite distal pad landmarks')
    size = np.array([length, width, height], float)
    if not np.isfinite(size).all() or np.any(size <= 0):
        raise ValueError('Pad dimensions must be positive and finite')
    span = tips[1]-tips[0]
    if np.linalg.norm(span) < .01:
        raise ValueError('Degenerate jaw span')
    span /= np.linalg.norm(span)
    long = MEASURED_LONG_AXIS-span*(MEASURED_LONG_AXIS@span)
    if np.linalg.norm(long) < .1:
        raise ValueError('Degenerate pad direction')
    long /= np.linalg.norm(long)
    axes = np.column_stack([long, span, np.cross(long, span)])
    return [{'name': f'contact_pad_{i}', 'center': tip-long*length/2,
             'axes': axes.copy(), 'half_size': size/2} for i, tip in enumerate(tips)]


def full_face_height_window(exposed_height, *, pad_height, pad_length,
                            pitch_deg=0., height_uncertainty=0.,
                            rim_margin=0., top_margin=0.):
    """Centre-height interval above rim for the entire pad face to fit.

    A negative interval does not rule out partial-face gripping. It means that
    full-height contact cannot meet these particular clearance/error bounds.
    This is a geometric diagnostic, not a grip-force or friction assessment.
    """
    values = np.array([exposed_height, pad_height, pad_length, pitch_deg,
                       height_uncertainty, rim_margin, top_margin], float)
    if not np.isfinite(values).all() or np.any(values[:3] <= 0) or np.any(values[4:] < 0):
        raise ValueError('Invalid pad/rim dimensions or uncertainty')
    if abs(pitch_deg) > 90:
        raise ValueError('Pitch must lie between -90 and 90 degrees')
    angle = np.deg2rad(pitch_deg)
    extent = pad_height*abs(np.cos(angle))+pad_length*abs(np.sin(angle))
    low = extent/2+rim_margin+height_uncertainty
    high = exposed_height-extent/2-top_margin-height_uncertainty
    return {'vertical_extent_m': float(extent),
            'minimum_center_above_rim_m': float(low),
            'maximum_center_above_rim_m': float(high),
            'window_width_m': float(high-low),
            'positive_full_face_window': bool(high > low+1e-12),
            'motion_ready': False}


def camera_parts_path_clearance(geometry, waypoints_rad, wrist_from_camera, parts,
                               obstacles, margin=.015):
    """Sample supplied camera-frame part OBBs against inflated obstacles.

    Excludes unsupplied parts, cables, self-collision and unsupplied scene objects.
    A pass is never a complete tool-clearance or execution authorization.
    """
    q = np.asarray(waypoints_rad, float)
    X = rigid(wrist_from_camera)
    if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or not np.isfinite(q).all():
        raise ValueError('Invalid joint path')
    if not parts or not obstacles or not np.isfinite(margin) or margin < .005:
        raise ValueError('Parts, obstacles and a clearance margin are required')
    scene = []
    for item in obstacles:
        T = rigid(item['T_base_obstacle'])
        size = np.asarray(item['size_m'], float)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('Invalid obstacle size')
        scene.append({'name': item['name'], 'center': T[:3, 3],
                      'axes': T[:3, :3], 'half_size': size/2+margin})
    minimum = float('inf'); closest = None; collisions = []; samples = 0
    for segment, (start, end) in enumerate(zip(q[:-1], q[1:])):
        count = max(1, int(np.ceil(max(abs(end-start))/np.deg2rad(.1))))
        for u in np.linspace(0, 1, count+1):
            T = geometry.transform(start+u*(end-start))@X
            samples += 1
            for part in parts:
                body = transform_box(part, T)
                for obstacle in scene:
                    gap = box_gap(body, obstacle)
                    pair = {'part': part['name'], 'obstacle': obstacle['name'],
                            'segment': segment, 'fraction': float(u), 'gap_m': gap}
                    if gap < minimum:
                        minimum, closest = gap, pair
                    if gap <= 0 and len(collisions) < 30:
                        collisions.append(pair)
    return {'sampled_parts_clear': not collisions, 'samples': samples,
            'minimum_sat_gap_m': minimum, 'closest': closest,
            'first_collisions': collisions, 'obstacle_inflation_m': margin,
            'motion_ready': False,
            'scope': 'Supplied parts only at fixed opening; incomplete tool and scene'}


def pad_path_clearance(geometry, waypoints_rad, wrist_from_camera, pads,
                       obstacles, margin=.015):
    result = camera_parts_path_clearance(geometry, waypoints_rad, wrist_from_camera,
                                         pads, obstacles, margin)
    result['sampled_pads_clear'] = result.pop('sampled_parts_clear')
    result['scope'] = 'Contact pads only at fixed opening; incomplete tool and scene'
    return result
