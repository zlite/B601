"""Fresh plate/bed registration and noncontact replanning. No motor access.

Candidate plans never become executable merely because IK succeeds. In
particular, an old replay or an empty/incomplete scene is not a fallback.
"""
import hashlib
import json

import cv2
import numpy as np

from printer_approach import rigid, plan, stock_path_clearance
from printer_grasp_clearance import camera_parts_path_clearance


def fingerprint(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, allow_nan=False).encode()).hexdigest()


def bracketed_pose(history, stamp):
    """Associate an exposure only with stationary readback on both sides."""
    rows = [(t, q) for t, q in history if abs(t-stamp) <= .25]
    times = np.array([t for t, _ in rows], float)
    q = np.asarray([q for _, q in rows], float)
    if (len(rows) < 3 or q.shape != (len(rows), 6) or
            not np.isfinite(q).all() or not np.isfinite(times).all() or
            not times[0] < stamp < times[-1] or
            times[-1]-times[0] < .15 or np.any(np.diff(times) <= 0) or
            np.max(np.diff(times)) > .2 or np.max(np.ptp(q, axis=0)) > .05):
        raise ValueError('Need stationary joint readback bracketing the stereo exposure')
    return np.radians(np.median(q, axis=0))


def register_observation(plate, history, geometry, wrist_camera, rail, now, session):
    """Bed and independently detected plate must share the same stereo pair."""
    stamp = plate.get('observation_time', float('nan'))
    if (plate.get('valid') is not True or not np.isfinite(stamp) or
            not 0 <= now-stamp <= .5 or not session):
        raise ValueError('Need a fresh complete stereo plate grid')
    if plate.get('method') != 'complete_stereo_well_grid' or plate.get('well_count') != 96:
        raise ValueError('Plate must be observed independently; bed offsets are insufficient')
    bed = plate.get('bed_reference', {})
    if (bed.get('valid') is not True or bed.get('observation_time') != stamp or
            bed.get('tag', {}).get('id') != 18 or bed.get('tag', {}).get('family') != '36h11' or
            bed.get('tag', {}).get('size_m') != .018):
        raise ValueError('Need bed tag 18 and plate in the same stereo pair')
    if (rail.get('state') != 'Idle' or rail.get('error') or
            not rail.get('connected') or not np.isfinite(rail.get('x_mm', float('nan'))) or
            not 0 <= now-rail.get('status_time', -1.) <= .4 or
            rail.get('last_motion_time', now) >= stamp):
        raise ValueError('Rail must be stopped throughout target acquisition')
    q = bracketed_pose(history, stamp)
    base_camera = geometry.transform(q) @ rigid(wrist_camera)
    bed_pose = base_camera @ rigid(bed['T_camera_b_tag'])
    plate_pose = base_camera @ rigid(plate['T_camera_b_plate'])
    return dict(valid=True, observation_time=stamp, session=session,
                rail_x_mm=float(rail['x_mm']), rail_revision=rail['commands'],
                arm_raw_rad=q.tolist(), geometry_fingerprint=geometry.fingerprint,
                T_base_bed=bed_pose.tolist(), T_base_plate=plate_pose.tolist(),
                T_bed_plate=np.linalg.solve(bed_pose, plate_pose).tolist(),
                method=plate['method'], motion_ready=False)


def require_fresh(observation, now, session, rail, geometry_fingerprint):
    if (observation.get('valid') is not True or observation.get('session') != session or
            not np.isfinite(now) or
            not 0 <= now-observation.get('observation_time', float('nan')) <= .5):
        raise ValueError('Target expired or camera restarted; reacquire')
    if observation.get('geometry_fingerprint') != geometry_fingerprint:
        raise ValueError('Arm calibration changed; reacquire')
    if (rail.get('state') != 'Idle' or rail.get('error') or not rail.get('connected') or
            not 0 <= now-rail.get('status_time', -1.) <= .4 or
            rail.get('commands') != observation.get('rail_revision') or
            not np.isfinite(rail.get('x_mm', float('nan'))) or
            abs(rail['x_mm']-observation['rail_x_mm']) > .001):
        raise ValueError('Rail frame changed or is stale; reacquire')
    for key in ('T_base_bed', 'T_base_plate', 'T_bed_plate'):
        rigid(observation[key])


def reference_change(before, after):
    """A plate can move independently; accept 180-degree grid-axis symmetry."""
    changes = {}
    for name in ('bed', 'plate'):
        a, b = rigid(before['T_base_'+name]), rigid(after['T_base_'+name])
        rotations = [b[:3, :3]]
        if name == 'plate':
            rotations.append(b[:3, :3] @ np.diag([-1., -1., 1.]))
        angle = min(float(np.degrees(np.linalg.norm(cv2.Rodrigues(a[:3, :3].T@r)[0])))
                    for r in rotations)
        distance = float(np.linalg.norm(a[:3, 3]-b[:3, 3]))
        changes[name] = dict(translation_m=distance, rotation_deg=angle)
    return dict(changes=changes, requires_replan=any(
        v['translation_m'] > .002 or v['rotation_deg'] > 2. for v in changes.values()))


def resolve_scene(scene, observation):
    """Only bed-attached obstacles follow the bed. Plate uses its own pose."""
    if scene.get('registered') is not True or not scene.get('revision'):
        raise ValueError('Register the printer, holder, syringe and tool before planning')
    if (scene.get('rail_x_mm') != observation['rail_x_mm'] or
            scene.get('geometry_fingerprint') != observation['geometry_fingerprint']):
        raise ValueError('Scene belongs to a different arm/rail calibration')
    items = scene.get('obstacles', [])
    required = {'bed', 'holder', 'plate', 'frame', 'gantry', 'syringe', 'bench'}
    if not required.issubset({item['name'] for item in items}):
        raise ValueError('Scene is missing printer/holder/plate/syringe/bench volumes')
    parents = {'base': np.eye(4), 'bed': rigid(observation['T_base_bed']),
               'plate': rigid(observation['T_base_plate'])}
    resolved = []
    for item in items:
        frame = item['frame']; name = item['name']
        if frame not in parents:
            raise ValueError('Unknown obstacle frame')
        expected = ('bed' if name in ('bed', 'holder') else
                    'plate' if name == 'plate' else
                    'base' if name in ('frame', 'gantry', 'syringe', 'bench') else None)
        if expected and frame != expected:
            raise ValueError('Obstacle attached to the wrong frame: '+name)
        size = np.asarray(item['size_m'], float)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('Invalid obstacle size')
        resolved.append(dict(name=name, size_m=size.tolist(),
            T_base_obstacle=(parents[frame] @ rigid(item['T_frame_obstacle'])).tolist()))
    return resolved


def preview(geometry, current_rad, wrist_camera, observation, scene, tool,
            *, now, session, rail):
    """Re-solve a side approach to the measured plate; stop 65 mm above it.

    This is a local plan from a stationary observation pose, not a blind path
    from rest. Full self-collision and camera/cable registration remain separate
    admission checks; a sampled environment pass cannot authorize execution.
    """
    require_fresh(observation, now, session, rail, geometry.fingerprint)
    q = np.asarray(current_rad, float)
    if q.shape != (6,) or not np.isfinite(q).all() or np.max(abs(
            q-np.asarray(observation['arm_raw_rad']))) > np.deg2rad(.05):
        raise ValueError('Arm moved after target capture; reacquire')
    obstacles = resolve_scene(scene, observation)
    if (tool.get('registered') is not True or not tool.get('camera_and_cables_included') or
            not tool.get('parts_camera_b') or not tool.get('revision')):
        raise ValueError('Need measured finger, camera and cable collision envelopes')
    camera = geometry.transform(q) @ rigid(wrist_camera)
    relative = np.linalg.solve(camera, rigid(observation['T_base_plate']))
    midpoint = np.asarray(tool['tips_camera_b_m'], float).mean(axis=0)
    local_tip = np.linalg.solve(relative, np.r_[midpoint, 1])[:3]
    if local_tip[0] < 0:
        # The full grid has no A1 identity. Pick the equivalent long-axis
        # direction facing the tool instead of making a 180-degree detour.
        relative = relative @ np.diag([-1., -1., 1., 1.])
    candidate = plan(geometry, q, wrist_camera, relative, tool['tips_camera_b_m'],
                     standoff=.065, alignment_height=.08)
    path = np.radians(candidate['waypoints_raw_deg'])
    # Inflate by registered scene uncertainty and the maximum 1.5 degree joint
    # tracking displacement over each downstream reach (sum, not maximum).
    reaches = np.asarray(tool.get('downstream_reach_bounds_m', []), float)
    uncertainty = scene.get('uncertainty_m', float('nan'))
    if (reaches.shape != (6,) or not np.isfinite(reaches).all() or np.any(reaches <= 0)
            or not np.isfinite(uncertainty) or not 0 <= uncertainty <= .02):
        raise ValueError('Need scene uncertainty and conservative joint reach bounds')
    margin = .015 + uncertainty + float(np.sum(2*reaches*np.sin(np.deg2rad(1.5)/2)))
    stock = stock_path_clearance(geometry, path, obstacles, margin)
    parts = camera_parts_path_clearance(geometry, path, wrist_camera,
                                       tool['parts_camera_b'], obstacles, margin)
    if not stock['sampled_stock_arm_clear'] or not parts['sampled_parts_clear']:
        raise ValueError('Fresh target path collides with the registered scene')
    candidate.update(observation=observation, scene_sha256=fingerprint(scene),
                     tool_sha256=fingerprint(tool), stock_check=stock, tool_check=parts,
                     motion_ready=False, scope='Fresh-target noncontact preview only',
                     unresolved=['Full self-collision check of this new path',
                                 'Validate scene/tool registration and local acquisition corridor',
                                 'Fresh target recheck before each motion segment',
                                 'Bounded physical rehearsal before automatic execution'])
    candidate['plan_sha256'] = fingerprint(candidate)
    return candidate
