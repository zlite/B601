"""Offline plate-frame side-approach planning. Never opens a hardware device.

This module produces a candidate only. It deliberately has no execution option:
printer/fixture registration and installed-tool collision geometry must be
validated before any candidate can become a physical rehearsal.
"""
import argparse
import hashlib
import json
from pathlib import Path
import struct

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation, Slerp

from arm_geometry import Geometry, URDF, origin_matrix
from plate_grasp_geometry import grasp_rotation


def rigid(value):
    matrix = np.asarray(value, float)
    if (matrix.shape != (4, 4) or not np.isfinite(matrix).all() or
            not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-7) or
            not np.allclose(matrix[:3, :3].T @ matrix[:3, :3], np.eye(3), atol=1e-5) or
            np.linalg.det(matrix[:3, :3]) < .999):
        raise ValueError('Expected a finite rigid transform')
    return matrix


def link_transforms(geometry, raw_rad):
    q = np.asarray(raw_rad, float)
    if q.shape != (6,) or not np.isfinite(q).all():
        raise ValueError('Expected six finite raw joint angles')
    result = {'base_link': np.eye(4)}
    transform = np.eye(4)
    for index, (joint, angle) in enumerate(zip(geometry.joints, q*geometry.signs+geometry.offsets), 1):
        transform = transform @ origin_matrix(joint.find('origin'))
        axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
        rotation = np.eye(4)
        rotation[:3, :3] = cv2.Rodrigues(axis*angle)[0]
        transform = transform @ rotation
        result[f'link{index}'] = transform.copy()
    return result


def stl_bounds(path):
    data = Path(path).read_bytes()
    if len(data) < 84:
        raise ValueError('Truncated collision STL')
    count = struct.unpack_from('<I', data, 80)[0]
    if not count or len(data) != 84+50*count:
        raise ValueError('Expected a complete binary collision STL')
    dtype = np.dtype([('normal', '<f4', 3), ('vertices', '<f4', (3, 3)), ('attribute', '<u2')])
    points = np.frombuffer(data, dtype=dtype, count=count, offset=84)['vertices'].reshape(-1, 3)
    if not np.isfinite(points).all():
        raise ValueError('Nonfinite collision mesh')
    return points.min(0), points.max(0), hashlib.sha256(data).hexdigest()


def mesh_boxes():
    """Conservative local boxes of every pinned stock-arm collision mesh."""
    import xml.etree.ElementTree as ET
    root = ET.parse(URDF).getroot()
    result = []
    for name in ['base_link', *[f'link{i}' for i in range(1, 7)]]:
        collision = root.find(f"link[@name='{name}']/collision")
        mesh = collision.find('geometry/mesh')
        path = (URDF.parent/mesh.get('filename')).resolve()
        low, high, checksum = stl_bounds(path)
        scale = np.array(list(map(float, mesh.get('scale', '1 1 1').split())))
        if np.any(scale <= 0): raise ValueError('Unsupported mesh scale')
        low, high = low*scale, high*scale
        transform = origin_matrix(collision.find('origin'))
        center = transform[:3, :3] @ ((low+high)/2) + transform[:3, 3]
        result.append({'name': name, 'center': center, 'axes': transform[:3, :3],
                       'half_size': (high-low)/2, 'mesh_sha256': checksum})
    return result


def box_gap(a, b):
    """Maximum separating-axis gap; <=0 means the two OBBs overlap.

    A positive result is a lower bound on Euclidean separation, not the exact
    distance. Checks all 15 SAT axes, including edge/edge cross products.
    """
    ca, cb = np.asarray(a['center']), np.asarray(b['center'])
    ra, rb = np.asarray(a['axes']), np.asarray(b['axes'])
    ha, hb = np.asarray(a['half_size']), np.asarray(b['half_size'])
    axes = [*ra.T, *rb.T, *[np.cross(x, y) for x in ra.T for y in rb.T]]
    gaps = []
    for axis in axes:
        norm = np.linalg.norm(axis)
        if norm < 1e-10: continue
        axis = axis/norm
        gaps.append(abs((cb-ca)@axis) - abs(ra.T@axis)@ha - abs(rb.T@axis)@hb)
    return float(max(gaps))


def transform_box(box, transform):
    return {**box, 'center': transform[:3, :3]@box['center']+transform[:3, 3],
            'axes': transform[:3, :3]@box['axes']}


def stock_path_clearance(geometry, waypoints_rad, obstacles, margin=.015):
    """Reject stock-arm box intersections along finely sampled joint segments.

    This is only one part of a complete collision check: installed custom tool,
    self-collision, moving obstacles and unobserved scene regions are not covered.
    Both endpoints and intermediate joint configurations are checked. The caller
    must supply registered obstacles; an empty scene never counts as clear.
    """
    q = np.asarray(waypoints_rad, float)
    if q.ndim != 2 or q.shape[1] != 6 or len(q) < 2 or not np.isfinite(q).all():
        raise ValueError('Invalid joint path')
    if not obstacles or not np.isfinite(margin) or margin < .005:
        raise ValueError('Registered obstacles and a clearance margin are required')
    scene = []
    for item in obstacles:
        transform = rigid(item['T_base_obstacle'])
        size = np.asarray(item['size_m'], float)
        if size.shape != (3,) or not np.isfinite(size).all() or np.any(size <= 0):
            raise ValueError('Invalid obstacle dimensions')
        scene.append({'name': item['name'], 'center': transform[:3, 3],
                      'axes': transform[:3, :3], 'half_size': size/2+margin})
    local_boxes = mesh_boxes()
    minimum = float('inf'); closest = None; samples = 0; collisions = []
    for segment, (start, finish) in enumerate(zip(q[:-1], q[1:])):
        # Small joint increments for this diagnostic; no continuous-collision
        # claim is made. Execution would also need tracking-error envelopes.
        count = max(1, int(np.ceil(np.max(abs(finish-start))/np.deg2rad(.1))))
        for u in np.linspace(0, 1, count+1):
            transforms = link_transforms(geometry, start+u*(finish-start)); samples += 1
            for local in local_boxes:
                body = transform_box(local, transforms[local['name']])
                for obstacle in scene:
                    gap = box_gap(body, obstacle)
                    pair = {'link': local['name'], 'obstacle': obstacle['name'],
                            'segment': segment, 'fraction': float(u), 'gap_m': gap}
                    if gap < minimum: minimum, closest = gap, pair
                    if gap <= 0 and len(collisions) < 30: collisions.append(pair)
    return {'sampled_stock_arm_clear': not collisions, 'samples': samples,
            'margin_m': margin, 'minimum_sat_gap_m': minimum, 'closest': closest,
            'first_collisions': collisions, 'motion_ready': False,
            'scope': 'Sampled conservative stock-link boxes only; excludes custom tool, self-collision and unseen geometry'}


def plan(geometry, start_rad, wrist_from_camera, camera_from_plate, tips,
         standoff=.04, alignment_height=.065, retreat=.025, max_joint_excursion=.8):
    """Back out and rise before aligning, then approach in the new plate frame.

    No old base-coordinate target, rest pose, or printer/table height enters
    this solve. Orientation and translation are both solved at every waypoint.
    """
    if not .035 <= standoff <= .080 or not standoff <= alignment_height <= .12:
        raise ValueError('Expected a conservative noncontact planning height')
    if not .015 <= retreat <= .05: raise ValueError('Invalid side-entry retreat')
    if not np.isfinite(max_joint_excursion) or not .2 <= max_joint_excursion <= 1.4:
        raise ValueError('Invalid offline joint-search radius')
    X, P = rigid(wrist_from_camera), rigid(camera_from_plate)
    tips = np.asarray(tips, float)
    if tips.shape != (2, 3) or not np.isfinite(tips).all():
        raise ValueError('Expected two measured distal pad landmarks')
    midpoint = tips.mean(0)
    q = np.asarray(start_rad, float).copy()
    base_camera = geometry.transform(q) @ X
    base_plate = base_camera @ P
    initial_tip_plate = np.linalg.solve(P, np.r_[midpoint, 1])[:3]
    if initial_tip_plate[0] < .070:
        raise ValueError('This candidate entry requires the pads to begin outside the near plate edge')
    # The parallel jaws admit two equivalent in-plane orientations. The new
    # grid's row direction is arbitrary; never request a 180-degree wrist
    # flip merely because its axes differ from an old plate reference.
    # Align the measured pad direction, not the nominal 45-degree bend. The
    # printer rim leaves little vertical room for a tilted 61 mm contact pad.
    # The new plate frame already defines its own surface normal; do not use
    # the historical TABLE_NORMAL_IN_GRID from the former workspace.
    grasp = grasp_rotation(tips, normal_in_grid=np.array([0., 0., 1.]))
    rotations = [base_plate[:3, :3] @ flip @ grasp for flip in (np.eye(3), np.diag([-1., -1., 1.]))]
    target_rotation = min(rotations, key=lambda r: np.linalg.norm(cv2.Rodrigues(base_camera[:3, :3].T@r)[0]))
    outside = max(.110, initial_tip_plate[0]+retreat)
    stages = [
        ('retreat_and_raise', np.array([outside, initial_tip_plate[1], alignment_height]), base_camera[:3, :3]),
        ('align_outside_fixture', np.array([outside, 0., alignment_height]), target_rotation),
        ('side_entry_above_plate', np.array([.025, 0., alignment_height]), target_rotation),
        ('noncontact_standoff', np.array([.025, 0., standoff]), target_rotation),
    ]
    lows, highs, interior_lows, interior_highs = [], [], [], []
    for i, joint in enumerate(geometry.joints):
        limit = joint.find('limit')
        lo, hi = sorted((float(limit.get(k))-geometry.offsets[i])/geometry.signs[i] for k in ('lower', 'upper'))
        if not lo <= q[i] <= hi:
            raise ValueError(f'Starting joint {i+1} is outside the nominal model limits')
        interior_lows.append(lo+.005); interior_highs.append(hi-.005)
        # A manually positioned joint may already be inside the extra margin.
        # Admit its actual start without widening the model's hard bounds;
        # subsequent samples must move inward until the margin is recovered.
        lows.append(max(min(lo+.005, q[i]), q[i]-max_joint_excursion))
        highs.append(min(max(hi-.005, q[i]), q[i]+max_joint_excursion))
    lows, highs = np.array(lows), np.array(highs)
    if np.any(q < lows) or np.any(q > highs): raise ValueError('Starting pose outside local/model joint limits')
    recovery_needed = bool(np.any(q < interior_lows) or np.any(q > interior_highs))
    waypoints = [q.copy()]; stage_rows = []
    initial_rotation = base_camera[:3, :3]
    initial_point = (base_camera @ np.r_[midpoint, 1])[:3]
    for name, point_plate, rotation in stages:
        goal = (base_plate @ np.r_[point_plate, 1])[:3]
        angle = np.linalg.norm(cv2.Rodrigues(initial_rotation.T@rotation)[0])
        count = max(2, int(np.ceil(np.linalg.norm(goal-initial_point)/.001)), int(np.ceil(angle/np.deg2rad(.25))))
        interpolation = Slerp([0, 1], Rotation.from_matrix([initial_rotation, rotation]))
        first = len(waypoints)
        for u in np.linspace(0, 1, count+1)[1:]:
            point = initial_point+(goal-initial_point)*u
            target_r = interpolation(u).as_matrix()
            def error(joints):
                camera = geometry.transform(joints) @ X
                return np.r_[10*((camera @ np.r_[midpoint, 1])[:3]-point),
                             cv2.Rodrigues(target_r.T@camera[:3, :3])[0].ravel()]
            solved = least_squares(error, q, bounds=(lows, highs), max_nfev=150)
            residual = error(solved.x)
            if np.linalg.norm(residual[:3]) > .002 or np.linalg.norm(residual[3:]) > np.deg2rad(.1):
                raise ValueError(f'No accurate bounded IK solution for {name}')
            if max(abs(solved.x-q)) > np.deg2rad(2.):
                raise ValueError(f'IK discontinuity at {name}: step degrees {np.degrees(solved.x-q).round(3).tolist()}')
            if (np.any((q < interior_lows) & (solved.x < q-1e-9)) or
                    np.any((q > interior_highs) & (solved.x > q+1e-9))):
                raise ValueError('Joint near model limit would move outward')
            q = solved.x
            # Shrink any start-only exception as the joint moves inward. Once
            # recovered, the normal margin cannot be entered again.
            lows = np.maximum(lows, np.minimum(interior_lows, q))
            highs = np.minimum(highs, np.maximum(interior_highs, q))
            waypoints.append(q.copy())
        stage_rows.append({'name': name, 'first_waypoint': first,
                           'last_waypoint': len(waypoints)-1, 'tip_target_plate_m': point_plate.tolist()})
        initial_point, initial_rotation = goal, rotation
    # Do not propose re-entering a near-limit starting pose on the way back.
    # Its return candidate ends at the raised outside-fixture pose instead.
    return_stop = stage_rows[0]['last_waypoint'] if recovery_needed else 0
    if recovery_needed and (np.any(waypoints[return_stop] < interior_lows) or
                            np.any(waypoints[return_stop] > interior_highs)):
        raise ValueError('Raised return pose did not recover normal joint margins')
    return {'T_base_plate_candidate': base_plate.tolist(), 'stages': stage_rows,
            'start_tips_plate_m': (np.linalg.inv(P)@np.c_[tips, np.ones(2)].T).T[:, :3].tolist(),
            'waypoints_raw_deg': np.degrees(waypoints).tolist(),
            'return_waypoints_raw_deg': np.degrees(waypoints[return_stop:][::-1]).tolist(),
            'near_limit_start_recovery': recovery_needed,
            'return_destination': 'raised outside-fixture pose' if recovery_needed else 'starting pose',
            'motion_ready': False,
            'offline_joint_search_radius_rad': max_joint_excursion,
            'unresolved': ['Plate pose seed needs independent stereo validation',
                           'Register printer bed, fixture, adjacent container and gantry collision volumes',
                           'Validate installed finger, camera mount and cable collision envelopes',
                           'Check complete swept arm/tool volumes and tracking-error margins against the registered scene'],
            'scope': 'Offline noncontact IK candidate only; no collision-free or motion-ready claim'}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--plate-reference', type=Path, required=True)
    parser.add_argument('--joint-inspection', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    reference = json.loads(args.plate_reference.read_text())
    inspection = json.loads(args.joint_inspection.read_text())
    X = json.loads(Path('calibration/stereo_sweep_handeye_20261004.json').read_text())['T_wrist_camera']
    tips = json.loads(Path('outputs/gripper/20261004T204827125655Z/jaw_geometry.json').read_text())[-1]['points_B_m']
    result = plan(Geometry(), inspection['arm_raw_rad'], X, reference['T_camera_b_plate'], tips)
    boxes = mesh_boxes()
    result['stock_arm_collision_meshes'] = [{k: v.tolist() if isinstance(v, np.ndarray) else v for k, v in b.items()} for b in boxes]
    result['sources'] = {'plate_reference': str(args.plate_reference), 'joint_inspection': str(args.joint_inspection)}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({'output': str(args.output), 'waypoints': len(result['waypoints_raw_deg']),
                      'stages': result['stages'], 'motion_ready': False, 'unresolved': result['unresolved']}, indent=2))


if __name__ == '__main__':
    main()
