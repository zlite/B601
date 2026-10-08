"""Offline FCL diagnostic for precomputed stock-link transforms, in metres.

Run with .venv-cad/bin/python. Input transforms are base <- collision mesh,
not base <- link. This sampled check cannot authorize physical motion.
"""
import argparse
import hashlib
import itertools
from functools import lru_cache
import json
from pathlib import Path

import fcl
import numpy as np
import trimesh


@lru_cache(maxsize=8)
def prepared_meshes(mesh_json):
    bodies, fingerprints, bounds = {}, {}, {}
    for item in json.loads(mesh_json):
        path = Path(item['mesh'])
        mesh = trimesh.load_mesh(path, process=False)
        scale = np.asarray(item['scale'], float)
        if scale.shape != (3,) or not np.isfinite(scale).all() or (scale <= 0).any():
            raise ValueError('Invalid mesh scale')
        vertices = np.asarray(mesh.vertices)*scale
        if not np.isfinite(vertices).all() or not len(mesh.faces):
            raise ValueError('Invalid mesh')
        model = fcl.BVHModel()
        model.beginModel(len(vertices), len(mesh.faces))
        model.addSubModel(vertices, np.asarray(mesh.faces, dtype=np.int32))
        model.endModel()
        bodies[item['name']] = model
        bounds[item['name']] = (vertices.min(0), vertices.max(0))
        fingerprints[item['name']] = hashlib.sha256(path.read_bytes()).hexdigest()
    return bodies, fingerprints, bounds


def check(data, margin_m=.005):
    if not np.isfinite(margin_m) or margin_m < .005:
        raise ValueError('At least 5 mm diagnostic clearance required')
    names = ['base_link', *[f'link{i}' for i in range(1, 7)]]
    if data['meshes'] and data['meshes'][-1]['name'] == 'end_link':
        names.append('end_link')
    if [m['name'] for m in data['meshes']] != names or not data['rows']:
        raise ValueError('Expected the seven ordered stock links, optional fixed gripper base, and samples')
    bodies, fingerprints, bounds = prepared_meshes(json.dumps(data['meshes'],sort_keys=True))
    # The stock gripper base is fixed to link6; they form one rigid body.
    # That rigid body shares the engineered joint6 with link5.
    body_index = {name: (6 if name == 'end_link' else i) for i, name in enumerate(names)}
    pairs = [(a, b) for a, b in itertools.combinations(names, 2)
             if abs(body_index[a]-body_index[b]) > 1]
    rows, closest_all = [], None
    for index, sample in enumerate(data['rows']):
        objects, world_bounds = {}, {}
        for name in names:
            t = np.asarray(sample['transforms'][name], float)
            if (t.shape != (4, 4) or not np.isfinite(t).all() or
                    not np.allclose(t[3], [0, 0, 0, 1]) or
                    not np.allclose(t[:3, :3].T@t[:3, :3], np.eye(3), atol=1e-6) or
                    np.linalg.det(t[:3, :3]) < .999999):
                raise ValueError('Invalid collision transform')
            objects[name] = fcl.CollisionObject(bodies[name], fcl.Transform(t[:3, :3], t[:3, 3]))
            lo, hi = bounds[name]
            center = t[:3, :3]@((lo+hi)/2)+t[:3, 3]
            half = np.abs(t[:3, :3])@((hi-lo)/2)
            world_bounds[name] = (center-half, center+half)
        near = []
        for a, b in pairs:
            alo, ahi = world_bounds[a]
            blo, bhi = world_bounds[b]
            lower_bound = float(np.linalg.norm(np.maximum(0, np.maximum(alo-bhi, blo-ahi))))
            # Disjoint world AABBs give a conservative lower separation bound.
            # Skip only when neither the margin result nor global minimum can change.
            threshold = max(margin_m, closest_all['distance_m']) if closest_all else float('inf')
            if lower_bound > threshold+1e-9:
                continue
            collision = fcl.CollisionResult()
            hit = fcl.collide(objects[a], objects[b],
                              fcl.CollisionRequest(), collision)
            distance = 0. if hit else float(fcl.distance(objects[a], objects[b],
                                                        fcl.DistanceRequest(), fcl.DistanceResult()))
            if not np.isfinite(distance) or distance < 0:
                raise ValueError('Invalid separation result')
            result = {'a': a, 'b': b, 'distance_m': distance, 'intersects': bool(hit)}
            if closest_all is None or distance < closest_all['distance_m']:
                closest_all = {'sample': index, **result}
            if distance < margin_m:
                near.append(result)
        rows.append({'sample': index, 'below_margin': near})
    return {'sampled_stock_self_clear': not any(r['below_margin'] for r in rows),
            'margin_m': margin_m, 'closest': closest_all, 'samples': len(rows),
            'samples_below_margin': sum(bool(r['below_margin']) for r in rows),
            'mesh_sha256': fingerprints, 'rows': rows, 'motion_ready': False,
            'scope': 'Sampled stock nonadjacent triangle surfaces only; excludes solid containment, custom tool, scene, tracking error and model uncertainty'}


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path)
    parser.add_argument('output', type=Path)
    args = parser.parse_args()
    result = check(json.loads(args.input.read_text()))
    result['input_sha256'] = hashlib.sha256(args.input.read_bytes()).hexdigest()
    args.output.write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: v for k, v in result.items() if k not in ('rows', 'mesh_sha256')}, indent=2))
