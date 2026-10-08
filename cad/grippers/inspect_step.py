"""Extract named gripper solids and conservative section boxes; no hardware.

Run with .venv-cad/bin/python. STEP input and all exported geometry use mm.
The reference plate is explicitly excluded from the robot collision model.
"""
import hashlib
import json
from pathlib import Path

import cadquery as cq
import numpy as np

HERE = Path(__file__).resolve().parent
SOURCE = HERE/'B601_clean_braced45_assembly.step'
NAMES = ('LEFT', 'LEFT_PAD', 'RIGHT', 'RIGHT_PAD', 'REFERENCE_PLATE_NOT_FOR_PRINTING')


def bounds(shape):
    b = shape.BoundingBox()
    return np.array([b.xmin, b.ymin, b.zmin]), np.array([b.xmax, b.ymax, b.zmax])


def main():
    assembly = cq.Assembly.load(str(SOURCE), unit='MM')
    result = {'source': str(SOURCE), 'sha256': hashlib.sha256(SOURCE.read_bytes()).hexdigest(),
              'units': 'mm (explicit STEP LENGTH_UNIT)', 'cadquery_version': cq.__version__,
              'components': {}, 'motion_ready': False,
              'scope': 'CAD geometry only; installed transform and physical pad thickness require validation'}
    derived = HERE/'derived'; derived.mkdir(exist_ok=True)
    for name in NAMES:
        obj = assembly.objects[name]
        shape = obj.obj.moved(obj.loc)
        if not shape.isValid() or len(shape.Solids()) != 1:
            raise ValueError(f'Invalid component: {name}')
        lo, hi = bounds(shape)
        path = derived/(name+'.stl')
        if not shape.exportStl(str(path), tolerance=.05, angularTolerance=.1, ascii=False):
            raise RuntimeError(f'Failed STL export: {name}')
        vertices, triangles = shape.tessellate(.05, .1)
        np.savez_compressed(derived/(name+'.npz'), vertices_mm=[v.toTuple() for v in vertices],
                            triangles=triangles)
        boxes = []
        if name in ('LEFT', 'RIGHT'):
            # Exact BRep intersections; boxes cover every slice of the body,
            # including braces and mounts, rather than only the contact pad.
            cuts = np.linspace(lo[0]-.001, hi[0]+.001, 15)
            for left, right in zip(cuts[:-1], cuts[1:]):
                slab = cq.Solid.makeBox(right-left, hi[1]-lo[1]+2, hi[2]-lo[2]+2,
                    cq.Vector(left, lo[1]-1, lo[2]-1))
                section = shape.intersect(slab)
                if not section.Solids():
                    continue
                a, b = bounds(section)
                boxes.append({'min_mm': a.tolist(), 'max_mm': b.tolist()})
        elif name.endswith('_PAD'):
            boxes = [{'min_mm': lo.tolist(), 'max_mm': hi.tolist()}]
        result['components'][name] = {'valid_single_solid': True,
            'is_robot_collision_part': name != 'REFERENCE_PLATE_NOT_FOR_PRINTING',
            'bounds_min_mm': lo.tolist(), 'bounds_max_mm': hi.tolist(),
            'dimensions_mm': (hi-lo).tolist(), 'volume_mm3': shape.Volume(),
            'center_mm': shape.Center().toTuple(), 'triangles': len(triangles),
            'stl': str(path.relative_to(HERE)),
            'stl_sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'section_boxes': boxes}
    result['nominal_pad_mm'] = {'length': 61., 'height': 8., 'thickness': 1.}
    result['nominal_inner_gap_mm'] = 85.48
    result['distal_pad_body_min_z_mm'] = 3.
    result['distal_pad_min_z_mm'] = 4.
    result['nominal_body_below_pad_mm'] = 1.
    (HERE/'inspection.json').write_text(json.dumps(result, indent=2)+'\n')
    print(json.dumps({k: {'dimensions_mm': v['dimensions_mm'], 'triangles': v['triangles'],
                          'collision_boxes': len(v['section_boxes'])}
                      for k, v in result['components'].items()}, indent=2))


if __name__ == '__main__':
    main()
