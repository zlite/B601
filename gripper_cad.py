"""Offline installed-finger registration hypothesis from supplied STEP geometry.

The two stereo landmarks and pad direction constrain a candidate, not a validated
tool calibration. In particular the landmarks' exact location on the rounded pad
end remains uncertain. Pad dimensions are operator measurements; their vertical
placement and the body registration remain hypotheses. No hardware access.
"""
import hashlib
import json
from pathlib import Path

import numpy as np

from plate_grasp_geometry import MEASURED_LONG_AXIS
from printer_approach import rigid

ROOT = Path(__file__).resolve().parent
CAD = ROOT/'cad/grippers'
COLLISION_PARTS = ('LEFT', 'LEFT_PAD', 'RIGHT', 'RIGHT_PAD')
PAD_SPEC = json.loads((CAD/'installed_pads.json').read_text())


def installed_pad_bounds(name):
    """Measured uncompressed pads, provisionally centered on the plastic face."""
    if name not in ('LEFT_PAD', 'RIGHT_PAD'):
        raise ValueError('Expected a named installed pad')
    p = PAD_SPEC
    x = p['distal_x_mm']
    y = p['backing_abs_y_mm']
    z = p['center_z_mm_candidate']
    low = np.array([x, y-p['uncompressed_thickness_mm'], z-p['height_mm']/2])
    high = np.array([x+p['length_mm'], y, z+p['height_mm']/2])
    if name == 'RIGHT_PAD':
        low[1], high[1] = -high[1], -low[1]
    return low*.001, high*.001


def finger_registration(tips, anchor_height_m=.008):
    tips = np.asarray(tips, float)
    if tips.shape != (2, 3) or not np.isfinite(tips).all():
        raise ValueError('Expected two finite distal landmarks')
    low, high = installed_pad_bounds('LEFT_PAD')
    if not np.isfinite(anchor_height_m) or not low[2] <= anchor_height_m <= high[2]:
        raise ValueError('Anchor must lie within the candidate installed pad height')
    span = tips[1]-tips[0]
    if np.linalg.norm(span) < .01:
        raise ValueError('Degenerate jaw span')
    span /= np.linalg.norm(span)
    long = MEASURED_LONG_AXIS-span*(MEASURED_LONG_AXIS@span)
    if np.linalg.norm(long) < .1:
        raise ValueError('Degenerate pad axis')
    long /= np.linalg.norm(long)
    # In the supplied assembly the distal pad end is -X; LEFT is +Y.
    # Z points up from the reference plate, away from the holder.
    R = np.column_stack([-long, -span, np.cross(long, span)])
    result = {}
    for name, tip, sign in [('LEFT', tips[0], 1), ('RIGHT', tips[1], -1)]:
        anchor = np.array([low[0], sign*PAD_SPEC['backing_abs_y_mm']*.001, anchor_height_m])
        T = np.eye(4); T[:3, :3] = R; T[:3, 3] = tip-R@anchor
        result[name] = rigid(T)
    return result


def registered_boxes(tips, anchor_height_range=(.008, .013)):
    """Enclose the two tested landmark-height hypotheses, not a certified bound.

    Pad vertical placement, landmark correspondence and unobserved mounting details
    can introduce further errors. The returned boxes alone never validate them.
    """
    inspection = json.loads((CAD/'inspection.json').read_text())
    checksum = hashlib.sha256((CAD/'B601_clean_braced45_assembly.step').read_bytes()).hexdigest()
    if checksum != inspection['sha256']:
        raise ValueError('Gripper STEP changed; re-extract geometry before checking')
    low_anchor, high_anchor = anchor_height_range
    pad_low, pad_high = installed_pad_bounds('LEFT_PAD')
    if not pad_low[2] <= low_anchor <= high_anchor <= pad_high[2]:
        raise ValueError('Invalid landmark-height hypothesis range')
    transforms = finger_registration(tips, (low_anchor+high_anchor)/2)
    boxes = []
    for name in COLLISION_PARTS:
        component = inspection['components'][name]
        if not component['is_robot_collision_part']:
            raise ValueError('Unexpected collision component classification')
        T = transforms[name.split('_')[0]]
        # Supplied STEP pad solids are explicitly replaced, never combined with
        # or mistaken for the thicker installed pads.
        bounds = ([installed_pad_bounds(name)] if name.endswith('_PAD') else
                  [(np.array(s['min_mm'])*.001, np.array(s['max_mm'])*.001)
                   for s in component['section_boxes']])
        for i, (low, high) in enumerate(bounds):
            half = (high-low)/2
            half[2] += (high_anchor-low_anchor)/2
            boxes.append({'name': f'{name}_{i}', 'component': name,
                          'center': T[:3, :3]@((low+high)/2)+T[:3, 3],
                          'axes': T[:3, :3].copy(), 'half_size': half})
    return boxes
