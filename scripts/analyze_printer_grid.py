#!/usr/bin/env python3
"""Analyze saved raw printer stereo captures; no device or motion access."""
import argparse
import json
from pathlib import Path
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from printer_plate_vision import detect_wells, stereo_plate_pose


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path, help='Directory containing report.json and B/C images')
    args = parser.parse_args()
    report = json.loads((args.capture/'report.json').read_text())
    results = []
    for index, frame in enumerate(report['frames']):
        cameras = frame['cameras']; pixels = {}
        result = dict(frame=index, valid=False, motion_ready=False)
        try:
            if abs(cameras['B']['observation_monotonic_s']-cameras['C']['observation_monotonic_s']) > .015:
                raise ValueError('Raw camera timestamps exceed stereo synchronization limit')
            for role in ('B', 'C'):
                image = cv2.imread(str(args.capture/cameras[role]['image']))
                if image is None:
                    raise ValueError('Raw stereo image unavailable')
                pixels[role] = detect_wells(image)
                for p in pixels[role]:
                    cv2.circle(image, tuple(np.round(p).astype(int)), 2, (0,255,0), 1)
                cv2.imwrite(str(args.capture/f'{index:02d}_{role}_grid.jpg'), image)
            result.update(stereo_plate_pose(pixels['B'], pixels['C'], cameras['B'], cameras['C'],
                                           report['extrinsics']['B_C']), valid=True)
        except (ValueError, cv2.error) as error:
            result['reason'] = str(error)
        results.append(result)
    output = dict(frames=results, valid_count=sum(r['valid'] for r in results), motion_ready=False)
    (args.capture/'plate_grid_analysis.json').write_text(json.dumps(output, indent=2)+'\n')
    print(json.dumps(output, indent=2))


if __name__ == '__main__': main()
