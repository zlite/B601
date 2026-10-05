#!/usr/bin/env python3
"""Find AprilTag ID 0 without moving the arm; save detections and an annotated image."""
import argparse
import json
import time
from contextlib import ExitStack
from pathlib import Path

import cv2
import numpy as np
from hello_world import Camera
from camera_view import upright, VIEW_ROTATION_DEG

TASK_CONFIG = json.loads((Path(__file__).resolve().parent / 'config/tag_task.json').read_text())

FAMILIES = {name: getattr(cv2.aruco, 'DICT_APRILTAG_' + name)
            for name in ('16h5', '25h9', '36h10', '36h11')}


def detect(frame, family='auto', tag_id=0):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    found = []
    for name in FAMILIES if family == 'auto' else [family]:
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detector = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(FAMILIES[name]), parameters)
        corners, ids, _ = detector.detectMarkers(gray)
        if ids is None:
            continue
        for points, value in zip(corners, ids.flatten()):
            if tag_id is None or int(value) == tag_id:
                xy = points.reshape(4, 2)
                found.append({'family': name, 'id': int(value),
                              'corners_px': xy.tolist(), 'center_px': xy.mean(axis=0).tolist()})
    return found


def detect_small(frame, family='36h11', tag_id=1):
    """Retry a small foreshortened tag at 2x; return original-image pixels."""
    found=detect(frame,family,tag_id)
    if found:return found
    gray=cv2.cvtColor(frame,cv2.COLOR_BGR2GRAY)
    parameters=cv2.aruco.DetectorParameters()
    parameters.cornerRefinementMethod=cv2.aruco.CORNER_REFINE_SUBPIX
    parameters.adaptiveThreshWinSizeMax=53
    parameters.adaptiveThreshWinSizeStep=4
    detector=cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(FAMILIES[family]),parameters)
    corners,ids,_=detector.detectMarkers(cv2.resize(gray,None,fx=2,fy=2))
    if ids is not None:
        for points,value in zip(corners,ids.flatten()):
            if int(value)==tag_id:
                xy=points.reshape(4,2)/2
                found.append({'family':family,'id':int(value),'corners_px':xy.tolist(),'center_px':xy.mean(0).tolist()})
    return found


def annotate(frame, detections):
    result = frame.copy()
    for item in detections:
        points = np.array(item['corners_px'], dtype=np.int32)
        cv2.polylines(result, [points], True, (0, 255, 0), 2)
        cv2.putText(result, f"{item['family']} ID:{item['id']}", tuple(points[0]),
                    cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 255, 0), 1)
    return result


def annotate_view(frame, detections):
    height, width = frame.shape[:2]
    displayed = []
    for item in detections:
        corners = np.array(item['corners_px'], dtype=float)
        corners = np.array([width - 1, height - 1]) - corners
        displayed.append({**item, 'corners_px': corners.tolist(),
                          'center_px': corners.mean(axis=0).tolist()})
    return annotate(upright(frame), displayed)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--image', type=Path, help='Analyze a saved image instead of opening OAK')
    parser.add_argument('--family', choices=['auto', *FAMILIES], default=TASK_CONFIG['tag']['family'])
    parser.add_argument('--id', type=int, default=TASK_CONFIG['tag']['id'])
    parser.add_argument('--seconds', type=float, default=15)
    parser.add_argument('--preview', action='store_true')
    parser.add_argument('--output', type=Path, default=Path('outputs/tag_detection.jpg'))
    args = parser.parse_args()
    if not 0 < args.seconds <= 300:
        parser.error('--seconds must be between 0 and 300')
    if args.image:
        frame = cv2.imread(str(args.image))
        if frame is None:
            parser.error(f'Cannot read {args.image}')
        detections = detect(frame, args.family, args.id)
    else:
        with ExitStack() as stack:
            camera = Camera(stack, False, args.output.with_name('tag_raw.jpg'), output_raw=True)
            deadline = time.monotonic() + args.seconds
            last_count = -1
            detections = []
            while time.monotonic() < deadline:
                camera.poll()
                if last_count != camera.count:
                    last_count = camera.count
                    frame = camera.frame.copy()
                    detections = detect(frame, args.family, args.id)
                    if args.preview:
                        cv2.imshow('AprilTag search (no motion)', annotate_view(frame, detections))
                    if detections or (args.preview and cv2.waitKey(1) & 0xff in (27, ord('q'))):
                        break
                time.sleep(.01)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    display = annotate(frame, detections) if args.image else annotate_view(frame, detections)
    if not cv2.imwrite(str(args.output), display):
        raise RuntimeError('Failed to save annotated image')
    report = {'detections': detections, 'image': str(args.output), 'motion_commanded': False,
              'configured_tag_size_m': TASK_CONFIG['tag']['size_m'],
              'coordinates': 'original_input_image_pixels',
              'image_rotation_deg': 0 if args.image else VIEW_ROTATION_DEG}
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
