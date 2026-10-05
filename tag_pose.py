#!/usr/bin/env python3
"""Observe the tag's metric pose in camera coordinates; never controls motors."""
import json
import time
from pathlib import Path
import cv2
import depthai as dai
import numpy as np
from tag_view import detect, annotate_view, TASK_CONFIG
from camera_view import VIEW_ROTATION_DEG
from camera_selection import wrist_pipeline, build_wrist_rgb, start_wrist_pipeline, wrist_camera_config


def estimate(corners, matrix, distortion, size):
    half = size / 2
    points = np.array([[-half, half, 0], [half, half, 0],
                       [half, -half, 0], [-half, -half, 0]], dtype=np.float64)
    count, rotations, translations, _ = cv2.solvePnPGeneric(
        points, np.array(corners, dtype=np.float64), np.array(matrix, dtype=np.float64),
        np.array(distortion, dtype=np.float64), flags=cv2.SOLVEPNP_IPPE_SQUARE)
    candidates = []
    for rotation, translation in zip(rotations, translations):
        R = cv2.Rodrigues(rotation)[0]
        if not np.all((points @ R.T + translation.reshape(3))[:, 2] > 0):
            continue
        projected, _ = cv2.projectPoints(points, rotation, translation, np.array(matrix), np.array(distortion))
        rms = float(np.sqrt(np.mean(np.sum((projected.reshape(4, 2) - corners)**2, axis=1))))
        if not np.isfinite(rms):
            continue
        candidates.append({'translation_camera_m': translation.reshape(3).tolist(),
                           'rotation_vector': rotation.reshape(3).tolist(),
                           'reprojection_rms_px': rms})
    return sorted(candidates, key=lambda item: item['reprojection_rms_px'])


def main():
    tag = TASK_CONFIG['tag']
    samples = []
    frames = 0
    detected = 0
    errors = []
    with wrist_pipeline() as pipeline:
        camera = build_wrist_rgb(pipeline)
        queue = camera.requestOutput((1280, 800), type=dai.ImgFrame.Type.BGR888p,
                                     fps=15, enableUndistortion=False).createOutputQueue(maxSize=2, blocking=False)
        start_wrist_pipeline(pipeline, camera)
        deadline = time.monotonic() + 15
        while time.monotonic() < deadline and len(samples) < 20:
            packet = queue.tryGet()
            if packet is not None and packet.getLensPosition() != wrist_camera_config()['manual_focus']:
                packet = None
            if packet is None:
                time.sleep(.01)
                continue
            frame = packet.getCvFrame()
            frames += 1
            detections = detect(frame, tag['family'], tag['id'])
            if len(detections) != 1:
                continue
            detected += 1
            transform = packet.getTransformation()
            matrix = np.array(transform.getIntrinsicMatrix(), dtype=float)
            distortion = np.array(transform.getDistortionCoefficients(), dtype=float)
            model = str(transform.getDistortionModel())
            if not transform.isValid() or not np.isfinite(matrix).all() or matrix[0, 0] <= 0 or matrix[1, 1] <= 0:
                raise RuntimeError('Invalid per-frame camera calibration')
            if 'Perspective' not in model:
                raise RuntimeError(f'Unsupported distortion model: {model}')
            poses = estimate(detections[0]['corners_px'], matrix, distortion, tag['size_m'])
            if poses:
                errors.append(poses[0]['reprojection_rms_px'])
            if poses and poses[0]['reprojection_rms_px'] < 2:
                samples.append({'corners_px': detections[0]['corners_px'], 'candidates': poses,
                                'lens_position': packet.getLensPosition()})
        if len(samples) < 20:
            if frames:
                cv2.imwrite('outputs/tag_pose_debug.jpg', annotate_view(frame, detections))
            raise RuntimeError(f'Only {len(samples)} usable observations; need 20. Frames={frames}, detections={detected}, RMS range={min(errors) if errors else None}..{max(errors) if errors else None}')
        translations = np.array([s['candidates'][0]['translation_camera_m'] for s in samples])
        report = {'tag': tag, 'wrist_camera': wrist_camera_config(),
                  'camera_matrix': matrix.tolist(), 'distortion': distortion.tolist(),
                  'distortion_model': model, 'frame_size': [frame.shape[1], frame.shape[0]],
                  'median_translation_camera_m': np.median(translations, axis=0).tolist(),
                  'translation_std_m': translations.std(axis=0).tolist(), 'samples': samples,
                  'motion_commanded': False, 'contact_target_valid': False,
                  'image_rotation_deg': VIEW_ROTATION_DEG, 'coordinates': 'original_sensor_frame',
                  'note': 'Camera-frame estimate only; planar pose ambiguity retained. No calibrated fingertip or base transform.'}
        Path('outputs').mkdir(exist_ok=True)
        Path('outputs/tag_pose.json').write_text(json.dumps(report, indent=2)+'\n')
        cv2.imwrite('outputs/tag_pose.jpg', annotate_view(frame, detections))
        print(json.dumps({k:v for k,v in report.items() if k != 'samples'}, indent=2))


if __name__ == '__main__':
    main()
