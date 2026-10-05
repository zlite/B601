#!/usr/bin/env python3
"""Read-only joint, RGB, and stereo-depth checks; never commands motors."""
import argparse
from datetime import datetime, timedelta, timezone
import hashlib
import json
from pathlib import Path
import time

import cv2
import depthai as dai
import numpy as np

from arm_geometry import Geometry
from calibrate_arm import Reader
from camera_selection import wrist_camera_config, wrist_pipeline, build_wrist_rgb, start_wrist_pipeline
from hello_world import PORT
from tag_pose import estimate
from tag_view import TASK_CONFIG, annotate_view, detect
from camera_view import upright, VIEW_ROTATION_DEG

ROOT = Path(__file__).resolve().parent


def compare_tag_depth(depth, corners, matrix, pose):
    """Compare RGB-aligned depth Z with the pose's plane at each interior pixel."""
    corners = np.asarray(corners)
    interior = corners.mean(axis=0) + .6 * (corners - corners.mean(axis=0))
    mask = np.zeros(depth.shape, np.uint8)
    cv2.fillConvexPoly(mask, interior.astype(np.int32), 1)
    valid = (mask != 0) & (depth > 0)
    y, x = np.nonzero(valid)
    fraction = float(valid.sum() / max(1, mask.sum()))
    if len(x) < 20:
        return {'valid_fraction': fraction, 'valid_pixels': len(x)}
    rays = np.linalg.solve(matrix, np.vstack([x, y, np.ones(len(x))])).T
    normal = cv2.Rodrigues(np.asarray(pose['rotation_vector']))[0][:, 2]
    predicted = (normal @ np.asarray(pose['translation_camera_m'])) / (rays @ normal)
    observed = depth[y, x] * .001
    return {'valid_fraction': fraction, 'valid_pixels': len(x),
            'median_depth_z_m': float(np.median(observed)),
            'median_expected_z_m': float(np.median(predicted)),
            'median_signed_error_m': float(np.median(observed - predicted)),
            'median_absolute_error_m': float(np.median(abs(observed - predicted)))}


def run(output):
    output.mkdir(parents=True, exist_ok=True)
    geometry = Geometry()
    report = {'captured_utc': datetime.now(timezone.utc).isoformat(),
              'wrist_camera': wrist_camera_config(), 'geometry_fingerprint': geometry.fingerprint,
              'motion_commanded': False, 'motion_ready': False, 'samples': [],
              'image_rotation_deg': VIEW_ROTATION_DEG, 'coordinates': 'original_sensor_frame'}
    with Reader(PORT) as reader:
        before, spans = reader.capture()
    report['joints_before_raw_rad'] = before
    report['joint_capture_span_rad'] = spans
    report['joint_delta_from_reference_deg'] = np.rad2deg(
        np.array(before) - geometry.profile['reference_raw_rad']).tolist()
    with wrist_pipeline() as pipeline:
        device = pipeline.getDefaultDevice()
        report['connected_sockets'] = [str(s) for s in device.getConnectedCameras()]
        calibration = device.readCalibration()
        report['calibration_sha256'] = hashlib.sha256(device.readCalibrationRaw()).hexdigest()
        report['stereo_baseline_cm'] = calibration.getBaselineDistance(
            dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C, False)
        rgb = build_wrist_rgb(pipeline)
        left = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        right = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        stereo = pipeline.create(dai.node.StereoDepth)
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.DEFAULT)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        stereo.setSubpixel(False)  # Extended disparity with the supported median-filter range.
        left.requestFullResolutionOutput(fps=30).link(stereo.left)
        right.requestFullResolutionOutput(fps=30).link(stereo.right)
        rgb_out = rgb.requestOutput((640, 400), type=dai.ImgFrame.Type.BGR888p,
                                    fps=15, enableUndistortion=True)
        rgb_out.link(stereo.inputAlignTo)
        sync = pipeline.create(dai.node.Sync)
        sync.setSyncThreshold(timedelta(milliseconds=30))
        rgb_out.link(sync.inputs['rgb'])
        stereo.depth.link(sync.inputs['depth'])
        queue = sync.out.createOutputQueue(maxSize=2, blocking=False)
        start_wrist_pipeline(pipeline, rgb)
        deadline = time.monotonic() + 20
        frame_count = 0
        warmup_count = 0
        last_stamp = None
        while time.monotonic() < deadline and frame_count < 60:
            group = queue.tryGet()
            if group is None:
                time.sleep(.005)
                continue
            packet, depth_packet = group['rgb'], group['depth']
            stamp = packet.getTimestamp().total_seconds()
            if last_stamp is not None and stamp <= last_stamp:
                raise RuntimeError('RGB timestamps did not advance')
            last_stamp = stamp
            frame, depth = packet.getCvFrame(), depth_packet.getFrame()
            if depth.shape != frame.shape[:2] or depth.dtype != np.uint16:
                raise RuntimeError('Unexpected RGB/depth shape or depth type')
            transform = packet.getTransformation()
            matrix = np.asarray(transform.getIntrinsicMatrix(), float)
            distortion = np.asarray(transform.getDistortionCoefficients(), float)
            if not transform.isValid() or not np.isfinite(matrix).all() or min(matrix[0,0],matrix[1,1]) <= 0:
                raise RuntimeError('Invalid RGB calibration')
            tags = detect(frame, TASK_CONFIG['tag']['family'], TASK_CONFIG['tag']['id'])
            if warmup_count < 30:
                warmup_count += 1
                continue
            sample = {'rgb_timestamp_s': stamp,
                      'lens_position': packet.getLensPosition(),
                      'sync_difference_ms': abs(stamp-depth_packet.getTimestamp().total_seconds())*1000,
                      'depth_valid_fraction': float(np.mean(depth > 0))}
            if len(tags) == 1:
                poses = estimate(tags[0]['corners_px'], matrix, distortion, TASK_CONFIG['tag']['size_m'])
                if poses:
                    sample['tag_pose'] = poses[0]
                    sample['tag_pose_candidates'] = poses
                    sample['tag_depth'] = compare_tag_depth(depth, tags[0]['corners_px'], matrix, poses[0])
            report['samples'].append(sample)
            frame_count += 1
        if not frame_count:
            raise RuntimeError('No synchronized RGB/depth frames within 20 seconds')
        report['frame_size'] = [frame.shape[1], frame.shape[0]]
        report['rgb_camera_matrix'] = matrix.tolist()
        report['rgb_distortion'] = distortion.tolist()
        cv2.imwrite(str(output/'rgb_tag.jpg'), annotate_view(frame, tags))
        np.save(output/'depth_mm.npy', depth)
        colored = cv2.applyColorMap(np.clip(depth.astype(float)*255/2000,0,255).astype(np.uint8), cv2.COLORMAP_TURBO)
        colored[depth == 0] = 0
        cv2.imwrite(str(output/'depth.png'), upright(colored))
    with Reader(PORT) as reader:
        after, _ = reader.capture()
    report['joints_after_raw_rad'] = after
    report['joint_change_during_check_deg'] = np.rad2deg(np.array(after)-before).tolist()
    tag_samples = [s for s in report['samples'] if 'tag_pose' in s]
    comparisons = [s['tag_depth'] for s in tag_samples if 'median_absolute_error_m' in s['tag_depth']]
    report['summary'] = {'synchronized_frames': frame_count, 'tag_frames': len(tag_samples),
                         'warmup_frames_excluded': warmup_count,
                         'median_depth_valid_fraction': float(np.median([s['depth_valid_fraction'] for s in report['samples']])),
                         'max_sync_difference_ms': max(s['sync_difference_ms'] for s in report['samples'])}
    if tag_samples:
        translations = np.array([s['tag_pose']['translation_camera_m'] for s in tag_samples])
        report['summary'].update(median_tag_translation_camera_m=np.median(translations,axis=0).tolist(),
                                 tag_translation_std_m=translations.std(axis=0).tolist())
    if comparisons:
        report['summary']['median_tag_depth_error_m'] = float(np.median([c['median_absolute_error_m'] for c in comparisons]))
        report['summary']['median_tag_depth_valid_fraction'] = float(np.median([c['valid_fraction'] for c in comparisons]))
    report['sensor_checks_passed'] = bool(
        frame_count >= 30 and len(tag_samples) >= 20 and len(comparisons) >= 20
        and report['summary']['max_sync_difference_ms'] <= 30
        and report['summary'].get('median_tag_depth_error_m', 1) <= .02
        and report['summary'].get('median_tag_depth_valid_fraction', 0) >= .5
        and max(report['summary'].get('tag_translation_std_m', [1])) <= .004
        and max(abs(v) for v in report['joint_change_during_check_deg']) <= .3)
    report['focus_verified'] = bool(all(s['lens_position']==report['wrist_camera']['manual_focus'] for s in report['samples']))
    report['sensor_checks_passed'] = report['sensor_checks_passed'] and report['focus_verified']
    report['starting_pose_near_saved_reference'] = bool(max(abs(v) for v in report['joint_delta_from_reference_deg']) <= 5)
    report['validation_scope'] = 'Stationary sensor and joint-read checks. Depth/tag agreement is a consistency check using configured tag size, not independent metric or hand-eye validation.'
    (output/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print(json.dumps(report['summary'],indent=2), flush=True)
    print(f"Stationary sensor checks passed: {report['sensor_checks_passed']}", flush=True)
    print(f'Saved {output}/report.json; no motion commanded.', flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, default=ROOT/'outputs/wrist_revalidation')
    run(parser.parse_args().output)
