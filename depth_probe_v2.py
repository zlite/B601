#!/usr/bin/env python3
"""Camera-only OAK-D Lite stereo check using the isolated DepthAI 2.32 environment."""
import argparse
from datetime import datetime, timedelta, timezone
import json
from pathlib import Path
import time

import cv2
import depthai as dai
import numpy as np

from camera_selection import wrist_camera_config
from camera_view import upright, VIEW_ROTATION_DEG
from revalidate_setup import compare_tag_depth
from tag_pose import estimate
from tag_view import TASK_CONFIG, annotate_view, detect


def run(output):
    if not dai.__version__.startswith('2.32.'):
        raise RuntimeError('Run with .venv-depth-v2/bin/python depth_probe_v2.py')
    config = wrist_camera_config()
    pipeline = dai.Pipeline()
    rgb = pipeline.create(dai.node.ColorCamera)
    rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb.setIspScale(1, 3)
    rgb.setFps(15)
    rgb.initialControl.setManualFocus(config['manual_focus'])
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    for camera, socket in [(left, dai.CameraBoardSocket.CAM_B), (right, dai.CameraBoardSocket.CAM_C)]:
        camera.setBoardSocket(socket)
        camera.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        camera.setFps(15)
    stereo = pipeline.create(dai.node.StereoDepth)
    stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
    stereo.setLeftRightCheck(True)
    stereo.setExtendedDisparity(True)
    stereo.setSubpixel(False)
    # Validate depth in its native rectified-right frame; RGB alignment is separate.
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    sync = pipeline.create(dai.node.Sync)
    sync.setSyncThreshold(timedelta(milliseconds=30))
    rgb.isp.link(sync.inputs['rgb'])
    stereo.depth.link(sync.inputs['depth'])
    stereo.rectifiedRight.link(sync.inputs['right'])
    out = pipeline.create(dai.node.XLinkOut)
    out.setStreamName('rgb_depth')
    sync.out.link(out.input)
    report = {'captured_utc': datetime.now(timezone.utc).isoformat(),
              'depthai_version': dai.__version__, 'wrist_camera': config,
              'motion_commanded': False, 'motion_ready': False,
              'image_rotation_deg': VIEW_ROTATION_DEG, 'coordinates': 'rectified_right_camera',
              'rgb_depth_aligned': False, 'samples': []}
    output.mkdir(parents=True, exist_ok=True)
    with dai.Device(pipeline, dai.DeviceInfo(config['device_id'])) as device:
        if device.getDeviceName() != config['model']:
            raise RuntimeError('Wrong camera model')
        calibration = device.readCalibration()
        if calibration.getLensPosition(dai.CameraBoardSocket.CAM_A) != config['manual_focus']:
            raise RuntimeError('Focus differs from stored calibration')
        report['usb_speed'] = str(device.getUsbSpeed())
        print(f"Camera: {config['model']}, DepthAI {dai.__version__}, USB {report['usb_speed']}", flush=True)
        matrix = np.array(calibration.getCameraIntrinsics(dai.CameraBoardSocket.CAM_C, 640, 480))
        distortion = np.zeros(5)
        queue = device.getOutputQueue('rgb_depth', maxSize=2, blocking=False)
        deadline = time.monotonic() + 20
        warmup = 0
        while time.monotonic() < deadline and len(report['samples']) < 60:
            group = queue.tryGet()
            if group is None:
                time.sleep(.005)
                continue
            if warmup < 30:
                warmup += 1
                continue
            packet, depth_packet = group['rgb'], group['depth']
            rgb_frame = packet.getCvFrame()
            frame = cv2.cvtColor(group['right'].getFrame(), cv2.COLOR_GRAY2BGR)
            depth = depth_packet.getFrame()
            if frame.shape[:2] != depth.shape:
                raise RuntimeError('RGB/depth dimensions do not match')
            tags = detect(frame, TASK_CONFIG['tag']['family'], TASK_CONFIG['tag']['id'])
            sample = {'lens_position': packet.getLensPosition(),
                      'rgb_timestamp_s': packet.getTimestamp().total_seconds(),
                      'sync_difference_ms': abs((packet.getTimestamp()-depth_packet.getTimestamp()).total_seconds())*1000,
                      'depth_valid_fraction': float(np.mean(depth > 0))}
            if len(tags) == 1:
                poses = estimate(tags[0]['corners_px'], matrix, distortion, TASK_CONFIG['tag']['size_m'])
                if poses:
                    sample['tag_pose'] = poses[0]
                    sample['tag_depth'] = compare_tag_depth(depth, tags[0]['corners_px'], matrix, poses[0])
            report['samples'].append(sample)
        if not report['samples']:
            raise RuntimeError('No synchronized RGB/depth frames')
        cv2.imwrite(str(output/'stereo_tag.jpg'), annotate_view(frame, tags))
        rgb_tags = detect(rgb_frame, TASK_CONFIG['tag']['family'], TASK_CONFIG['tag']['id'])
        cv2.imwrite(str(output/'rgb_tag.jpg'), annotate_view(rgb_frame, rgb_tags))
        np.save(output/'depth_mm.npy', depth)
        colored = cv2.applyColorMap(np.clip(depth.astype(float)*255/2000,0,255).astype(np.uint8), cv2.COLORMAP_TURBO)
        colored[depth == 0] = 0
        cv2.imwrite(str(output/'depth.png'), upright(colored))
        report['camera_matrix'] = matrix.tolist()
        report['distortion'] = distortion.tolist()
        report['frame_size'] = [frame.shape[1], frame.shape[0]]
    samples = report['samples']
    comparisons = [s['tag_depth'] for s in samples if 'median_absolute_error_m' in s.get('tag_depth', {})]
    summary = {'synchronized_frames': len(samples), 'tag_frames': sum('tag_pose' in s for s in samples),
               'max_sync_difference_ms': max(s['sync_difference_ms'] for s in samples),
               'lens_positions': sorted(set(s['lens_position'] for s in samples)),
               'median_depth_valid_fraction': float(np.median([s['depth_valid_fraction'] for s in samples]))}
    if comparisons:
        summary['median_tag_depth_error_m'] = float(np.median([s['median_absolute_error_m'] for s in comparisons]))
        summary['median_tag_depth_valid_fraction'] = float(np.median([s['valid_fraction'] for s in comparisons]))
    report['summary'] = summary
    report['sensor_checks_passed'] = bool(len(samples) >= 30 and len(comparisons) >= 20
        and summary['max_sync_difference_ms'] <= 30
        and summary['lens_positions'] == [config['manual_focus']]
        and summary.get('median_tag_depth_error_m', 1) <= .02
        and summary.get('median_tag_depth_valid_fraction', 0) >= .5)
    report['note'] = 'Stationary depth/tag consistency in the rectified-right camera. RGB/depth alignment, hand-eye and contact geometry remain unvalidated.'
    (output/'report.json').write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    print(json.dumps(summary,indent=2),flush=True)
    print(f"Sensor checks passed: {report['sensor_checks_passed']}. Saved {output}/report.json",flush=True)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,default=Path(__file__).resolve().parent/'outputs/wrist_depth_v2')
    run(parser.parse_args().output)
