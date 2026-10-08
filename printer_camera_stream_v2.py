#!/usr/bin/env python3
"""Camera-only JSON stream for the read-only printer dashboard; DepthAI 2.32."""
import base64
import argparse
import json
from pathlib import Path
import signal
import time

import cv2
import depthai as dai
import numpy as np

from camera_selection import wrist_camera_config
from contact_geometry import stereo_tag_pose
from tag_view import annotate_view, detect, detect_small
from printer_plate_vision import PlateVisionWorker


def interrupted(*_):
    raise KeyboardInterrupt


def main(capture_root=None):
    if not dai.__version__.startswith('2.32.'):
        raise RuntimeError('This camera stream requires the verified DepthAI 2.32 environment')
    config = wrist_camera_config()
    plate_worker = PlateVisionWorker()
    last_plate_request = 0.
    target = json.loads(Path('config/printer_bed_target.json').read_text())['tag']
    pipeline = dai.Pipeline()
    rgb = pipeline.create(dai.node.ColorCamera)
    rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
    rgb.setIspScale(1, 3)
    rgb.setFps(10)
    rgb.initialControl.setManualFocus(config['manual_focus'])
    ports = {'A': rgb.isp}
    for role in ('B', 'C'):
        mono = pipeline.create(dai.node.MonoCamera)
        mono.setBoardSocket(getattr(dai.CameraBoardSocket, 'CAM_'+role))
        mono.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P)
        mono.setFps(10)
        # Current oblique tag view clips white detail at 2000 us; the stationary
        # 1000 us capture recovered 10/12 metric stereo fits at unchanged gates.
        mono.initialControl.setManualExposure(1000, 100)
        ports[role] = mono.out
    for role, port in ports.items():
        out = pipeline.create(dai.node.XLinkOut)
        out.setStreamName(role)
        port.link(out.input)
    with dai.Device(pipeline, dai.DeviceInfo(config['device_id'])) as device:
        if device.getDeviceName() != config['model']:
            raise RuntimeError('Unexpected wrist camera')
        cal = device.readCalibration()
        T = np.asarray(cal.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B, dai.CameraBoardSocket.CAM_C))
        T[:3, 3] *= .01
        extrinsics = {'B_C': T.tolist()}
        if capture_root is not None:
            capture_root.mkdir(parents=True, exist_ok=True)
            for a, b in [('B', 'A'), ('C', 'A')]:
                t = np.asarray(cal.getCameraExtrinsics(getattr(dai.CameraBoardSocket, 'CAM_'+a),
                                                       getattr(dai.CameraBoardSocket, 'CAM_'+b)))
                t[:3, 3] *= .01
                extrinsics[a+'_'+b] = t.tolist()
        capture_id = None
        capture_report = None
        calibration = {r: {'camera_matrix': cal.getCameraIntrinsics(getattr(dai.CameraBoardSocket, 'CAM_'+r), 640, 480),
                           'distortion': cal.getDistortionCoefficients(getattr(dai.CameraBoardSocket, 'CAM_'+r))}
                       for r in ('B', 'C')}
        queues = {r: device.getOutputQueue(r, maxSize=1, blocking=False) for r in ports}
        latest = {}; seen = {}; last = time.monotonic(); began = last
        while True:
            for role, queue in queues.items():
                packet = queue.tryGet()
                if packet is not None:
                    latest[role] = packet
            if time.monotonic()-last > 10:
                raise RuntimeError('Camera frames stopped')
            if (len(latest) != 3 or time.monotonic()-began < 3 or
                    any(p.getSequenceNum() <= seen.get(r, -1) for r, p in latest.items())):
                time.sleep(.005)
                continue
            seen = {r: p.getSequenceNum() for r, p in latest.items()}
            stamps = {r: p.getTimestamp().total_seconds() for r, p in latest.items()}
            sdk_now = dai.Clock.now().total_seconds()
            stamp = time.monotonic()-(sdk_now-min(stamps[r] for r in ('B', 'C')))
            observation = {'valid': False, 'observation_time': stamp, 'tag': target,
                           'motion_ready': False, 'runtime': dai.__version__}
            cameras = {}
            try:
                if abs(stamps['B']-stamps['C']) > .015 or not 0 <= time.monotonic()-stamp <= .25:
                    raise ValueError('Stereo pair stale or unsynchronized')
                cameras = {}
                for role in ('B', 'C'):
                    gray = latest[role].getCvFrame()
                    # Existing small-tag fallback preserves original sensor
                    # coordinates; all stereo metric checks still apply.
                    cameras[role] = {**calibration[role], 'tags': detect_small(cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR), target['family'], target['id'])}
                observation.update(stereo_tag_pose({'cameras': cameras}, extrinsics, target['size_m']), valid=True)
            except (ValueError, cv2.error) as error:
                observation['reason'] = str(error)
            if latest['A'].getLensPosition() != config['manual_focus']:
                raise RuntimeError('Wrist focus changed')
            frame = latest['A'].getCvFrame()
            tags = detect(frame, target['family'], target['id'])
            view = annotate_view(frame, tags)
            ok, jpeg = cv2.imencode('.jpg', view, [cv2.IMWRITE_JPEG_QUALITY, 85])
            if not ok:
                raise RuntimeError('Image encoding failed')
            observation['frame_age_s'] = time.monotonic()-stamp
            if observation['frame_age_s'] > .25:
                observation.update(valid=False, reason='Stereo processing exceeded freshness limit')
            if (time.monotonic()-last_plate_request >= .2 and
                    abs(stamps['B']-stamps['C']) <= .015 and
                    0 <= time.monotonic()-stamp <= .25):
                plate_worker.submit({r: latest[r].getCvFrame().copy() for r in ('B', 'C')},
                                    calibration, T, observation)
                last_plate_request = time.monotonic()
            row = {'kind': 'printer_camera_frame', 'target': observation,
                   'plate': plate_worker.latest,
                   'rgb_observation_time': time.monotonic()-(dai.Clock.now().total_seconds()-stamps['A']),
                   'visible_tags': [{'family': t['family'], 'id': t['id']} for t in tags],
                   'jpeg_base64': base64.b64encode(jpeg).decode('ascii')}
            print(json.dumps(row, allow_nan=False), flush=True)
            # Optional raw triplets from this same device owner. No camera restart
            # or motor-thread blocking is needed at a stopped observation pose.
            request_path = capture_root/'request.json' if capture_root is not None else None
            if request_path is not None and request_path.exists():
                request = json.loads(request_path.read_text())
                ident = request['stage']
                if type(ident) is not int or not 0 <= ident <= 100:
                    raise ValueError('Invalid capture stage')
                if ident != capture_id:
                    capture_id = ident
                    capture_report = {'frames': [], 'extrinsics': extrinsics, 'metric_tag': target,
                                      'wrist_camera': config, 'motion_ready': False,
                                      'scope': 'Raw held-pose triplets; validate against motor timestamps before metric use'}
                times = {r: time.monotonic()-(dai.Clock.now().total_seconds()-s) for r, s in stamps.items()}
                previous = capture_report['frames'][-1]['cameras']['B']['observation_monotonic_s'] if capture_report['frames'] else -1
                if (len(capture_report['frames']) < 3 and min(times.values()) >= request['requested_at']+.4
                        and times['B']-previous >= .18 and abs(stamps['B']-stamps['C']) <= .015):
                    destination = capture_root/f'stage_{ident:02d}'
                    destination.mkdir(exist_ok=True)
                    sample = {'sync_span_s': max(stamps.values())-min(stamps.values()), 'cameras': {},
                              'stereo_tag_pose': observation}
                    for role, packet in latest.items():
                        raw = packet.getCvFrame()
                        height, width = raw.shape[:2]
                        socket = getattr(dai.CameraBoardSocket, 'CAM_'+role)
                        filename = f'{len(capture_report["frames"]):02d}_{role}.png'
                        if not cv2.imwrite(str(destination/filename), raw):
                            raise RuntimeError('Raw camera capture failed')
                        sample['cameras'][role] = {
                            'image': filename, 'camera_matrix': cal.getCameraIntrinsics(socket, width, height),
                            'distortion': cal.getDistortionCoefficients(socket), 'timestamp_s': stamps[role],
                            'observation_monotonic_s': times[role], 'exposure_s': packet.getExposureTime().total_seconds(),
                            'sensitivity_iso': packet.getSensitivity(), 'tags': tags if role == 'A' else cameras.get(role, {}).get('tags', [])}
                    capture_report['frames'].append(sample)
                    temp = destination/'report.tmp'
                    temp.write_text(json.dumps(capture_report, indent=2, allow_nan=False)+'\n')
                    temp.replace(destination/'report.json')
            last = time.monotonic()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--capture-root', type=Path)
    args = parser.parse_args()
    signal.signal(signal.SIGTERM, interrupted)
    try:
        main(args.capture_root)
    except KeyboardInterrupt:
        pass
