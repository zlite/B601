"""Explicit Linux UVC overview feed; never supplies invented lens calibration."""
import math
from pathlib import Path
import time

import cv2


def validate_device(config):
    path = Path(config['device_path'])
    if path.parent != Path('/dev/v4l/by-id') or not path.is_symlink():
        raise RuntimeError('Configured UVC camera stable device path is unavailable')
    node = path.resolve(strict=True)
    info = Path('/sys/class/video4linux') / node.name
    if (info / 'name').read_text().strip() != config['model']:
        raise RuntimeError('UVC camera model mismatch')
    for parent in (info / 'device').resolve().parents:
        if (parent / 'idVendor').exists():
            if ((parent / 'idVendor').read_text().strip() != config['usb_vendor_id'] or
                    (parent / 'idProduct').read_text().strip() != config['usb_product_id']):
                raise RuntimeError('UVC camera USB identity mismatch')
            if (parent / 'serial').read_text().strip() != config['device_id']:
                raise RuntimeError('UVC camera serial mismatch')
            return str(path)
    raise RuntimeError('Cannot verify UVC camera USB identity')


def frame_timing(timestamp_ms, now, previous=None):
    """OpenCV V4L2 POS_MSEC exposes the dequeued driver buffer timestamp.

    Require advancing timestamps in the host monotonic clock domain. This
    measures driver-buffer age, not independently measured exposure latency.
    Never substitute receipt time for an absent or stale capture timestamp.
    """
    stamp = timestamp_ms / 1000.
    age = now - stamp
    if not math.isfinite(stamp) or stamp <= 0 or not 0 <= age < .5:
        raise RuntimeError('UVC frame timestamp is missing, delayed, or in an unsupported clock domain')
    if previous is not None and stamp <= previous:
        raise RuntimeError('UVC frame timestamp did not advance')
    return {'frame_time': stamp, 'frame_age_s': age,
            'timestamp_source': 'v4l2_driver_buffer',
            'exposure_latency_validated': False}


def camera_worker(workbench, role, config):
    if role != 'tripod' or config.get('usage') != 'overview_only':
        workbench.error(role, 'Uncalibrated UVC backend supports only the tripod overview')
        return
    while not workbench.stop.is_set():
        capture = None
        try:
            path = validate_device(config)
            capture = cv2.VideoCapture(path, cv2.CAP_V4L2)
            if not capture.isOpened():
                raise RuntimeError('Unable to open configured UVC camera')
            width, height = config['size']
            for prop, value in (
                (cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*config['fourcc'])),
                (cv2.CAP_PROP_FRAME_WIDTH, width), (cv2.CAP_PROP_FRAME_HEIGHT, height),
                (cv2.CAP_PROP_FPS, config['fps']), (cv2.CAP_PROP_BUFFERSIZE, 1),
            ):
                if not capture.set(prop, value):
                    raise RuntimeError(f'UVC capture setting rejected: {prop}')
            actual = [round(capture.get(p)) for p in
                      (cv2.CAP_PROP_FRAME_WIDTH, cv2.CAP_PROP_FRAME_HEIGHT)]
            if actual != [width, height] or abs(capture.get(cv2.CAP_PROP_FPS)-config['fps']) > .1:
                raise RuntimeError('UVC capture mode differs from configuration')
            if int(capture.get(cv2.CAP_PROP_FOURCC)) != cv2.VideoWriter_fourcc(*config['fourcc']):
                raise RuntimeError('UVC pixel format differs from configuration')
            previous = None
            warmup = 5
            while not workbench.stop.is_set():
                ok, frame = capture.read()
                if not ok or frame is None or frame.shape[:2] != (height, width):
                    raise RuntimeError('UVC frames stopped or changed dimensions')
                timing = frame_timing(capture.get(cv2.CAP_PROP_POS_MSEC), time.monotonic(), previous)
                previous = timing['frame_time']
                if warmup:
                    warmup -= 1
                    continue
                ok, jpeg = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 85])
                if not ok:
                    raise RuntimeError('UVC preview encoding failed')
                metadata = {**timing, 'device_id': config['device_id'], 'model': config['model'],
                            'frame_size': [width, height], 'calibration_status': 'uncalibrated',
                            'lens_model': 'fisheye', 'usage': 'overview_only',
                            'display_rotation_deg': config.get('display_rotation_deg', 0)}
                with workbench.lock:
                    workbench.images[role] = jpeg.tobytes()
                    if getattr(workbench, 'survey_mode', False):
                        workbench.survey_frames = getattr(workbench, 'survey_frames', {})
                        workbench.survey_frames[role] = {**metadata, 'image': frame.copy(),
                                                       'received': time.monotonic()}
                workbench.publish(role, **metadata)
        except Exception as error:
            workbench.error(role, error)
            with workbench.lock:
                getattr(workbench, 'survey_frames', {}).pop(role, None)
        finally:
            if capture is not None:
                capture.release()
        workbench.stop.wait(3)
