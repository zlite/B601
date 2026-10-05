"""Explicit camera selection for arm experiments; never fall back to the tripod."""
from contextlib import contextmanager
import json
from pathlib import Path
import time


def wrist_camera_config():
    return json.loads((Path(__file__).resolve().parent / 'config/cameras.json').read_text())['wrist']


def build_wrist_rgb(pipeline):
    """Use the calibrated lens position; temporary sensor control, no EEPROM write."""
    import depthai as dai
    focus = wrist_camera_config()['manual_focus']
    if not isinstance(focus, int) or not 0 <= focus <= 255:
        raise ValueError('Expected manual focus in 0..255')
    device = pipeline.getDefaultDevice()
    calibrated_focus = device.readCalibration().getLensPosition(dai.CameraBoardSocket.CAM_A)
    if focus != calibrated_focus:
        raise RuntimeError('Configured focus differs from camera calibration; review camera configuration')
    camera = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
    return camera


def start_wrist_pipeline(pipeline, camera):
    """Apply focus in RAM after camera sources have had time to initialize."""
    import depthai as dai
    control = camera.inputControl.createInputQueue()
    pipeline.start()
    time.sleep(3)
    focus = wrist_camera_config()['manual_focus']
    command = dai.CameraControl()
    command.setManualFocus(focus)
    control.send(command)
    print(f'RGB focus locked to calibrated lens position {focus}', flush=True)


@contextmanager
def wrist_pipeline():
    import depthai as dai
    camera = wrist_camera_config()
    matches = [info for info in dai.Device.getAllAvailableDevices()
               if info.deviceId == camera['device_id']]
    if len(matches) != 1:
        raise RuntimeError(f"Wrist camera {camera['device_id']} unavailable; check USB connection and permissions")
    with dai.Device(matches[0]) as device:
        device.setMaxReconnectionAttempts(0)
        model = device.getDeviceName()
        if model != camera['model']:
            raise RuntimeError(f'Unexpected wrist camera model: {model}')
        print(f"Wrist camera: {model} ({camera['device_id']})", flush=True)
        with dai.Pipeline(device) as pipeline:
            yield pipeline
