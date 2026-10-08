#!/usr/bin/env python3
"""Local visual pairing workbench. Reads hardware; never commands motors."""
import argparse
from collections import deque
from datetime import datetime, timezone
import json
import math
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
import secrets
import threading
import time
from urllib.parse import urlparse

import cv2
import numpy as np
from arm_geometry import Geometry, origin_matrix
from calibrate_arm import Reader
from hello_world import PORT
from leader_read import LeaderReader, LEADER_PORT, PREVIEW_SIGNS, NAMES

ROOT = Path(__file__).resolve().parent
ASSETS = ROOT / 'visual_pairing'


def joint_points(geometry, model_deg):
    """Joint origins and wrist axes in base coordinates, metres."""
    T = np.eye(4)
    points = [[0., 0., 0.]]
    for joint, angle in zip(geometry.joints, np.radians(model_deg)):
        T = T @ origin_matrix(joint.find('origin'))
        points.append(T[:3, 3].tolist())
        axis = np.array(list(map(float, joint.find('axis').get('xyz').split())))
        R = np.eye(4)
        R[:3, :3] = cv2.Rodrigues(axis * angle)[0]
        T = T @ R
    axes = [(T[:3, 3] + T[:3, i]*.07).tolist() for i in range(3)]
    return {'points': points, 'axes': axes}


def stable_median(history, now, dimensions=7):
    rows = [(t, q) for t, q in history if now-t <= 1.6]
    if len(rows) < 5 or rows[-1][0]-rows[0][0] < .8 or now-rows[-1][0] > .5:
        raise ValueError('Hold the leader still for a full second with live readings.')
    q = np.array([r[1] for r in rows])
    if q.ndim != 2 or q.shape[1] != dimensions or not np.isfinite(q).all():
        raise ValueError('Invalid leader readings.')
    if np.max(np.ptp(q, axis=0)) > .5:
        raise ValueError('Leader is moving. Hold still for a second, then retry.')
    return np.median(q, axis=0).tolist()


def direction_feedback(history, guided, now):
    """Judge only the selected hinge; coupled joints are feedback, not a veto."""
    joint = guided['joint']-1
    rows = [(t,q) for t,q in history if t >= guided['began'] and now-t <= .7]
    result = {'ready': False, 'delta_deg': [0.]*6, 'selected_delta_deg': 0.,
              'message': 'Move the selected hinge a comfortable 5–15 degrees.'}
    if not rows or now-rows[-1][0] > .5:
        result['message'] = 'Waiting for fresh leader readings.'
        return result
    q = np.array([q for _,q in rows],dtype=float)
    if q.ndim != 2 or q.shape[1] != 7 or not np.isfinite(q).all():
        result['message'] = 'Invalid leader readings.'
        return result
    observed = np.median(q,axis=0)
    delta = observed[:6]-np.array(guided['baseline'][:6])
    result.update(delta_deg=delta.tolist(), selected_delta_deg=float(delta[joint]),
                  observed_deg=observed.tolist(),
                  most_active_joint=int(np.argmax(np.abs(delta)))+1)
    if abs(delta[joint]) < 3:
        result['message'] = f'Move {NAMES[joint]} a little farther (at least 3 degrees). Other joints may move.'
    elif abs(delta[joint]) > 45:
        result['message'] = 'That was a large movement. Click Start here to use this comfortable pose as the new baseline.'
    elif len(rows) < 4 or rows[-1][0]-rows[0][0] < .4 or np.ptp(q[:,joint]) > 1:
        result['message'] = 'Pause the selected hinge briefly. The other joints do not need to be still.'
    else:
        result['ready'] = True
        result['message'] = 'Ready. If the highlighted hinge moves the same way as yours, save this direction.'
    return result


class Workbench:
    def __init__(self, demo=False):
        self.demo = demo
        self.geometry = Geometry()
        self.lock = threading.RLock()
        self.stop = threading.Event()
        self.sources = {}
        self.history = deque(maxlen=50)
        self.follower_history = deque(maxlen=50)
        self.images = {}
        self.reference = None
        self.signs = PREVIEW_SIGNS.copy()
        self.checks = {}
        self.guided = None
        self.message = 'Start by matching the leader to its reference photo.'
        self.session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        self.output = ROOT / 'calibration' / f'leader_pairing_{self.session}.json'

    def publish(self, name, **values):
        with self.lock:
            self.sources[name] = {'time': time.monotonic(), **values}
            if name == 'leader' and 'angles' in values:
                self.history.append((time.monotonic(), values['angles']))
            if name == 'follower' and 'angles' in values:
                self.follower_history.append((time.monotonic(), values['angles']))

    def error(self, name, error):
        with self.lock:
            self.sources[name] = {'time': time.monotonic(), 'error': str(error)}
            if name == 'follower':
                self.follower_history.clear()
            if name == 'leader':
                self.history.clear()
                # Any link loss invalidates the software origin for this run.
                self.reference = None
                self.guided = None
                self.checks = {}

    def bus_worker(self, name, factory, poll_interval=.08):
        while not self.stop.is_set():
            try:
                with factory() as reader:
                    while not self.stop.is_set():
                        values = reader.read()
                        if name == 'follower':
                            values = np.degrees(values).tolist()
                        self.publish(name, angles=values)
                        self.stop.wait(poll_interval)
            except Exception as error:
                self.error(name, error)
                self.stop.wait(2)

    def camera_worker(self, role):
        config = json.loads((ROOT / 'config/cameras.json').read_text())[role]
        if config.get('backend') == 'v4l2':
            from uvc_camera import camera_worker
            return camera_worker(self, role, config)
        if role == 'wrist' and getattr(self, 'track_printer_target', False):
            from printer_camera_worker import camera_worker
            return camera_worker(self, config)
        import depthai as dai
        from camera_selection import build_wrist_rgb, start_wrist_pipeline
        from calibrate_camera import frame_pose
        from tag_view import annotate_view
        while not self.stop.is_set():
            try:
                matches = [d for d in dai.Device.getAllAvailableDevices() if d.deviceId == config['device_id']]
                if len(matches) != 1:
                    raise RuntimeError(f"{role} camera unavailable ({config['device_id']})")
                with dai.Device(matches[0]) as device:
                    device.setMaxReconnectionAttempts(0)
                    if device.getDeviceName() != config['model']:
                        raise RuntimeError('Camera identity mismatch')
                    with dai.Pipeline(device) as pipeline:
                        imu_queue = None
                        if role == 'wrist' and getattr(self,'capture_imu',False):
                            self.imu_type = device.getConnectedIMU()
                            if self.imu_type != 'BMI270':
                                raise RuntimeError('Expected the verified BMI270 for the inertial diagnostic')
                            imu = pipeline.create(dai.node.IMU)
                            imu.enableFirmwareUpdate(False)
                            imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW,dai.IMUSensor.GYROSCOPE_RAW],100)
                            imu.setBatchReportThreshold(1)
                            imu.setMaxBatchReports(10)
                            imu_queue = imu.out.createOutputQueue(maxSize=200,blocking=False)
                        cam = build_wrist_rgb(pipeline) if role == 'wrist' else pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
                        # Both cameras share USB 2 on this machine. Preserve wrist
                        # calibration pixels; use a smaller workspace preview and
                        # NV12 transport to avoid ~1 s of device-side backlog.
                        size = (1280,800) if role == 'wrist' else (640,400)
                        size = getattr(self,'camera_sizes',{}).get(role,size)
                        fps = getattr(self,'camera_fps',{}).get(role,10)
                        queue = cam.requestOutput(size, type=dai.ImgFrame.Type.NV12, fps=fps, enableUndistortion=False).createOutputQueue(maxSize=1, blocking=False)
                        if role == 'wrist':
                            start_wrist_pipeline(pipeline, cam)
                        else:
                            pipeline.start()
                        last = time.monotonic()
                        while not self.stop.is_set():
                            if imu_queue is not None:
                                batch=[]
                                while True:
                                    imu_packet=imu_queue.tryGet()
                                    if imu_packet is None:break
                                    offset=time.monotonic()-dai.Clock.now().total_seconds()
                                    for p in imu_packet.packets:
                                        a,g=p.acceleroMeter,p.gyroscope
                                        batch.append({'accel_time':a.getTimestamp().total_seconds()+offset,
                                            'gyro_time':g.getTimestamp().total_seconds()+offset,
                                            'a':[a.x,a.y,a.z],'g':[g.x,g.y,g.z]})
                                if batch:
                                    with self.lock:
                                        self.imu_rows.extend(batch)
                            packet = queue.tryGet()
                            if packet is None:
                                if time.monotonic()-last > 10:
                                    raise RuntimeError('Camera frames stopped')
                                self.stop.wait(.01)
                                continue
                            frame = packet.getCvFrame()
                            if getattr(self,'survey_mode',False):
                                tr=packet.getTransformation()
                                if tr.isValid():
                                    with self.lock:
                                        self.survey_frames=getattr(self,'survey_frames',{})
                                        self.survey_frames[role]={'image':frame.copy(),'camera_matrix':np.array(tr.getIntrinsicMatrix()).tolist(),'distortion':np.array(tr.getDistortionCoefficients()).tolist(),'received':time.monotonic()}
                                        if role=='wrist':self.survey_frame=self.survey_frames[role]
                            info = {'device_id': config['device_id']}
                            observation = None
                            raw_frame = None
                            if role == 'wrist':
                                if packet.getLensPosition() != config['manual_focus']:
                                    if time.monotonic()-last > 10:
                                        raise RuntimeError('Wrist focus not at calibrated setting')
                                    continue
                                try:
                                    frame, found, pose, matrix, distortion = frame_pose(packet)
                                    raw_frame = frame
                                    frame = annotate_view(frame, found)
                                    info.update(tag='Visible', rms_px=pose['reprojection_rms_px'], distance_mm=1000*np.linalg.norm(pose['translation_camera_m']))
                                    from calibrate_camera import transform
                                    observation = {'valid':True, 'T_camera_tag':transform(pose).tolist(),
                                        'camera_matrix':matrix.tolist(), 'distortion':distortion.tolist(),
                                        'corners_px':found[0]['corners_px'], 'rms_px':pose['reprojection_rms_px'],
                                        'device_id':config['device_id'], 'lens_position':packet.getLensPosition(),
                                        'coordinates':'original_sensor_frame', 'image_rotation_deg':180}

                                except ValueError as error:
                                    from tag_view import detect
                                    visible = detect(frame, 'auto', None)
                                    frame = annotate_view(frame, visible)
                                    info['visible_tags'] = [{'family': t['family'], 'id': t['id']} for t in visible]
                                    info['tag'] = str(error)
                                    observation = {'valid':False, 'reason':str(error)}
                            ok, jpeg = cv2.imencode('.jpg', cv2.resize(frame, (800,500)), [cv2.IMWRITE_JPEG_QUALITY,80])
                            if not ok:
                                raise RuntimeError('Image encoding failed')
                            with self.lock:
                                self.images[role] = jpeg.tobytes()
                            age = dai.Clock.now().total_seconds()-packet.getTimestamp().total_seconds()
                            stamp = time.monotonic()-age
                            info['frame_age_s'] = age
                            last = time.monotonic()
                            self.publish(role, **info)
                            check = getattr(self, 'camera_check', None)
                            if check is not None and observation is not None:
                                # Map the SDK host clock to Python's monotonic clock.
                                from camera_check import MAX_FRAME_AGE_S
                                if not 0 <= age <= MAX_FRAME_AGE_S:
                                    observation = {'valid':False, 'reason':'Camera image is delayed; waiting for a fresh frame.'}
                                observation['time'] = stamp
                                try:
                                    check.add_frame(observation, frame, raw_image=raw_frame)
                                except Exception as error:
                                    # A diagnostic failure must not restart a healthy camera.
                                    with check.lock:
                                        check.problem = 'Camera check error: '+str(error)
            except Exception as error:
                self.error(role, error)
                self.stop.wait(3)

    def save(self):
        data = {'session_id': self.session, 'demo': self.demo,
                'leader_port': LEADER_PORT, 'servo_ids': list(range(7)),
                'reference_raw_deg': self.reference, 'signs': self.signs,
                'direction_checks': self.checks,
                'geometry_fingerprint': self.geometry.fingerprint,
                'mapping': 'model_deg = sign * (leader_raw_deg - leader_reference_deg), first six joints only',
                'reference_validity': 'Current uninterrupted connection only; re-reference after restart or read failure.',
                'motion_ready': False, 'hardware_zero_changed': False,
                'scope': 'Operator-aligned software reference and direction candidates. No scale, range, collision, gripper, or motion validation.'}
        target = self.output if not self.demo else ROOT / 'outputs' / self.output.name
        target.parent.mkdir(exist_ok=True)
        temp = target.with_suffix('.tmp')
        temp.write_text(json.dumps(data, indent=2, allow_nan=False)+'\n')
        temp.replace(target)

    def action(self, request):
        with self.lock:
            now = time.monotonic()
            source = self.sources.get('leader', {})
            if 'angles' not in source or now-source['time'] > .5:
                raise ValueError('Leader readings are unavailable or stale.')
            op = request.get('action')
            if op == 'reference':
                if request.get('pose_confirmed') is not True:
                    raise ValueError('Confirm the physical leader matches the reference photo.')
                self.reference = stable_median(self.history, now)
                self.checks = {}
                self.guided = None
                self.signs = PREVIEW_SIGNS.copy()
                self.message = 'Software reference saved. Move the leader to watch the cyan preview. Check directions below.'
                self.save()
            elif op == 'begin_direction':
                if self.reference is None:
                    raise ValueError('Save the folded leader reference first.')
                joint = int(request['joint'])
                if joint not in range(1,7):
                    raise ValueError('Choose joints 1 through 6.')
                self.guided = {'joint': joint, 'baseline': list(source['angles']),
                               'began': now, 'preview_sign': self.signs[joint-1]}
                self.message = f'Move the {NAMES[joint-1]} hinge relative to its neighboring link. Other joints may move too.'
            elif op == 'flip_direction':
                if self.guided is None:
                    raise ValueError('Start a visual check first.')
                self.guided['preview_sign'] *= -1
                self.message = 'Selected hinge preview reversed. Compare it with your physical movement, then save if it matches.'
            elif op == 'cancel_direction':
                self.guided = None
                self.message = 'Check cancelled. Previously saved directions are unchanged.'
            elif op == 'record_direction':
                if self.guided is None or request.get('direction_confirmed') is not True:
                    raise ValueError('Start a direction check and confirm you followed its instruction.')
                feedback = direction_feedback(self.history, self.guided, now)
                if not feedback['ready']:
                    raise ValueError(feedback['message'])
                observed = feedback['observed_deg']
                joint = self.guided['joint']
                sign = self.guided['preview_sign']
                self.signs[joint-1] = sign
                self.checks[str(joint)] = {**self.guided, 'observed_deg': observed, 'sign': sign,
                                          'delta_deg': feedback['delta_deg'],
                                          'method': 'operator_confirmed_visual_mapping_with_coupled_motion',
                                          'angle_scale_validated': False}
                self.guided = None
                self.message = f'Joint {joint} direction saved. Choose the next joint and start from your current comfortable pose.'
                self.save()
            else:
                raise ValueError('Unknown action')

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            sources = {name: {**s, 'age_s': now-s['time']} for name,s in self.sources.items()}
            target = sources.get('printer_target')
            if target is not None and not 0 <= now-target.get('observation_time', 0) <= .25:
                target.update(valid=False, reason='Stereo reference expired; reacquire before planning')
            result = {'demo': self.demo, 'sources': sources, 'names': NAMES,
                      'message': self.message, 'reference': self.reference,
                      'signs': self.signs, 'checks': self.checks, 'guided': self.guided,
                      'motion_ready': False}
            follower = sources.get('follower',{})
            if 'angles' in follower and follower['age_s'] < 1:
                model = np.array(follower['angles'])*self.geometry.signs+np.degrees(self.geometry.offsets)
                result['follower_model_deg'] = model.tolist()
                result['follower_shape'] = joint_points(self.geometry, model)
            leader = sources.get('leader',{})
            if self.reference is not None and 'angles' in leader and leader['age_s'] < .5:
                model = (np.array(leader['angles'][:6])-self.reference[:6])*self.signs
                result['leader_model_deg'] = model.tolist()
                result['leader_shape'] = joint_points(self.geometry, model)
                result['outside_model_limits'] = [i+1 for i,(j,q) in enumerate(zip(self.geometry.joints,np.radians(model))) if not float(j.find('limit').get('lower')) <= q <= float(j.find('limit').get('upper'))]
                if self.guided:
                    result['direction_feedback'] = direction_feedback(self.history,self.guided,now)
                    baseline = (np.array(self.guided['baseline'][:6])-self.reference[:6])*self.signs
                    isolated = baseline.copy()
                    i = self.guided['joint']-1
                    isolated[i] += (leader['angles'][i]-self.guided['baseline'][i])*self.guided['preview_sign']
                    result['check_baseline_shape'] = joint_points(self.geometry,baseline)
                    result['check_preview_shape'] = joint_points(self.geometry,isolated)
            return result


def make_handler(workbench, token):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def reply(self, code, content, mime='application/json'):
            self.send_response(code)
            self.send_header('Content-Type', mime)
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.send_header('Content-Length', str(len(content)))
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            path = urlparse(self.path).path
            if path == '/':
                page = (ASSETS/'index.html').read_text().replace('__TOKEN__',token)
                self.reply(200, page.encode(), 'text/html; charset=utf-8')
            elif path == '/state':
                self.reply(200, json.dumps(workbench.snapshot(),allow_nan=False).encode())
            elif path in ('/wrist.jpg','/tripod.jpg'):
                role = path[1:-4]
                with workbench.lock:
                    data = workbench.images.get(role)
                self.reply(200 if data else 503, data or b'Camera not ready','image/jpeg' if data else 'text/plain')
            elif path == '/leader_reference.jpg':
                self.reply(200,(ASSETS/'leader_reference.jpg').read_bytes(),'image/jpeg')
            else:
                self.reply(404,b'Not found','text/plain')

        def do_POST(self):
            if urlparse(self.path).path != '/action' or self.headers.get('X-Pairing-Token') != token:
                self.reply(403,b'{}')
                return
            try:
                length = int(self.headers.get('Content-Length','0'))
                if not 0 < length <= 2048:
                    raise ValueError('Invalid request size')
                workbench.action(json.loads(self.rfile.read(length)))
                self.reply(200,b'{"ok":true}')
            except (ValueError, KeyError, TypeError) as error:
                self.reply(400,json.dumps({'error':str(error)}).encode())
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--printer-target', action='store_true',
                        help='Read-only live stereo tracking of the movable printer-bed tag')
    parser.add_argument('--demo', action='store_true', help='UI preview; no hardware access')
    args = parser.parse_args()
    workbench = Workbench(args.demo)
    workbench.track_printer_target = args.printer_target
    workers = []
    if args.demo:
        def demo_loop():
            while not workbench.stop.is_set():
                workbench.publish('leader', angles=[0.]*7)
                workbench.publish('follower', angles=np.degrees(workbench.geometry.profile['reference_raw_rad']).tolist())
                workbench.stop.wait(.1)
        workers.append(threading.Thread(target=demo_loop,daemon=True))
    else:
        workers.extend([threading.Thread(target=workbench.bus_worker,args=('leader',LeaderReader),daemon=True),
                        threading.Thread(target=workbench.bus_worker,args=('follower',lambda: Reader(PORT)),daemon=True)])
        workers.extend(threading.Thread(target=workbench.camera_worker,args=(role,),daemon=True) for role in ('wrist','tripod'))
    server = ThreadingHTTPServer(('127.0.0.1', args.port), make_handler(workbench,secrets.token_urlsafe(24)))
    for worker in workers:
        worker.start()
    print(f'Visual pairing: http://127.0.0.1:{args.port} — {"DEMO" if args.demo else "READ ONLY; no motor writes"}',flush=True)
    try:
        server.serve_forever(poll_interval=.2)
    except KeyboardInterrupt:
        pass
    finally:
        workbench.stop.set()
        server.server_close()
        for worker in workers:
            worker.join(timeout=16 if args.printer_target else 4)


if __name__ == '__main__':
    main()
