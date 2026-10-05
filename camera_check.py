"""Stationary camera/joint comparison. No motor or camera-device commands."""
from collections import deque
from datetime import datetime, timezone
import json
from pathlib import Path
import re
import threading
import time

import cv2
import numpy as np

MAX_FRAME_AGE_S = 1.5
STEPS = [(None, 'Starting view'), (5, 'Wrist roll'), (4, 'Wrist yaw'), (3, 'Wrist bend')]
BEND_STEPS = [(None, 'Starting view'), (3, 'Bend outward'), (3, 'Bend return'), (3, 'Bend repeat')]


def rotation_degrees(rotation):
    return float(np.degrees(np.linalg.norm(cv2.Rodrigues(np.asarray(rotation, dtype=float))[0])))


def compare_views(before, after):
    """Relative rotation angle is independent of the unknown rigid camera mount.

    Coupled joint movements are included in forward kinematics. This test does
    not determine axis direction, translation, or a calibrated hand-eye transform.
    """
    a, b = np.array(before['T_base_wrist']), np.array(after['T_base_wrist'])
    c, d = np.array(before['T_camera_tag']), np.array(after['T_camera_tag'])
    model = rotation_degrees(a[:3, :3].T @ b[:3, :3])
    camera = rotation_degrees(c[:3, :3] @ d[:3, :3].T)
    error = abs(model-camera)
    return {'model_rotation_deg': model, 'camera_rotation_deg': camera,
            'difference_deg': error, 'assessment': 'close' if error <= 2 else 'needs investigation',
            'diagnostic_threshold_deg': 2., 'calibration_validated': False}


class CameraCheck:
    def __init__(self, geometry, output=Path('outputs/camera_checks'), steps=None):
        self.geometry, self.output = geometry, Path(output)
        self.steps = STEPS if steps is None else steps
        if self.steps not in (STEPS,BEND_STEPS):
            raise ValueError('Unknown camera diagnostic sequence')
        self.lock = threading.RLock()
        self.joints = deque(maxlen=120)
        self.frames = deque(maxlen=22)
        self.pending = False
        self.saving = False
        self.samples = []
        self.independent_imu = None
        self.problem = None
        self.session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')

    def add_joints(self, stamp, angles, *, powered, following, all_powered, fault):
        with self.lock:
            self.joints.append({'time': stamp, 'angles': list(angles), 'powered': powered,
                                'following': following, 'all_powered': all_powered, 'fault': fault})

    def arm_capture(self):
        with self.lock:
            if len(self.samples) >= len(self.steps):
                raise ValueError('Check complete. Start a new check to collect more views.')
            if self.saving:
                raise ValueError('Saving the current view')
            self.pending = True
            self.problem = None

    def load_completed(self, path):
        """Display a completed local check after restart; never reuse old frames."""
        path = Path(path)
        report = json.loads(path.read_text())
        session = report.get('session','')
        samples = report.get('samples',[])
        steps = [tuple(row) for row in report.get('steps',STEPS)]
        if steps not in (STEPS,BEND_STEPS):
            raise ValueError('Unknown saved diagnostic sequence')
        if (not re.fullmatch(r'[0-9]{8}T[0-9]{12}Z',session) or
                path.resolve() != (self.output/session/'report.json').resolve() or
                report.get('geometry_fingerprint') != self.geometry.fingerprint or len(samples) != 4):
            raise ValueError('Expected a completed check from this geometry and output directory')
        for i,sample in enumerate(samples):
            if sample.get('index') != i or sample.get('step') != steps[i][1]:
                raise ValueError('Saved camera steps are out of order')
            for key,shape in [('raw_joint_deg',(6,)),('T_base_wrist',(4,4)),('T_camera_tag',(4,4))]:
                value = np.asarray(sample.get(key),dtype=float)
                if value.shape != shape or not np.isfinite(value).all():
                    raise ValueError('Invalid saved camera measurement')
            if i:
                sample['comparison'] = compare_views(samples[i-1],sample)
        independent = None
        imu_path = self.output.parent/'camera_diagnosis'/f'imu_comparison_{session}.json'
        if imu_path.exists():
            imu_report = json.loads(imu_path.read_text())
            rows = imu_report.get('comparisons',[])
            if imu_report.get('session') != session or len(rows) != 3:
                raise ValueError('Independent sensor report does not match the camera check')
            keys = ('encoder_deg','tag_deg','gyro_deg','gyro_vs_encoder_deg','tag_vs_gyro_deg','gravity_tilt_deg')
            for i,row in enumerate(rows):
                if row.get('step') != steps[i+1][1] or not all(np.isfinite(float(row.get(k,np.nan))) for k in keys):
                    raise ValueError('Invalid independent sensor comparison')
            independent = {'method':imu_report['method'], 'comparisons':[
                {'step':r['step'],**{k:float(r[k]) for k in keys}} for r in rows]}
        with self.lock:
            if self.saving or self.pending:
                raise ValueError('Cannot load a report during capture')
            self.session, self.samples = session, samples
            self.steps = steps
            self.independent_imu = independent
            self.frames.clear()
            self.joints.clear()
            self.problem = None

    def restart(self):
        with self.lock:
            if self.saving:
                raise ValueError('Wait for the current view to finish saving')
            self.pending = False
            self.samples = []
            self.steps = STEPS
            self.independent_imu = None
            self.frames.clear()
            self.problem = None
            self.session = datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')

    def readiness(self, now):
        """Called under lock. Fresh steady windows bracket the chosen image."""
        result = {'ready': False, 'steady_fraction': 0., 'delta_deg': [0.]*6,
                  'selected_joint': self.steps[min(len(self.samples),3)][0], 'message': ''}
        def blocked(message):
            result['message'] = message
            return result, None
        if self.problem:
            return blocked(self.problem)
        if len(self.samples) == len(self.steps):
            return blocked('Check complete. Review the three comparisons below.')
        if not self.joints or now-self.joints[-1]['time'] > .25:
            return blocked('Waiting for fresh arm readings.')
        latest = self.joints[-1]
        if latest['fault']:
            return blocked('Arm controller fault. Resolve it before capturing.')
        if self.samples:
            result['delta_deg'] = (np.array(latest['angles'])-self.samples[-1]['raw_joint_deg']).tolist()
        if latest['following']:
            return blocked('Release the follow button when the movement gauge reaches the green zone.')
        if not self.frames or now-self.frames[-1]['received'] > .35 or not 0 <= now-self.frames[-1]['time'] <= MAX_FRAME_AGE_S:
            return blocked('Waiting for a fresh wrist-camera image.')
        if not self.frames[-1].get('valid'):
            return blocked(self.frames[-1].get('reason', 'Keep the whole tag visible.'))
        joint = result['selected_joint']
        if joint is not None:
            movement = abs(result['delta_deg'][joint])
            if movement < 6:
                return blocked(f'Move {self.steps[len(self.samples)][1].lower()} into the green zone: 6–15°, either direction. Other joints may move a little.')
            if movement > 20:
                return blocked('Move back toward the previous pose until the gauge is below 20°.')
        # Evaluate an image-time window, then require continuous stationarity
        # from that window through NOW. Delayed video cannot be paired with a
        # newer pose just because it was recently delivered.
        end = self.frames[-1]['time']
        rows = [r for r in self.joints if r['time'] >= end-1.4]
        frames = [f for f in self.frames if end-1.2 <= f['time'] <= rows[-1]['time']-.05]
        rows = [r for r in rows if not r['following'] and not r['fault']]
        if len(rows) < 12 or len(frames) < 7:
            return blocked('Hold still for a moment; collecting steady camera and joint readings.')
        # Only a continuous trailing paused interval qualifies.
        start = max(rows[0]['time'], frames[0]['time'])
        relevant = [r for r in self.joints if start <= r['time']]
        if any(r['following'] or r['fault'] for r in relevant):
            return blocked('Settling after movement…')
        if any(not f.get('valid') for f in frames):
            return blocked('Keep the tag visible and slightly tilted while the image settles.')
        span = min(rows[-1]['time'], frames[-1]['time'])-start
        result['steady_fraction'] = min(1., max(0., span/1.))
        if span < 1.:
            return blocked('Hold steady — collecting one second of measurements.')
        if max(np.diff([r['time'] for r in relevant])) > .2 or max(np.diff([f['time'] for f in frames])) > .25:
            return blocked('Waiting for an uninterrupted stream of readings.')
        q = np.array([r['angles'] for r in relevant], dtype=float)
        if q.shape[1:] != (6,) or not np.isfinite(q).all() or np.ptp(q, axis=0).max() > .25:
            return blocked('The arm is still settling. Leave the leader released.')
        rotations = [np.array(f['T_camera_tag'])[:3,:3] for f in frames]
        spread = max(rotation_degrees(rotations[-1].T @ r) for r in rotations)
        positions = np.array([np.array(f['T_camera_tag'])[:3,3] for f in frames])
        if spread > 1. or np.max(np.std(positions,axis=0)) > .003:
            return blocked('Camera estimate is changing. Keep the tag fixed and avoid a straight-on view.')
        chosen = frames[-1]
        before = [r for r in relevant if r['time'] <= chosen['time']]
        after = [r for r in relevant if r['time'] >= chosen['time']]
        if not before or not after or chosen['time']-before[-1]['time'] > .12 or after[0]['time']-chosen['time'] > .12:
            return blocked('Waiting for joint readings on both sides of the image timestamp.')
        a,b = before[-1],after[0]
        alpha = (chosen['time']-a['time'])/(b['time']-a['time']) if b['time']>a['time'] else 0.
        angles = (np.array(a['angles'])*(1-alpha)+np.array(b['angles'])*alpha).tolist()
        result.update(ready=True, steady_fraction=1., message='Ready — capture this view.' if not self.pending else 'Ready — capturing…')
        return result, (chosen, angles, np.ptp(q,axis=0).tolist(), spread)

    def add_frame(self, observation, image, now=None, raw_image=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            if self.frames and observation['time'] <= self.frames[-1]['time']:
                return
            self.frames.append({**observation, 'received':now, 'image':image, 'raw_image':raw_image})
            status, capture = self.readiness(now)
            if not self.pending or self.saving or not status['ready']:
                return
            chosen, angles, joint_span, rotation_span = capture
            index, session = len(self.samples), self.session
            sample = {k:v for k,v in chosen.items() if k not in ('image','raw_image')}
            sample['stationary_observations'] = [{k:v for k,v in f.items() if k not in ('image','raw_image')}
                for f in self.frames if chosen['time']-1.1 <= f['time'] <= chosen['time'] and f.get('valid')]
            if chosen['raw_image'] is not None:
                sample['raw_image'] = f'{index}_raw.png'
            sample.update(index=index, step=self.steps[index][1], raw_joint_deg=angles,
                T_base_wrist=self.geometry.transform(np.radians(angles)).tolist(),
                joint_window_span_deg=joint_span, camera_window_rotation_span_deg=rotation_span,
                captured_utc=datetime.now(timezone.utc).isoformat(),
                timestamp_method='Host-mapped image timestamp bracketed by sequential joint-read completion times; stationary window required')
            if self.samples:
                sample['comparison'] = compare_views(self.samples[-1], sample)
            samples = self.samples+[sample]
            self.saving = True
        # Encoding and disk I/O never hold the motor thread's lock or our lock.
        try:
            directory = self.output/session
            directory.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(directory/f'{index}.jpg'), chosen['image']):
                raise RuntimeError('Could not save the image')
            if chosen['raw_image'] is not None and not cv2.imwrite(str(directory/f'{index}_raw.png'),chosen['raw_image']):
                raise RuntimeError('Could not save the raw image')
            report = {'session':session, 'geometry_fingerprint':self.geometry.fingerprint,
                'steps':self.steps,
                'scope':'Relative rotation consistency only. Not hand-eye, stereo, translation, or fingertip calibration.',
                'calibration_validated':False, 'motion_commanded_by_capture':False, 'samples':samples}
            temp=directory/'report.tmp';temp.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
            temp.replace(directory/'report.json')
            with self.lock:
                self.samples = samples
                self.pending = False
        except Exception as error:
            with self.lock:
                self.problem = 'Capture could not be saved: '+str(error)
                self.pending = False
        finally:
            with self.lock:
                self.saving = False

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        with self.lock:
            status,_ = self.readiness(now)
            end = self.frames[-1]['time'] if self.frames else now
            recent_joints = [r for r in self.joints if r['time'] >= end-1.4]
            recent_frames = [f for f in self.frames if f['time'] >= end-1.2]
            diagnostics = {'joint_samples':len(recent_joints), 'camera_samples':len(recent_frames),
                'latest_frame_age_s':now-self.frames[-1]['time'] if self.frames else None,
                'latest_frame_valid':self.frames[-1].get('valid') if self.frames else False,
                'latest_frame_reason':self.frames[-1].get('reason') if self.frames else None,
                'camera_window_s':recent_frames[-1]['time']-recent_frames[0]['time'] if len(recent_frames)>1 else 0.,
                'joint_span_deg':np.ptp([r['angles'] for r in recent_joints],axis=0).tolist() if recent_joints else None}
            valid = [f for f in recent_frames if f.get('valid')]
            if valid:
                rotation=np.array(valid[-1]['T_camera_tag'])[:3,:3]
                diagnostics['camera_rotation_span_deg']=max(rotation_degrees(rotation.T@np.array(f['T_camera_tag'])[:3,:3]) for f in valid)
            return {**status, 'diagnostics':diagnostics, 'pending':self.pending, 'saving':self.saving, 'session':self.session,
                'steps':self.steps,'step':len(self.samples), 'step_name':self.steps[min(len(self.samples),3)][1],
                'complete':len(self.samples)==len(self.steps),
                'independent_imu':self.independent_imu,
                'results':[{'step':s['step'], **s['comparison']} for s in self.samples if 'comparison' in s],
                'images':[f'/camera-check/image/{self.session}/{s["index"]}.jpg' for s in self.samples]}
