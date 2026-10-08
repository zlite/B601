"""Read-only target preview work; never occupies or commands the motor owner."""
from collections import deque
from copy import deepcopy
from datetime import datetime, timezone
import json
from pathlib import Path
import threading
import time

import numpy as np

from printer_target_plan import preview, require_fresh, reference_change, fingerprint


class TargetPreview:
    def __init__(self, workbench):
        self.w = workbench
        self.busy = False
        self.message = 'Waiting for a stationary view of the plate grid and bed tag'
        self.result = None
        self.planned_observation = None
        self.history = deque(maxlen=3)
        self.thread = None
        self.capture_stage = 0

    def rail_state(self):
        r = self.w.rail
        return dict(state=r.state, connected=r.connected, error=r.error,
                    x_mm=r.x_mm, commands=r.commands, status_time=r.status_time)

    def observe(self, plate):
        with self.w.lock:
            current = plate.get('registration')
            if not current:
                self.history.clear()
                return
            if self.history and current['observation_time'] == self.history[-1]['observation_time']:
                return
            if self.history and (current['session'] != self.history[-1]['session'] or
                    current['rail_revision'] != self.history[-1]['rail_revision'] or
                    current['observation_time'] <= self.history[-1]['observation_time'] or
                    current['observation_time']-self.history[-1]['observation_time'] > 1. or
                    reference_change(self.history[-1], current)['requires_replan']):
                self.history.clear()
            self.history.append(deepcopy(current))

    def current(self):
        plate = self.w.sources.get('printer_plate', {})
        observation = plate.get('registration')
        if observation is None:
            raise ValueError(plate.get('registration_error') or plate.get('reason') or
                             'Need a complete stereo grid and bed tag')
        require_fresh(observation, time.monotonic(), plate['session'],
                      self.rail_state(), self.w.geometry.fingerprint)
        if len(self.history) < 3 or any(reference_change(observation, r)['requires_replan']
                                       for r in self.history):
            raise ValueError('Waiting for three stable independent plate observations')
        return deepcopy(observation)

    def snapshot(self):
        try:
            observation = self.current()
            reason = None
        except (ValueError, KeyError) as error:
            observation = None
            reason = str(error)
        if self.result is not None and (observation is None or
                self.planned_observation is None or
                observation['session'] != self.planned_observation['session'] or
                observation['rail_revision'] != self.planned_observation['rail_revision'] or
                reference_change(self.planned_observation, observation)['requires_replan']):
            self.result = None
            self.planned_observation = None
            self.message = 'Previous preview invalidated; reacquire and replan'
        # Preview readiness is explicitly not motion readiness.
        return dict(preview_ready=reason is None and not self.busy,
                    busy=self.busy, reason=reason, message=self.message,
                    result=self.result, motion_ready=False,
                    scope='Camera-guided noncontact preview; execution not commissioned')

    def action(self, request):
        if request['action'] == 'target_capture':
            if self.capture_stage >= 100:
                raise ValueError('Capture limit reached; retain evidence and restart')
            self.capture_stage += 1
            root = self.w.printer_capture_root
            root.mkdir(parents=True, exist_ok=True)
            temp = root/'request.tmp'
            temp.write_text(json.dumps(dict(stage=self.capture_stage, requested_at=time.monotonic())))
            temp.replace(root/'request.json')
            self.message = 'Saving raw stereo images for calibration; no motion commanded'
            return
        if request['action'] != 'target_preview':
            raise ValueError('Unknown target action')
        if self.busy:
            raise ValueError('Target preview is already running')
        observation = self.current()
        q = np.radians(self.w.sources['follower']['angles'])
        rail = self.rail_state()
        self.result = None
        self.planned_observation = None
        self.busy = True
        self.message = 'Checking measured target and registered scene'
        self.thread = threading.Thread(target=self.run, args=(observation, q, rail), daemon=True)
        self.thread.start()

    def run(self, observation, q, rail):
        folder = self.w.recording_root/'target_plans'/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        try:
            config = json.loads(Path('config/printer_target_scene.json').read_text())
            X = json.loads(Path('calibration/stereo_sweep_handeye_20261004.json').read_text())['T_wrist_camera']
            candidate = preview(self.w.geometry, q, X, observation, config['scene'], config['tool'],
                                now=time.monotonic(), session=observation['session'], rail=rail)
            folder.mkdir(parents=True)
            (folder/'candidate.json').write_text(json.dumps(candidate, indent=2, allow_nan=False)+'\n')
            with self.w.lock:
                latest = self.current()
                if (latest['session'] != observation['session'] or
                        latest['rail_revision'] != observation['rail_revision'] or
                        reference_change(observation, latest)['requires_replan']):
                    raise ValueError('Target moved while planning; discard and reacquire')
                if fingerprint(config) != fingerprint(json.loads(Path('config/printer_target_scene.json').read_text())):
                    raise ValueError('Scene calibration changed while planning')
                self.result = dict(path=str(folder/'candidate.json'),
                                   plan_sha256=candidate['plan_sha256'], motion_ready=False)
                self.planned_observation = observation
                self.message = 'Noncontact preview saved; physical validation still required'
        except Exception as error:
            with self.w.lock:
                self.result = None
                self.message = 'Preview blocked: '+str(error)
        finally:
            with self.w.lock:
                self.busy = False
