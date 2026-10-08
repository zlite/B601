"""Attended, relative six-axis teaching with camera/path recording.

Starts disabled. Uses relative leader control and separate attended rail jogs.
Gripper is not controlled. Reviewed same-scene approach/return demo is optional.
"""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import secrets
import signal
import threading
import time
from http.server import ThreadingHTTPServer

import axis_follow
from axis_follow import AxisWorkbench, ReferencedTarget, make_axis_handler
from leader_read import LeaderReader, LEADER_PORT
from pairing_dashboard import ASSETS
from rail_jog import RailJog
from printer_demo_ui import DemoController
from printer_target_ui import TargetPreview


def teaching_profiles():
    # Restore the existing attended-following profile, not the 3 degree/s
    # observation profile that could not keep up with the operator's hand.
    return [{**p, 'speed': 18. if i < 3 else 24.,
             'acceleration': 60. if i < 3 else 80.,
             # The shoulder's old +/-60-degree teaching window stopped the
             # demonstrated descent. axis_bounds still intersects this window
             # with the calibrated URDF limits and its existing endpoint margin.
             'range': 180. if i == 1 else p['range'],
             'measured_speed': 60. if i < 3 else 75.}
            for i, p in enumerate(axis_follow.PROFILES)]


class TeachWorkbench(AxisWorkbench):
    target_mapper = ReferencedTarget
    response_time = .02
    brake_at_target = True
    resume_alignment_tolerance_deg = 1.

    def after_motor_shutdown(self):
        self.demo.after_motor_shutdown()

    def __init__(self, pairing, output=None, rail_monitor_only=False, rail_reboot_offset_mm=0.):
        super().__init__(pairing)
        self.track_printer_target = True
        self.recording_root = Path(output or Path('outputs/printer_teach') / self.session)
        self.recording_root.mkdir(parents=True, exist_ok=False)
        self.log_path = self.recording_root / 'control.jsonl'
        self.recorder_time = None
        self.recorder_error = None
        self.recording_started = time.monotonic()
        self.recording_rows = 0
        self.recording_images = 0
        self.initial_pair_saved = False
        self.teaching_mappers = None
        self.press_leader_reference = None
        self.rail = RailJog(self, control_enabled=not rail_monitor_only,reboot_offset_mm=rail_reboot_offset_mm)
        self.demo = DemoController(self)
        self.printer_capture_root = self.recording_root/'target_captures'
        self.target_preview = TargetPreview(self)
        self.metadata = {
            'session': self.session, 'created_utc': datetime.now(timezone.utc).isoformat(),
            'geometry_fingerprint': self.geometry.fingerprint,
            'leader_port': LEADER_PORT,
            'leader_mapping': 'Capture on first press; preserve through startup, limits and ordinary pauses; explicit re-align while paused',
            'operator_reported_base_orientation': 'Follower manually rotated to align with leader before teaching',
            'mapping_signs': self.follow_signs, 'profiles': teaching_profiles(),
            'target_mapping': 'Fixed relative reference within each hold; saturation never shifts alignment',
            'response_time_s': self.response_time,
            'brake_at_target': self.brake_at_target,
            'leader_poll_pause_s': .005,
            'resume_alignment_tolerance_deg': self.resume_alignment_tolerance_deg,
            'gripper_commanded': False, 'rail_control_enabled': not rail_monitor_only,
            'rail_monitor_only': rail_monitor_only,
            'rail_scope': 'Attended keyboard jogs and confirmed 300 mm left/return demo with arm at rest and motors off; no homing or generalized positioning',
            'autonomous_motion_ready': False,
            'scope': 'Operator teaching and confirmed same-scene open-jaw replay loop; no grasp or lift',
            'camera_scope': 'Displayed JPEG snapshots with source metadata; not synchronized metric frames',
            'recording_limit_s': None,
        }
        (self.recording_root / 'session.json').write_text(json.dumps(self.metadata, indent=2)+'\n')

    def make_target_mappers(self, baseline, leader, following, limits):
        if self.teaching_mappers is None:
            captured = self.press_leader_reference if self.press_leader_reference is not None else leader
            self.teaching_mappers = super().make_target_mappers(baseline, captured, following, limits)
        else:
            # Recheck after setup as well as at the HTTP press: leader input can
            # change between the request and the next motor-loop read.
            errors = self.alignment_errors(leader, baseline)
            if any(abs(v)>self.resume_alignment_tolerance_deg for v in errors.values()):
                self.gate.cancel('leader_alignment_changed')
        return self.teaching_mappers

    def update_take_up_reference(self, mapped, leader, following):
        # Keep the press-time reference. The trajectory starts at the held pose
        # and catches up under unchanged speed/acceleration limits after take-up.
        pass

    def alignment_errors(self, leader, follower):
        if self.teaching_mappers is None:
            return {}
        return {j:max(m.low, min(m.high, m.origin+m.sign*(leader[j]-m.leader_reference)))-follower[j]
                for j,m in self.teaching_mappers.items()}

    def run_auxiliary_motion(self, arm):
        return self.demo.run_pending(arm)

    def action(self, request):
        op = request.get('action')
        with self.lock:
            if isinstance(op, str) and op.startswith('target_'):
                self.target_preview.action(request)
                return
            if isinstance(op,str) and op.startswith('demo_'):
                self.demo.action(request)
                return
            if self.demo.busy():
                if op == 'disable' and request.get('support_confirmed') is True:
                    self.demo.stop_requested=True
                    self.demo.stop_loop=True
                    return
                if op == 'release':
                    if self.demo.runner:self.demo.runner.pause_requested=True
                    return
                if op != 'rail_stop':raise ValueError('Pause or finish the demo before other controls')
            if isinstance(op,str) and op.startswith('rail_'):
                self.rail.action(request)
                if op == 'rail_start':self.demo.invalidate()
                return
            if op == 'press' and self.rail.busy():
                raise ValueError('Wait for the rail to stop before arm following')
            if op in ('press', 'realign'):
                self.fresh_leader()
                leader = self.sources['leader']['angles'][:6]
                follower = self.sources.get('follower', {})
                if 'angles' not in follower or time.monotonic()-follower['time']>.25:
                    raise ValueError('Wait for fresh follower readings')
                if op == 'realign':
                    if self.fault or self.active_id is not None or self.gate.lease is not None:
                        raise ValueError('Pause following before explicitly re-aligning')
                    if not self.powered or self.teaching_mappers is None:
                        raise ValueError('Re-align is available during powered pause')
                    q = follower['angles']
                    new = {j:ReferencedTarget(q[j], leader[j], m.sign, m.low, m.high)
                           for j,m in self.teaching_mappers.items()}
                    self.teaching_mappers = new
                    self.record('reference_realigned', leader_reference=leader,
                                follower_reference=q, motion_commanded=False)
                    return
                if self.powered and self.teaching_mappers is not None:
                    errors = self.alignment_errors(leader, follower['angles'])
                    mismatches = {j+1:round(e,2) for j,e in errors.items()
                                  if abs(e)>self.resume_alignment_tolerance_deg}
                    if mismatches:
                        raise ValueError('Match leader to held follower (within 1 degree), or use Re-align explicitly. Joint errors: '+str(mismatches))
                else:
                    self.teaching_mappers = None
                    self.press_leader_reference = list(leader)
            super().action(request)
            if op in ('select', 'reverse', 'set_control_mode') or (op == 'disable' and self.disable_requested):
                self.teaching_mappers = None
                self.press_leader_reference = None

    def cameras_ready(self):
        healthy = (self.recorder_error is None and self.recorder_time is not None and
                   0 <= time.monotonic()-self.recorder_time < .5)
        if not self.rail.control_enabled and self.rail.busy():
            return False
        return healthy and super().cameras_ready()

    def snapshot(self):
        result = super().snapshot()
        result['teaching'] = {
            'folder': str(self.recording_root), 'rows': self.recording_rows,
            'images': self.recording_images, 'error': self.recorder_error,
            'age_s': None if self.recorder_time is None else time.monotonic()-self.recorder_time,
            'gripper_control': False,
        }
        with self.lock:
            result['rail'] = self.rail.snapshot()
            result['demo'] = self.demo.snapshot()
            result['target_preview'] = self.target_preview.snapshot()
            if self.demo.busy():
                result['rail']['ready']=False
                result['message']=result['demo']['phase']
            blocked=self.rail.busy() or self.demo.busy()
            result['ready']=bool(result.get('ready')) and not blocked
            result['teaching']['arm_block_reason']=(
                'Teaching paused while demo is active' if self.demo.busy() else
                'Arm blocked: waiting for a confirmed stopped rail' if blocked else None)
            leader, follower = self.sources.get('leader', {}), self.sources.get('follower', {})
            fresh = all('angles' in s and time.monotonic()-s['time']<.25 for s in (leader,follower))
            errors = self.alignment_errors(leader['angles'], follower['angles']) if fresh else {}
            result['teaching'].update(reference_captured=self.teaching_mappers is not None,
                resume_errors_deg={j+1:e for j,e in errors.items()},
                resume_aligned=fresh and all(abs(e)<=self.resume_alignment_tolerance_deg for e in errors.values()))
        return result

    def record(self, event, **values):
        super().record(event, monotonic_s=time.monotonic(), **values)

    def recording_worker(self):
        next_image = 0.
        try:
            with (self.recording_root / 'observations.jsonl').open('a', buffering=1) as output:
                while not self.stop.is_set():
                    now = time.monotonic()
                    # Continue attended repeating demos until stopped; disk or
                    # recorder failure still closes the motion-readiness gate.
                    with self.lock:
                        sources = {k: dict(v) for k, v in self.sources.items()}
                        row = {'sample_monotonic_s': now, 'sources': sources,
                               'powered': self.powered, 'following': self.active_id is not None,
                               'fault': self.fault, 'images': {}}
                        images = dict(self.images) if now >= next_image else {}
                    if not self.initial_pair_saved:
                        pair = [sources.get(role, {}) for role in ('leader', 'follower')]
                        if all('angles' in s and 0 <= now-s['time'] < .25 for s in pair):
                            (self.recording_root / 'initial_pair.json').write_text(json.dumps({
                                'sample_monotonic_s': now, 'leader': pair[0], 'follower': pair[1],
                                'scope': 'Read-only connected pose; actual motion baselines captured on each press'
                            }, indent=2)+'\n')
                            self.initial_pair_saved = True
                    for role, data in images.items():
                        source = sources.get(role, {})
                        if 'error' in source or not 0 <= now-source.get('time', 0) < .4:
                            continue
                        filename = f'{self.recording_rows:06d}_{role}.jpg'
                        (self.recording_root / filename).write_bytes(data)
                        row['images'][role] = filename
                        self.recording_images += 1
                    if images:
                        next_image = now+.5
                    output.write(json.dumps(row, allow_nan=False)+'\n')
                    self.recording_rows += 1
                    self.recorder_time = time.monotonic()
                    self.stop.wait(.1)
        except Exception as error:
            with self.lock:
                self.recorder_error = str(error)
                self.gate.cancel('recorder_failed')
        finally:
            summary = {**self.metadata, 'rows': self.recording_rows,
                       'images': self.recording_images, 'recorder_error': self.recorder_error,
                       'rail_commands': self.rail.commands,
                       'finished_utc': datetime.now(timezone.utc).isoformat()}
            try:
                (self.recording_root / 'recording_summary.json').write_text(json.dumps(summary, indent=2)+'\n')
            except OSError:
                pass  # Original recording fault remains visible to the motor gate/UI.


def handler(workbench, token):
    parent = make_axis_handler(workbench, token)
    class Handler(parent):
        def do_GET(self):
            if self.path == '/':
                page = (ASSETS / 'printer_teach.html').read_text().replace('__TOKEN__', token)
                self.reply(200, page.encode(), 'text/html; charset=utf-8')
            else:
                super().do_GET()
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pairing', type=Path, default=Path('calibration/leader_pairing_20261002T202428971124Z.json'))
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--rail-monitor-only', action='store_true',
                        help='Disable rail jogs; require fresh stopped-rail status for arm teaching')
    parser.add_argument('--rail-reboot-offset-mm',type=float,default=0.,help='Known unchanged physical rail displacement if this connection reboots GRBL')
    args = parser.parse_args()
    # Process-local profiles only; calibration and motor zeros are unchanged.
    axis_follow.PROFILES[:] = teaching_profiles()
    w = TeachWorkbench(args.pairing, rail_monitor_only=args.rail_monitor_only,rail_reboot_offset_mm=args.rail_reboot_offset_mm)
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler(w, secrets.token_urlsafe(24)))
    workers = [threading.Thread(target=w.recording_worker),
               threading.Thread(target=w.rail.worker),
               threading.Thread(target=w.bus_worker, args=('leader', LeaderReader, .005), daemon=True),
               threading.Thread(target=w.motor_worker)]
    workers += [threading.Thread(target=w.camera_worker, args=(role,), daemon=True)
                for role in ('wrist', 'tripod')]
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    for worker in workers:
        worker.start()
    print(f'Teaching: http://127.0.0.1:{args.port} — starts disabled; recordings: {w.recording_root}', flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        w.stop.set()
        server.server_close()
        for worker in workers:
            worker.join(timeout=16)


if __name__ == '__main__':
    main()
