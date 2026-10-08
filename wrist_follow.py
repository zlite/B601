#!/usr/bin/env python3
"""Attended, hold-to-run joint6 experiment. Starts disabled; never changes zeros."""
import argparse
from datetime import datetime, timezone
import json
import math
from pathlib import Path
import secrets
import signal
import threading
import time
from http.server import ThreadingHTTPServer

from calibrate_arm import Reader
from hello_world import PORT
from leader_read import LeaderReader
from pairing_dashboard import Workbench, make_handler, ASSETS

LIMIT = 15.0
SPEED = 12.0
ACCELERATION = 40.0
MEASURED_SPEED_LIMIT = 35.0
TRACKING_ERROR_LIMIT = 4.0
ENVELOPE_MARGIN = 2.0
LEASE = .4
MOTOR_TIMEOUT_MS = 500
# MotorBridge motor_abi/src/motor_register_ffi.rs converts ms to 50 us ticks.
DM_TIMEOUT_TICKS_PER_MS = 20


class HoldGate:
    """HTTP intent only. Expiry/release permanently cancels that press."""
    def __init__(self):
        self.lease = None
        self.sequences = {}

    def action(self, request, now):
        client, seq = request.get('client'), request.get('seq')
        if not isinstance(client, str) or len(client) > 80 or not isinstance(seq, int):
            raise ValueError('Invalid control request')
        if seq <= self.sequences.get(client, -1):
            return
        self.sequences[client] = seq
        op = request.get('action')
        if op == 'release':
            self.lease = None
        elif op == 'press':
            if self.lease is not None:
                raise ValueError('Release the current hold first')
            self.lease = {'client': client, 'id': seq, 'began': now, 'heartbeat': now}
        elif op == 'heartbeat':
            if self.valid(now) and self.lease['client'] == client and self.lease['id'] == request.get('press_id'):
                self.lease['heartbeat'] = now
        else:
            raise ValueError('Unknown control')

    def valid(self, now):
        if self.lease and now-self.lease['heartbeat'] > LEASE:
            self.lease = None
        return self.lease is not None


def mapped_target(origin, baseline, leader_baseline, leader, sign):
    values = (origin, baseline, leader_baseline, leader)
    if not all(math.isfinite(x) for x in values) or sign not in (-1, 1):
        raise ValueError('Invalid leader mapping input')
    desired = baseline + sign*(leader-leader_baseline)
    return max(origin-LIMIT, min(origin+LIMIT, desired))


class WristTrajectory:
    """Bounded position trajectory with speed, acceleration and edge braking."""
    def __init__(self, origin, position, *, low=None, high=None, speed=SPEED, acceleration=ACCELERATION, response_time=.15, brake_at_target=False):
        low = origin-LIMIT if low is None else low
        high = origin+LIMIT if high is None else high
        if not all(math.isfinite(v) for v in (origin, position, low, high, speed, acceleration, response_time)) or not low <= position <= high or speed <= 0 or acceleration <= 0 or response_time <= 0:
            raise ValueError('Trajectory must start inside the wrist envelope')
        self.low, self.high = low, high
        self.speed, self.acceleration = speed, acceleration
        self.response_time = response_time
        self.brake_at_target = brake_at_target
        self.position, self.velocity = position, 0.

    def step(self, desired, dt):
        if not math.isfinite(desired) or not math.isfinite(dt) or not 0 < dt <= .3:
            raise ValueError('Invalid input or control loop stalled')
        desired = max(self.low, min(self.high, desired))
        count = math.ceil(dt/.005)
        h = dt/count
        for _ in range(count):
            # v*h + v²/(2*a) <= remaining distance: reserve room to brake
            # before the fixed session boundary, including during reversals.
            def braking_speed(distance):
                return math.sqrt((self.acceleration*h)**2+2*self.acceleration*max(0.,distance))-self.acceleration*h
            low_v = max(-self.speed, self.velocity-self.acceleration*h, -braking_speed(self.position-self.low))
            high_v = min(self.speed, self.velocity+self.acceleration*h, braking_speed(self.high-self.position))
            if low_v > high_v+1e-8:
                raise RuntimeError('Wrist trajectory cannot brake inside its envelope')
            wanted = (desired-self.position)/self.response_time
            if self.brake_at_target:
                goal_speed = braking_speed(abs(desired-self.position))
                wanted = max(-goal_speed, min(goal_speed, wanted))
            self.velocity = max(low_v, min(high_v, wanted))
            self.position += self.velocity*h
        if not self.low-1e-8 <= self.position <= self.high+1e-8:
            raise RuntimeError('Wrist target outside its envelope')
        return self.position


def check_envelope(q, origin):
    if max(abs(a-b) for a,b in zip(q[:5], origin[:5])) > 1:
        raise RuntimeError('A supported follower joint moved more than 1 degree')
    if abs(q[5]-origin[5]) > LIMIT+ENVELOPE_MARGIN:
        raise RuntimeError('Wrist exceeded the trial envelope')
    # Do not engage torque outside the command envelope, even if just inside
    # the wider measured-position fault margin.
    if abs(q[5]-origin[5]) > LIMIT:
        raise RuntimeError('Wrist moved outside the starting envelope; restart from a supported pose')


def relative_wrist_sign(saved, follower_sign):
    """A physical paired check can supersede the earlier model-preview mapping."""
    physical = saved.get('relative_follow_checks', {}).get('6')
    if physical is None:
        return int(saved['direction_checks']['6']['sign'] * follower_sign)
    sign = physical.get('raw_leader_to_raw_follower_sign')
    if type(sign) is not int or sign not in (-1, 1):
        raise ValueError('Invalid physical wrist-follow mapping')
    return sign


class FollowArm(Reader):
    """One owner thread, fresh position transactions; only J6 has write calls."""
    def __init__(self, port=PORT):
        super().__init__(port)
        self.active = False
        self.timeout_saved = None
        self.timeout_changed = False
        self.wrist_velocity_deg_s = 0.
        self.joint_velocities_deg_s = [0.]*6

    def read(self):
        values = []
        fresh=getattr(self,'_fresh_reader',None)
        if fresh is not None:
            readings=fresh.read()
        else:
            readings=[]
            for motor in self.motors:
                position=motor.get_register_f32(80,100)
                motor.request_feedback();time.sleep(.005);self.ctrl.poll_feedback_once()
                readings.append((position,motor.get_state()))
        for i,(motor,(position,s)) in enumerate(zip(self.motors,readings)):
            selected = getattr(self, 'selected', 5)
            enabled_indices = getattr(self, 'enabled_indices', None)
            enabled = (i == selected and self.active) if enabled_indices is None else i in enabled_indices
            expected = 1 if enabled else 0
            if s is None or s.status_code != expected:
                raise RuntimeError(f'Joint {i+1}: unexpected status; stopped')
            if not all(math.isfinite(v) for v in (position, s.vel, s.torq, s.t_mos, s.t_rotor)):
                raise RuntimeError('Invalid motor feedback')
            if max(s.t_mos, s.t_rotor) > 60:
                raise RuntimeError('Motor temperature exceeds 60 C')
            velocity = self.joint_velocities_deg_s[i] = math.degrees(s.vel)
            if i == selected:
                self.wrist_velocity_deg_s = velocity
            speed_limit = getattr(self,'speed_limits',{}).get(i,getattr(self, 'measured_speed_limit', MEASURED_SPEED_LIMIT))
            if enabled and abs(velocity) > speed_limit:
                raise RuntimeError(f'Joint {i+1} velocity {velocity:.1f} degrees/second exceeds {speed_limit:g}')
            values.append(math.degrees(position))
        return values

    def prepare(self):
        m = self.motors[5]
        # Previous physical wave established MIT operation. Refuse a mode change.
        if m.get_register_u32(10, 200) != 1:
            raise RuntimeError('Wrist is not in the tested MIT mode')
        if not self.timeout_changed:
            self.timeout_saved = m.get_register_u32(9, 200)
            self.timeout_changed = True  # cleanup also covers a partial write
            m.set_can_timeout_ms(MOTOR_TIMEOUT_MS)
            expected = MOTOR_TIMEOUT_MS * DM_TIMEOUT_TICKS_PER_MS
            actual = m.get_register_u32(9, 200)
            if actual != expected:
                raise RuntimeError(f'{MOTOR_TIMEOUT_MS} ms motor timeout readback mismatch: expected {expected} ticks, got {actual}')

    def enable(self, position):
        self.active = True  # cleanup on partial enable failure
        self.motors[5].enable()
        self.command(position)

    def command(self, position):
        self.motors[5].send_mit(math.radians(position), 0., 18., 2., 0.)

    def disable(self):
        if self.active:
            self.motors[5].disable()
            self.active = False

    def __exit__(self, *args):
        try:
            self.disable()
            if self.timeout_changed:
                self.read()  # only restore after confirming disabled
                self.motors[5].write_register_u32(9, self.timeout_saved)
                if self.motors[5].get_register_u32(9, 200) != self.timeout_saved:
                    raise RuntimeError('Wrist timeout restoration failed')
        finally:
            super().__exit__(*args)


class WristWorkbench(Workbench):
    def __init__(self, pairing):
        super().__init__()
        saved = json.loads(Path(pairing).read_text())
        check = saved.get('direction_checks', {}).get('6', {})
        if (saved.get('demo') is not False or saved.get('geometry_fingerprint') != self.geometry.fingerprint
                or check.get('sign') not in (-1, 1) or check.get('sign') != saved['signs'][5]):
            raise ValueError('Expected a matching, physically checked wrist direction')
        # Reuse the verified direction only, never an old absolute software origin.
        self.mapping_sign = relative_wrist_sign(saved, self.geometry.signs[5])
        self.gate = HoldGate()
        self.ready = False
        self.fault = None
        self.active_id = None
        self.trial = {}
        self.message = 'Connecting; motors disabled'
        self.log_path = Path('outputs') / f'wrist_follow_{self.session}.jsonl'
        self.log_path.parent.mkdir(exist_ok=True)

    def record(self, event, **values):
        with self.log_path.open('a') as output:
            output.write(json.dumps({'utc': datetime.now(timezone.utc).isoformat(), 'event': event, **values}, allow_nan=False)+'\n')

    def action(self, request):
        with self.lock:
            if request.get('action') == 'press':
                if not self.ready or self.fault or request.get('clearance_confirmed') is not True:
                    raise ValueError('Wait for ready, and confirm support and cable clearance')
                self.fresh_leader()
            self.gate.action(request, time.monotonic())

    def fresh_leader(self):
        source = self.sources.get('leader', {})
        if 'angles' not in source or time.monotonic()-source['time'] > .25:
            raise RuntimeError('Leader disconnected or stale')
        return source['angles'][5]

    def cameras_ready(self):
        now = time.monotonic()
        return all('error' not in self.sources.get(role, {}) and
                   now-self.sources.get(role, {}).get('time', 0) < 1 for role in ('wrist', 'tripod'))

    def snapshot(self):
        with self.lock:
            state = super().snapshot()
            state.update(ready=self.ready and not self.fault, fault=self.fault,
                         active=self.active_id is not None, trial=self.trial.copy(),
                         speed_limit_deg_s=SPEED, travel_limit_deg=LIMIT,
                         acceleration_limit_deg_s2=ACCELERATION,
                         hold_requested=self.gate.lease is not None,
                         hold_elapsed_s=time.monotonic()-self.gate.lease['began'] if self.gate.lease else 0)
            return state

    def motor_worker(self, factory=FollowArm):
        try:
            with factory() as arm:
                origin = arm.read()
                model_wrist = origin[5]*self.geometry.signs[5]+math.degrees(self.geometry.offsets[5])
                limit = self.geometry.joints[5].find('limit')
                low, high = math.degrees(float(limit.get('lower'))), math.degrees(float(limit.get('upper')))
                if not low+LIMIT+ENVELOPE_MARGIN < model_wrist < high-LIMIT-ENVELOPE_MARGIN:
                    raise RuntimeError('Wrist too near its model limit')
                previous = origin[5]
                last = time.monotonic()
                self.record('connected', raw_deg=origin, mapping_sign=self.mapping_sign,
                            speed_limit_deg_s=SPEED, travel_limit_deg=LIMIT,
                            acceleration_limit_deg_s2=ACCELERATION, measured_speed_limit_deg_s=MEASURED_SPEED_LIMIT,
                            tracking_error_limit_deg=TRACKING_ERROR_LIMIT)
                while not self.stop.is_set():
                    # Release before a potentially blocking read.
                    with self.lock:
                        held = self.gate.valid(time.monotonic())
                    if arm.active and not held:
                        arm.disable()
                        q = arm.read()
                        with self.lock:
                            self.active_id = None
                            self.message = 'Released; all six motors confirmed disabled'
                        self.record('disabled', raw_deg=q)
                    q = arm.read()
                    self.publish('follower', angles=q)
                    if self.stop.is_set():
                        break
                    now = time.monotonic()
                    with self.lock:
                        held = self.gate.valid(now)
                        cameras = self.cameras_ready()
                        try:
                            leader = self.fresh_leader()
                            leader_age = time.monotonic()-self.sources['leader']['time']
                            leader_ok = True
                        except RuntimeError:
                            leader_ok = False
                        self.ready = leader_ok and cameras and self.active_id is None
                        if held and (not leader_ok or not cameras):
                            raise RuntimeError('Leader or camera stream stale; release and restart this server')
                        lease_id = (self.gate.lease['client'], self.gate.lease['id']) if held else None
                    if held:
                        if max(abs(a-b) for a,b in zip(q[:5], origin[:5])) > 1 or abs(q[5]-origin[5]) > LIMIT+ENVELOPE_MARGIN:
                            raise RuntimeError('Follower moved outside the trial envelope')
                        if not arm.active:
                            check_envelope(q, origin)
                            arm.prepare()
                            q = arm.read()
                            check_envelope(q, origin)
                            with self.lock:
                                # A release/expired hold during setup must never enable.
                                if not self.gate.valid(time.monotonic()) or lease_id != (self.gate.lease['client'], self.gate.lease['id']):
                                    continue
                                leader_baseline = self.fresh_leader()
                                baseline = previous = q[5]
                                trajectory = WristTrajectory(origin[5], baseline)
                                arm.enable(previous)
                                self.active_id = lease_id
                                self.message = 'Following wrist roll; release to disable'
                            self.record('enabled', leader_baseline=leader_baseline, follower_baseline=baseline)
                            last = time.monotonic()
                            self.stop.wait(.02)
                            continue
                        if lease_id != self.active_id:
                            raise RuntimeError('Control ownership changed; stopped')
                        if abs(q[5]-previous) > TRACKING_ERROR_LIMIT:
                            raise RuntimeError(f'Wrist tracking error {abs(q[5]-previous):.2f} degrees exceeds {TRACKING_ERROR_LIMIT:g}')
                        desired = mapped_target(origin[5], baseline, leader_baseline, leader, self.mapping_sign)
                        target = trajectory.step(desired, now-last)
                        with self.lock:
                            # Do not send a movement command after a processed release.
                            if self.gate.valid(time.monotonic()) and lease_id == (self.gate.lease['client'], self.gate.lease['id']):
                                arm.command(target)
                                previous = target
                        self.record('sample', leader_delta_deg=leader-leader_baseline,
                                    target_offset_deg=previous-origin[5], actual_offset_deg=q[5]-origin[5],
                                    control_dt_s=now-last, leader_age_s=leader_age,
                                    target_velocity_deg_s=trajectory.velocity,
                                    measured_velocity_deg_s=arm.wrist_velocity_deg_s)
                    with self.lock:
                        self.trial = {'leader_delta_deg': leader-leader_baseline if arm.active and leader_ok else 0,
                                      'target_offset_deg': previous-origin[5] if arm.active else q[5]-origin[5],
                                      'actual_offset_deg': q[5]-origin[5]}
                        if not arm.active:
                            self.message = 'Ready; wrist disabled' if self.ready else 'Waiting for leader and both cameras; wrist disabled'
                    last = now
                    self.stop.wait(.005)
        except Exception as error:
            with self.lock:
                self.fault = str(error)
                self.ready = False
                self.gate.lease = None
                self.active_id = None
                self.message = 'Stopped: '+str(error)
            self.record('fault', error=str(error))
            print(self.message, flush=True)


def handler(workbench, token):
    parent = make_handler(workbench, token)
    class Handler(parent):
        def do_GET(self):
            if self.path == '/':
                self.reply(200, (ASSETS/'wrist_follow.html').read_text().replace('__TOKEN__', token).encode(), 'text/html; charset=utf-8')
            else:
                super().do_GET()

        def do_POST(self):
            try:
                super().do_POST()
            except RuntimeError as error:
                self.reply(400, json.dumps({'error': str(error)}).encode())
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pairing', type=Path, required=True)
    parser.add_argument('--port', type=int, default=8765)
    parser.add_argument('--check-timeout', action='store_true', help='Verify timeout setup/restoration while disabled, then exit; never enable')
    args = parser.parse_args()
    if args.check_timeout:
        with FollowArm() as arm:
            arm.read()
            old = arm.motors[5].get_register_u32(9, 200)
            arm.prepare()
            arm.read()
            actual = arm.motors[5].get_register_u32(9, 200)
            print(f'Timeout verified: {actual} ticks = {actual/DM_TIMEOUT_TICKS_PER_MS:g} ms; all six motors disabled', flush=True)
        with FollowArm() as arm:
            arm.read()
            restored = arm.motors[5].get_register_u32(9, 200)
            if restored != old:
                raise RuntimeError(f'Timeout restoration failed: expected {old}, got {restored}')
            print(f'Original timeout restored: {restored} ticks; all six motors disabled', flush=True)
        return
    workbench = WristWorkbench(args.pairing)
    workers = [threading.Thread(target=workbench.bus_worker, args=('leader', LeaderReader, .02), daemon=True),
               threading.Thread(target=workbench.motor_worker)]
    workers += [threading.Thread(target=workbench.camera_worker, args=(role,), daemon=True) for role in ('wrist', 'tripod')]
    server = ThreadingHTTPServer(('127.0.0.1', args.port), handler(workbench, secrets.token_urlsafe(24)))
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    for worker in workers:
        worker.start()
    print(f'Wrist-only hold-to-run: http://127.0.0.1:{args.port} — starts disabled', flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        workbench.stop.set()
        server.server_close()
        for worker in workers:
            worker.join(timeout=5)


if __name__ == '__main__':
    main()
