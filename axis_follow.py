#!/usr/bin/env python3
"""Selected-axis or coordinated REbot following while the operator holds a button."""
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

from camera_check import CameraCheck
from arm_control import B601_GAINS, GAIN_REGISTERS
from calibrate_arm import Reader
from leader_read import LeaderReader, NAMES
from pairing_dashboard import ASSETS
from wrist_follow import (FollowArm, WristWorkbench, WristTrajectory, HoldGate, handler,
                          MOTOR_TIMEOUT_MS, DM_TIMEOUT_TICKS_PER_MS)

# First three axes use the existing B601 position/velocity gain profile.
# Wrist bend/yaw retain their current position gains. Roll keeps its tested MIT gains.
PROFILES = [
    dict(range=60., speed=18., acceleration=60., measured_speed=60., mode=2),
    dict(range=60., speed=18., acceleration=60., measured_speed=60., mode=2),
    dict(range=90., speed=18., acceleration=60., measured_speed=60., mode=2),
    dict(range=45., speed=24., acceleration=80., measured_speed=75., mode=2),
    dict(range=45., speed=24., acceleration=80., measured_speed=75., mode=2),
    dict(range=45., speed=24., acceleration=80., measured_speed=75., mode=1),
]
TRACKING_ERROR = 8.0


class AxisGate(HoldGate):
    """Keep the cause of cancellation, including server-side heartbeat expiry."""
    def __init__(self):
        super().__init__()
        self.reason = 'not_started'

    def cancel(self, reason):
        self.lease = None
        self.reason = reason

    def valid(self, now):
        had_lease = self.lease is not None
        valid = super().valid(now)
        if had_lease and not valid:
            self.reason = 'heartbeat_expired'
        return valid

    def action(self, request, now):
        client, seq = request.get('client'), request.get('seq')
        accepted = isinstance(client, str) and isinstance(seq, int) and seq > self.sequences.get(client, -1)
        super().action(request, now)
        if accepted and request.get('action') == 'release':
            reason = request.get('reason', 'operator_release')
            self.reason = reason[:100] if isinstance(reason, str) else 'operator_release'


class ClutchedTarget:
    """Discard leader travel beyond a limit so reversing responds immediately."""
    def __init__(self, position, leader, sign, low, high):
        self.position, self.leader, self.sign = position, leader, sign
        self.low, self.high = low, high
        self.limited = False

    def update(self, leader):
        if not math.isfinite(leader):
            raise RuntimeError('Invalid leader position')
        desired = self.position + self.sign * (leader-self.leader)
        self.leader = leader
        self.position = max(self.low, min(self.high, desired))
        self.limited = desired <= self.low or desired >= self.high
        return self.position

    def rebase_leader(self, leader):
        if not math.isfinite(leader):
            raise RuntimeError('Invalid leader position')
        self.leader = leader


class ReferencedTarget:
    """Clamp output without changing the reference when leader travel saturates.

    Returning the leader to its press-time reference returns the requested
    follower pose, even after excursions beyond either follower limit.
    """
    def __init__(self, position, leader, sign, low, high):
        if (not all(math.isfinite(v) for v in (position, leader, low, high)) or
                sign not in (-1, 1) or not low <= position <= high):
            raise ValueError('Invalid relative reference')
        self.origin = self.position = position
        self.leader_reference = leader
        self.sign, self.low, self.high = sign, low, high
        self.limited = False

    def rebase_leader(self, leader):
        if not math.isfinite(leader):
            raise RuntimeError('Invalid leader position')
        self.leader_reference = leader
        self.origin = self.position

    def update(self, leader):
        if not math.isfinite(leader):
            raise RuntimeError('Invalid leader position')
        desired = self.origin + self.sign*(leader-self.leader_reference)
        self.position = max(self.low, min(self.high, desired))
        self.limited = desired <= self.low or desired >= self.high
        return self.position


def axis_bounds(geometry, selected, position):
    """Intersect local travel with model limits; permit only inward travel if
    a <=1° software-reference discrepancy places the starting pose just outside.
    No command goes farther outside than the existing supported position.
    """
    limit = geometry.joints[selected].find('limit')
    sign, offset = geometry.signs[selected], math.degrees(geometry.offsets[selected])
    model_low, model_high = [math.degrees(float(limit.get(k))) for k in ('lower','upper')]
    raw_low, raw_high = sorted(((model_low-offset)/sign, (model_high-offset)/sign))
    if not math.isfinite(position) or not raw_low-1 <= position <= raw_high+1:
        raise ValueError(f'{NAMES[selected]} starting pose is outside its model limits; check the supported pose')
    radius = PROFILES[selected]['range']
    return max(position-radius, min(position, raw_low+.25)), min(position+radius, max(position, raw_high-.25))


def raw_signs(saved, geometry):
    signs = []
    for i in range(6):
        check = saved.get('direction_checks', {}).get(str(i+1), {})
        if check.get('sign') not in (-1,1) or check['sign'] != saved['signs'][i]:
            raise ValueError(f'Missing visual direction for joint {i+1}')
        physical = saved.get('relative_follow_checks', {}).get(str(i+1))
        sign = int(check['sign']*geometry.signs[i]) if physical is None else physical.get('raw_leader_to_raw_follower_sign')
        if type(sign) is not int or sign not in (-1,1):
            raise ValueError('Invalid saved relative direction')
        signs.append(sign)
    return signs


def starting_targets(q,limits):
    """Tolerate <=0.1° of idle encoder/settling variation without expanding
    any target boundary. The enable/hold targets remain clipped inside limits.
    """
    targets={}
    for i,(low,high) in limits.items():
        if not math.isfinite(q[i]) or q[i]<low-.1 or q[i]>high+.1:
            raise RuntimeError(f'{NAMES[i]} outside starting envelope: {q[i]:.3f} degrees; allowed {low:.3f} to {high:.3f}. Refresh the supported starting pose.')
        targets[i]=max(low,min(high,q[i]))
    return targets


class AxisArm(FollowArm):
    def __init__(self):
        super().__init__()
        from fast_feedback import configure_library
        self.use_fresh_feedback=configure_library()
        self._fresh_reader=None
        self.selected = 4
        self.original = {}
        self.enabled_indices = set()
        self.speed_limits = {i:p['measured_speed'] for i,p in enumerate(PROFILES)}
        self.measured_speed_limit = PROFILES[self.selected]['measured_speed']

    def __enter__(self):
        super().__enter__()
        try:
            if self.use_fresh_feedback:
                from fast_feedback import FreshMotorReader
                self._fresh_reader=FreshMotorReader(self.motors)
            return self
        except BaseException:
            Reader.__exit__(self,None,None,None)
            raise

    def select(self, selected):
        if self.active or selected not in range(6):
            raise ValueError('Release and disable before selecting a joint')
        self.selected = selected
        self.measured_speed_limit = PROFILES[selected]['measured_speed']

    def prepare(self):
        if self.active:
            raise RuntimeError('Cannot configure an enabled joint')
        self.read()
        i, m = self.selected, self.motors[self.selected]
        if i not in self.original:
            old = {'mode':m.get_register_u32(10,200), 'timeout':m.get_register_u32(9,200)}
            if old['mode'] not in (1,2,3,4):
                raise RuntimeError('Invalid original motor mode')
            if i < 5:
                gains = [m.get_register_f32(rid,200) for rid in GAIN_REGISTERS]
                if not all(math.isfinite(x) and x>=0 for x in gains) or gains[0]<=0 or gains[2]<=0:
                    raise RuntimeError('Invalid or zero position-controller gains')
                if i < 3:
                    old['gains'] = gains
            self.original[i] = old  # preserve before any writes
        if i < 3:
            for rid,value in zip(GAIN_REGISTERS,B601_GAINS[i]):
                m.write_register_f32(rid,value)
                if not math.isclose(m.get_register_f32(rid,200),value,rel_tol=1e-5,abs_tol=1e-7):
                    raise RuntimeError(f'Joint {i+1} gain readback mismatch')
        m.ensure_mode(PROFILES[i]['mode'],1000)
        if m.get_register_u32(10,200) != PROFILES[i]['mode']:
            raise RuntimeError('Control-mode readback mismatch')
        m.set_can_timeout_ms(MOTOR_TIMEOUT_MS)
        actual = m.get_register_u32(9,200)
        if actual != MOTOR_TIMEOUT_MS*DM_TIMEOUT_TICKS_PER_MS:
            raise RuntimeError(f'Timeout readback mismatch: {actual} ticks')

    def enable(self, position):
        self.enable_group({self.selected:position})

    def prepare_group(self,indices):
        selected = self.selected
        try:
            for i in indices:
                self.select(i)
                self.prepare()
        finally:
            self.select(selected)

    def enable_group(self,targets):
        self.command_group(targets,take_up=True)
        self.active = True
        for i,position in targets.items():
            self.enabled_indices.add(i)  # cleanup also covers partial enable
            self.motors[i].enable()
            self.command_axis(i,position,take_up=True)

    def command_group(self,targets,take_up=False):
        for i,position in targets.items():
            self.command_axis(i,position,take_up=take_up)

    def command(self, position, take_up=False):
        self.command_axis(self.selected,position,take_up)

    def command_axis(self,i,position,take_up=False):
        if i not in range(6) or not math.isfinite(position):
            raise ValueError('Invalid arm target')
        m = self.motors[i]
        if i == 5:
            m.send_mit(math.radians(position),0.,18.,2.,0.)
        else:
            speed=getattr(self,'command_speed_limits',{}).get(i,PROFILES[i]['speed']*1.5)
            if not math.isfinite(speed) or not 0<speed<=60.:
                raise ValueError('Invalid transient command speed limit')
            m.send_pos_vel(math.radians(position), .01 if take_up else math.radians(speed))

    def disable(self):
        errors=[]
        for i in sorted(self.enabled_indices):
            try:
                self.motors[i].disable()
                self.enabled_indices.remove(i)
            except Exception as error:
                errors.append(f'Joint {i+1}: {error}')
        self.active = bool(self.enabled_indices)
        if errors:
            raise RuntimeError('Disable failed; use physical power cutoff. '+'; '.join(errors))

    def __exit__(self,*args):
        try:
            self.disable()
            if self.original:
                self.read()
                for i,old in self.original.items():
                    m = self.motors[i]
                    m.ensure_mode(old['mode'],1000)
                    if m.get_register_u32(10,200) != old['mode']:
                        raise RuntimeError(f'Joint {i+1} mode restoration failed')
                    for rid,value in zip(GAIN_REGISTERS,old.get('gains',[])):
                        m.write_register_f32(rid,value)
                        if not math.isclose(m.get_register_f32(rid,200),value,rel_tol=1e-5,abs_tol=1e-7):
                            raise RuntimeError(f'Joint {i+1} gain restoration failed')
                    m.write_register_u32(9,old['timeout'])
                    if m.get_register_u32(9,200) != old['timeout']:
                        raise RuntimeError(f'Joint {i+1} timeout restoration failed')
        finally:
            if self._fresh_reader:self._fresh_reader.close();self._fresh_reader=None
            Reader.__exit__(self,*args)


class AxisWorkbench(WristWorkbench):
    target_mapper = ClutchedTarget
    response_time = .08
    brake_at_target = False

    def make_target_mappers(self, baseline, leader, following, limits):
        return {j:self.target_mapper(baseline[j], leader[j], self.follow_signs[j], *limits[j])
                for j in following}

    def update_take_up_reference(self, mapped, leader, following):
        for j in following:
            mapped[j].rebase_leader(leader[j])

    def __init__(self,pairing):
        super().__init__(pairing)
        self.pairing = Path(pairing)
        self.saved = json.loads(self.pairing.read_text())
        self.follow_signs = raw_signs(self.saved,self.geometry)
        self.selected = 4  # next hinge inward from the successful roll test
        self.control_mode = 'follow_all'
        self.axis_trials = []
        self.revision = 0
        self.bounds = None
        self.observed = set()
        self.gate = AxisGate()
        self.powered = False
        self.feedback_reader = None
        self.disable_requested = False
        self.pause_reason = None
        self.camera_check = CameraCheck(self.geometry)
        self.joint_calibration = None
        self.log_path = Path('outputs')/f'axis_follow_{self.session}.jsonl'

    def publish(self, name, **values):
        super().publish(name, **values)
        if name == 'follower' and hasattr(self, 'camera_check'):
            self.camera_check.add_joints(time.monotonic(), values['angles'], powered=self.powered,
                following=self.active_id is not None or self.gate.lease is not None,
                all_powered=self.control_mode == 'follow_all', fault=self.fault)

    def fresh_leader(self):
        source = self.sources.get('leader',{})
        if 'angles' not in source or time.monotonic()-source['time'] > .25:
            raise RuntimeError('Leader disconnected or stale')
        return source['angles'][self.selected]

    def save_direction(self,confirmed):
        checks = self.saved.setdefault('relative_follow_checks',{})
        key = str(self.selected+1)
        old = checks.get(key,{})
        updated = {**old, 'raw_leader_to_raw_follower_sign':self.follow_signs[self.selected],
                   'corrected_direction_physically_confirmed':confirmed,
                   'method':'operator_confirmed_physical_following' if confirmed else 'operator_reversed_direction_in_dashboard',
                   'updated_utc':datetime.now(timezone.utc).isoformat(), 'source_log':str(self.log_path)}
        if old:
            self.saved.setdefault('relative_follow_history',[]).append({'joint':self.selected+1,**old})
        checks[key] = updated
        temp = self.pairing.with_suffix('.tmp')
        temp.write_text(json.dumps(self.saved,indent=2,allow_nan=False)+'\n')
        temp.replace(self.pairing)

    def action(self,request):
        if request.get('action') == 'camera_capture':
            self.camera_check.arm_capture()
            return
        if request.get('action') == 'camera_restart':
            self.camera_check.restart()
            return
        with self.lock:
            op = request.get('action')
            if op in ('select','reverse','confirm_direction','set_control_mode'):
                if self.powered or self.active_id is not None or self.gate.lease is not None:
                    raise ValueError('Support the arm and remove motor power before changing configuration')
                if self.fault:
                    raise ValueError('Restart the server after this fault')
                if request.get('revision') != self.revision:
                    raise ValueError('The selected joint changed; refresh status')
                if op == 'set_control_mode':
                    mode=request.get('control_mode')
                    if mode not in ('single','hold_all','follow_all'):
                        raise ValueError('Unknown following mode')
                    self.control_mode=mode
                    self.bounds=None
                elif op == 'select':
                    joint = request.get('joint')
                    if type(joint) is not int or joint not in range(1,7):
                        raise ValueError('Select joint 1 through 6')
                    self.selected = joint-1
                    self.bounds = None
                elif op == 'reverse':
                    self.follow_signs[self.selected] *= -1
                    self.observed.discard(self.selected)
                    self.save_direction(False)
                else:
                    if self.selected not in self.observed:
                        raise ValueError('First move this axis using a hold, then confirm the physical direction')
                    self.save_direction(True)
                self.revision += 1
                self.ready = False
                return
            if op == 'disable':
                if request.get('support_confirmed') is not True:
                    raise ValueError('Support the arm before removing motor power')
                release = {**request, 'action':'release', 'reason':'operator_torque_off'}
                client, seq = request.get('client'), request.get('seq')
                previous_seq = self.gate.sequences.get(client, -1) if isinstance(client, str) else -1
                self.gate.action(release, time.monotonic())
                if seq > previous_seq:
                    self.disable_requested = True
                    self.ready = False
                    self.record('torque_off_requested', reason='operator_torque_off')
                return
            if op == 'press' and self.disable_requested:
                raise ValueError('Wait for motor power removal to finish')
            if op == 'release':
                before = self.gate.lease
                super().action(request)
                if before is not None and self.gate.lease is None:
                    self.record('pause_requested', reason=self.gate.reason)
                return
            if op == 'press' and (request.get('joint') != self.selected+1 or request.get('revision') != self.revision):
                raise ValueError('Joint selection changed; try the hold again')
            super().action(request)

    def snapshot(self):
        camera_status = self.camera_check.snapshot()
        with self.lock:
            result = super().snapshot()
            result['camera_check'] = camera_status
            result['joint_calibration'] = self.joint_calibration
            cfg = PROFILES[self.selected]
            result.update(selected_joint=self.selected+1, selected_name=NAMES[self.selected], revision=self.revision,
                feedback_reader=self.feedback_reader,
                control_mode=self.control_mode, axis_trials=self.axis_trials,
                active=self.powered, powered=self.powered, following=self.active_id is not None,
                pause_reason=self.pause_reason, tracking_error_limit_deg=TRACKING_ERROR,
                power_state='unknown_after_fault' if self.fault else ('on' if self.powered else 'off'),
                speed_limit_deg_s=cfg['speed'], travel_limit_deg=cfg['range'], acceleration_limit_deg_s2=cfg['acceleration'],
                bounds=self.bounds, directions=[bool(self.saved.get('relative_follow_checks',{}).get(str(i+1),{}).get('corrected_direction_physically_confirmed')) for i in range(6)],
                follow_sign=self.follow_signs[self.selected], can_confirm=self.selected in self.observed)
            return result

    def motor_worker(self, factory=AxisArm):
        original_error=None
        try:
            with factory() as arm:
                self.feedback_reader='concurrent_experimental' if getattr(arm,'_fresh_reader',None) is not None else 'sequential'
                try:
                    self.run_motor(arm)
                except Exception as error:
                    original_error=error
                    raise
            with self.lock:
                self.powered = False
                self.active_id = None
            self.record('shutdown_complete', reason='server_shutdown', motors_disabled=True)
        except Exception as error:
            primary=original_error if original_error is not None else error
            cleanup_error=str(error) if original_error is not None and error is not original_error else None
            with self.lock:
                self.ready = False
                self.fault = str(primary)
                self.powered = None  # unknown until fresh hardware verification
                self.gate.cancel('motor_or_control_fault')
                self.active_id = None
                self.message = 'Fault; motor state requires verification: '+str(primary)
            self.record('shutdown_requested',reason='motor_or_control_fault',error=str(primary))
            self.record('fault', reason='motor_or_control_fault', error=str(primary), cleanup_error=cleanup_error, torque_off_attempted=True)
            print(self.message, flush=True)
        finally:
            try:self.after_motor_shutdown()
            except Exception as error:
                self.record('report_cleanup_failed',error=str(error))

    def after_motor_shutdown(self):
        """Subclass hook for work that must not delay motor-owner cleanup."""
        pass

    def run_auxiliary_motion(self, arm):
        return False

    def run_motor(self, arm):
        revision = -1
        last = time.monotonic()
        while not self.stop.is_set():
            if self.run_auxiliary_motion(arm):
                revision = -1
                last = time.monotonic()
                continue
            with self.lock:
                if self.disable_requested:
                    arm.disable()
                    self.powered = False
                    self.active_id = None
                    self.disable_requested = False
                    self.gate.cancel('operator_torque_off')
                    self.pause_reason = 'operator_torque_off'
                    self.record('disabled', reason='operator_torque_off')
            q = arm.read()
            self.publish('follower', angles=q)
            if self.stop.is_set():
                break
            now = time.monotonic()
            # An unsupported starting envelope blocks teaching, but disabled
            # readback must stay alive so an operator can reposition and retry.
            if not arm.active and self.bounds is None:
                with self.lock:
                    candidates=[self.selected] if self.control_mode=='single' else list(range(6))
                    try:
                        for joint in candidates:axis_bounds(self.geometry,joint,q[joint])
                    except ValueError as error:
                        self.ready=False
                        self.gate.cancel('starting_pose_invalid')
                        self.message=str(error)+' · motors disabled; reposition to recover'
                        blocked=True
                    else:blocked=False
                if blocked:
                    self.stop.wait(.02)
                    last=time.monotonic()
                    continue
            with self.lock:
                if revision != self.revision:
                    if arm.active:
                        raise RuntimeError('Selection changed while powered')
                    i, mode = self.selected, self.control_mode
                    controlled = [i] if mode == 'single' else list(range(6))
                    following = controlled if mode == 'follow_all' else [i]
                    arm.select(i)
                    if self.bounds is None:
                        limits = {j:axis_bounds(self.geometry, j, q[j]) for j in controlled}
                        origin = list(q)
                        self.bounds = {'low_deg':limits[i][0]-q[i], 'high_deg':limits[i][1]-q[i]}
                    revision = self.revision
                    previous = {j:q[j] for j in controlled}
                    self.record('selected', joint=i+1, control_mode=mode, raw_deg=q,
                                limits=limits, mapping_signs=self.follow_signs, profiles=PROFILES)
                held = self.gate.valid(now)
                leader = None
                leader_age = None
                input_reason = None
                try:
                    self.fresh_leader()
                    leader = list(self.sources['leader']['angles'][:6])
                    leader_age = now-self.sources['leader']['time']
                except RuntimeError:
                    input_reason = 'leader_stale'
                if input_reason is None and not self.cameras_ready():
                    input_reason = 'camera_stale'
                if held and input_reason:
                    self.gate.cancel(input_reason)
                    held = False
                    self.record('pause_requested', reason=input_reason)
                lease_id = (self.gate.lease['client'], self.gate.lease['id']) if held else None
                if arm.active:
                    if now-last > .3:
                        raise RuntimeError('Motor control loop stalled')
                    if any(not limits[j][0]-2 <= q[j] <= limits[j][1]+2 for j in controlled):
                        raise RuntimeError('A controlled joint left its travel envelope')
                    for j in controlled:
                        error = abs(q[j]-previous[j])
                        if error > TRACKING_ERROR:
                            raise RuntimeError(f'{NAMES[j]} tracking error {error:.2f} degrees exceeds {TRACKING_ERROR:g}')
                if arm.active or held:
                    if any(abs(q[j]-origin[j]) > 2 for j in range(6) if j not in controlled):
                        raise RuntimeError('An unpowered follower joint moved more than 2 degrees; use an all-joints mode')
                # Stop chasing the old target on release. Hold the latest measured
                # position (clipped to the unchanged command bounds), with torque on.
                if arm.active and self.active_id is not None and lease_id != self.active_id:
                    previous = {j:max(limits[j][0], min(limits[j][1], q[j])) for j in controlled}
                    self.active_id = None
                    self.pause_reason = self.gate.reason if not held else 'new_press'
                    self.record('paused', reason=self.pause_reason, hold_raw_deg=previous, powered=True)
                if held and self.active_id is None:
                    newly_enabled = not arm.active
                    if newly_enabled:
                        starting_targets(q, limits)
                        # Configuration is slow; keep HTTP heartbeats and input
                        # publication running, then revalidate ownership before enable.
                        self.lock.release()
                        try:
                            arm.prepare_group(controlled)
                            q = arm.read()
                        finally:
                            self.lock.acquire()
                        if any(abs(q[j]-origin[j]) > 2 for j in range(6) if j not in controlled):
                            raise RuntimeError('Supported pose changed during setup')
                        previous = starting_targets(q, limits)
                    # Setup can take longer than the lease, even with the lock held.
                    if (self.stop.is_set() or self.disable_requested or revision != self.revision or
                            not self.gate.valid(time.monotonic()) or
                            lease_id != (self.gate.lease['client'], self.gate.lease['id'])):
                        last = time.monotonic()
                        continue
                    try:
                        self.fresh_leader()
                    except RuntimeError:
                        self.gate.cancel('leader_stale')
                        last = time.monotonic()
                        continue
                    if not self.cameras_ready():
                        self.gate.cancel('camera_stale')
                        last = time.monotonic()
                        continue
                    leader = list(self.sources['leader']['angles'][:6])
                    leader_baseline = list(leader)
                    baseline = list(q)
                    for j in controlled:
                        baseline[j] = previous[j]
                    trajectories = {j:WristTrajectory(origin[j], baseline[j], low=limits[j][0], high=limits[j][1],
                        speed=PROFILES[j]['speed'], acceleration=PROFILES[j]['acceleration'], response_time=self.response_time,
                        brake_at_target=self.brake_at_target) for j in following}
                    mapped = self.make_target_mappers(baseline, leader, following, limits)
                    if not self.gate.valid(time.monotonic()):
                        last = time.monotonic()
                        continue
                    if newly_enabled:
                        arm.enable_group(previous)
                    self.powered = True
                    self.active_id = lease_id
                    self.pause_reason = None
                    enabled_at = last = time.monotonic()
                    settled = not newly_enabled or controlled == [5]
                    self.record('enabled' if newly_enabled else 'resumed', control_mode=mode,
                        controlled_joints=[j+1 for j in controlled], leader_baseline=leader_baseline,
                        follower_baseline=baseline, measured_start_raw_deg=q)
                elif held:
                    taking_load = not settled and now-enabled_at < .8
                    if not settled and not taking_load:
                        if any(abs(q[j]-baseline[j]) > 1 for j in controlled):
                            raise RuntimeError('Arm did not settle at its starting pose')
                        settled = True
                    targets = {j:baseline[j] for j in controlled}
                    if taking_load:
                        self.update_take_up_reference(mapped, leader, following)
                    for j in following:
                        if not taking_load:
                            targets[j] = trajectories[j].step(mapped[j].update(leader[j]), now-last)
                    if self.gate.valid(time.monotonic()):
                        arm.command_group(targets, take_up=taking_load)
                        previous = targets
                        for j in following:
                            if abs(leader[j]-leader_baseline[j]) >= 2 and abs(q[j]-baseline[j]) >= 1:
                                self.observed.add(j)
                    self.record('sample', control_mode=mode, leader_delta_deg=[a-b for a,b in zip(leader,leader_baseline)],
                        target_offset_deg={j+1:previous[j]-origin[j] for j in controlled},
                        actual_offset_deg=[a-b for a,b in zip(q,origin)], measured_velocity_deg_s=arm.joint_velocities_deg_s,
                        control_dt_s=now-last, leader_age_s=leader_age,
                        mapped_offset_deg={j+1:mapped[j].position-origin[j] for j in following},
                        limited_joints=[j+1 for j in following if mapped[j].limited])
                elif arm.active:
                    arm.command_group(previous)  # maintain motor watchdog and support without browser/leader
                self.ready = input_reason is None and self.active_id is None and not self.disable_requested
                self.axis_trials = [{'joint':j+1, 'name':NAMES[j],
                    'leader_delta_deg':leader[j]-leader_baseline[j] if self.active_id and leader else 0,
                    'target_offset_deg':previous[j]-origin[j], 'actual_offset_deg':q[j]-origin[j],
                    'low_deg':limits[j][0]-origin[j], 'high_deg':limits[j][1]-origin[j],
                    'speed_deg_s':PROFILES[j]['speed'],
                    'following_error_deg':mapped[j].position-q[j] if self.active_id and j in following else None,
                    'limited':bool(self.active_id and j in following and (mapped[j].limited or
                        min(abs(mapped[j].position-limits[j][0]), abs(mapped[j].position-limits[j][1])) < .05))}
                    for j in controlled]
                self.trial = next(row for row in self.axis_trials if row['joint'] == i+1)
                if self.active_id:
                    self.message = 'Taking hold; keep the arm supported' if not settled else 'Following; release to pause and hold'
                elif arm.active:
                    self.message = 'Paused — motors holding position · '+str(self.pause_reason)
                else:
                    self.message = 'Ready; motors disabled' if self.ready else 'Waiting for leader and cameras; motors disabled'
            last = time.monotonic() if now < last else now
            self.stop.wait(.005)


def make_axis_handler(workbench,token):
    parent = handler(workbench,token)
    class Handler(parent):
        def do_GET(self):
            if self.path.startswith('/camera-check/image/'):
                import re
                match = re.fullmatch(r'/camera-check/image/([0-9]{8}T[0-9]{12}Z)/([0-3])\.jpg', self.path)
                if match:
                    path = workbench.camera_check.output/match[1]/(match[2]+'.jpg')
                    if path.is_file():
                        self.reply(200, path.read_bytes(), 'image/jpeg')
                        return
                self.reply(404,b'Not found','text/plain')
            elif self.path=='/':
                self.reply(200,(ASSETS/'axis_follow.html').read_text().replace('__TOKEN__',token).encode(),'text/html; charset=utf-8')
            else:
                super().do_GET()
    return Handler


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pairing',type=Path,required=True)
    parser.add_argument('--port',type=int,default=8765)
    parser.add_argument('--camera-report',type=Path,help='Display a completed camera check from this geometry; does not apply calibration or move motors')
    parser.add_argument('--joint-calibration',type=Path,help='Display camera-referenced local motion results; does not change motor mapping')
    parser.add_argument('--check-settings',action='store_true',help='Verify each selected-axis configuration and restoration while all motors stay disabled; then exit')
    args = parser.parse_args()
    if args.check_settings:
        with AxisArm() as arm:
            start = arm.read()
            for i in range(6):
                arm.select(i)
                arm.prepare()
                q = arm.read()
                if max(abs(a-b) for a,b in zip(q,start))>.5:
                    raise RuntimeError('Supported arm moved during configuration check')
                print(f'{NAMES[i]}: mode {PROFILES[i]["mode"]} and 500 ms timeout verified; all motors disabled',flush=True)
            original = dict(arm.original)
        with AxisArm() as arm:
            arm.read()
            for i,old in original.items():
                m = arm.motors[i]
                if any(m.get_register_u32(rid,200)!=old[key] for rid,key in ((9,'timeout'),(10,'mode'))):
                    raise RuntimeError(f'Joint {i+1} settings not restored')
                if any(not math.isclose(m.get_register_f32(rid,200),v,rel_tol=1e-5,abs_tol=1e-7) for rid,v in zip(GAIN_REGISTERS,old.get('gains',[]))):
                    raise RuntimeError(f'Joint {i+1} gains not restored')
        print('All six original modes/timeouts and changed gains restored; no motors enabled',flush=True)
        return
    w = AxisWorkbench(args.pairing)
    if args.camera_report:
        w.camera_check.load_completed(args.camera_report)
    if args.joint_calibration:
        from joint_motion_calibration import load_profile
        w.joint_calibration = load_profile(args.joint_calibration, w.geometry.fingerprint)
    server = ThreadingHTTPServer(('127.0.0.1',args.port),make_axis_handler(w,secrets.token_urlsafe(24)))
    workers = [threading.Thread(target=w.bus_worker,args=('leader',LeaderReader,.02),daemon=True),threading.Thread(target=w.motor_worker)]
    workers += [threading.Thread(target=w.camera_worker,args=(role,),daemon=True) for role in ('wrist','tripod')]
    def stop(*_): raise KeyboardInterrupt
    signal.signal(signal.SIGTERM,stop)
    for worker in workers: worker.start()
    print(f'Axis following: http://127.0.0.1:{args.port} — all motors start disabled',flush=True)
    try:
        server.serve_forever(poll_interval=.1)
    except KeyboardInterrupt:
        pass
    finally:
        w.stop.set()
        server.server_close()
        for worker in workers: worker.join(timeout=8)


if __name__=='__main__': main()
