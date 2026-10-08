"""Attended GRBL jogs; separate port, bounded requests, no homing or unlock."""
import json
import math
from pathlib import Path
import time

import serial

PORT = '/dev/serial/by-path/pci-0000:05:00.4-usb-0:1.2:1.0-port0'
HEARTBEAT_S = .3
STEP_MM = 2.
FEED_MM_MIN = 1920.
ACCEL_MM_S2 = 40.
# GRBL blends collinear jog blocks. Keep only a short distance ahead of the
# latest reported position, including the one command awaiting acknowledgement.
LOOKAHEAD_MM = 18.
ACK_TIMEOUT_S = .5
PROGRESS_TIMEOUT_S = 2.


def parse_status(line):
    if not line.startswith('<') or not line.endswith('>'):
        raise ValueError('Invalid GRBL status')
    fields=line[1:-1].split('|'); values=dict(x.split(':',1) for x in fields[1:] if ':' in x)
    position=[float(x) for x in values.get('MPos','').split(',')]
    if len(position)!=3 or not all(math.isfinite(x) for x in position):
        raise ValueError('GRBL machine position missing or invalid')
    return {'state':fields[0], 'x_mm':position[0], 'pins':values.get('Pn','')}


def validate_settings(settings):
    expected={1:0.,3:1.,5:0.,13:0.,20:0.,21:1.,22:0.,100:5.,120:ACCEL_MM_S2}
    if any(settings.get(k)!=v for k,v in expected.items()):
        raise ValueError('Rail settings differ from the inspected, operator-confirmed configuration')
    if not settings.get(110,0)>=FEED_MM_MIN:
        raise ValueError('Rail feed/acceleration settings invalid')


def jog_command(direction):
    if direction not in ('left','right'):
        raise ValueError('Use left or right')
    # Operator: switch at right of ArduCam. Verified calibration: +X away.
    distance=STEP_MM if direction=='left' else -STEP_MM
    return f'$J=G91 G21 X{distance:.3f} F{FEED_MM_MIN:.0f}\n'.encode()


class RailJog:
    def __init__(self, workbench, control_enabled=True, reboot_offset_mm=0.):
        self.w=workbench;self.lease=None;self.sequences={}
        self.control_enabled=control_enabled
        self.reboot_offset_mm=float(reboot_offset_mm);self.coordinate_offset_mm=0.
        if not math.isfinite(self.reboot_offset_mm) or abs(self.reboot_offset_mm)>300:raise ValueError('Invalid rail reboot offset')
        self.state='Disconnected';self.x_mm=None;self.pins='';self.status_time=0.
        self.error=None;self.connected=False;self.in_flight=False;self.awaiting_ack=False
        self.motion_uncertain=False;self.cancelled_at=None;self.command_at=0.;self.target=None
        self.last_cancel_send=0.
        self.progress_at=0.;self.progress_x=None;self.direction_sign=1.
        self.travel=0.;self.commands=0
        self.log=workbench.recording_root/'rail.jsonl'

    def record(self,event,**values):
        with self.log.open('a') as output:
            output.write(json.dumps({'monotonic_s':time.monotonic(),'event':event,**values})+'\n')

    def busy(self):
        if not self.control_enabled:
            # Arm-only teaching requires current evidence the rail is stopped,
            # even when its alarm prevents jogging. Never infer stop from a
            # disconnected controller or an old status report.
            if not (self.connected and self.state in ('Idle','Alarm') and
                    0<=time.monotonic()-self.status_time<.4):
                return True
        return self.lease is not None or self.in_flight or self.state=='Jog' or self.motion_uncertain

    def arm_is_parked(self):
        follower=self.w.sources.get('follower',{})
        q=follower.get('angles')
        if (self.w.powered or self.w.active_id is not None or self.w.gate.lease is not None or
                self.w.fault or q is None or time.monotonic()-follower['time']>.25):
            return False
        reference=[math.degrees(v) for v in self.w.geometry.profile['reference_raw_rad']]
        return max(abs(q[j]-reference[j]) for j in (1,2))<=2.

    def allowed(self):
        now=time.monotonic()
        return (self.control_enabled and self.connected and not self.error and self.state in ('Idle','Jog') and
                'X' not in self.pins and 0<=now-self.status_time<.4 and self.arm_is_parked() and
                self.w.cameras_ready() and all(now-self.w.sources.get(role,{}).get('time',0)<.4
                                               for role in ('wrist','tripod')))

    def action(self,request):
        client,seq=request.get('client'),request.get('seq')
        if not isinstance(client,str) or len(client)>80 or type(seq) is not int:
            raise ValueError('Invalid rail request')
        if seq<=self.sequences.get(client,-1):return
        self.sequences[client]=seq
        op=request['action']
        if op=='rail_stop':
            self.lease=None
        elif op=='rail_start':
            if not self.control_enabled:
                raise ValueError('Rail jogging disabled: monitoring stopped rail for arm teaching only')
            if (request.get('clearance_confirmed') is not True or self.busy() or
                    not self.allowed() or self.state!='Idle'):
                raise ValueError('Rail requires a clear path, fresh cameras, idle GRBL, and the arm rested with motor power off')
            direction=request.get('direction');jog_command(direction)
            self.lease={'client':client,'id':seq,'direction':direction,'heartbeat':time.monotonic()}
            self.travel=0.
            self.target=self.x_mm;self.progress_x=self.x_mm;self.progress_at=time.monotonic()
            self.direction_sign=1. if direction=='left' else -1.
        elif op=='rail_heartbeat':
            if (self.lease and self.lease['client']==client and self.lease['id']==request.get('press_id') and
                    time.monotonic()-self.lease['heartbeat']<=HEARTBEAT_S):
                self.lease['heartbeat']=time.monotonic()
        else:raise ValueError('Unknown rail action')

    def begin_demo_leg(self, target_mm):
        """Bounded endpoint, same heartbeat/cancel/lookahead as attended jogs."""
        if (self.busy() or not self.allowed() or self.state!='Idle' or
                not math.isfinite(target_mm) or abs(target_mm-self.x_mm)>300.01):
            raise ValueError('Demo rail leg requires a parked arm, fresh views and at most 300 mm travel')
        if abs(target_mm-self.x_mm)<.1:return
        if abs((target_mm-self.x_mm)*5-round((target_mm-self.x_mm)*5))>1e-6:
            raise ValueError('Rail endpoint must match the 0.2 mm step scale')
        direction='left' if target_mm>self.x_mm else 'right'
        self.lease={'client':'internal-demo','id':0,'direction':direction,
                    'heartbeat':time.monotonic(),'endpoint_mm':target_mm}
        self.travel=0.;self.target=self.x_mm;self.progress_x=self.x_mm;self.progress_at=time.monotonic()
        self.direction_sign=1. if direction=='left' else -1.

    def snapshot(self):
        return {'connected':self.connected,'state':self.state,'x_mm':self.x_mm,'pins':self.pins,
                'jog_enabled':self.control_enabled,
                'error':self.error,'busy':self.busy(),'ready':self.allowed() and not self.busy(),
                'homed':False,'coordinate_offset_mm':self.coordinate_offset_mm,'feed_mm_s':FEED_MM_MIN/60.,'max_hold_mm':None,
                'lookahead_mm':LOOKAHEAD_MM,'acceleration_mm_s2':ACCEL_MM_S2,
                'commands':self.commands,'arm_parked':self.arm_is_parked()}

    def query(self,port,command):
        port.write(command.encode()+b'\n');lines=[];deadline=time.monotonic()+2
        while time.monotonic()<deadline:
            raw=port.readline()
            if not raw:continue
            line=raw.decode('ascii',errors='replace').strip();lines.append(line)
            if line=='ok':return lines
            if line.startswith(('error:','ALARM:')):raise RuntimeError('GRBL: '+line)
        raise RuntimeError('GRBL query timeout for '+repr(command)+': '+repr(lines))

    def accept_line(self,line):
        if line.startswith('<'):
            s=parse_status(line);self.state=s['state'];self.x_mm=s['x_mm']+self.coordinate_offset_mm;self.pins=s['pins']
            s['machine_x_mm']=s['x_mm'];s['x_mm']=self.x_mm
            self.status_time=time.monotonic()
            if self.in_flight and (self.progress_x is None or abs(self.x_mm-self.progress_x)>.1):
                self.progress_at=self.status_time;self.progress_x=self.x_mm
            if self.in_flight:
                self.record('status',**s)
        elif line=='ok':self.awaiting_ack=False
        elif line.startswith(('error:','ALARM:')):
            if line.startswith('ALARM:'):
                self.state='Alarm';self.status_time=0.
                if not self.control_enabled:
                    self.error='GRBL: '+line;self.record('alarm',line=line)
                    return
            raise RuntimeError('GRBL: '+line)
        elif line.startswith('Grbl'):raise RuntimeError('GRBL restarted; rail position invalidated')

    def tick(self,port):
        now=time.monotonic()
        if self.lease and (now-self.lease['heartbeat']>HEARTBEAT_S or not self.allowed()):
            self.lease=None
        if self.in_flight:
            if self.lease is None and now-self.last_cancel_send>=.1:
                # Repeat after the jog's acknowledgement: GRBL can ignore a
                # realtime cancel that arrives before it enters STATE_JOG.
                port.write(b'\x85');self.last_cancel_send=now
                if not self.awaiting_ack and self.cancelled_at is None:
                    self.cancelled_at=now;self.record('jog_cancel')
            fresh=self.status_time>max(self.command_at,(self.cancelled_at+.1) if self.cancelled_at else 0.)
            complete=(not self.awaiting_ack and self.state=='Idle' and fresh and
                      ((self.cancelled_at is not None) or
                       (self.lease is not None and abs(self.x_mm-self.target)<=.21)))
            if complete:
                self.in_flight=False;self.cancelled_at=None
                if self.lease and 'endpoint_mm' in self.lease and abs(self.x_mm-self.lease['endpoint_mm'])<=.1:
                    self.lease=None
                    self.record('demo_leg_complete',x_mm=self.x_mm,idle_release_setting_ms=0)
            elif self.awaiting_ack and now-self.command_at>ACK_TIMEOUT_S:
                raise RuntimeError('Rail jog acknowledgement timed out')
            elif now-self.progress_at>PROGRESS_TIMEOUT_S:
                raise RuntimeError('Rail jog stopped reporting progress')
        if self.lease and not self.awaiting_ack and self.cancelled_at is None:
            # Held-key heartbeat authorizes continued travel. Bound only the
            # outstanding distance, never enqueue an entire rail-length move.
            ahead=self.direction_sign*(self.target-self.x_mm)
            if ahead < -.21:
                raise RuntimeError('Rail reported position beyond the commanded endpoint')
            if ahead+STEP_MM>LOOKAHEAD_MM+1e-6:return
            direction=self.lease['direction'];distance=STEP_MM
            if 'endpoint_mm' in self.lease:
                remaining=self.direction_sign*(self.lease['endpoint_mm']-self.target)
                if remaining<-.1:raise RuntimeError('Rail demo endpoint exceeded')
                if remaining<=.1:return
                distance=min(distance,remaining)
            command=f'$J=G91 G21 X{self.direction_sign*distance:.3f} F{FEED_MM_MIN:.0f}\n'.encode()
            if not self.in_flight:self.progress_at=now
            self.command_at=now;self.target+=self.direction_sign*distance
            self.in_flight=True;self.awaiting_ack=True;self.travel+=distance
            port.write(command);self.commands+=1
            self.record('jog',direction=direction,command=command.decode().strip(),start_x=self.x_mm,
                        target_x=self.target,queued_mm=self.direction_sign*(self.target-self.x_mm),
                        scope='Unhomed relative jog; not an absolute rail position')

    def worker(self):
        port=serial.Serial(port=None,baudrate=115200,timeout=.05,write_timeout=.2,exclusive=True)
        port.dtr=False;port.rts=False;port.port=PORT
        try:
            port.open()
            # Some Arduino USB bridges reboot on open despite requested DTR.
            # Drain the boot banner before sending the first identity query.
            deadline=time.monotonic()+3.5
            while time.monotonic()<deadline:
                line=port.readline()
                if line:
                    decoded=line.decode('ascii',errors='replace').strip();self.record('startup',line=decoded)
                    if decoded.startswith('Grbl'):self.coordinate_offset_mm=self.reboot_offset_mm
                if self.w.stop.is_set():return
            self.query(port,'')
            identity=self.query(port,'$I')
            if not any(x.startswith('[VER:1.1') for x in identity):raise RuntimeError('Wrong rail controller identity')
            settings={int(x[1:].split('=')[0]):float(x.split('=')[1]) for x in self.query(port,'$$') if x.startswith('$')}
            validate_settings(settings);self.record('connected',port=PORT,settings=settings)
            port.timeout=0.;buffer=bytearray();next_status=0.
            with self.w.lock:self.connected=True
            while not self.w.stop.is_set():
                buffer.extend(port.read(max(1,port.in_waiting)))
                with self.w.lock:
                    while b'\n' in buffer:
                        line,_,rest=buffer.partition(b'\n');buffer=bytearray(rest)
                        self.accept_line(line.decode('ascii',errors='replace').strip())
                    if time.monotonic()>=next_status:
                        port.write(b'?');next_status=time.monotonic()+.1
                    self.tick(port)
                self.w.stop.wait(.01)
        except Exception as error:
            with self.w.lock:
                self.error=str(error);self.lease=None;self.motion_uncertain=self.in_flight or self.state=='Jog'
            self.record('fault',error=str(error),motion_uncertain=self.motion_uncertain)
        finally:
            if port.is_open:
                if self.in_flight or self.state=='Jog':
                    try:
                        port.write(b'\x85');port.timeout=.05;deadline=time.monotonic()+2
                        while time.monotonic()<deadline:
                            port.write(b'?');line=port.readline().decode('ascii',errors='replace').strip()
                            if line.startswith('<') and parse_status(line)['state']=='Idle':
                                self.motion_uncertain=False;self.in_flight=False;self.state='Idle';break
                        else:self.motion_uncertain=True
                    except Exception:self.motion_uncertain=True
                port.close()
            with self.w.lock:self.connected=False;self.lease=None
            self.record('closed',motion_uncertain=self.motion_uncertain)
