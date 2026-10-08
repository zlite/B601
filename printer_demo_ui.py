"""Attended approach/return demo, executed by the teaching motor owner."""
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import numpy as np
from axis_follow import DEMO_ROLL_MIT_GAINS
from printer_demo_plan import PreparedDemoPlan, REST_TOLERANCE_DEG
from printer_demo_replay import ReplayRunner, ReturnRequested, load_plan, timed_path
from replay_timing import TimedCurve

CONFIG=Path('config/printer_demo.json')

class SupportedStop(Exception):pass
class DemoCancelled(Exception):pass

class UIRunner(ReplayRunner):
    def __init__(self,*args,demo,**kwargs):
        super().__init__(*args,**kwargs);self.demo=demo;self.review_callback=demo.review
    def vision_ready(self):
        return self.demo.attended() and self.w.cameras_ready() and super().vision_ready()
    def tick(self,*args,**kwargs):
        if self.demo.stop_requested:raise SupportedStop('Operator confirmed support and requested motor power off')
        if self.w.stop.is_set():raise RuntimeError('Server shutdown')
        return super().tick(*args,**kwargs)

class DemoController:
    def __init__(self,w):
        self.w=w;self.pending=False;self.active=False;self.paused=False
        self.runner=None;self.owner=None;self.heartbeat=0.;self.decision=None
        self.stop_requested=False;self.message='Demo unavailable';self.error=None
        self.pause_requested=False;self.stop_loop=False;self.cycle=0;self.rail_commands_allowed=0;self.recover_only=False
        self.deferred_reports=[];self.deferred_checkers=[]
        self.last_seq={};self.folder=None;self.config={};self.expected=None;self.prepared=None;self.rail_returned_at=None
        if CONFIG.exists():
            self.config=json.loads(CONFIG.read_text())
            plan=json.loads((Path(self.config['source'])/'simplified_plan.json').read_text())
            self.expected=np.asarray(plan['start_raw_deg'])
            self.message='Ready for resting-pose confirmation'
    def busy(self):return self.pending or self.active
    def attended(self):return 0<=time.monotonic()-self.heartbeat<.75
    def reason(self):
        w=self.w;r=w.rail;f=w.sources.get('follower',{})
        if not self.config.get('enabled'):return 'Demo needs a reviewed route for this scene'
        if self.busy():return 'Demo is active'
        if w.fault:return str(w.fault)
        if w.powered or w.active_id is not None or w.gate.lease:return 'Rest the arm and remove motor power first'
        if (not r.connected or r.state!='Idle' or r.error or time.monotonic()-r.status_time>=.4 or abs(r.x_mm)>.001 or r.commands!=self.rail_commands_allowed):return 'Rail must remain at the demonstrated position; recheck route after rail movement'
        if not w.cameras_ready():return 'Waiting for live cameras and recorder'
        if 'angles' not in f or time.monotonic()-f['time']>.25:return 'Waiting for fresh arm readings'
        if np.any(abs(np.asarray(f['angles'])-self.expected)>REST_TOLERANCE_DEG):return 'Return arm to the demonstrated resting pose (0.75°; roll 1.5°)'
        return None
    def snapshot(self):
        reason=self.reason()
        return dict(ready=reason is None,active=self.busy(),paused=self.paused,
            phase=self.runner.phase if self.runner else self.message,reason=reason,error=self.error,cycle=self.cycle,repeat=not self.stop_loop,
            folder=str(self.folder) if self.folder else None,speed_scale=4,
            last_sample=self.runner.rows[-1] if self.runner and self.runner.rows else None)
    def invalidate(self):
        if self.config.get('enabled'):
            self.config['enabled']=False;self.config['disabled_reason']='Rail jogging changed the reviewed scene'
            CONFIG.write_text(json.dumps(self.config,indent=2)+'\n')
    def action(self,request):
        op=request['action'];client=request.get('client');seq=request.get('seq')
        if not isinstance(client,str) or not client or not isinstance(seq,int) or seq<=self.last_seq.get(client,-1):raise ValueError('Stale demo request')
        self.last_seq[client]=seq
        if op in ('demo_start','demo_recover'):
            self.recover_only=op=='demo_recover'
            reason=self.reason()
            if self.recover_only:
                if self.busy() or self.w.powered or not self.w.rail.allowed() or not 0<=self.w.rail.x_mm<=300:raise ValueError('Rail recovery is unavailable')
                if np.any(abs(np.asarray(self.w.sources['follower']['angles'])-self.expected)>REST_TOLERANCE_DEG):raise ValueError('Resting pose changed')
            elif reason:raise ValueError(reason)
            if request.get('rest_confirmed') is not True or request.get('scene_unchanged') is not True or request.get('rail_clear_confirmed') is not True:raise ValueError('Confirm resting pose, unchanged scene and clear 30 cm rail sweep')
            self.owner=client;self.heartbeat=time.monotonic();self.pending=True
            self.stop_requested=False;self.pause_requested=False;self.stop_loop=False;self.cycle=0;self.decision=None;self.error=None;self.runner=None
            self.message='Checking fresh resting pose and entry clearance'
        elif op=='demo_heartbeat':
            if client==self.owner and self.busy():self.heartbeat=time.monotonic()
        elif op=='demo_stop':
            self.stop_loop=True
        elif op=='demo_pause':
            self.pause_requested=True
            self.w.rail.lease=None
            if self.pending:self.pending=False;self.message='Cancelled before motor enable'
            if self.runner:self.runner.pause_requested=True
            # During rail travel, remain stopped with the arm motors off.
            # During arm preflight, cancellation is caught before enabling.
        elif op in ('demo_resume','demo_return'):
            if not self.paused:raise ValueError('Pause the demo before selecting Resume or Return')
            if request.get('clearance_confirmed') is not True:raise ValueError('Confirm the path is still clear')
            self.owner=client;self.heartbeat=time.monotonic();self.decision='return' if op=='demo_return' else 'advance'
            if op=='demo_return':self.stop_loop=True
        else:raise ValueError('Unknown demo action')
    def review(self,runner,output,index,label):
        self.paused=True;self.decision=None;runner.phase='Paused and holding: '+label
        runner.save_view('paused_'+str(index))
        try:
            while True:
                runner.tick()
                with self.w.lock:
                    if self.decision:
                        runner.pause_requested=False
                        if runner.vision_ready():
                            decision=self.decision;self.decision=None;self.pause_requested=False;return decision
                        runner.pause_requested=True
        finally:self.paused=False
    def run_pending(self,arm):
        with self.w.lock:
            if not self.pending:return False
            self.pending=False;self.active=True;self.w.ready=False
        try:
            if not self.recover_only:self.prepare_demo(arm)
            while not self.stop_loop and not self.w.stop.is_set():
                self.cycle+=1;self.runner=None
                if not self.rail_cycle(arm):break
                self._arm_cycle(arm)
                if self.error or self.stop_requested:break
                self.runner=None
                self.message=f'Cycle {self.cycle} complete · motors off'
            if not self.error:self.message=f'Stopped after cycle {self.cycle} · motors off'
        except DemoCancelled as error:self.message=str(error)
        except Exception as error:
            self.error=str(error);self.message='Demo stopped: '+str(error)
            if arm.active:raise
        finally:
            if self.prepared:
                if arm.active:self.deferred_checkers.append(self.prepared)
                else:self.prepared.close()
                self.prepared=None
            self.w.rail.lease=None
            with self.w.lock:self.active=False;self.paused=False;self.w.powered=arm.active
        return True

    def save_report(self,report,arm):
        """Never serialize a recording while a fault unwinds a powered owner."""
        if arm.active:
            self.deferred_reports.append((self.folder/'report.json',report))
        else:
            (self.folder/'report.json').write_text(json.dumps(report,indent=2)+'\n')

    def after_motor_shutdown(self):
        # Called only after the motor context has completed its cleanup attempt
        # and closed hardware handles. No report/checker delay can starve it.
        try:
            for path,report in self.deferred_reports:
                path.write_text(json.dumps(report,indent=2)+'\n')
        finally:
            self.deferred_reports.clear()
            for checker in self.deferred_checkers:checker.close()
            self.deferred_checkers.clear()

    def prepare_demo(self,arm):
        if arm.active:raise ValueError('Preparation requires disabled motors')
        self.message='Preparing repeating demo before rail travel'
        self.prepared=PreparedDemoPlan(Path(self.config['source']))
        self.home_entry=TimedCurve(self.prepared.paths[0][1][0],4.)
        self.timed_tail=[(name,[timed_path(p,4.) for p in chunks]) for name,chunks in self.prepared.paths[1:]]
        arm.prepare_group(range(6))
        self.w.publish('follower',angles=arm.read())
        if self.stop_requested or self.pause_requested or not self.attended():raise DemoCancelled('Demo stopped during preparation')

    def rail_cycle(self,arm):
        rail=self.w.rail
        if arm.active:raise ValueError('Rail requires arm motor power off')
        park=np.asarray(arm.read());self.w.publish('follower',angles=park.tolist())
        if not self.attended() or self.pause_requested:raise DemoCancelled('Demo stopped before rail travel')
        if not self.recover_only and abs(rail.x_mm)>.1:raise ValueError('Rail is not at the demonstrated starting position')
        origin=0.;targets=[origin] if self.recover_only else [origin+300.,origin];return_only=self.recover_only
        for target in targets:
            started=False;paused=False
            while True:
                q=np.asarray(arm.read());self.w.publish('follower',angles=q.tolist())
                # The unpowered wrist can settle as the carriage moves. The
                # rail's folded-arm interlock stays active; capture a new
                # stationary pose after returning instead of chasing this pose.
                if self.w.stop.is_set() or self.stop_requested:raise DemoCancelled('Demo stopped · arm motors off')
                with self.w.lock:
                    if not rail.connected or rail.error or rail.state=='Alarm':raise ValueError('Rail unavailable; arm remains disabled')
                    if started and not rail.busy() and abs(rail.x_mm-target)<=.1:break
                    if self.pause_requested or not self.attended() or not self.w.cameras_ready() or (started and rail.lease is None):
                        paused=True;self.paused=True;rail.lease=None
                    if paused:
                        self.message=f'Rail paused at X {rail.x_mm:.1f} mm · arm motors off'
                        if self.decision and not rail.busy() and self.attended() and rail.allowed():
                            if self.decision=='return':target=origin;return_only=True
                            self.decision=None;self.pause_requested=False;paused=False;self.paused=False;started=False
                    if not paused and not started:
                        rail.begin_demo_leg(target);started=True
                    if not paused:
                        self.message=f'Cycle {self.cycle} · rail '+('30 cm left' if target>origin else 'returning to start')+f' · X {rail.x_mm:.1f} mm'
                        if rail.lease:rail.lease['heartbeat']=time.monotonic()
                time.sleep(.02)
            self.rail_commands_allowed=rail.commands
            # Fresh Idle acknowledgement is required before the next leg or arm.
            self.message='Rail stopped · idle motor release configured'
            if return_only:return False
        if abs(rail.x_mm-origin)>.1:raise ValueError('Rail did not return to its original position')
        self.rail_returned_at=time.monotonic()
        return True

    def settled_disabled_pose(self,arm):
        if arm.active:raise ValueError('Rest capture requires disabled arm motors')
        self.message='Rail returned · checking fresh arm entry'
        if self.w.stop.is_set() or self.stop_requested or self.pause_requested or not self.attended():raise DemoCancelled('Demo stopped before arm engagement')
        if self.w.rail.busy() or self.w.rail.state!='Idle' or abs(self.w.rail.x_mm)>.1:raise ValueError('Rail must be stopped at its original position')
        q=arm.read();self.w.publish('follower',angles=q)
        return q

    def gripper_for(self,arm):
        # The motor owner lives across demo cycles and separate Demo presses.
        # SDK registration must happen once per owner, even though it is read-only.
        if getattr(self,'gripper_arm',None) is not arm:
            self.gripper_motor=arm.add_motor(7,23,'4310');self.gripper_arm=arm
        return self.gripper_motor

    def _arm_cycle(self,arm):
        old_speed=arm.speed_limits;old_command=getattr(arm,'command_speed_limits',None)
        report={'requested_speed_scale':4,'returned_to_start':False,'motors_disabled_verified':False,'gripper_commanded':False,'rail_commanded':False,'completed_stages':[]}
        self.folder=Path('outputs/printer_replay/ui_demo')/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');self.folder.mkdir(parents=True)
        enabled=False
        try:
            if arm.active:raise ValueError('Arm must be disabled before demo')
            start=self.settled_disabled_pose(arm);report['start_raw_deg']=start
            report['starting_pose_method']='Fresh disabled readback immediately after rail Idle at origin; entry checked from this pose; <=0.05 degree change verified before enable'
            grip=self.gripper_for(arm);position=grip.get_register_f32(80,300)
            grip.request_feedback();time.sleep(.01);arm.ctrl.poll_feedback_once()
            state=grip.get_state()
            if state is None or state.status_code!=0:raise ValueError('Gripper must be disabled')
            plan,limits,poses,paths,evidence=self.prepared.refresh(start,position,self.folder/'plan')
            report['entry_check_s']=evidence['check_seconds']
            home_entry=self.home_entry
            poses=poses+home_entry.points.tolist()
            report['fixed_home_raw_deg']=self.prepared.plan['start_raw_deg']
            paths=[(paths[0][0],[TimedCurve(paths[0][1][0],4.)]),*self.timed_tail]
            report['planned_outbound_s']=sum(p.duration for _,chunks in paths for p in chunks)
            runner=UIRunner(self.w,arm,start,limits,self.folder,poses,rail=self.w.rail,demo=self);self.runner=runner;runner.speed=12.
            deadline=time.monotonic()+10
            while True:
                if self.stop_requested or self.pause_requested or not self.attended():raise ValueError('Demo cancelled or browser heartbeat lost during preflight')
                if np.max(abs(np.asarray(arm.read())-start))>.05:raise ValueError('Resting pose changed during preflight')
                if runner.vision_ready():break
                if time.monotonic()>deadline:raise ValueError('Live cameras or rail unavailable after preflight')
                time.sleep(.02)
            runner.save_view('before')
            if self.stop_requested or not runner.vision_ready() or np.max(abs(np.asarray(arm.read())-start))>.05:raise ValueError('Readiness or resting pose changed during setup')
            arm.speed_limits={i:48. for i in range(6)};arm.command_speed_limits={i:18. for i in range(6)}
            arm.set_demo_roll_stiffness(True)
            report['roll_mit_gains']={'kp':DEMO_ROLL_MIT_GAINS[0],'kd':DEMO_ROLL_MIT_GAINS[1],'feedforward_nm':0.}
            enabled=True;arm.enable_group(dict(enumerate(start)));self.w.powered=True;runner.last=time.monotonic()
            began=time.monotonic()
            for _ in range(2):runner.tick(take_up=True)
            report['rail_to_arm_motion_s']=time.monotonic()-self.rail_returned_at if self.rail_returned_at is not None else None
            visited=[start]
            try:
                for name,chunks in paths:
                    runner.phase='Approaching: '+name
                    for path in chunks:runner.follow_curve(path,visited)
                    runner.save_view(name);report['completed_stages'].append(name)
            except ReturnRequested:report['early_return']=True
            except ValueError as error:
                self.review(runner,self.folder,999,str(error));report['early_return']=True
            runner.returning=True;runner.phase='Returning to rest'
            full_route=len(report['completed_stages'])==len(paths)
            report['fixed_home_return']=full_route
            report['return_segments']=runner.return_executed_curves(home_entry if full_route else None)
            report['returned_to_start']=True;report['motion_elapsed_s']=time.monotonic()-began
            # Each returned curve already waits for the measured endpoint tolerance.
            for _ in range(2):runner.tick()
            runner.save_view('returned');arm.disable();self.w.powered=False;report['final_raw_deg']=arm.read();report['motors_disabled_verified']=True
            self.message=runner.phase='Complete · returned to rest · motors off'
        except SupportedStop as error:
            report['supported_stop']=str(error);arm.disable();arm.read();report['motors_disabled_verified']=True
            self.message='Stopped with operator support · motors off'
            if self.runner:self.runner.phase=self.message
        except Exception as error:
            report['error']=repr(error);self.error=str(error);self.message='Demo stopped: '+str(error)
            if self.runner:self.runner.phase=self.message
            if enabled:raise
        finally:
            # Do not change gains on a powered fault; the motor-owner cleanup
            # must disable first. Successful/supported stops restore teaching.
            if not arm.active:arm.set_demo_roll_stiffness(False)
            if self.runner:report['samples']=self.runner.rows
            self.save_report(report,arm)
            arm.speed_limits=old_speed
            if old_command is None:
                if hasattr(arm,'command_speed_limits'):del arm.command_speed_limits
            else:arm.command_speed_limits=old_command
            with self.w.lock:
                self.paused=False;self.w.powered=arm.active
                self.w.bounds=None;self.w.revision+=1;self.w.teaching_mappers=None;self.w.press_leader_reference=None
        return True
