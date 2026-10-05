"""Restore the measured open jaw position with the arm disabled at rest."""
import math,time
import numpy as np


def restore_open_at_rest(arm,grip,geometry,vision_ready):
    q=arm.read();reference=np.degrees(geometry.profile['reference_raw_rad'])
    if arm.active or max(abs(np.array(q)[1:3]-reference[1:3]))>=1.5:
        raise RuntimeError('Jaw reset requires disabled arm at rest')
    start=grip.get_register_f32(80,100)
    if not math.isfinite(start) or not -1.22<start<-1.07:
        raise RuntimeError('Jaw reset outside small opening correction range')
    goal=-1.17
    def feedback(enabled):
        arm.read();grip.request_feedback();time.sleep(.005);arm.ctrl.poll_feedback_once()
        state=grip.get_state();actual=grip.get_register_f32(80,100)
        if state is None or state.status_code!=int(enabled):raise RuntimeError('Jaw reset motor status invalid')
        if not all(math.isfinite(v) for v in (actual,state.vel,state.torq,state.t_mos,state.t_rotor)):
            raise RuntimeError('Jaw reset feedback invalid')
        if not -1.22<actual<start+.02 or abs(state.vel)>.35 or abs(state.torq)>.5 or max(state.t_mos,state.t_rotor)>50:
            raise RuntimeError('Jaw reset feedback guard')
        return actual
    feedback(False)
    if start<=goal+.004:return {'adjusted':False,'q_rad':start}
    mode=grip.get_register_u32(10,200);timeout=grip.get_register_u32(9,200)
    enabled=False;rows=[]
    try:
        if not vision_ready():raise RuntimeError('Jaw reset cameras not fresh')
        grip.ensure_mode(4,1000);grip.set_can_timeout_ms(500)
        if grip.get_register_u32(10,200)!=4 or grip.get_register_u32(9,200)!=10000:
            raise RuntimeError('Jaw reset mode/watchdog readback mismatch')
        grip.send_force_pos(start,.03,.02);enabled=True;grip.enable();began=time.monotonic();settled=None
        while time.monotonic()-began<8:
            if not vision_ready():raise RuntimeError('Jaw reset cameras stale')
            target=max(goal,start-.02*(time.monotonic()-began))
            grip.send_force_pos(target,.03,.02);actual=feedback(True)
            rows.append({'target_rad':target,'q_rad':actual})
            if target==goal and abs(actual-goal)<.004:
                if settled is None:settled=time.monotonic()
                if time.monotonic()-settled>=.3:break
            else:settled=None
        else:raise RuntimeError('Jaw reset did not settle')
    finally:
        if enabled:grip.disable()
        feedback(False)
        grip.ensure_mode(mode,1000);grip.write_register_u32(9,timeout)
        if grip.get_register_u32(10,200)!=mode or grip.get_register_u32(9,200)!=timeout:
            raise RuntimeError('Jaw reset settings not restored')
    return {'adjusted':True,'q_rad':feedback(False),'samples':rows}
