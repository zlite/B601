"""Keep a healthy, extended arm holding after a software exception."""
import json,time
from pathlib import Path
import cv2
import numpy as np
from axis_follow import AxisArm

def read_object(path):
    try:
        value=json.loads(path.read_text())
        return value if isinstance(value,dict) else {}
    except (OSError,ValueError):return {}

class PlateArm(AxisArm):
    def __init__(self,folder,workbench):
        super().__init__();self.output=Path(folder);self.workbench=workbench
    def folded(self,q):
        reference=np.degrees(self.workbench.geometry.profile['reference_raw_rad'])
        return max(abs(np.array(q)[1:3]-reference[1:3]))<1.5
    def hold_after_fault(self,q,error):
        targets=dict(enumerate(q));saved=0;requests=set();attempts=0
        causes=[];cause=error
        while cause is not None:
            causes.append(str(cause));cause=cause.__cause__
        print('FAULT HOLD: motors remain powered at the stopped pose.',self.output,' -> '.join(causes),flush=True)
        while True:
            self.command_group(targets);actual=self.read()
            if max(abs(actual[i]-q[i]) for i in range(6))>3:raise RuntimeError('Fault hold position cannot be maintained')
            confirmation=read_object(self.output/'supported_remove_power.json')
            if confirmation.get('physically_supported_confirmed') is True:break
            callback=getattr(self,'recovery_callback',None)
            request=read_object(self.output/'recovery_request.json')
            request_id=request.get('request_id')
            if (callback is not None and request.get('action')=='recover_to_rest'
                    and isinstance(request_id,str) and request_id and request_id not in requests and attempts<2):
                requests.add(request_id);attempts+=1
                try:
                    try:
                        self.output.joinpath('fault_hold.json').write_text(json.dumps({'holding':False,'recovering':True,'motors_powered':True,'request_id':request_id,'recovery_attempts':attempts,'causes':causes}))
                    except OSError:pass
                    if callback() is not True:raise RuntimeError('Recovery did not verify rest')
                    actual=self.read();rest=getattr(self,'recovery_target',None)
                    verified=self.folded(actual) if rest is None else bool(np.all(abs(np.array(actual)-rest)<np.array([.2]*5+[.5])))
                    if not verified:raise RuntimeError('Recovery final pose is not verified rest')
                    try:self.output.joinpath('fault_hold.json').write_text(json.dumps({'holding':False,'recovering':False,'recovered_to_rest':True,'request_id':request_id,'raw_deg':actual,'causes':causes}))
                    except OSError:pass
                    return True
                except Exception as recovery_error:
                    # A rejected/partial recovery must hold its NEW measured
                    # pose, never jump back to the original fault position.
                    q=self.read();targets=dict(enumerate(q));causes.append(str(recovery_error))
                    print('Recovery request stopped; continuing powered hold:',recovery_error,flush=True)
            now=time.monotonic()
            if now-saved>1:
                data={'holding':True,'reason':str(error),'causes':causes,'raw_deg':actual,'time':now,'requires_supported_power_removal':True,'recovery_available':callback is not None,'recovery_attempts':attempts}
                try:
                    temp=self.output/'fault_hold.tmp';temp.write_text(json.dumps(data,indent=2));temp.replace(self.output/'fault_hold.json')
                    for role in ('wrist','tripod'):
                        with self.workbench.lock:frame=getattr(self.workbench,'survey_frames',{}).get(role,{})
                        if 'image' in frame:cv2.imwrite(str(self.output/f'fault_{role}.jpg'),frame['image'])
                except (OSError,ValueError,cv2.error):pass
                saved=now
            time.sleep(.005)
    def __exit__(self,typ,error,tb):
        recovered=False
        if typ is not None and issubclass(typ,Exception) and self.active:
            try:q=self.read()
            except Exception:
                # A drive/feedback failure must retain the driver's existing
                # hardware-fault shutdown behavior; it is not a software hold.
                return super().__exit__(typ,error,tb)
            if not self.folded(q):
                try:recovered=self.hold_after_fault(q,error) is True
                except BaseException:
                    super().__exit__(typ,error,tb)
                    raise
        result=super().__exit__(typ,error,tb)
        return True if recovered else result
