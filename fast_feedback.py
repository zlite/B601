"""Fresh, concurrent read-only motor transactions using the pinned SDK extension."""
from concurrent.futures import ThreadPoolExecutor, wait
import ctypes
import hashlib
import json
import os
from pathlib import Path
import time
from motorbridge.errors import CallError

ROOT=Path(__file__).resolve().parent
LIBRARY=ROOT/'native/motorbridge/build/libmotor_abi.so'
UPSTREAM_COMMIT='c652ce420e7da1008fa1864cc7f47d7d7c57fdb6'

def configure_library():
    """Experimental reader is opt-in after long-hold hardware failures."""
    if os.environ.get('B601_FRESH_FEEDBACK','0')!='1' or not LIBRARY.exists():return False
    manifest=json.loads(LIBRARY.with_name('manifest.json').read_text())
    if (manifest['upstream_commit'] != UPSTREAM_COMMIT or
            manifest['patch_sha256'] != hashlib.sha256((LIBRARY.parent.parent/'fresh_state.patch').read_bytes()).hexdigest()):
        raise RuntimeError('Fresh-feedback build does not match the pinned source')
    if hashlib.sha256(LIBRARY.read_bytes()).hexdigest()!=manifest['library_sha256']:raise RuntimeError('Fresh-feedback library checksum mismatch')
    if os.environ.get('MOTORBRIDGE_LIB') not in (None,str(LIBRARY)):raise RuntimeError('Conflicting motor library override')
    os.environ['MOTORBRIDGE_LIB']=str(LIBRARY)
    return True

class FreshMotorReader:
    def __init__(self,motors):
        from motorbridge.abi import CState
        from motorbridge.models import MotorState
        if len(motors)!=6:raise ValueError('Exactly six arm motors required')
        self.motors=motors;self.CState=CState;self.MotorState=MotorState
        self.call=motors[0]._abi.lib.b601_motor_request_fresh_state
        self.call.argtypes=[ctypes.c_void_p,ctypes.c_uint32,ctypes.POINTER(CState)];self.call.restype=ctypes.c_int32
        self.pool=ThreadPoolExecutor(max_workers=6,thread_name_prefix='fresh-motor-read')
        self.last_duration_s=None
        self.retry_count=0
    def position(self,motor):
        try:
            return motor.get_register_f32(80,40)
        except CallError as error:
            # Only a missing reply is transient. Transport/handle errors remain
            # immediate faults, and a second timeout propagates to the owner.
            if 'register 80 not received within 40ms' not in str(error):
                raise
            self.retry_count+=1
            return motor.get_register_f32(80,40)
    def one(self,motor):
        # The full-precision register transaction and status transaction each
        # reject cached replies. No motor command is issued by these workers.
        position=self.position(motor)
        state=self.CState()
        if self.call(motor._require_open(),40,ctypes.byref(state)) or not state.has_value:
            # One lost status packet may be re-requested, never replaced by a
            # cached value. The batch deadline still includes this retry.
            self.retry_count+=1
            if self.call(motor._require_open(),40,ctypes.byref(state)) or not state.has_value:
                raise RuntimeError('Fresh motor feedback timed out or failed after one retry')
        return position,self.MotorState(can_id=int(state.can_id),arbitration_id=int(state.arbitration_id),status_code=int(state.status_code),pos=float(state.pos),vel=float(state.vel),torq=float(state.torq),t_mos=float(state.t_mos),t_rotor=float(state.t_rotor))
    def read(self):
        began=time.monotonic();jobs=[self.pool.submit(self.one,m) for m in self.motors]
        # Finish every read before the owner can send commands or close handles,
        # including when one motor fails. Native transactions have finite timeouts.
        wait(jobs)
        self.last_duration_s=time.monotonic()-began
        if self.last_duration_s>.12:
            failures=[repr(job.exception()) for job in jobs if job.exception() is not None]
            raise RuntimeError(f'Fresh motor read deadline exceeded: {self.last_duration_s:.3f}s; failures={failures}')
        return [job.result() for job in jobs]
    def close(self):self.pool.shutdown(wait=True,cancel_futures=True)
