"""Bounded reacquisition while the caller holds a measured, checked pose."""
import time


def observe_or_hold(get,freeze,hold_tick,save,timeout_s=2.):
    try:return get()
    except ValueError as error:reason=str(error)
    began=time.monotonic();deadline=began+timeout_s
    freeze();reacquired=False
    try:
        while time.monotonic()<deadline:
            hold_tick()
            try:
                observation=get();reacquired=True;return observation
            except ValueError:pass
        from plate_recovery import PlateRecoveryNeeded
        raise PlateRecoveryNeeded('Fresh plate/table tracking did not return during stationary pause')
    finally:
        save({'reason':reason,'duration_s':time.monotonic()-began,'reacquired':reacquired})
