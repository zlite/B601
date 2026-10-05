"""Bounded open-jaw touchdown, using motor load as a contact indication.

Motor torques are not calibrated fingertip forces and cannot identify the
contacted object. The caller must establish the holder approach corridor.
"""
import numpy as np
from plate_contact_guard import check_descent_load
from plate_recovery import PlateRecoveryNeeded

CONTACT_DELTA_NM = np.array([.9, 2.5, 2.5, .8, .5, .3])


def load_score(baseline, current,load_check=check_descent_load):
    load_check(baseline, current)
    return float(np.max(np.abs(np.asarray(current)-baseline)/CONTACT_DELTA_NM))


def remaining_contact_load(baseline, contact, released):
    """Fraction remaining along the measured contact-load direction.

    Gravity support decreases during table contact. On withdrawal the motor
    torque can cross its initial value; that opposite change is unloading,
    not additional contact. Absolute overload checks remain independent.
    """
    check_descent_load(baseline,contact)
    check_descent_load(baseline,released)
    contact_delta=(np.asarray(contact)-baseline)/CONTACT_DELTA_NM
    released_delta=(np.asarray(released)-baseline)/CONTACT_DELTA_NM
    norm=float(contact_delta@contact_delta)
    if norm<1e-12:raise RuntimeError('Missing contact-load direction; hold')
    return float(released_delta@contact_delta/norm)


def touch_and_backoff(sample, move, hold, save, normal, backoff_m=.001,max_descent_m=.008,load_check=check_descent_load):
    """Bounded 0.5 mm commands; optionally withdraw after resistance.

    sample returns settled motor loads and the observed open-pad midpoint.
    move accepts a displacement in the fixed, observed plate coordinate frame.
    Any failure propagates to a powered hold, never a lateral recovery.
    """
    if backoff_m not in (0.,.001):raise ValueError('Unsupported contact backoff')
    if max_descent_m not in (.008,.010):raise ValueError('Unsupported touchdown command budget')
    steps=int(round(max_descent_m/.0005))
    normal = np.asarray(normal, float)
    normal /= np.linalg.norm(normal)
    initial = sample()
    baseline = np.asarray(initial['load_nm'], float)
    origin = np.asarray(initial['center_m'], float)
    load_check(baseline, baseline)
    rows = [];previous_center=origin.copy();blocked_steps=0
    for step in range(steps+1):
        state = sample()
        center = np.asarray(state['center_m'], float)
        if center.shape != (3,) or not np.isfinite(center).all():
            raise RuntimeError('Touchdown position unavailable; hold')
        sideways = center-origin-normal*((center-origin)@normal)
        if np.linalg.norm(sideways) > .0015 or abs(center[1]) > .0015:
            raise RuntimeError('Touchdown moved off center; hold without lateral correction')
        if center[2] < -.001 or np.linalg.norm(center-origin) > .009:
            raise RuntimeError('Touchdown exceeded observed travel bound; hold')
        score = load_score(baseline, state['load_nm'],load_check)
        descent_progress=float((previous_center-center)@normal)
        if step:
            blocked_steps=blocked_steps+1 if descent_progress<.00025 else 0
        previous_center=center.copy()
        rows.append(dict(state, step=step, contact_score=score,
                         observed_descent_step_m=descent_progress,
                         consecutive_blocked_steps=blocked_steps))
        save({'samples': rows, 'commanded_descent_m': step*.0005})
        # Direction-dependent drivetrain friction can change torque even
        # during unobstructed motion. Require two observed shortfalls in the
        # 0.5 mm descent as well as sustained load before calling it contact.
        # Settled shoulder load varies with approach direction. Three blocked
        # half-millimetre commands provide stronger geometric evidence than
        # two, allowing a smaller sustained load change to establish contact.
        # Zero load or freely moving drivetrain friction still cannot qualify.
        contact_threshold=.5 if blocked_steps>=3 else 1.
        if score >= contact_threshold and blocked_steps>=2:
            hold(.3)
            confirmed = sample()
            if load_score(baseline, confirmed['load_nm'],load_check) < contact_threshold:
                raise RuntimeError('Touchdown resistance was not sustained; hold')
            if backoff_m==0:
                result={'samples':rows,'contact_confirmed':confirmed,
                        'commanded_descent_m':step*.0005,'commanded_backoff_m':0.,
                        'observed_backoff_m':0.,'grip_at_contact':True,
                        'approach_complete':True,'contact_load_threshold':contact_threshold,
                        'contact_blocked_steps':blocked_steps}
                save(result)
                return result
            contact_center = np.asarray(confirmed['center_m'], float)
            rise=0.;lateral=0.;backoff_commanded=0.;backoff_samples=[]
            for _ in range(4):
                correction=min(.001,.001-rise)
                if correction<=0 or backoff_commanded+correction>.003:break
                move(normal*correction);backoff_commanded+=correction
                released=sample()
                displacement=np.asarray(released['center_m'])-contact_center
                rise=float(displacement@normal)
                lateral=float(np.linalg.norm(displacement-normal*rise))
                backoff_samples.append(dict(released,observed_rise_m=rise,
                                            lateral_m=lateral))
                save({'samples':rows,'contact_confirmed':confirmed,
                      'backoff_samples':backoff_samples,
                      'commanded_backoff_m':backoff_commanded})
                if lateral>.001 or abs(released['center_m'][1])>.0015 or rise<-.0003:
                    raise RuntimeError('Backoff moved off its upward corridor; hold')
                if rise>=.00075:break
            unload_score = load_score(baseline, released['load_nm'])
            remaining=remaining_contact_load(baseline,confirmed['load_nm'],released['load_nm'])
            result = {'samples': rows, 'contact_confirmed': confirmed,
                      'backoff': released, 'observed_backoff_m': rise,
                      'backoff_lateral_m': lateral,
                      'commanded_descent_m': step*.0005,
                      'commanded_backoff_m': backoff_commanded,
                      'backoff_samples':backoff_samples,'backoff_load_score': unload_score,
                      'remaining_contact_load_fraction':remaining}
            save(result)
            if not .00075 <= rise <= .00125 or lateral > .001 or abs(released['center_m'][1]) > .0015:
                raise RuntimeError('One millimetre backoff/alignment not verified; hold')
            if remaining > .75:
                raise PlateRecoveryNeeded('Resistance did not ease after backoff; hold')
            return result
        if step == steps:
            raise PlateRecoveryNeeded(f'No holder resistance within {max_descent_m*1000:.0f} mm command budget; hold')
        move(-normal*.0005)
