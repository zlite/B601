"""Qualify jaw stalls against the repeated, visually verified plate grasps."""
import math

# Seven successful current-holder trials stalled between -0.3623 and -0.3362.
# Leave substantial positional margin, but do not mistake -0.85 rad rubbing
# against the holder for plate contact. Torque and velocity guards are separate.
MIN_PLATE_CONTACT_RAD=-.55

def closure_action(q,relief_steps):
    if not math.isfinite(q) or not -1.22<=q<=.05:raise ValueError('Invalid closure position')
    if q>=MIN_PLATE_CONTACT_RAD:return 'grasp'
    if relief_steps>=3:raise ValueError('Jaws blocked before plate contact after bounded relief')
    return 'relieve_half_mm'
