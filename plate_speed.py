"""Separate clear travel from precision plate work, within existing limits."""

def motion_profile(scale,travel_scale):
    if scale not in (1.,2.,4.) or travel_scale not in (1.,2.):
        raise ValueError('Unsupported plate speed scale')
    return {
        'clear_pacing_lag_deg':min(2.4,1.8*travel_scale),
        'approach_pacing_lag_deg':1.8,
        'transit_deg_s':min(48.,16.*scale*travel_scale),
        'transit_acceleration_deg_s2':min(288.,72.*(scale*travel_scale)**2),
        'clear_approach_deg_s':min(48.,8.*scale*travel_scale),
        'clear_approach_acceleration_deg_s2':min(288.,36.*(scale*travel_scale)**2),
        'approach_deg_s':8.*scale,
        'approach_acceleration_deg_s2':min(288.,36.*scale**2),
        'gripper_ramp_rad_s':.04*min(2.,scale),
        'gripper_velocity_rad_s':.05*min(2.,scale),
    }

def apply_local_profile(runner,profile,clear):
    prefix='clear_approach' if clear else 'approach'
    runner.speed=profile[prefix+'_deg_s']
    runner.acceleration=profile[prefix+'_acceleration_deg_s2']
    runner.pacing_lag=profile['clear_pacing_lag_deg' if clear else 'approach_pacing_lag_deg']
