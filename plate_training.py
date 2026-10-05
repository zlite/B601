"""Read-only evidence for holder repetitions and reusable setup calibration."""
import hashlib,json
from pathlib import Path

SERIES_START='20261005T190906767335Z'

def require_current_workspace(path=Path('calibration/workspace_revalidation_required.json')):
    if path.exists():
        raise ValueError('Workspace relocation is pending: acquire and validate new holder/table references before replaying plate routes')

def holder_successes(root=Path('outputs/plate_hover')):
    result=[]
    for folder in sorted(Path(root).glob('*')):
        if folder.name<SERIES_START:continue
        try:
            r=json.loads((folder/'report.json').read_text())
            p=json.loads((folder/'pickup_report.json').read_text())
            assisted=json.loads((folder/'shutdown_verification.json').read_text()).get('physically_supported_shutdown',False) if (folder/'shutdown_verification.json').exists() else False
        except (OSError,ValueError):continue
        if (not assisted and r.get('returned_to_rest') is True and r.get('motors_disabled_verified') is True
            and all(p.get(k) is True for k in ('lift_following_verified','released_verified','jaw_open_verified'))
            and .019<=p.get('model_lift_height_m',0)<=.023 and not p.get('transfer')):
            result.append({'run':folder.name,'approach':r.get('approach'),
                           'lift_mm':1000*p['model_lift_height_m'],
                           'placement_shift_mm':1000*p.get('release_plate_translation_m',0)})
    return result

def require_ten_holder_cycles(root=Path('outputs/plate_hover')):
    successes=holder_successes(root)
    if len(successes)<10:raise ValueError(f'Table transfer requires ten verified holder cycles; have {len(successes)}')
    return successes

def reusable_profile():
    reusable=['calibration/leader_pairing_20261002T202428971124Z.json',
              'calibration/camera_joint_motion_summary_20261004.json',
              'calibration/stereo_sweep_handeye_20261004.json',
              'config/cameras.json','plate_speed.py',
              'outputs/gripper/20261004T204827125655Z/jaw_geometry.json']
    workspace=['calibration/plate_surface_dense_20261005.json',
               'calibration/plate_grid_tripod_20261004.json',
               'calibration/plate_tripod_candidate_20261004.json',
               'outputs/plate_grid_reseed_candidate.json']
    def fingerprints(paths):
        return {p:hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in paths}
    return {'reusable_if_mounts_and_hardware_unchanged':fingerprints(reusable),
            'workspace_specific_must_reacquire_after_move':fingerprints(workspace),
            'motion_ready_after_relocation':False,
            'required_relocation_checks':['Verify camera serials, resolution and locked focus',
                'Verify resting joint readback and gripper opening without enabling arm',
                'Acquire new holder/tag pose, plate grid and independent table landmarks',
                'Recompute reachable approach and return paths in the new workspace',
                'Pass an open-jaw standoff/return rehearsal',
                'Pass a 20 mm holder lift, placement, release and autonomous return'],
            'holder_successes':holder_successes(),
            'scope':'Evidence and reuse manifest; does not authorize replaying old workspace trajectories'}

if __name__=='__main__':
    profile=reusable_profile()
    path=Path('calibration/plate_reuse_manifest.json')
    path.write_text(json.dumps(profile,indent=2)+'\n')
    print(json.dumps({'verified_holder_cycles':len(profile['holder_successes']),'manifest':str(path)},indent=2))
