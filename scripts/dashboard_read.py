#!/usr/bin/env python3
"""Read-only diagnostics for the fixed local REbot dashboard. No action requests."""
import argparse
import json
from datetime import datetime, timezone
from pathlib import Path
import time
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
URL = 'http://127.0.0.1:8765'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', action='store_true', help='Save the two currently served JPEGs under outputs/camera_diagnosis')
    parser.add_argument('--brief', action='store_true', help='Show motor state and source readings only')
    parser.add_argument('--target', action='store_true', help='Include the current read-only printer stereo observation')
    parser.add_argument('--sample-target-seconds', type=float, default=0,
                        help='Record up to 60 seconds of read-only target status')
    args = parser.parse_args()
    if not 0 <= args.sample_target_seconds <= 60:
        parser.error('Target sampling must be between 0 and 60 seconds')
    if args.sample_target_seconds:
        rows = []; deadline = time.monotonic()+args.sample_target_seconds
        while time.monotonic() < deadline:
            with urlopen(URL+'/state', timeout=3) as response:
                state = json.load(response)
            rows.append({'sample_time': time.monotonic(),
                         'target': state.get('sources', {}).get('printer_target'),
                         'follower': state.get('sources', {}).get('follower')})
            time.sleep(.2)
        folder = ROOT/'outputs/printer_tracking'
        folder.mkdir(parents=True, exist_ok=True)
        path = folder/(datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')+'.json')
        valid = [r['target'] for r in rows if r['target'] and r['target'].get('valid')]
        summary = {'samples': len(rows), 'valid_samples': len(valid),
                   'sessions': sorted({r['session'] for r in valid}),
                   'max_observation_age_s': max((r['sample_time']-r['target']['observation_time']
                       for r in rows if r['target'] and r['target'].get('valid')), default=None),
                   'scene_revisions': sorted({r['scene_revision'] for r in valid}),
                   'motion_ready': False}
        path.write_text(json.dumps({'summary': summary, 'rows': rows}, indent=2, allow_nan=False)+'\n')
        print(json.dumps({'report': str(path), **summary}, indent=2))
        return
    with urlopen(URL+'/state', timeout=3) as response:
        state = json.load(response)
    summary = {k:state.get(k) for k in ('ready','powered','following','fault','feedback_reader','message','control_mode','camera_check','teaching','rail','phase','pause_requested','samples','pause_cause','last_sample','demo','target_preview')}
    summary['sources'] = {k:{key:v.get(key) for key in ('angles','age_s','frame_age_s','error','tag','rms_px') if key in v}
                          for k,v in state.get('sources',{}).items()}
    profile = state.get('joint_calibration')
    summary['joint_calibration'] = None if not profile else {
        'session':profile['session'], 'global_mapping_changed':profile['global_mapping_changed'],
        'joints':[{key:m.get(key) for key in ('name','model_type','validation_passed','max_validation_error_deg')}
                  for m in profile['joints']]}
    with urlopen(URL+'/', timeout=3) as response:
        page = response.read().decode()
    summary['page'] = {'guided_camera_check':'cameraStepTitle' in page,
                       'joint_calibration_results':'jointCalibrationResults' in page,
                       'explicit_capture_outcome':'captureOutcome' in page,
                       'specific_waiting_reason':'need 6–15°' in page}
    if args.images:
        directory = ROOT/'outputs/camera_diagnosis'
        directory.mkdir(parents=True,exist_ok=True)
        for role in ('wrist','tripod'):
            with urlopen(URL+'/'+role+'.jpg',timeout=3) as response:
                (directory/(role+'_latest.jpg')).write_bytes(response.read())
        summary['images_directory'] = str(directory)
    if args.brief:summary={k:v for k,v in summary.items() if k in ('ready','powered','following','fault','feedback_reader','message','sources','images_directory','teaching','rail','phase','pause_requested','samples','pause_cause','last_sample','demo','target_preview')}
    if args.target:
        summary['printer_target'] = state.get('sources', {}).get('printer_target')
        summary['printer_plate'] = state.get('sources', {}).get('printer_plate')
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
