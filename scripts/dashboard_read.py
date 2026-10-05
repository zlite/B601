#!/usr/bin/env python3
"""Read-only diagnostics for the fixed local REbot dashboard. No action requests."""
import argparse
import json
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(__file__).resolve().parents[1]
URL = 'http://127.0.0.1:8765'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', action='store_true', help='Save the two currently served JPEGs under outputs/camera_diagnosis')
    parser.add_argument('--brief', action='store_true', help='Show motor state and source readings only')
    args = parser.parse_args()
    with urlopen(URL+'/state', timeout=3) as response:
        state = json.load(response)
    summary = {k:state.get(k) for k in ('powered','following','fault','message','control_mode','camera_check')}
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
    if args.brief:summary={k:v for k,v in summary.items() if k in ('powered','following','fault','message','sources','images_directory')}
    print(json.dumps(summary,indent=2))


if __name__=='__main__':
    main()
