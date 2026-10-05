#!/usr/bin/env python3
"""Manual eye-in-hand capture/validation; no motor motion or settings writes."""
import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import time
import cv2
import depthai as dai
import numpy as np
from arm_geometry import Geometry
from calibrate_arm import Reader
from hello_world import PORT
from tag_pose import estimate
from tag_view import detect, annotate_view, TASK_CONFIG
from camera_view import VIEW_ROTATION_DEG
from camera_selection import wrist_camera_config, build_wrist_rgb, start_wrist_pipeline

ROOT = Path(__file__).resolve().parent
DATA = ROOT / 'calibration/arm2_handeye_samples.json'
RESULT = ROOT / 'calibration/arm2_handeye_candidate.json'


def transform(pose):
    T = np.eye(4)
    T[:3,:3] = cv2.Rodrigues(np.array(pose['rotation_vector']))[0]
    T[:3,3] = pose['translation_camera_m']
    return T


def residuals(boards, reference):
    errors = [np.linalg.inv(reference) @ T for T in boards]
    return ([float(np.linalg.norm(T[:3,3])) for T in errors],
            [float(np.linalg.norm(cv2.Rodrigues(T[:3,:3])[0])*180/np.pi) for T in errors])


def solve_samples(samples):
    if len(samples) < 20:
        raise ValueError('Capture at least 20 distinct stationary poses')
    wrist = [np.array(s['T_base_wrist']) for s in samples]
    tag = [np.array(s['T_camera_tag']) for s in samples]
    # Every fifth view is held out, never passed to the fitting algorithm.
    train = [i for i in range(len(samples)) if i % 5 != 4]
    test = [i for i in range(len(samples)) if i % 5 == 4]
    vectors = np.array([cv2.Rodrigues(wrist[train[0]][:3,:3].T @ wrist[i][:3,:3])[0].ravel() for i in train[1:]])
    singular = np.linalg.svd(vectors, compute_uv=False)
    if len(singular) < 2 or singular[1] < .2:
        raise ValueError('Insufficient rotation diversity: vary wrist orientation about multiple axes')
    R,t = cv2.calibrateHandEye([wrist[i][:3,:3] for i in train], [wrist[i][:3,3] for i in train],
                              [tag[i][:3,:3] for i in train], [tag[i][:3,3] for i in train],
                              method=cv2.CALIB_HAND_EYE_PARK)
    X = np.eye(4); X[:3,:3] = R; X[:3,3] = t.ravel()
    if not np.isfinite(X).all() or abs(np.linalg.det(R)-1) > 1e-5:
        raise ValueError('Hand-eye solve failed')
    boards = [A @ X @ B for A,B in zip(wrist,tag)]
    # Medoid of training board estimates, using translation and rotation disagreement.
    scores = []
    for i in train:
        pe,re = residuals([boards[j] for j in train], boards[i])
        scores.append(sum(pe)+.01*sum(re))
    reference = boards[train[int(np.argmin(scores))]]
    pe,re = residuals([boards[i] for i in test], reference)
    passed = max(pe) <= .005 and max(re) <= 2
    return {'T_wrist_camera':X.tolist(), 'tool_frame':'link6', 'T_base_tag_candidate':reference.tolist(),
            'heldout_indices':test, 'heldout_translation_errors_m':pe, 'heldout_rotation_errors_deg':re,
            'validation_passed':bool(passed), 'motion_ready':False,
            'note':'Candidate only. Reference alignment, fingertip offset and clearance still need independent physical verification.'}


def load_data(geometry):
    if DATA.exists():
        data=json.loads(DATA.read_text())
        if data['geometry_fingerprint'] != geometry.fingerprint or data['tag'] != TASK_CONFIG['tag']:
            raise ValueError('Reference/model/tag changed; archive old sample file before starting a new collection')
        if data.get('wrist_camera') != wrist_camera_config():
            raise ValueError('Camera changed or old session has no camera identity; use capture --new-session')
        return data
    return {'geometry_fingerprint':geometry.fingerprint, 'tag':TASK_CONFIG['tag'],
            'wrist_camera':wrist_camera_config(), 'samples':[]}


def save_data(data):
    tmp=DATA.with_suffix('.tmp')
    tmp.write_text(json.dumps(data,indent=2,allow_nan=False)+'\n')
    tmp.replace(DATA)


def frame_pose(packet):
    image=packet.getCvFrame()
    tag=TASK_CONFIG['tag']
    found=detect(image,tag['family'],tag['id'])
    if len(found)!=1:
        raise ValueError('Need exactly one visible tag 0')
    tr=packet.getTransformation()
    if not tr.isValid() or 'Perspective' not in str(tr.getDistortionModel()):
        raise ValueError('Missing/unsupported frame calibration')
    K=np.array(tr.getIntrinsicMatrix(),dtype=float)
    D=np.array(tr.getDistortionCoefficients(),dtype=float)
    poses=estimate(found[0]['corners_px'],K,D,tag['size_m'])
    if not poses or poses[0]['reprojection_rms_px'] > 1.5:
        raise ValueError('Tag corners too uncertain; adjust view')
    if len(poses)>1 and poses[1]['reprojection_rms_px']-poses[0]['reprojection_rms_px'] < .2:
        raise ValueError('Ambiguous front-on pose; tilt the camera slightly')
    return image, found, poses[0], K, D


def capture(port, new_session=False):
    from camera_selection import wrist_pipeline
    geometry=Geometry()
    if new_session:
        stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
        if DATA.exists():
            archive=DATA.with_name(f'arm2_handeye_samples.{stamp}.json')
            archive.write_bytes(DATA.read_bytes())
            print(f'Preserved previous dataset: {archive}')
        data={'geometry_fingerprint':geometry.fingerprint, 'tag':TASK_CONFIG['tag'],
              'wrist_camera':wrist_camera_config(), 'session_id':stamp, 'samples':[]}
        save_data(data)
    else:
        data=load_data(geometry)
    if 'session_id' not in data:
        data['session_id']=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    print('Keep tag AND robot base fixed throughout this session. Support the disabled arm.')
    print('SPACE: save a steady pose. Q: finish. Capture 20 varied views; no motors are commanded.',flush=True)
    with Reader(port) as reader, wrist_pipeline() as pipeline:
        reader.read()  # refuse enabled/faulted joints before starting
        cam=build_wrist_rgb(pipeline)
        queue=cam.requestOutput((1280,800),type=dai.ImgFrame.Type.BGR888p,fps=15,enableUndistortion=False).createOutputQueue(maxSize=2,blocking=False)
        start_wrist_pipeline(pipeline, cam)
        message='Hold steady, then SPACE'; last=time.monotonic(); began=last
        try:
            while True:
                packet=queue.tryGet()
                if packet is None:
                    if time.monotonic()-last>10: raise RuntimeError('Camera frames stopped')
                    time.sleep(.01); continue
                last=time.monotonic()
                image=packet.getCvFrame()
                found=detect(image,TASK_CONFIG['tag']['family'],TASK_CONFIG['tag']['id'])
                preview=annotate_view(image,found)
                cv2.putText(preview,f"Saved {len(data['samples'])}/20 | SPACE capture | Q finish",(15,30),cv2.FONT_HERSHEY_SIMPLEX,.7,(0,255,255),2)
                cv2.putText(preview,message[:105],(15,65),cv2.FONT_HERSHEY_SIMPLEX,.5,(0,255,255),1)
                cv2.imshow('Second arm - read-only hand-eye calibration',preview)
                key=cv2.waitKey(1)&255
                if key in (ord('q'),27):break
                if key!=32:continue
                try:
                    if time.monotonic()-began<3:
                        raise ValueError('Camera settling; wait three seconds before capturing')
                    before=reader.read()
                    earliest=dai.Clock.now().total_seconds()
                    deadline=time.monotonic()+3
                    fresh=None
                    while time.monotonic()<deadline:
                        candidate=queue.tryGet()
                        if candidate is not None and candidate.getTimestamp().total_seconds()>=earliest:
                            fresh=candidate;break
                        time.sleep(.005)
                    if fresh is None:raise ValueError('No fresh synchronized camera frame')
                    if fresh.getLensPosition()!=wrist_camera_config()['manual_focus']:
                        raise ValueError('Lens has not reached the calibrated focus position')
                    image,found,pose,K,D=frame_pose(fresh)
                    after=reader.read()
                    if max(abs(a-b) for a,b in zip(before,after))>np.deg2rad(.3):
                        raise ValueError('Arm moved during capture; hold steady and retry')
                    q=((np.array(before)+after)/2).tolist()
                    A=geometry.transform(q); B=transform(pose)
                    for old in data['samples']:
                        O=np.array(old['T_base_wrist'])
                        rot=np.linalg.norm(cv2.Rodrigues(O[:3,:3].T@A[:3,:3])[0])
                        if np.linalg.norm(O[:3,3]-A[:3,3])<.01 and rot<np.deg2rad(5):
                            raise ValueError('Too similar to an earlier pose; change position/orientation')
                    index=len(data['samples'])
                    image_path=ROOT/f"calibration/handeye_{data['session_id']}_{index:03d}.jpg"
                    raw_path=image_path.with_name(image_path.stem+'_raw.jpg')
                    if not cv2.imwrite(str(raw_path),image):
                        raise ValueError('Cannot save raw image')
                    if not cv2.imwrite(str(image_path),annotate_view(image,found)):
                        raise ValueError('Cannot save image')
                    data['samples'].append({'captured_utc':datetime.now(timezone.utc).isoformat(),
                        'raw_before':before,'raw_after':after,'T_base_wrist':A.tolist(),'T_camera_tag':B.tolist(),
                        'image':str(image_path),'raw_image':str(raw_path),
                        'image_rotation_deg':VIEW_ROTATION_DEG,'coordinates':'original_sensor_frame',
                        'corners_px':found[0]['corners_px'],
                        'frame_timestamp_s':fresh.getTimestamp().total_seconds(),
                        'lens_position':fresh.getLensPosition(),
                        'camera_matrix':K.tolist(),'distortion':D.tolist(),
                        'reprojection_rms_px':pose['reprojection_rms_px']})
                    save_data(data);message=f'Saved pose {index+1}'
                except ValueError as error:message=str(error)
                print(message,flush=True)
        finally:cv2.destroyAllWindows()
    print(f"Saved {len(data['samples'])} poses. You can rest the arm. Run: uv run calibrate_camera.py solve")


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('action',choices=['capture','solve']);p.add_argument('--port',default=PORT)
    p.add_argument('--new-session',action='store_true',help='Archive previous samples and start fresh; old images are preserved')
    a=p.parse_args()
    if a.new_session and a.action!='capture':p.error('--new-session is only for capture')
    if a.action=='capture':capture(a.port,a.new_session)
    else:
        g=Geometry();data=load_data(g);result=solve_samples(data['samples'])
        result['geometry_fingerprint']=g.fingerprint
        result['wrist_camera']=data['wrist_camera']
        result['session_id']=data.get('session_id')
        RESULT.write_text(json.dumps(result,indent=2,allow_nan=False)+'\n')
        print(json.dumps(result,indent=2))

if __name__=='__main__':
    try:main()
    except (KeyboardInterrupt,EOFError):raise SystemExit('Stopped; saved samples retained. Motors unchanged.')
    except Exception as error:raise SystemExit(f'Calibration stopped: {error}')
