#!/usr/bin/env python3
"""Record an operator-held contact reference. Read-only; does not prove contact."""
import json,time
from datetime import datetime,timezone
from pathlib import Path
import cv2
import depthai as dai
import numpy as np
from arm_geometry import Geometry
from calibrate_arm import Reader
from hello_world import PORT
from tag_view import detect,annotate_view,TASK_CONFIG
from camera_selection import wrist_pipeline, build_wrist_rgb, start_wrist_pipeline, wrist_camera_config
from tag_pose import estimate


def main():
    stamp=datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    folder=Path(__file__).resolve().parent/'calibration'/f'contact_{stamp}'
    folder.mkdir()
    samples=[]
    with Reader(PORT) as reader, wrist_pipeline() as pipeline:
        gripper=reader.add_motor(7,23,'4310')
        camera=build_wrist_rgb(pipeline)
        queue=camera.requestOutput((1280,800),type=dai.ImgFrame.Type.BGR888p,fps=15,enableUndistortion=False).createOutputQueue(maxSize=2,blocking=False)
        start_wrist_pipeline(pipeline, camera)
        deadline=time.monotonic()+12
        attempts=0
        while len(samples)<5 and time.monotonic()<deadline:
            attempts+=1
            before=reader.read()
            earliest=dai.Clock.now().total_seconds()
            packet=None
            end=min(deadline,time.monotonic()+3)
            while time.monotonic()<end:
                p=queue.tryGet()
                if p is not None and p.getTimestamp().total_seconds()>=earliest and p.getLensPosition()==wrist_camera_config()['manual_focus']:
                    packet=p;break
                time.sleep(.005)
            if packet is None:continue
            image=packet.getCvFrame()
            found=detect(image,TASK_CONFIG['tag']['family'],TASK_CONFIG['tag']['id'])
            after=reader.read()
            grip=gripper.get_register_f32(80,500)
            cv2.imwrite(str(folder/'latest_raw.jpg'),image)
            cv2.imwrite(str(folder/'latest_detection.jpg'),annotate_view(image,found))
            if max(abs(a-b) for a,b in zip(before,after))>np.deg2rad(.5):continue
            tr=packet.getTransformation();K=np.array(tr.getIntrinsicMatrix());D=np.array(tr.getDistortionCoefficients())
            poses=[]
            if len(found)==1 and tr.isValid() and 'Perspective' in str(tr.getDistortionModel()):
                poses=estimate(found[0]['corners_px'],K,D,TASK_CONFIG['tag']['size_m'])
            samples.append({'raw_before':before,'raw_after':after,'gripper_raw_rad':grip,
                'frame_timestamp_s':packet.getTimestamp().total_seconds(),
                'detections':found,'pose_candidates':poses,'camera_matrix':K.tolist(),'distortion':D.tolist()})
            cv2.imwrite(str(folder/f'frame_{len(samples):02d}.jpg'),image)
    report={'captured_utc':datetime.now(timezone.utc).isoformat(),'tag':TASK_CONFIG['tag'],
        'geometry_fingerprint':Geometry().fingerprint,'contact_basis':'operator-reported light contact; no force sensor confirmation',
        'contacting_finger':'not yet identified','samples':samples,
        'motion_commanded':False,'motion_ready':False,'attempts':attempts}
    out=folder/'reference.json';out.write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Saved',out,flush=True)
    print('Stable bracketed samples:',len(samples),'with tag detections:',sum(bool(s['detections']) for s in samples),flush=True)
    for i,s in enumerate(samples):
        print(i, 'tag pose candidates',[(round(p['reprojection_rms_px'],2),np.round(p['translation_camera_m'],4).tolist()) for p in s['pose_candidates']],flush=True)

if __name__=='__main__':main()
