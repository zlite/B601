#!/usr/bin/env python3
"""Camera-only bandwidth/latency A/B experiment. Never opens the motor bus."""
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime,timezone
import json
from pathlib import Path
import time
import cv2
import depthai as dai
import numpy as np
from camera_selection import build_wrist_rgb,start_wrist_pipeline

CONFIG=json.loads(Path('config/cameras.json').read_text())
OUTPUT=Path('outputs/camera_latency');OUTPUT.mkdir(parents=True,exist_ok=True)


def capture(role,size,kind,seconds=7):
    cfg=CONFIG[role];result={'role':role,'size':size,'format':kind,'motion_commanded':False}
    matches=[d for d in dai.Device.getAllAvailableDevices() if d.deviceId==cfg['device_id']]
    if len(matches)!=1:raise RuntimeError(role+' camera unavailable; stop other camera clients')
    with dai.Device(matches[0]) as device:
        device.setMaxReconnectionAttempts(0)
        result['usb_speed']=str(device.getUsbSpeed())
        with dai.Pipeline(device) as pipeline:
            cam=build_wrist_rgb(pipeline) if role=='wrist' else pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_A)
            q=cam.requestOutput(tuple(size),type=getattr(dai.ImgFrame.Type,kind),fps=10,enableUndistortion=False).createOutputQueue(maxSize=1,blocking=False)
            if role=='wrist':start_wrist_pipeline(pipeline,cam)
            else:pipeline.start()
            ages=[];stamps=[];processing=[];lens=[];started=time.monotonic();end=started+seconds+3
            while time.monotonic()<end:
                packet=q.tryGet()
                if packet is None:time.sleep(.003);continue
                last_packet=packet
                t=time.monotonic()
                frame=packet.getCvFrame()
                if role=='wrist':
                    from calibrate_camera import frame_pose
                    try:frame,found,pose,K,D=frame_pose(packet)
                    except ValueError:pose=None
                age=dai.Clock.now().total_seconds()-packet.getTimestamp().total_seconds()
                if time.monotonic()-started>3:
                    ages.append(age);stamps.append(packet.getTimestamp().total_seconds());processing.append(time.monotonic()-t);lens.append(packet.getLensPosition())
            if len(ages)<20:raise RuntimeError('Insufficient '+role+' frames')
            result.update(frames=len(ages),delivered_fps=(len(stamps)-1)/(stamps[-1]-stamps[0]),
                frame_age_ms=dict(zip(['median','p95','max'],(np.percentile(ages,[50,95,100])*1000).tolist())),
                processing_ms=float(np.median(processing)*1000),lens_positions=sorted(set(lens)),
                frame_shape=list(frame.shape),camera_matrix=np.array(last_packet.getTransformation().getIntrinsicMatrix()).tolist())
            if role=='wrist':result['last_tag_rms_px']=pose['reprojection_rms_px'] if pose else None
            cv2.imwrite(str(OUTPUT/f'{role}_{kind}_{size[0]}.jpg'),frame)
    return result


def main():
    results=[]
    for name,wrist,tripod,kind in [('existing',(1280,800),(1280,800),'BGR888p'),('smaller_workspace',(1280,800),(640,400),'BGR888p'),('nv12_small_workspace',(1280,800),(640,400),'NV12')]:
        print('CASE',name,flush=True)
        with ThreadPoolExecutor(max_workers=2) as pool:
            jobs=[pool.submit(capture,'wrist',wrist,kind),pool.submit(capture,'tripod',tripod,kind)]
            rows=[]
            for job in jobs:
                try:rows.append(job.result())
                except Exception as e:rows.append({'error':str(e)})
        results.append({'case':name,'cameras':rows})
        (OUTPUT/'report.json').write_text(json.dumps({'utc':datetime.now(timezone.utc).isoformat(),'depthai':dai.__version__,'motion_commanded':False,'experiments':results},indent=2)+'\n')
        print(json.dumps(results[-1]),flush=True)
        time.sleep(1)

if __name__=='__main__':main()
