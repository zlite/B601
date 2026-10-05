#!/usr/bin/env python3
"""Read-only stereo pose publisher in camera B coordinates, DepthAI 2.32."""
import argparse,json,time
from pathlib import Path
import cv2
import depthai as dai
import numpy as np
from camera_selection import wrist_camera_config
from tag_view import detect,detect_small,TASK_CONFIG
from contact_geometry import stereo_tag_pose


def run(folder,tag_id=TASK_CONFIG['tag']['id'],tag_size=TASK_CONFIG['tag']['size_m'],small_tag=False):
    if not 0<tag_size<=.2 or tag_id<0:raise ValueError('Invalid tag definition')
    root=Path('outputs/stereo_contact').resolve();folder=folder.resolve()
    if not folder.is_relative_to(root):raise ValueError('Output must be inside outputs/stereo_contact')
    if not dai.__version__.startswith('2.32.'):raise RuntimeError('Use installed DepthAI 2.32 environment')
    folder.mkdir(parents=True,exist_ok=True);config=wrist_camera_config();pipeline=dai.Pipeline()
    for role in ('B','C'):
        node=pipeline.create(dai.node.MonoCamera);node.setBoardSocket(getattr(dai.CameraBoardSocket,'CAM_'+role))
        node.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P);node.setFps(15)
        node.initialControl.setManualExposure(2000,200)
        out=pipeline.create(dai.node.XLinkOut);out.setStreamName(role);node.out.link(out.input)
    with dai.Device(pipeline,dai.DeviceInfo(config['device_id'])) as device:
        if device.getDeviceName()!=config['model']:raise RuntimeError('Unexpected wrist camera')
        cal=device.readCalibration();T=np.array(cal.getCameraExtrinsics(dai.CameraBoardSocket.CAM_B,dai.CameraBoardSocket.CAM_C));T[:3,3]*=.01
        extrinsics={'B_C':T.tolist()};calibration={}
        for role in ('B','C'):
            socket=getattr(dai.CameraBoardSocket,'CAM_'+role)
            calibration[role]={'camera_matrix':cal.getCameraIntrinsics(socket,640,480),'distortion':cal.getDistortionCoefficients(socket)}
        (folder/'calibration.json').write_text(json.dumps({'cameras':calibration,'extrinsics':extrinsics,'coordinates':'camera B, meters','motion_commanded':False},indent=2))
        queues={r:device.getOutputQueue(r,maxSize=2,blocking=False) for r in ('B','C')};latest={};last=-1;saved=0;began=time.monotonic()
        while not (folder/'stop').exists() and time.monotonic()-began<900:
            for role,q in queues.items():
                packet=q.tryGet()
                if packet is not None:latest[role]=packet
            if len(latest)<2 or latest['B'].getSequenceNum()==last:time.sleep(.003);continue
            stamps=[p.getTimestamp().total_seconds() for p in latest.values()]
            if max(stamps)-min(stamps)>.015:time.sleep(.003);continue
            last=latest['B'].getSequenceNum();frame={'cameras':{}}
            received=time.monotonic();age=dai.Clock.now().total_seconds()-min(stamps)
            result={'valid':False,'received':received,'frame_time':received-age,'age_s':age,'sync_span_s':max(stamps)-min(stamps),'sequence':last}
            for role,packet in latest.items():
                image=packet.getCvFrame();tags=(detect_small if small_tag else detect)(cv2.cvtColor(image,cv2.COLOR_GRAY2BGR),TASK_CONFIG['tag']['family'],tag_id)
                frame['cameras'][role]={**calibration[role],'tags':tags}
                if received-saved>.5:cv2.imwrite(str(folder/f'{role}.jpg'),image)
            if received-saved>.5:saved=received
            try:
                if age>.3:raise ValueError('Stereo image is stale')
                for cam in frame['cameras'].values():
                    if len(cam['tags'])!=1:raise ValueError('Need the whole tag in both stereo cameras')
                    px=np.array(cam['tags'][0]['corners_px'])
                    if np.any(px<6) or np.any(px>np.array([633,473])):raise ValueError('Tag approaches image edge')
                result.update(stereo_tag_pose(frame,extrinsics,size=tag_size),valid=True,tag_id=tag_id,tag_size_m=tag_size)
            except ValueError as error:result['reason']=str(error)
            temp=folder/'latest.tmp';temp.write_text(json.dumps(result,allow_nan=False));temp.replace(folder/'latest.json')
    print('Stereo observer stopped',flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--tag-id',type=int,default=TASK_CONFIG['tag']['id']);p.add_argument('--tag-size',type=float,default=TASK_CONFIG['tag']['size_m']);p.add_argument('--small-tag',action='store_true')
    a=p.parse_args();run(a.output,a.tag_id,a.tag_size,a.small_tag)
