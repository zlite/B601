#!/usr/bin/env python3
"""Read-only raw three-camera capture using the installed DepthAI 2 runtime."""
import argparse,json,time
from datetime import datetime,timezone
from pathlib import Path
import cv2
import depthai as dai
import numpy as np
from camera_selection import wrist_camera_config
from tag_view import detect,TASK_CONFIG
from tag_pose import estimate
from contact_geometry import check_tag_triplet,stereo_tag_pose


def main(all_tags=False):
    if not dai.__version__.startswith('2.32.'):raise RuntimeError('Use .venv-depth-v2')
    folder=Path('outputs/contact_camera')/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ');folder.mkdir(parents=True)
    config=wrist_camera_config();pipeline=dai.Pipeline()
    rgb=pipeline.create(dai.node.ColorCamera);rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P);rgb.setIspScale(1,3);rgb.setFps(10)
    rgb.initialControl.setManualFocus(config['manual_focus'])
    ports={'A':rgb.isp}
    for role in ('B','C'):
        node=pipeline.create(dai.node.MonoCamera);node.setBoardSocket(getattr(dai.CameraBoardSocket,'CAM_'+role))
        node.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P);node.setFps(10)
        node.initialControl.setManualExposure(2000,200);ports[role]=node.out
    for role,port in ports.items():
        out=pipeline.create(dai.node.XLinkOut);out.setStreamName(role);port.link(out.input)
    report={'motion_commanded':False,'frames':[],'depthai_version':dai.__version__,
            'coordinates':'original sensor images; extrinsics in meters','wrist_camera':config}
    with dai.Device(pipeline,dai.DeviceInfo(config['device_id'])) as device:
        if device.getDeviceName()!=config['model']:raise RuntimeError('Unexpected camera')
        cal=device.readCalibration();report['extrinsics']={}
        for a,b in [('B','C'),('B','A'),('C','A')]:
            T=np.array(cal.getCameraExtrinsics(getattr(dai.CameraBoardSocket,'CAM_'+a),getattr(dai.CameraBoardSocket,'CAM_'+b)));T[:3,3]*=.01;report['extrinsics'][a+'_'+b]=T.tolist()
        queues={r:device.getOutputQueue(r,maxSize=3,blocking=False) for r in ports}
        latest={};began=time.monotonic();last=-1
        while time.monotonic()-began<25 and len(report['frames'])<12:
            for role,q in queues.items():
                packet=q.tryGet()
                if packet is not None:latest[role]=packet
            if len(latest)!=3 or time.monotonic()-began<3:time.sleep(.005);continue
            stamps=[p.getTimestamp().total_seconds() for p in latest.values()]
            if latest['A'].getSequenceNum()==last or max(stamps)-min(stamps)>.045:time.sleep(.005);continue
            if latest['A'].getLensPosition()!=config['manual_focus']:continue
            last=latest['A'].getSequenceNum();row={'sync_span_s':max(stamps)-min(stamps),'cameras':{}}
            for role,packet in latest.items():
                frame=packet.getCvFrame();h,w=frame.shape[:2];socket=getattr(dai.CameraBoardSocket,'CAM_'+role)
                K=np.array(cal.getCameraIntrinsics(socket,w,h));D=np.array(cal.getDistortionCoefficients(socket))
                color=cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR) if frame.ndim==2 else frame
                tags=detect(color,'auto' if all_tags else TASK_CONFIG['tag']['family'],None if all_tags else TASK_CONFIG['tag']['id'])
                poses=estimate(tags[0]['corners_px'],K,D,TASK_CONFIG['tag']['size_m']) if len(tags)==1 else []
                name=f'{len(report["frames"]):02d}_{role}.png';cv2.imwrite(str(folder/name),frame)
                row['cameras'][role]={'image':name,'camera_matrix':K.tolist(),'distortion':D.tolist(),
                    'timestamp_s':packet.getTimestamp().total_seconds(),'exposure_s':packet.getExposureTime().total_seconds(),'sensitivity_iso':packet.getSensitivity(),'tags':tags,'poses':poses}
            row['tag_metric_check']=check_tag_triplet(row,report['extrinsics'])
            try:row['stereo_tag_pose']=stereo_tag_pose(row,report['extrinsics'])
            except ValueError as error:row['stereo_tag_pose']={'error':str(error)}
            report['frames'].append(row)
        if not report['frames']:raise RuntimeError('No synchronized calibrated triplet')
    (folder/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Camera report:',folder/'report.json',flush=True)
    print('Frames:',len(report['frames']),'tag counts:',{r:sum(bool(f['cameras'][r]['poses']) for f in report['frames']) for r in ('A','B','C')},flush=True)
    print('Passed metric checks:',sum(f['tag_metric_check']['passed'] for f in report['frames']),flush=True)
    print(report['frames'][-1]['tag_metric_check'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--all-tags',action='store_true');a=p.parse_args();main(a.all_tags)
