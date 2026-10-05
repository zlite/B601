#!/usr/bin/env python3
"""Read-only three-camera OAK capture for fingertip triangulation; no motor access."""
import json,time
from datetime import datetime,timezone
from pathlib import Path
import cv2
import depthai as dai
import numpy as np
from camera_selection import wrist_pipeline,build_wrist_rgb,start_wrist_pipeline
from tag_view import detect,TASK_CONFIG
from tag_pose import estimate


def main():
    folder=Path('outputs/contact_camera')/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    folder.mkdir(parents=True)
    report={'motion_commanded':False,'frames':[],'coordinates':'original sensor images; extrinsics in meters'}
    with wrist_pipeline() as pipeline:
        device=pipeline.getDefaultDevice();cal=device.readCalibration()
        nodes={'A':build_wrist_rgb(pipeline)}
        for role in ('B','C'):
            node=pipeline.create(dai.node.MonoCamera);node.setBoardSocket(getattr(dai.CameraBoardSocket,'CAM_'+role))
            node.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P);node.setFps(10)
            node.initialControl.setManualExposure(2000,200);nodes[role]=node
        queues={}
        for role,node in nodes.items():
            size=(1280,800) if role=='A' else (640,480)
            kind=dai.ImgFrame.Type.NV12 if role=='A' else dai.ImgFrame.Type.GRAY8
            port=node.requestOutput(size,type=kind,fps=10,enableUndistortion=False) if role=='A' else node.out
            queues[role]=port.createOutputQueue(maxSize=3,blocking=False)
        report['extrinsics']={a+'_'+b:cal.getCameraExtrinsics(getattr(dai.CameraBoardSocket,'CAM_'+a),getattr(dai.CameraBoardSocket,'CAM_'+b),unit=dai.LengthUnit.METER) for a,b in [('B','C'),('B','A'),('C','A')]}
        start_wrist_pipeline(pipeline,nodes['A'])
        latest={};deadline=time.monotonic()+20;last=-1
        while time.monotonic()<deadline and len(report['frames'])<12:
            for role,q in queues.items():
                packet=q.tryGet()
                if packet is not None:latest[role]=packet
            if len(latest)!=3:time.sleep(.005);continue
            stamps=[p.getTimestamp().total_seconds() for p in latest.values()]
            if latest['A'].getSequenceNum()==last or max(stamps)-min(stamps)>.045:
                time.sleep(.005);continue
            if latest['A'].getLensPosition()!=76:continue
            last=latest['A'].getSequenceNum();row={'sync_span_s':max(stamps)-min(stamps),'cameras':{}}
            for role,packet in latest.items():
                frame=packet.getCvFrame();tr=packet.getTransformation()
                if role=='A':
                    if not tr.isValid() or 'Perspective' not in str(tr.getDistortionModel()):raise RuntimeError('Invalid image calibration')
                    K=np.array(tr.getIntrinsicMatrix());D=np.array(tr.getDistortionCoefficients())
                else:
                    socket=getattr(dai.CameraBoardSocket,'CAM_'+role)
                    K=np.array(cal.getCameraIntrinsics(socket,640,480));D=np.array(cal.getDistortionCoefficients(socket))
                color=cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR) if frame.ndim==2 else frame
                tags=detect(color,TASK_CONFIG['tag']['family'],TASK_CONFIG['tag']['id'])
                poses=estimate(tags[0]['corners_px'],K,D,TASK_CONFIG['tag']['size_m']) if len(tags)==1 else []
                name=f'{len(report["frames"]):02d}_{role}.png';cv2.imwrite(str(folder/name),frame)
                row['cameras'][role]={'image':name,'camera_matrix':K.tolist(),'distortion':D.tolist(),
                    'timestamp_s':packet.getTimestamp().total_seconds(),'tags':tags,'poses':poses}
            report['frames'].append(row)
        if not report['frames']:raise RuntimeError('No synchronized calibrated triplet')
    (folder/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Camera report:',folder/'report.json',flush=True)
    print('Frames:',len(report['frames']),'tag counts:',{r:sum(bool(f['cameras'][r]['poses']) for f in report['frames']) for r in ('A','B','C')},flush=True)


if __name__=='__main__':main()
