#!/usr/bin/env python3
"""Read-only raw three-camera capture using the installed DepthAI 2 runtime."""
import argparse,json,time
from datetime import datetime,timezone
from pathlib import Path
import cv2
import depthai as dai
import numpy as np
from camera_selection import wrist_camera_config
from tag_view import detect,detect_small,TASK_CONFIG
from tag_pose import estimate
from contact_geometry import check_tag_triplet,stereo_tag_pose


def metric_tag_definition(all_tags=False, tag_id=None, tag_size=None):
    if (tag_id is None) != (tag_size is None):
        raise ValueError('Provide both --tag-id and --tag-size (black square side in metres)')
    if tag_id is not None:
        if tag_id < 0 or not np.isfinite(tag_size) or not 0 < tag_size <= .2:
            raise ValueError('Invalid tag definition')
        return {'family': '36h11', 'id': tag_id, 'size_m': tag_size}
    # Discovery must not apply the old 60 mm size to a newly discovered tag.
    return None if all_tags else dict(TASK_CONFIG['tag'])


def main(all_tags=False, tag_id=None, tag_size=None, mono_exposure_us=2000, mono_iso=200, output=None):
    metric_tag = metric_tag_definition(all_tags, tag_id, tag_size)
    if not 100 <= mono_exposure_us <= 20000 or not 100 <= mono_iso <= 800:
        raise ValueError('Invalid stationary mono exposure settings')
    if not dai.__version__.startswith('2.32.'):raise RuntimeError('Use .venv-depth-v2')
    folder=Path(output) if output else Path('outputs/contact_camera')/datetime.now(timezone.utc).strftime('%Y%m%dT%H%M%S%fZ')
    folder.mkdir(parents=True,exist_ok=output is not None)
    if (folder/'report.json').exists() or any(folder.glob('??_?.png')):
        raise FileExistsError('Refusing to overwrite camera evidence')
    config=wrist_camera_config();pipeline=dai.Pipeline()
    rgb=pipeline.create(dai.node.ColorCamera);rgb.setBoardSocket(dai.CameraBoardSocket.CAM_A)
    rgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P);rgb.setIspScale(1,3);rgb.setFps(10)
    rgb.initialControl.setManualFocus(config['manual_focus'])
    ports={'A':rgb.isp}
    for role in ('B','C'):
        node=pipeline.create(dai.node.MonoCamera);node.setBoardSocket(getattr(dai.CameraBoardSocket,'CAM_'+role))
        node.setResolution(dai.MonoCameraProperties.SensorResolution.THE_480_P);node.setFps(10)
        node.initialControl.setManualExposure(mono_exposure_us,mono_iso);ports[role]=node.out
    for role,port in ports.items():
        out=pipeline.create(dai.node.XLinkOut);out.setStreamName(role);port.link(out.input)
    report={'motion_commanded':False,'frames':[],'depthai_version':dai.__version__,
            'coordinates':'original sensor images; extrinsics in meters','wrist_camera':config,
            'metric_tag':metric_tag,'motion_ready':False}
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
            last=latest['A'].getSequenceNum();clock_offset=time.monotonic()-dai.Clock.now().total_seconds()
            row={'sync_span_s':max(stamps)-min(stamps),'cameras':{}}
            for role,packet in latest.items():
                frame=packet.getCvFrame();h,w=frame.shape[:2];socket=getattr(dai.CameraBoardSocket,'CAM_'+role)
                K=np.array(cal.getCameraIntrinsics(socket,w,h));D=np.array(cal.getDistortionCoefficients(socket))
                color=cv2.cvtColor(frame,cv2.COLOR_GRAY2BGR) if frame.ndim==2 else frame
                tags=(detect(color,'auto',None) if all_tags else
                      detect_small(color,metric_tag['family'],metric_tag['id']))
                selected=metric_tag is not None and len(tags)==1 and all(tags[0][k]==metric_tag[k] for k in ('family','id'))
                poses=estimate(tags[0]['corners_px'],K,D,metric_tag['size_m']) if selected else []
                name=f'{len(report["frames"]):02d}_{role}.png';cv2.imwrite(str(folder/name),frame)
                row['cameras'][role]={'image':name,'camera_matrix':K.tolist(),'distortion':D.tolist(),
                    'timestamp_s':packet.getTimestamp().total_seconds(),
                    'observation_monotonic_s':packet.getTimestamp().total_seconds()+clock_offset,
                    'exposure_s':packet.getExposureTime().total_seconds(),'sensitivity_iso':packet.getSensitivity(),'tags':tags,'poses':poses}
            selected=metric_tag is not None and all(len(c['tags'])==1 and all(c['tags'][0][k]==metric_tag[k] for k in ('family','id')) for c in row['cameras'].values())
            row['tag_metric_check']=check_tag_triplet(row,report['extrinsics'],size=metric_tag['size_m']) if selected else {'passed':False,'reason':'No confirmed-size matching tag in all three views'}
            try:
                if metric_tag is None or any(len(row['cameras'][r]['tags'])!=1 or
                    any(row['cameras'][r]['tags'][0][k]!=metric_tag[k] for k in ('family','id'))
                    for r in ('B','C')):raise ValueError('No confirmed-size matching tag in both stereo views')
                row['stereo_tag_pose']=stereo_tag_pose(row,report['extrinsics'],size=metric_tag['size_m'])
            except ValueError as error:row['stereo_tag_pose']={'error':str(error)}
            report['frames'].append(row)
        if not report['frames']:raise RuntimeError('No synchronized calibrated triplet')
    (folder/'report.json').write_text(json.dumps(report,indent=2,allow_nan=False)+'\n')
    print('Camera report:',folder/'report.json',flush=True)
    print('Frames:',len(report['frames']),'tag detections:',{r:sum(bool(f['cameras'][r]['tags']) for f in report['frames']) for r in ('A','B','C')},flush=True)
    print('Passed metric checks:',sum(f['tag_metric_check']['passed'] for f in report['frames']),flush=True)
    print(report['frames'][-1]['tag_metric_check'],flush=True)


if __name__=='__main__':
    p=argparse.ArgumentParser();p.add_argument('--all-tags',action='store_true');p.add_argument('--tag-id',type=int);p.add_argument('--tag-size',type=float);p.add_argument('--mono-exposure-us',type=int,default=2000);p.add_argument('--mono-iso',type=int,default=200);p.add_argument('--output',type=Path);a=p.parse_args();main(a.all_tags,a.tag_id,a.tag_size,a.mono_exposure_us,a.mono_iso,a.output)
