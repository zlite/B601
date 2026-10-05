#!/usr/bin/env python3
"""Read the wrist camera's inertial sensor; no motor bus or firmware writes."""
import json
from pathlib import Path
import time

import depthai as dai
import numpy as np
from camera_selection import wrist_pipeline


def main():
    result = {'motion_commanded':False,'firmware_update_requested':False}
    with wrist_pipeline() as pipeline:
        device=pipeline.getDefaultDevice()
        kind=device.getConnectedIMU()
        result['imu_type']=kind
        if kind and kind != 'NONE':
            imu=pipeline.create(dai.node.IMU)
            imu.enableFirmwareUpdate(False)
            imu.enableIMUSensor([dai.IMUSensor.ACCELEROMETER_RAW,dai.IMUSensor.GYROSCOPE_RAW],100)
            imu.setBatchReportThreshold(1)
            imu.setMaxBatchReports(10)
            queue=imu.out.createOutputQueue(maxSize=10,blocking=False)
            pipeline.start()
            end=time.monotonic()+5
            rows=[]
            while time.monotonic()<end:
                packet=queue.tryGet()
                if packet is None:
                    time.sleep(.005);continue
                for p in packet.packets:
                    a,g=p.acceleroMeter,p.gyroscope
                    rows.append({'a':[a.x,a.y,a.z],'g':[g.x,g.y,g.z],
                                 'time':a.getTimestamp().total_seconds()})
            result['samples']=rows
            if rows:
                result.update(mean_acceleration=np.mean([r['a'] for r in rows],axis=0).tolist(),
                              mean_gyro=np.mean([r['g'] for r in rows],axis=0).tolist())
    out=Path('outputs/camera_diagnosis/imu_probe.json')
    out.parent.mkdir(parents=True,exist_ok=True)
    out.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps({k:v for k,v in result.items() if k!='samples'}))
    print('samples',len(result.get('samples',[])))


if __name__=='__main__':main()
