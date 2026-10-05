#!/usr/bin/env python3
"""Offline independent gyro/gravity versus tag and encoder rotation comparison."""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

from camera_check import rotation_degrees


def vector_angle(a,b):
    a,b=np.asarray(a,dtype=float),np.asarray(b,dtype=float)
    return float(np.degrees(np.arccos(np.clip(a@b/(np.linalg.norm(a)*np.linalg.norm(b)),-1,1))))


def integrate(times, rates, start, end, bias_start, bias_end):
    times,rates=np.asarray(times),np.asarray(rates)
    if end<=start or start<times[0] or end>times[-1]:
        raise ValueError('Gyro timestamps do not bracket the interval')
    selected=(times>=start-.02)&(times<=end+.02)
    if np.max(np.diff(times[selected]))>.03:
        raise ValueError('Gyro sample gap exceeds 30 ms')
    t=np.r_[start,times[(times>start)&(times<end)],end]
    w=np.column_stack([np.interp(t,times,rates[:,i]) for i in range(3)])
    alpha=((t-start)/(end-start))[:,None]
    w-=np.asarray(bias_start)*(1-alpha)+np.asarray(bias_end)*alpha
    R=np.eye(3)
    for i,dt in enumerate(np.diff(t)):
        R=R@cv2.Rodrigues((w[i]+w[i+1])*.5*dt)[0]
    return R


def analyze(path):
    trial=json.loads(Path(path).read_text())
    session=trial['session']
    report=json.loads((Path('outputs/camera_checks')/session/'report.json').read_text())
    samples=report['samples']
    if len(samples)!=4 or trial.get('imu_type')!='BMI270':
        raise ValueError('Expected four completed captures with BMI270 data')
    # Duplicate gyro timestamps can occur when accel/gyro are batched differently.
    rows=list({r['gyro_time']:r for r in trial['imu_samples']}.values())
    rows.sort(key=lambda r:r['gyro_time'])
    times=np.array([r['gyro_time'] for r in rows]);rates=np.array([r['g'] for r in rows])
    poses=[]
    for sample in samples:
        t=sample['time'];idx=(times>=t-.6)&(times<=t)
        accel=[r['a'] for r in rows if t-.6<=r['accel_time']<=t]
        if idx.sum()<30 or len(accel)<30:
            raise ValueError('Insufficient stationary IMU samples')
        std=rates[idx].std(axis=0)
        if np.max(std)>.01:
            raise ValueError('Stationary gyro window contains excessive motion')
        poses.append({'time':t,'gyro_bias_rad_s':rates[idx].mean(axis=0).tolist(),
            'gyro_std_rad_s':std.tolist(),'gravity_m_s2':np.mean(accel,axis=0).tolist(),
            'gravity_std_m_s2':np.std(accel,axis=0).tolist(),'samples':int(idx.sum())})
    comparisons=[]
    for i,(a,b) in enumerate(zip(poses,poses[1:])):
        R=integrate(times,rates,a['time'],b['time'],a['gyro_bias_rad_s'],b['gyro_bias_rad_s'])
        measured=rotation_degrees(R)
        comparison=samples[i+1]['comparison']
        comparisons.append({'step':samples[i+1]['step'],'encoder_deg':comparison['model_rotation_deg'],
            'tag_deg':comparison['camera_rotation_deg'],'gyro_deg':measured,
            'gyro_vs_encoder_deg':abs(measured-comparison['model_rotation_deg']),
            'tag_vs_gyro_deg':abs(measured-comparison['camera_rotation_deg']),
            'gravity_tilt_deg':vector_angle(a['gravity_m_s2'],b['gravity_m_s2']),
            'gyro_predicted_gravity_error_deg':vector_angle(R.T@a['gravity_m_s2'],b['gravity_m_s2']),
            'gyro_rotation_matrix':R.tolist()})
    return {'session':session,'method':'Integrate 100 Hz raw body-frame gyro with interpolated stationary endpoint biases; independently compare gravity tilt',
        'calibration_applied':False,'pose_windows':poses,'comparisons':comparisons}


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('trial',type=Path)
    args=parser.parse_args()
    result=analyze(args.trial)
    output=args.trial.with_name('imu_comparison_'+result['session']+'.json')
    output.write_text(json.dumps(result,indent=2)+'\n')
    print(json.dumps(result,indent=2))


if __name__=='__main__':main()
