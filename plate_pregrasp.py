"""Require several fresh stationary samples at the existing grasp boundary."""
import time
import numpy as np


def wait_pregrasp(get,tick,tips,save,timeout=2.):
    deadline=time.monotonic()+timeout;last=None;good=0
    while time.monotonic()<deadline:
        tick()
        try:data=get()
        except ValueError:good=0;continue
        if data['received']==last:continue
        last=data['received'];P=np.asarray(data['T_camera_b_plate'],float)
        pin=(np.linalg.inv(P)@np.c_[tips,np.ones(2)].T).T[:,:3]
        valid=bool(.006<=pin[:,2].min()<=.010 and abs(pin[:,1].mean())<=.0015 and min(abs(pin[:,1]))>=.045)
        save({'received':last,'tips_in_plate_m':pin.tolist(),'accepted':valid})
        good=good+1 if valid else 0
        if good>=3:return data
    raise ValueError('Pregrasp geometry did not stabilize inside the existing bounds')
