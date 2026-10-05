"""Track the plate well grid from the measured hover image; no motor access."""
import json,time,threading
from pathlib import Path
import cv2
import numpy as np

REFERENCE=Path('outputs/plate_hover/20261004T205718996871Z')

class GridTracker:
    def __init__(self,reference=None):
        if reference is not None:
            c=json.loads(Path(reference).read_text())
            self.K=np.asarray(c['camera_matrix'],float);self.D=np.asarray(c['distortion'],float)
            self.BA=np.asarray(c['T_camera_a_b'],float);P=np.asarray(c['T_camera_b_plate'],float)
            self.model=np.asarray(c['model'],float);self.pixels=np.asarray(c['pixels'],np.float32).reshape(-1,1,2)
            if self.K.shape!=(3,3) or P.shape!=(4,4) or self.BA.shape!=(4,4) or self.model.ndim!=2 or self.model.shape[1]!=3 or len(self.model)<20 or len(self.pixels)!=len(self.model):raise ValueError('Invalid grid bootstrap dimensions')
            if not np.isfinite(np.r_[self.K.ravel(),self.D.ravel(),P.ravel(),self.BA.ravel(),self.model.ravel(),self.pixels.ravel()]).all():raise ValueError('Nonfinite grid bootstrap')
            A=self.BA@P;self.rvec=cv2.Rodrigues(A[:3,:3])[0];self.tvec=A[:3,3].reshape(3,1)
            predicted=cv2.projectPoints(self.model,self.rvec,self.tvec,self.K,self.D)[0]
            if np.sqrt(np.mean(np.sum((predicted-self.pixels)**2,axis=2)))>1.5:raise ValueError('Grid bootstrap reprojection failed')
            im=cv2.imread(c['reference_image'])
            if im is None or im.shape[:2]!=(800,1280):raise ValueError('Grid bootstrap image unavailable')
            self.contrast=cv2.createCLAHE(2.,(8,8));self.previous=self.contrast.apply(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY))
            return
        s=json.loads((REFERENCE/'progress.json').read_text())['views'][-1]['cameras']['wrist']
        self.K=np.array(s['camera_matrix']);self.D=np.array(s['distortion'])
        self.contrast=cv2.createCLAHE(2.,(8,8))
        self.previous=self.contrast.apply(cv2.cvtColor(cv2.imread(str(REFERENCE/'hover_wrist.png')),cv2.COLOR_BGR2GRAY))
        self.model=np.array([[.0495-r*.009,.0315-c*.009,0] for r in range(7) for c in range(8)],float)
        P=np.array(json.loads((REFERENCE/'plate_grid_pose.json').read_text())['T_camera_b_plate'])
        self.BA=np.array(json.loads(Path('outputs/contact_camera/20261004T210338098024Z/report.json').read_text())['extrinsics']['B_A'])
        A=self.BA@P;self.rvec=cv2.Rodrigues(A[:3,:3])[0];self.tvec=A[:3,3].reshape(3,1)
        self.pixels=cv2.projectPoints(self.model,self.rvec,self.tvec,self.K,self.D)[0].astype(np.float32)
    def update(self,image):
        gray=self.contrast.apply(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY))
        uv,status,_=cv2.calcOpticalFlowPyrLK(self.previous,gray,self.pixels,None,winSize=(61,61),maxLevel=3,criteria=(3,30,.01))
        back,reverse,_=cv2.calcOpticalFlowPyrLK(gray,self.previous,uv,None,winSize=(61,61),maxLevel=3,criteria=(3,30,.01))
        xy=uv.reshape(-1,2);good=(status.ravel()==1)&(reverse.ravel()==1)&(np.linalg.norm(back-self.pixels,axis=2).ravel()<.8)&np.all(xy>18,1)&(xy[:,0]<1261)&(xy[:,1]<781)
        if good.sum()<20:raise ValueError('Too few visible, consistent plate wells')
        model=self.model[good];xy=xy[good]
        required=max(20,.8*len(model))
        # Repeated wells can pass forward/backward optical flow while landing
        # one row away. Reject those correspondences using the planar lattice
        # before fitting a 3D pose; retain the existing 80% / 2 px requirements.
        undistorted=cv2.undistortPoints(xy.reshape(-1,1,2),self.K,self.D,P=self.K).reshape(-1,2)
        _,mask=cv2.findHomography(model[:,:2],undistorted,cv2.RANSAC,2.)
        if mask is None or mask.sum()<required:raise ValueError('Plate grid planar consensus failed')
        consensus=mask.ravel().astype(bool);model=model[consensus];xy=xy[consensus]
        ok,r,t=cv2.solvePnP(model,xy,self.K,self.D,self.rvec.copy(),self.tvec.copy(),True,flags=cv2.SOLVEPNP_ITERATIVE)
        projected=cv2.projectPoints(model,r,t,self.K,self.D)[0].reshape(-1,2)
        idx=np.flatnonzero(np.linalg.norm(projected-xy,axis=1)<2.)
        if not ok or len(idx)<required:raise ValueError('Plate grid pose is inconsistent')
        cv2.solvePnPRefineLM(model[idx],xy[idx],self.K,self.D,r,t)
        projected=cv2.projectPoints(model[idx],r,t,self.K,self.D)[0].reshape(-1,2)
        rms=float(np.sqrt(np.mean(np.sum((projected-xy[idx])**2,1))))
        if rms>1.5 or not np.isfinite(t).all() or t[2,0]<=.08:raise ValueError('Plate grid pose quality failed')
        self.previous=gray;self.pixels=xy[idx].reshape(-1,1,2);self.model=model[idx];self.rvec=r;self.tvec=t
        A=np.eye(4);A[:3,:3]=cv2.Rodrigues(r)[0];A[:3,3]=t.ravel();B=np.linalg.inv(self.BA)@A
        return {'T_camera_b_plate':B.tolist(),'rms_px':rms,'well_count':len(idx)}

class GridObserver:
    def __init__(self,w):
        self.w=w;self.latest=None;self.tracker=None;self.stop=threading.Event();self.thread=threading.Thread(target=self.run,daemon=True)
        self.surface_enabled=False;self.surface_tracker=None;self.last_rejection=None
        self.surface_required=True
    def resume(self):
        if self.thread.is_alive():raise RuntimeError('Plate observer still running')
        self.latest=None;self.stop=threading.Event()
        self.thread=threading.Thread(target=self.run,daemon=True);self.thread.start()
    def run(self):
        if self.tracker is None:self.tracker=GridTracker(getattr(self.w,'grid_reference',None))
        tracker=self.tracker;last=0
        while not self.stop.is_set():
            with self.w.lock:frame=dict(self.w.survey_frames.get('wrist',{}))
            if frame.get('received',0)<=last:time.sleep(.005);continue
            last=frame['received']
            self.process_frame(frame)
    def process_frame(self,frame):
        try:
            data={**self.tracker.update(frame['image']),'received':frame['received'],'valid':True}
            if self.surface_enabled:
                from plate_surface import SurfaceTracker
                if self.surface_tracker is None:self.surface_tracker=SurfaceTracker()
                try:data['surface']=self.surface_tracker.update(frame['image'])
                except (ValueError,cv2.error) as error:
                    if self.surface_required:raise
                    data['surface_error']=str(error)
            # Publish a synchronized, fully accepted frame. Never combine an
            # old table transform with the current camera-relative plate pose.
            self.latest=data;self.last_rejection=None
        except (ValueError,cv2.error) as error:
            self.last_rejection={'reason':str(error),'received':frame['received']}
            # Keep retrying new frames. The last good observation retains its
            # ORIGINAL timestamp and expires after the existing 500 ms limit.
    def get(self):
        data=self.latest
        if not data or not data.get('valid') or time.monotonic()-data['received']>.5:raise ValueError('Plate grid unavailable: '+str(self.last_rejection or data))
        return data
