"""Track the wooden support plane independently of the transparent well grid.

Reference landmarks are ray/plane intersections in the calibrated stereo
camera frame. No robot kinematics or assumed finger angle enters this fit.
"""
import json
from pathlib import Path
import cv2
import numpy as np

CALIBRATION=Path('calibration/plate_surface_dense_20261005.json')

def normal_uncertainty_deg(model,r,t,K,D,BA,normal,rms):
    """Local normal uncertainty under a >=0.5 px isotropic noise model."""
    _,jac=cv2.projectPoints(model,r,t,K,D);J=jac[:,:6]
    if np.linalg.matrix_rank(J)<6:raise ValueError('Table pose is unobservable')
    covariance=np.linalg.inv(J.T@J)*max(float(rms),.5)**2
    base=BA[:3,:3].T@cv2.Rodrigues(r)[0]@normal;Jn=np.zeros((3,3))
    for i in range(3):
        perturbed=r.copy();perturbed[i]+=1e-5
        Jn[:,i]=(BA[:3,:3].T@cv2.Rodrigues(perturbed)[0]@normal-base)/1e-5
    sigma=float(np.degrees(np.sqrt(max(0.,np.trace(Jn@covariance[:3,:3]@Jn.T)))))
    if not np.isfinite(sigma) or sigma>.2:raise ValueError('Table normal uncertainty too high')
    return sigma

def plate_in_surface_frame(observation):
    """Remove wrist-camera movement before judging placement on the table."""
    try:
        B=np.asarray(observation['surface']['T_camera_b_reference'],float)
        P=np.asarray(observation['T_camera_b_plate'],float)
    except (KeyError,TypeError,ValueError) as error:raise RuntimeError('Independent table pose unavailable') from error
    if B.shape!=(4,4) or P.shape!=(4,4) or not np.isfinite(np.r_[B.ravel(),P.ravel()]).all():
        raise RuntimeError('Invalid table-relative plate pose')
    return np.linalg.solve(B,P)

class SurfaceTracker:
    def __init__(self, calibration=CALIBRATION):
        c=json.loads(Path(calibration).read_text())
        self.K=np.array(c['camera_matrix']);self.D=np.array(c['distortion'])
        self.BA=np.array(c['T_camera_a_b']);self.normal=np.array(c['normal_reference_b'])
        self.contrast=cv2.createCLAHE(2.,(8,8))
        self.previous=self.contrast.apply(cv2.cvtColor(cv2.imread(c['reference_image']),cv2.COLOR_BGR2GRAY))
        self.pixels=np.array(c['pixels'],np.float32).reshape(-1,1,2)
        self.model=np.array(c['points_reference_b_m'],float)
        self.rvec=cv2.Rodrigues(self.BA[:3,:3])[0];self.tvec=self.BA[:3,3].reshape(3,1)
        self.reference=self.previous.copy();self.reference_pixels=self.pixels.copy();self.reference_model=self.model.copy()
        self.frames=0

    def update(self,image):
        # Periodically recover landmarks from the fixed reference. Permanently
        # pruning every temporarily lost corner makes long holds brittle.
        modes=[True,False] if self.frames%10==0 else [False,True]
        error=None
        for anchored in modes:
            try:
                result=self._track(image,anchored);self.frames+=1;return result
            except (ValueError,cv2.error) as caught:error=caught
        raise ValueError(str(error))

    def _track(self,image,anchored):
        gray=self.contrast.apply(cv2.cvtColor(image,cv2.COLOR_BGR2GRAY))
        previous=self.reference if anchored else self.previous
        pixels=self.reference_pixels if anchored else self.pixels
        model=self.reference_model if anchored else self.model
        guess=cv2.projectPoints(model,self.rvec,self.tvec,self.K,self.D)[0].astype(np.float32) if anchored else None
        uv,status,_=cv2.calcOpticalFlowPyrLK(previous,gray,pixels,guess,winSize=(31,31),maxLevel=3,criteria=(3,30,.01),flags=cv2.OPTFLOW_USE_INITIAL_FLOW if anchored else 0)
        back,reverse,_=cv2.calcOpticalFlowPyrLK(gray,previous,uv,pixels.copy() if anchored else None,winSize=(31,31),maxLevel=3,criteria=(3,30,.01),flags=cv2.OPTFLOW_USE_INITIAL_FLOW if anchored else 0)
        xy=uv.reshape(-1,2)
        good=(status.ravel()==1)&(reverse.ravel()==1)&(np.linalg.norm(back-pixels,axis=2).ravel()<.6)
        good&=(xy[:,0]>15)&(xy[:,0]<gray.shape[1]-15)&(xy[:,1]>15)&(xy[:,1]<gray.shape[0]-15)
        if good.sum()<30:raise ValueError('Too few table landmarks')
        model=model[good];xy=xy[good]
        # The landmarks are coplanar by construction. Establish consensus in
        # that plane before solving 3D pose, avoiding unstable minimal PnP
        # subsets under changing highlights. Keep the existing pixel/inlier
        # thresholds and spatial coverage requirements.
        _,_,basis=np.linalg.svd(model-model.mean(0),full_matrices=False)
        plane=model@basis[:2].T
        undistorted=cv2.undistortPoints(xy.reshape(-1,1,2),self.K,self.D,P=self.K).reshape(-1,2)
        _,mask=cv2.findHomography(plane,undistorted,cv2.RANSAC,1.2)
        if mask is None or mask.sum()<max(30,.8*len(model)):raise ValueError('Table landmark pose inconsistent')
        idx=mask.ravel().astype(bool);model=model[idx];xy=xy[idx]
        if np.ptp(model[:,0])<.06 or np.ptp(model[:,1])<.02:raise ValueError('Table landmarks insufficiently spread')
        ok,r,t=cv2.solvePnP(model,xy,self.K,self.D,self.rvec.copy(),self.tvec.copy(),True,flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:raise ValueError('Table pose solve failed')
        cv2.solvePnPRefineLM(model,xy,self.K,self.D,r,t)
        projected=cv2.projectPoints(model,r,t,self.K,self.D)[0].reshape(-1,2)
        rms=float(np.sqrt(np.mean(np.sum((projected-xy)**2,axis=1))))
        if rms>.8 or not np.isfinite(t).all():raise ValueError('Table pose quality failed')
        sigma=normal_uncertainty_deg(model,r,t,self.K,self.D,self.BA,self.normal,rms)
        rotation=self.BA[:3,:3].T@cv2.Rodrigues(r)[0]
        normal=rotation@self.normal
        A=np.eye(4);A[:3,:3]=cv2.Rodrigues(r)[0];A[:3,3]=t.ravel()
        B=np.linalg.solve(self.BA,A)
        self.previous=gray;self.pixels=xy.astype(np.float32).reshape(-1,1,2);self.model=model;self.rvec=r;self.tvec=t
        return {'normal_camera_b':normal.tolist(),'T_camera_b_reference':B.tolist(),'rms_px':rms,'point_count':len(model),'reference_anchored':anchored,'normal_sigma_deg':sigma}
