"""Shape-preserving joint paths with zero-speed ends and no waypoint dwells."""
import math
import numpy as np
from scipy.interpolate import PchipInterpolator


def checked_blend_segments(points,speed,acceleration,between_checks=lambda:None):
    """Blend a recorded clear route; retain a stop only at an unblendable turn."""
    q=np.asarray(points,float)
    if q.ndim!=2 or q.shape[1]!=6 or len(q)<1 or not np.isfinite(q).all():
        raise ValueError('Invalid retrace route')
    q=q[np.r_[True,np.max(abs(np.diff(q,axis=0)),axis=1)>1e-8]]
    if len(q)<2:return []
    pending=[q];result=[]
    while pending:
        segment=pending.pop();between_checks()
        try:
            BlendedPath(segment,speed,acceleration)
            result.append(segment.tolist())
        except ValueError as error:
            if str(error)!='Blend cuts too far from the dense route' or len(segment)<=2:raise
            middle=len(segment)//2
            pending.extend([segment[middle:],segment[:middle+1]])
    return result

class BlendedPath:
    def __init__(self,points,speed,acceleration,max_deviation=.2):
        q=np.asarray(points,dtype=float)
        if q.ndim!=2 or q.shape[1]!=6 or len(q)<2 or not np.isfinite(q).all():raise ValueError('Invalid blended path')
        if not 0<speed<=48 or not 0<acceleration<=288:raise ValueError('Invalid blend limits')
        keep=np.r_[True,np.max(abs(np.diff(q,axis=0)),axis=1)>1e-8];q=q[keep]
        if len(q)<2:raise ValueError('Blended path has no travel')
        distance=np.max(abs(np.diff(q,axis=0)),axis=1);s=np.r_[0.,np.cumsum(distance)];s/=s[-1]
        self.points=q;self.knots=s;self.spline=PchipInterpolator(s,q,axis=0)
        # Exact component derivative extrema of each cubic piece.
        d1=d2=0.
        for k,h in enumerate(np.diff(s)):
            a,b,c,_=self.spline.c[:,k,:]
            for j in range(6):
                xs=[0.,float(h)]
                if abs(a[j])>1e-12:
                    root=-b[j]/(3*a[j])
                    if 0<root<h:xs.append(float(root))
                d1=max(d1,max(abs(3*a[j]*x*x+2*b[j]*x+c[j]) for x in xs))
                d2=max(d2,abs(2*b[j]),abs(6*a[j]*h+2*b[j]))
        # Quintic smoothstep: max first derivative 1.875; max second 10/sqrt(3).
        self.duration=max(.25,1.875*d1/speed,math.sqrt((1.875**2*d2+10/math.sqrt(3)*d1)/acceleration))*1.01
        dense=np.unique(np.r_[s,np.linspace(0,1,2001)])
        values=self.spline(dense)
        linear=np.array([np.interp(dense,s,q[:,j]) for j in range(6)]).T
        self.max_deviation=float(np.max(abs(values-linear)))
        if self.max_deviation>max_deviation:raise ValueError('Blend cuts too far from the dense route')
        if np.any(values<q.min(0)-1e-8) or np.any(values>q.max(0)+1e-8):raise ValueError('Blend overshoots joint envelope')
    def at(self,elapsed):
        u=float(np.clip(elapsed/self.duration,0,1));s=u**3*(10-15*u+6*u*u)
        return self.spline(s)
    def advance(self,elapsed,dt,actual,margin):
        proposed=min(self.duration,elapsed+dt)
        if np.max(abs(self.at(proposed)-actual))<=margin:return proposed
        low,high=elapsed,proposed
        for _ in range(16):
            m=(low+high)/2
            if np.max(abs(self.at(m)-actual))<=margin:low=m
            else:high=m
        return low
