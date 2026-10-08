"""Time scaling of checked curves with per-axis velocity/acceleration bounds."""
import math
import numpy as np
from scipy.optimize import brentq
from blended_motion import BlendedPath

ACCELERATION_LIMITS = np.array([60.,60.,60.,80.,80.,80.])


def derivative_bounds(path):
    """Conservative component bounds on each spline piece after time easing."""
    velocity=np.zeros(6);acceleration=np.zeros(6)
    ease=lambda u:u**3*(10-15*u+6*u*u)
    inverse=lambda s:0. if s<=0 else 1. if s>=1 else brentq(lambda u:ease(u)-s,0.,1.)
    us=[inverse(float(s)) for s in path.knots]
    for k,(lo,hi) in enumerate(zip(us,us[1:])):
        h=path.knots[k+1]-path.knots[k]
        a,b,c,_=path.spline.c[:,k,:]
        d1=np.maximum(abs(c),abs(3*a*h*h+2*b*h+c))
        for j in range(6):
            if abs(a[j])>1e-12:
                x=-b[j]/(3*a[j])
                if 0<x<h:d1[j]=max(d1[j],abs(3*a[j]*x*x+2*b[j]*x+c[j]))
        d2=np.maximum(abs(2*b),abs(6*a*h+2*b))
        candidates=[lo,hi]+[x for x in (.5,) if lo<x<hi]
        sp=max(30*u*u*(1-u)**2 for u in candidates)
        candidates=[lo,hi]+[x for x in ((3-math.sqrt(3))/6,(3+math.sqrt(3))/6) if lo<x<hi]
        spp=max(abs(60*u*(1-u)*(1-2*u)) for u in candidates)
        velocity=np.maximum(velocity,d1*sp/path.duration)
        acceleration=np.maximum(acceleration,(d2*sp*sp+d1*spp)/path.duration**2)
    return velocity,acceleration


class TimedCurve:
    advance=BlendedPath.advance

    def __init__(self,path,requested_scale=1.):
        if not math.isfinite(requested_scale) or not 1<=requested_scale<=4:
            raise ValueError('Replay speed scale must be between 1 and 4')
        v,a=derivative_bounds(path)
        self.scale=min(float(requested_scale),12./max(float(np.max(v)),1e-12),float(np.min(np.sqrt(.95*ACCELERATION_LIMITS/np.maximum(a,1e-12)))))
        self.path=path;self.duration=path.duration/self.scale
        self.points=path.points;self.max_deviation=path.max_deviation
        self.velocity_bounds=v*self.scale;self.acceleration_bounds=a*self.scale**2

    def at(self,elapsed):
        return self.path.at(float(np.clip(elapsed,0,self.duration))*self.scale)
