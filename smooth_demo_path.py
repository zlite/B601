"""Join compatible demonstrated curves and time them with local derivative bounds."""
import numpy as np
from scipy.interpolate import CubicHermiteSpline
from blended_motion import BlendedPath
from replay_timing import ACCELERATION_LIMITS

class JoinedPath:
    advance=BlendedPath.advance
    def __init__(self,groups):
        originals=[BlendedPath(q,3.,9.) for q in groups]
        lengths=[np.max(abs(np.diff(c.points,axis=0)),axis=1).sum() for c in originals]
        total=sum(lengths);knots=[];points=[];derivatives=[];offset=0.
        for index,(curve,length) in enumerate(zip(originals,lengths)):
            s=(offset+curve.knots*length)/total;d=curve.spline(curve.knots,1)*total/length
            if index:
                if np.max(abs(points[-1]-curve.points[0]))>1e-8:raise ValueError('Discontinuous joined route')
                hL=knots[-1]-knots[-2];hR=s[1]-s[0];left=derivatives[-1];right=d[0]
                average=(hL*left+hR*right)/(hL+hR)
                average=np.where(left*right<=0,0.,average)
                dl=(points[-1]-points[-2])/hL;dr=(curve.points[1]-curve.points[0])/hR
                bound=3*np.minimum(abs(dl),abs(dr))
                derivatives[-1]=np.sign(average)*np.minimum(abs(average),bound)
            first=1 if index else 0
            knots.extend(s[first:]);points.extend(curve.points[first:]);derivatives.extend(d[first:]);offset+=length
        self.knots=np.asarray(knots);self.points=np.asarray(points)
        self.spline=CubicHermiteSpline(knots,points,derivatives)
        self.max_deviation=0.;offset=0.;piece=0
        # Exact extrema of the difference of cubics in their shared arc coordinate.
        for curve,length in zip(originals,lengths):
            factor=total/length
            for k,h0 in enumerate(np.diff(curve.knots)):
                h=h0/factor
                old=curve.spline.c[:,k,:]*np.array([factor**3,factor**2,factor,1.])[:,None]
                difference=self.spline.c[:,piece,:]-old
                for j in range(6):
                    a,b,c,d=difference[:,j];xs=[0.,h]
                    roots=np.roots([3*a,2*b,c]) if abs(a)+abs(b)>1e-12 else []
                    xs += [float(x.real) for x in roots if abs(x.imag)<1e-9 and 0<x.real<h]
                    self.max_deviation=max(self.max_deviation,max(abs(((a*x+b)*x+c)*x+d) for x in xs))
                piece+=1
            offset+=length
        if self.max_deviation>.08:raise ValueError('Joined path exceeds 0.08 degree reviewed-curve corridor')
        # Timing-independent sampling parameter, used by the collision checker.
        self.duration=1.
        dense=self.spline(np.unique(np.r_[self.knots,np.linspace(0,1,10001)]))
        if np.any(dense<self.points.min(0)-1e-8) or np.any(dense>self.points.max(0)+1e-8):raise ValueError('Joined curve overshoots envelope')
    def at(self,elapsed):return self.spline(float(np.clip(elapsed,0,1)))

class LocalTimedCurve:
    advance=BlendedPath.advance
    def __init__(self,path,speed=12.):
        if not 0<speed<=12:raise ValueError('Unreviewed speed ceiling')
        self.path=path;self.points=path.points;self.max_deviation=path.max_deviation
        grid=np.unique(np.concatenate([np.linspace(a,b,5) for a,b in zip(path.knots,path.knots[1:])]))
        velocity=[];acceleration=[]
        for low,high in zip(grid,grid[1:]):
            k=min(len(path.knots)-2,int(np.searchsorted(path.knots,low,side='right')-1))
            x0=low-path.knots[k];x1=high-path.knots[k]
            a,b,c,_=path.spline.c[:,k,:]
            d1=np.maximum(abs(3*a*x0*x0+2*b*x0+c),abs(3*a*x1*x1+2*b*x1+c))
            for j in range(6):
                if abs(a[j])>1e-12:
                    x=-b[j]/(3*a[j])
                    if x0<x<x1:d1[j]=max(d1[j],abs(3*a[j]*x*x+2*b[j]*x+c[j]))
            d2=np.maximum(abs(6*a*x0+2*b),abs(6*a*x1+2*b))
            # Reserve half the acceleration for path curvature and half for
            # tangential acceleration; triangle inequality bounds each joint.
            limits=.9*ACCELERATION_LIMITS
            velocity.append(min(float(np.min(speed/np.maximum(d1,1e-12))),float(np.min(np.sqrt(.5*limits/np.maximum(d2,1e-12))))))
            acceleration.append(float(np.min(.5*limits/np.maximum(d1,1e-12))))
        vmax=np.array(velocity);amax=np.array(acceleration);ds=np.diff(grid)
        speeds=np.r_[0.,np.minimum(vmax[:-1],vmax[1:]),0.]
        for i in range(len(ds)):speeds[i+1]=min(speeds[i+1],np.sqrt(speeds[i]**2+2*amax[i]*ds[i]))
        for i in reversed(range(len(ds))):speeds[i]=min(speeds[i],np.sqrt(speeds[i+1]**2+2*amax[i]*ds[i]))
        dt=2*ds/(speeds[:-1]+speeds[1:])
        if not np.isfinite(dt).all() or np.any(dt<=0):raise ValueError('Invalid local timing')
        self.s=grid;self.v=speeds;self.a=(speeds[1:]-speeds[:-1])/dt
        self.times=np.r_[0.,np.cumsum(dt)];self.duration=float(self.times[-1])
        self.velocity_limit=speed;self.acceleration_limits=.9*ACCELERATION_LIMITS
    def parameter(self,elapsed):
        t=float(np.clip(elapsed,0,self.duration));i=min(len(self.a)-1,int(np.searchsorted(self.times,t,side='right')-1));dt=t-self.times[i]
        return float(np.clip(self.s[i]+self.v[i]*dt+.5*self.a[i]*dt*dt,0,1))
    def at(self,elapsed):return self.path.spline(self.parameter(elapsed))
