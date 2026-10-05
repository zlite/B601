"""Local plate-grasp orientation from the close stereo table measurement."""
import numpy as np

TABLE_NORMAL_IN_GRID=np.array([.05920843,-.02253963,.99799111])
CONFIRMED_CONTACT_Q=-.1488550901412964
# Stereo outer rubber/white seams, both fingers and 12 synchronized frames.
# Inner edges were occluded by the plate and are deliberately excluded.
MEASURED_LONG_AXIS=np.array([.00838460,.69790207,.71604213])

def grasp_rotation(tips,table_corrected=False,normal_in_grid=None):
    span=tips[1]-tips[0];span/=np.linalg.norm(span)
    long=MEASURED_LONG_AXIS.copy() if normal_in_grid is not None else np.array([0,2**-.5,2**-.5])
    long-=span*(long@span);long/=np.linalg.norm(long)
    rotation=np.array([long,span,np.cross(long,span)])
    if table_corrected or normal_in_grid is not None:
        normal=TABLE_NORMAL_IN_GRID if normal_in_grid is None else np.asarray(normal_in_grid,float)
        if normal.shape!=(3,) or not np.isfinite(normal).all() or np.linalg.norm(normal)<.9 or normal[2]<.95:
            raise ValueError('Table normal inconsistent with the plate approach')
        normal=normal/np.linalg.norm(normal)
        longitudinal=np.array([1.,0.,0.]);longitudinal-=normal*(normal@longitudinal);longitudinal/=np.linalg.norm(longitudinal)
        rotation=np.column_stack([longitudinal,np.cross(normal,longitudinal),normal])@rotation
    return rotation
