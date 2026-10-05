"""Offline fingertip triangulation with an independent AprilTag metric check."""
import cv2
import numpy as np


def triangulate(pixels_b,pixels_c,cal_b,cal_c,T_c_b):
    b=cv2.undistortPoints(np.array(pixels_b,dtype=float).reshape(-1,1,2),np.array(cal_b['camera_matrix']),np.array(cal_b['distortion'])).reshape(-1,2)
    c=cv2.undistortPoints(np.array(pixels_c,dtype=float).reshape(-1,1,2),np.array(cal_c['camera_matrix']),np.array(cal_c['distortion'])).reshape(-1,2)
    T=np.array(T_c_b,dtype=float)
    h=cv2.triangulatePoints(np.eye(3,4),T[:3],b.T,c.T)
    points=(h[:3]/h[3]).T
    other=points@T[:3,:3].T+T[:3,3]
    if not np.isfinite(points).all() or min(points[:,2].min(),other[:,2].min())<=0:
        raise ValueError('Triangulated point is not in front of both cameras')
    errors=[]
    for xyz,cal,px in [(points,cal_b,pixels_b),(other,cal_c,pixels_c)]:
        projected=cv2.projectPoints(xyz,np.zeros(3),np.zeros(3),np.array(cal['camera_matrix']),np.array(cal['distortion']))[0].reshape(-1,2)
        errors.append(np.linalg.norm(projected-np.array(px),axis=1))
    return points,np.maximum(*errors)


def check_tag_triplet(frame,extrinsics,size=.06):
    cams=frame['cameras'];a,b,c=[cams[k] for k in ('A','B','C')]
    if any(len(cam['tags'])!=1 or not cam['poses'] for cam in (a,b,c)):
        return {'passed':False,'reason':'Tag must be visible in all three calibrated images'}
    try:points,errors=triangulate(b['tags'][0]['corners_px'],c['tags'][0]['corners_px'],b,c,extrinsics['B_C'])
    except ValueError as error:return {'passed':False,'reason':str(error)}
    edges=np.linalg.norm(points-np.roll(points,1,axis=0),axis=1)
    T=np.array(extrinsics['B_A']);points_a=points@T[:3,:3].T+T[:3,3]
    pose=a['poses'][0];R=cv2.Rodrigues(np.array(pose['rotation_vector']))[0];t=np.array(pose['translation_camera_m'])
    half=size/2;tag=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]])
    reference=tag@R.T+t
    disagreement=np.linalg.norm(points_a-reference,axis=1)
    edge_error=float(np.max(abs(edges-size)));max_error=float(np.max(errors));rgb_error=float(np.max(disagreement))
    return {'passed':bool(edge_error<=.003 and max_error<=1. and rgb_error<=.005),
            'max_edge_error_m':edge_error,'max_reprojection_error_px':max_error,
            'max_rgb_pose_disagreement_m':rgb_error,'stereo_tag_corners_camera_a_m':points_a.tolist(),
            'tag_edge_lengths_m':edges.tolist()}


def stereo_tag_pose(frame,extrinsics,size=.06):
    """Estimate tag pose directly in camera B, without an RGB conversion.

    Metric edges, stereo reprojection, and rigid-square residual must agree.
    This is a camera observation, not a validation of contact geometry.
    """
    b,c=[frame['cameras'][k] for k in ('B','C')]
    if len(b['tags'])!=1 or len(c['tags'])!=1:
        raise ValueError('Whole tag must be visible in both stereo images')
    points,errors=triangulate(b['tags'][0]['corners_px'],c['tags'][0]['corners_px'],b,c,extrinsics['B_C'])
    edges=np.linalg.norm(points-np.roll(points,1,axis=0),axis=1)
    if np.max(abs(edges-size))>.003 or np.max(errors)>1.:
        raise ValueError('Stereo metric/reprojection check failed')
    half=size/2;model=np.array([[-half,half,0],[half,half,0],[half,-half,0],[-half,-half,0]])
    center=points.mean(0);u,_,v=np.linalg.svd(model.T@(points-center))
    correction=np.eye(3);correction[2,2]=np.linalg.det(v.T@u.T)
    R=v.T@correction@u.T
    residual=np.linalg.norm(model@R.T+center-points,axis=1)
    if np.max(residual)>.003:raise ValueError('Stereo corners do not fit the known rigid tag')
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=center
    return {'T_camera_b_tag':T.tolist(),'max_rigid_square_error_m':float(max(residual)),
            'max_stereo_reprojection_error_px':float(max(errors)),
            'max_tag_edge_error_m':float(max(abs(edges-size)))}
