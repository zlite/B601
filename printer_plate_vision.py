"""Acquire the complete 96-well lattice independently in synchronized stereo.

No saved image, bed-to-plate offset, robot state, or motor interface is used.
The rectangular grid has a 180-degree ambiguity: this reports its centre and
axes for a symmetric grasp, never an A1 identity. Partial grids fail closed.
"""
import cv2
import numpy as np
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import time

from contact_geometry import triangulate


def well_model():
    return np.array([[(col-5.5)*.009, (row-3.5)*.009, 0.]
                     for row in range(8) for col in range(12)])


def detect_wells(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    if gray.dtype != np.uint8 or gray.ndim != 2:
        raise ValueError('Expected an 8-bit camera image')
    p = cv2.SimpleBlobDetector_Params()
    p.minThreshold = 10; p.maxThreshold = 245; p.thresholdStep = 15
    p.filterByArea = True; p.minArea = 5; p.maxArea = 1500
    p.filterByCircularity = False; p.filterByConvexity = False
    p.filterByInertia = True; p.minInertiaRatio = .08
    p.filterByColor = True; p.blobColor = 0
    p.minDistBetweenBlobs = 3
    detector = cv2.SimpleBlobDetector_create(p)
    # Reject a second distinct lattice rather than choosing an arbitrary plate.
    solutions = []
    for frame in (gray, 255-gray):
        ok, pixels = cv2.findCirclesGrid(frame, (12, 8),
            flags=cv2.CALIB_CB_SYMMETRIC_GRID | cv2.CALIB_CB_CLUSTERING,
            blobDetector=detector)
        if ok:
            xy = pixels.reshape(96, 2).astype(float)
            if (np.min(xy) < 4 or np.max(xy[:, 0]) >= gray.shape[1]-4
                    or np.max(xy[:, 1]) >= gray.shape[0]-4):
                continue
            if not any(min(np.max(np.linalg.norm(xy-other, axis=1)),
                           np.max(np.linalg.norm(xy-other[::-1], axis=1))) < 1.
                       for other in solutions):
                solutions.append(xy)
    if len(solutions) != 1:
        raise ValueError('Need one complete, unambiguous 12 by 8 well grid')
    return solutions[0]


def fit_lattice(points):
    points = np.asarray(points, float)
    if points.shape != (96, 3) or not np.isfinite(points).all():
        raise ValueError('Need 96 finite stereo well centres')
    model = well_model(); centre = points.mean(0)
    u, _, vt = np.linalg.svd(model.T @ (points-centre))
    correction = np.eye(3); correction[2, 2] = np.linalg.det(vt.T @ u.T)
    rotation = vt.T @ correction @ u.T
    error = np.linalg.norm(model @ rotation.T+centre-points, axis=1)
    grid = points.reshape(8, 12, 3)
    pitch = np.r_[np.linalg.norm(np.diff(grid, axis=0), axis=2).ravel(),
                  np.linalg.norm(np.diff(grid, axis=1), axis=2).ravel()]
    if np.max(error) > .0015 or np.max(abs(pitch-.009)) > .0015:
        raise ValueError('Stereo wells do not fit the 9 mm rigid plate lattice')
    transform = np.eye(4); transform[:3, :3] = rotation; transform[:3, 3] = centre
    return transform, float(np.max(error)), float(np.max(abs(pitch-.009)))


def stereo_plate_pose(pixels_b, pixels_c, cal_b, cal_c, T_c_b):
    b, c = (np.asarray(p, float).reshape(-1, 2) for p in (pixels_b, pixels_c))
    if b.shape != (96, 2) or c.shape != (96, 2) or not np.isfinite(np.r_[b, c]).all():
        raise ValueError('Need the complete grid in both stereo images')
    # Each detector may choose a different starting corner. Test all rectangular
    # symmetries against epipolar and metric constraints; never assume ordering.
    candidates = []
    for rows, cols in ((1, 1), (-1, -1), (1, -1), (-1, 1)):
        other = c.reshape(8, 12, 2)[::rows, ::cols].reshape(96, 2)
        try:
            xyz, reprojection = triangulate(b, other, cal_b, cal_c, T_c_b)
            if np.max(reprojection) > .8:
                continue
            T, residual, pitch_error = fit_lattice(xyz)
            # Define +Z towards the observing camera (above the well surface).
            if T[:3, 2] @ T[:3, 3] > 0:
                xyz = xyz.reshape(8, 12, 3)[::-1].reshape(96, 3)
                T, residual, pitch_error = fit_lattice(xyz)
            if abs(T[:3, 2] @ (T[:3, 3]/np.linalg.norm(T[:3, 3]))) < .15:
                continue  # Edge-on normal is too poorly constrained.
            candidates.append(dict(T_camera_b_plate=T.tolist(), well_count=96,
                max_lattice_error_m=residual, max_pitch_error_m=pitch_error,
                max_stereo_reprojection_error_px=float(np.max(reprojection)),
                method='complete_stereo_well_grid', well_identity_validated=False,
                orientation_symmetry_deg=180, motion_ready=False))
        except (ValueError, cv2.error, np.linalg.LinAlgError):
            continue
    if len(candidates) != 1:
        raise ValueError('Stereo plate correspondence is missing or ambiguous')
    return candidates[0]


def observe_plate(images, calibration, T_c_b):
    b, c = [detect_wells(images[role]) for role in ('B', 'C')]
    return stereo_plate_pose(b, c, calibration['B'], calibration['C'], T_c_b)


def analyze_job(images, calibration, T, bed):
    cv2.setNumThreads(1)  # Only this separate worker; preserve camera-stream timing.
    result = dict(valid=False, observation_time=bed['observation_time'],
                  bed_reference=bed, motion_ready=False)
    try:
        result.update(observe_plate(images, calibration, T), valid=True)
    except (ValueError, cv2.error, np.linalg.LinAlgError) as error:
        result['reason'] = str(error)
    result['processing_finished_at'] = time.monotonic()
    return result


class PlateVisionWorker:
    """One job in a separate process; no queued backlog or camera-thread wait."""
    def __init__(self):
        self.executor = ProcessPoolExecutor(max_workers=1, mp_context=multiprocessing.get_context('spawn'))
        self.future = None
        self._latest = None

    @property
    def latest(self):
        if self.future is not None and self.future.done():
            try:
                self._latest = self.future.result()
            except Exception as error:
                self._latest = dict(valid=False, reason='Plate worker failed: '+str(error), motion_ready=False)
            self.future = None
        return self._latest

    def submit(self, images, calibration, T_c_b, bed_observation):
        self.latest  # Harvest completion without waiting.
        if self.future is None:
            self.future = self.executor.submit(analyze_job, images, calibration, T_c_b, dict(bed_observation))

    def close(self):
        self.executor.shutdown(wait=False, cancel_futures=True)
