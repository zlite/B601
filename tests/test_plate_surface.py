import unittest
import json
from pathlib import Path
import cv2
import numpy as np
from plate_surface import SurfaceTracker,plate_in_surface_frame,CALIBRATION,normal_uncertainty_deg
from plate_contact_guard import check_plate_release
from scipy.spatial.transform import Rotation
from plate_grasp_geometry import grasp_rotation,MEASURED_LONG_AXIS
from plate_hover import TIPS

class SurfaceTests(unittest.TestCase):
    def test_recorded_shadow_sequences_keep_table_tracking(self):
        for run in ('20261005T185718424449Z','20261005T190011333297Z'):
            folder=Path('outputs/plate_hover')/run
            if not folder.exists():continue
            views=json.loads((folder/'progress.json').read_text())['views']
            start=next(v['cameras']['wrist']['received'] for v in views if v['label']=='lower')
            tracker=SurfaceTracker();accepted=0
            for path in sorted((folder/'recording').glob('*_wrist.jpg')):
                stamp=float(path.name.split('_')[0])
                if start<=stamp<=start+3:
                    result=tracker.update(cv2.imread(str(path)))
                    self.assertLessEqual(result['normal_sigma_deg'],.2);accepted+=1
            self.assertGreaterEqual(accepted,5)

    def test_normal_uncertainty_rejects_insufficient_information(self):
        tracker=SurfaceTracker()
        with self.assertRaises(ValueError):
            normal_uncertainty_deg(np.zeros((30,3)),tracker.rvec,tracker.tvec,tracker.K,tracker.D,tracker.BA,tracker.normal,.5)
        with self.assertRaisesRegex(ValueError,'uncertainty'):
            normal_uncertainty_deg(tracker.model,tracker.rvec,tracker.tvec,tracker.K,tracker.D,tracker.BA,tracker.normal,50.)

    def test_global_brightness_and_local_shadow_preserve_normal(self):
        c=json.loads(CALIBRATION.read_text());image=cv2.imread(c['reference_image'])
        shadow=np.linspace(.45,1.,image.shape[1])[None,:,None]
        for variant in (np.clip(image*.55,0,255).astype(np.uint8),
                        np.clip(image*1.25,0,255).astype(np.uint8),
                        (image*shadow).astype(np.uint8)):
            result=SurfaceTracker().update(variant)
            angle=np.degrees(np.arccos(np.clip(np.array(c['normal_reference_b'])@result['normal_camera_b'],-1,1)))
            self.assertLess(angle,.5)

    def test_sunlit_holder_view_acquires_independent_table(self):
        p=Path('outputs/plate_hover/20261005T185718424449Z/lower_wrist.png')
        if not p.exists():self.skipTest('Recorded sunlight view unavailable')
        result=SurfaceTracker().update(cv2.imread(str(p)))
        self.assertGreaterEqual(result['point_count'],30)
        self.assertLess(result['rms_px'],.8)

    def test_camera_motion_does_not_look_like_plate_displacement(self):
        plate=np.eye(4);plate[:3,3]=[.03,.02,.15]
        camera=np.eye(4);camera[:3,:3]=Rotation.from_euler('y',3,degrees=True).as_matrix();camera[:3,3]=[.004,-.001,.002]
        observation={'T_camera_b_plate':(camera@plate).tolist(),'surface':{'T_camera_b_reference':camera.tolist()}}
        recovered=plate_in_surface_frame(observation)
        np.testing.assert_allclose(recovered,plate,atol=1e-12)
        check_plate_release(plate,recovered)
        moved=plate.copy();moved[0,3]+=.006
        observation['T_camera_b_plate']=(camera@moved).tolist()
        with self.assertRaisesRegex(RuntimeError,'shifted'):
            check_plate_release(plate,plate_in_surface_frame(observation))

    def test_missing_table_pose_cannot_pass_release(self):
        with self.assertRaises(RuntimeError):plate_in_surface_frame({'T_camera_b_plate':np.eye(4).tolist()})
    def test_captured_approach_acquires_independent_table_plane(self):
        p=Path('outputs/plate_hover/20261005T161455329882Z')
        tracker=SurfaceTracker()
        at25=tracker.update(cv2.imread(str(p/'lower_wrist.png')))
        self.assertGreater(at25['point_count'],100)
        at18=tracker.update(cv2.imread(str(p/'alignment_review_wrist.png')))
        expected=np.array([-.03013022,.6653912,-.74588653])
        self.assertLess(np.degrees(np.arccos(np.clip(expected@at18['normal_camera_b'],-1,1))),.3)
        self.assertLess(at18['rms_px'],.8)
    def test_occluded_table_is_rejected_without_overwriting_reference(self):
        tracker=SurfaceTracker();previous=tracker.previous.copy()
        with self.assertRaises(ValueError):tracker.update(np.zeros((800,1280,3),np.uint8))
        np.testing.assert_array_equal(tracker.previous,previous)
    def test_measured_fingers_lie_in_target_table_plane(self):
        normal=np.array([.015,-.025,1.]);normal/=np.linalg.norm(normal)
        R=grasp_rotation(TIPS,normal_in_grid=normal)
        for axis in ([.02038925,.6978619,.71594145],[-.00362006,.69794223,.71614281]):
            self.assertLess(abs(np.degrees(np.arcsin(normal@(R@axis)))),.1)
        np.testing.assert_allclose(R@R.T,np.eye(3),atol=1e-8)
        self.assertAlmostEqual(np.linalg.det(R),1.)
    def test_invalid_or_flipped_table_normal_rejected(self):
        for normal in ([0,0,-1],[float('nan'),0,1],[0,0,0],[1,0,0]):
            with self.assertRaises(ValueError):grasp_rotation(TIPS,normal_in_grid=normal)
