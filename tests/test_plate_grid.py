import unittest
from unittest.mock import patch,Mock
from pathlib import Path
import cv2
import numpy as np
from plate_grid import GridTracker,GridObserver,REFERENCE


class GridTests(unittest.TestCase):
    def test_recent_hover_sequence_keeps_consensus(self):
        reference=Path('outputs/plate_grid_reseed_candidate.json')
        root=Path('outputs/plate_hover/20261005T212418495446Z/recording')
        if not reference.exists() or not root.exists():self.skipTest('Reacquisition sequence unavailable')
        tracker=GridTracker(reference);count=0
        for path in sorted(root.glob('*_wrist.jpg')):
            if not 422340.7<float(path.name.split('_')[0])<422348.8:continue
            result=tracker.update(cv2.imread(str(path)));count+=1
            self.assertGreaterEqual(result['well_count'],30)
            self.assertLess(result['rms_px'],.5)
        self.assertGreaterEqual(count,10)

    def test_rejected_frame_retries_without_refreshing_old_timestamp(self):
        observer=GridObserver(None);observer.tracker=Mock()
        observer.tracker.update.side_effect=[{'pose':'first'},ValueError('shadow'),{'pose':'recovered'}]
        observer.process_frame({'image':None,'received':10.})
        observer.process_frame({'image':None,'received':10.1})
        with patch('plate_grid.time.monotonic',return_value=10.2):
            self.assertEqual(observer.get()['received'],10.)
        with patch('plate_grid.time.monotonic',return_value=10.6):
            with self.assertRaisesRegex(ValueError,'shadow'):observer.get()
        observer.process_frame({'image':None,'received':10.7})
        with patch('plate_grid.time.monotonic',return_value=10.7):
            self.assertEqual(observer.get()['pose'],'recovered')

    def test_surface_rejection_preserves_paired_frame_not_mixed_transforms(self):
        observer=GridObserver(None);observer.surface_enabled=True
        observer.tracker=Mock();observer.surface_tracker=Mock()
        observer.tracker.update.side_effect=[{'pose':'old'}, {'pose':'new'}]
        observer.surface_tracker.update.side_effect=[{'table':'old'},ValueError('glint')]
        observer.process_frame({'image':None,'received':10.})
        observer.process_frame({'image':None,'received':10.1})
        with patch('plate_grid.time.monotonic',return_value=10.2):
            self.assertEqual(observer.get()['pose'],'old')
            self.assertEqual(observer.get()['surface']['table'],'old')
            self.assertEqual(observer.get()['received'],10.)

    def test_changed_sunlight_acquires_consistent_wells(self):
        path=Path('outputs/plate_hover/20261005T185123742274Z/hover_wrist.png')
        if not path.exists():self.skipTest('Recorded sunlight image unavailable')
        result=GridTracker().update(cv2.imread(str(path)))
        self.assertGreaterEqual(result['well_count'],30)
        self.assertLess(result['rms_px'],.5)

    def test_clear_carry_can_track_plate_without_fabricating_table_pose(self):
        observer=GridObserver(None);observer.surface_enabled=True;observer.surface_required=False
        observer.tracker=Mock();observer.surface_tracker=Mock()
        observer.tracker.update.return_value={'pose':'current'}
        observer.surface_tracker.update.side_effect=ValueError('occluded wood')
        observer.process_frame({'image':None,'received':10.})
        with patch('plate_grid.time.monotonic',return_value=10.1):
            self.assertNotIn('surface',observer.get())
            self.assertEqual(observer.get()['surface_error'],'occluded wood')
        observer.surface_required=True
        observer.process_frame({'image':None,'received':10.2})
        with patch('plate_grid.time.monotonic',return_value=10.6):
            with self.assertRaises(ValueError):observer.get()

    def test_replacement_holder_rejects_wrong_row_matches(self):
        image=Path('outputs/plate_hover/20261005T173024832559Z/hover_wrist.png')
        if not image.exists():self.skipTest('Recorded replacement holder image unavailable')
        tracker=GridTracker()
        result=tracker.update(cv2.imread(str(image)))
        self.assertGreaterEqual(result['well_count'],33)
        self.assertLess(result['rms_px'],.5)
        self.assertGreaterEqual(np.ptp(tracker.model[:,0]),.045)
        self.assertGreaterEqual(np.ptp(tracker.model[:,1]),.054)

    def test_reference_image_preserves_pose(self):
        tracker=GridTracker();r=tracker.rvec.copy();t=tracker.tvec.copy()
        tracker.update(cv2.imread(str(REFERENCE/'hover_wrist.png')))
        np.testing.assert_allclose(tracker.rvec,r,atol=1e-4)
        np.testing.assert_allclose(tracker.tvec,t,atol=1e-5)

    def test_no_consensus_does_not_mutate_tracking_state(self):
        tracker=GridTracker();previous=tracker.previous.copy();model=tracker.model.copy()
        with patch('plate_grid.cv2.findHomography',return_value=(None,None)):
            with self.assertRaisesRegex(ValueError,'consensus'):
                tracker.update(cv2.imread(str(REFERENCE/'hover_wrist.png')))
        np.testing.assert_array_equal(tracker.previous,previous)
        np.testing.assert_array_equal(tracker.model,model)
