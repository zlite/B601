import json
from pathlib import Path
import tempfile
import unittest

import cv2
import numpy as np

from camera_check import CameraCheck, compare_views, BEND_STEPS


class Geometry:
    fingerprint = 'synthetic'
    def transform(self, q):
        T=np.eye(4)
        for axis,angle in [([0,0,1],q[3]),([0,1,0],q[4]),([1,0,0],q[5])]:
            T[:3,:3]=T[:3,:3]@cv2.Rodrigues(np.array(axis,dtype=float)*angle)[0]
        T[:3,3]=[.2,.1,.3]
        return T


class CameraCheckTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory()
        self.check=CameraCheck(Geometry(),Path(self.temp.name))
        self.now=100.
        self.image=np.zeros((20,30,3),dtype=np.uint8)
        self.X=np.eye(4);self.X[:3,:3]=cv2.Rodrigues(np.array([.3,-.2,.1]))[0]

    def tearDown(self):self.temp.cleanup()

    def feed(self,q=None,*,following=False,powered=True,valid=True,seconds=2.,camera_extra=0.,fault=None,delay=.08,raw_image=None):
        q=[0.]*6 if q is None else q
        for i in range(round(seconds/.05)):
            self.now+=.05
            self.check.add_joints(self.now,q,powered=powered,following=following,all_powered=True,fault=fault)
            if i%2==0:
                A=self.check.geometry.transform(np.radians(q))
                B=np.linalg.inv(self.X)@np.linalg.inv(A)
                B[:3,:3]=cv2.Rodrigues(np.array([camera_extra,0.,0.]))[0]@B[:3,:3]
                self.check.add_frame({'time':self.now-delay,'valid':valid,'reason':'Tag not visible',
                    'T_camera_tag':B.tolist(),'device_id':'synthetic','lens_position':76},self.image,now=self.now,raw_image=raw_image)

    def test_raw_pixels_and_stationary_measurements_are_preserved(self):
        raw=self.image.copy();raw[0,0]=[12,34,56]
        self.check.arm_capture();self.feed(raw_image=raw)
        directory=Path(self.temp.name)/self.check.session
        np.testing.assert_array_equal(cv2.imread(str(directory/'0_raw.png')),raw)
        sample=json.loads((directory/'report.json').read_text())['samples'][0]
        self.assertEqual(sample['raw_image'],'0_raw.png')
        self.assertGreaterEqual(len(sample['stationary_observations']),7)
        self.assertNotIn('image',sample['stationary_observations'][0])

    def test_bend_repeat_and_independent_results_survive_reload(self):
        output=Path(self.temp.name)/'camera_checks'
        self.check=CameraCheck(Geometry(),output,steps=BEND_STEPS)
        self.check.arm_capture();self.feed()
        for q in [[0,0,0,-8,0,0],[0]*6,[0,0,0,-8,0,0]]:
            self.check.arm_capture();self.feed(q)
        session=self.check.session
        comparisons=[{'step':step,'encoder_deg':8.,'tag_deg':6.1,'gyro_deg':6.2,
            'gyro_vs_encoder_deg':1.8,'tag_vs_gyro_deg':.1,'gravity_tilt_deg':6.4}
            for _,step in BEND_STEPS[1:]]
        directory=Path(self.temp.name)/'camera_diagnosis';directory.mkdir()
        (directory/f'imu_comparison_{session}.json').write_text(json.dumps({
            'session':session,'method':'synthetic','comparisons':comparisons}))
        restored=CameraCheck(Geometry(),output)
        restored.load_completed(output/session/'report.json')
        self.assertEqual(restored.snapshot(self.now)['steps'],BEND_STEPS)
        self.assertEqual(restored.snapshot(self.now)['independent_imu']['comparisons'],comparisons)
        restored.restart()
        self.assertIsNone(restored.snapshot(self.now)['independent_imu'])

    def test_capture_waits_for_stationary_pause_and_saves_once(self):
        self.check.arm_capture()
        self.feed(following=True)
        self.assertEqual(len(self.check.samples),0)
        self.feed(powered=False)  # a supported, unpowered arm is measurable too
        self.assertEqual(len(self.check.samples),1)
        self.feed()
        self.assertEqual(len(self.check.samples),1)
        self.assertFalse(self.check.pending)
        self.feed()
        self.assertEqual(len(self.check.samples),1)
        report=json.loads((Path(self.temp.name)/self.check.session/'report.json').read_text())
        self.assertFalse(report['calibration_validated'])
        self.assertFalse(report['motion_commanded_by_capture'])
        self.assertTrue((Path(self.temp.name)/self.check.session/'0.jpg').is_file())

    def test_four_views_compare_coupled_motion_without_mount_calibration(self):
        self.check.arm_capture();self.feed()
        for q in [[0,0,0,1,2,10],[0,0,0,2,12,11],[0,0,0,12,13,12]]:
            self.check.arm_capture();self.feed(q)
        state=self.check.snapshot(self.now)
        self.assertTrue(state['complete'])
        self.assertEqual(len(state['results']),3)
        for r in state['results']:
            self.assertAlmostEqual(r['difference_deg'],0.,places=6)
            self.assertFalse(r['calibration_validated'])
        with self.assertRaises(ValueError):self.check.arm_capture()

    def test_missing_tag_and_stale_readings_cannot_capture(self):
        self.check.arm_capture();self.feed(valid=False)
        self.assertFalse(self.check.samples)
        self.assertFalse(self.check.snapshot(self.now+2)['ready'])
        self.feed();self.assertEqual(len(self.check.samples),1)

    def test_completed_report_can_be_reviewed_after_restart_without_reusing_frames(self):
        self.check.arm_capture();self.feed()
        for q in [[0,0,0,0,0,8],[0,0,0,0,8,8],[0,0,0,-8,8,8]]:
            self.check.arm_capture();self.feed(q)
        path=Path(self.temp.name)/self.check.session/'report.json'
        restored=CameraCheck(Geometry(),Path(self.temp.name))
        restored.load_completed(path)
        self.assertTrue(restored.snapshot(self.now)['complete'])
        self.assertFalse(restored.frames)
        self.assertFalse(restored.joints)
        self.assertEqual(len(restored.snapshot(self.now)['results']),3)
        restored.restart()
        self.assertFalse(restored.snapshot(self.now)['complete'])

    def test_incomplete_report_cannot_be_loaded_as_completed(self):
        self.check.arm_capture();self.feed()
        path=Path(self.temp.name)/self.check.session/'report.json'
        restored=CameraCheck(Geometry(),Path(self.temp.name))
        with self.assertRaises(ValueError):restored.load_completed(path)

    def test_motion_too_small_or_too_large_waits_for_operator(self):
        self.check.arm_capture();self.feed()
        self.check.arm_capture();self.feed([0,0,0,0,0,2])
        self.assertEqual(len(self.check.samples),1)
        self.feed([0,0,0,0,0,25]);self.assertEqual(len(self.check.samples),1)
        self.feed([0,0,0,0,0,10]);self.assertEqual(len(self.check.samples),2)

    def test_unstable_joints_or_pose_do_not_capture(self):
        self.check.arm_capture()
        for i in range(50):
            self.feed([0,0,0,0,0,float(i%2)],seconds=.1)
        self.assertFalse(self.check.samples)
        self.check.restart();self.check.arm_capture()
        for i in range(50):self.feed(seconds=.1,camera_extra=.06*(i%2))
        self.assertFalse(self.check.samples)

    def test_fault_and_old_frame_do_not_capture(self):
        self.check.arm_capture();self.feed(fault='motor fault')
        self.assertFalse(self.check.samples)
        old=self.check.frames[-1]['time']
        self.check.add_frame({'time':old-1,'valid':True},self.image,now=self.now)
        self.assertEqual(self.check.frames[-1]['time'],old)

    def test_restart_preserves_saved_session(self):
        self.check.arm_capture();self.feed()
        directory=Path(self.temp.name)/self.check.session
        self.check.restart()
        self.assertTrue((directory/'report.json').exists())
        self.assertFalse(self.check.samples)
        self.assertFalse(self.check.pending)

    def test_delayed_video_uses_image_time_window_and_waits_past_motion(self):
        self.check.arm_capture()
        self.feed(following=True,delay=.85,seconds=2.)
        self.assertFalse(self.check.samples)
        self.feed(delay=.85,seconds=1.)
        self.assertFalse(self.check.samples)  # images still include movement interval
        self.feed(delay=.85,seconds=2.)
        self.assertEqual(len(self.check.samples),1)
        self.assertGreater(self.check.snapshot(self.now)['diagnostics']['latest_frame_age_s'],.8)

    def test_frozen_or_excessively_old_images_remain_rejected(self):
        self.check.arm_capture()
        self.feed(delay=2.,seconds=4.)
        self.assertFalse(self.check.samples)

    def test_bad_camera_rotation_is_reported_not_accepted_as_calibration(self):
        a={'T_base_wrist':np.eye(4).tolist(),'T_camera_tag':np.eye(4).tolist()}
        A=np.eye(4);A[:3,:3]=cv2.Rodrigues(np.array([.2,0.,0.]))[0]
        b={'T_base_wrist':A.tolist(),'T_camera_tag':np.eye(4).tolist()}
        result=compare_views(a,b)
        self.assertGreater(result['difference_deg'],10)
        self.assertEqual(result['assessment'],'needs investigation')


if __name__=='__main__':unittest.main()
