import math
import unittest
import json
import tempfile
from unittest.mock import Mock,patch
from types import SimpleNamespace
from pathlib import Path
import cv2
import numpy as np

from arm_geometry import Geometry
from calibrate_joint_motion import plan,Runner
from joint_motion_calibration import OFFSETS,TRAVEL_SIGNS,fit_joint,predict,load_profile,DENSE_OFFSETS,fit_dense_joint,predict_rotation


class JointMotionTests(unittest.TestCase):
    def test_plan_moves_one_joint_within_limits_and_returns(self):
        g=Geometry();start=np.degrees(g.profile['reference_raw_rad']).tolist()
        poses,limits=plan(g,start,list(range(6)))
        for j,route in poses.items():
            self.assertEqual(route[-1],start)
            for q,offset in zip(route,OFFSETS):
                self.assertEqual(q[:j]+q[j+1:],start[:j]+start[j+1:])
                self.assertAlmostEqual(abs(q[j]-start[j]),offset)
                self.assertTrue(limits[j][0]<=q[j]<=limits[j][1])
        start[1]-=10
        with self.assertRaises(ValueError):plan(g,start,[0])

    def synthetic(self,joint=2,validation_error=0):
        rows=[]
        for i,x in enumerate(OFFSETS):
            q=np.zeros(6);q[joint]=TRAVEL_SIGNS[joint]*x
            y=.8*x+(validation_error if i==5 else 0)
            C=np.eye(4);C[:3,:3]=cv2.Rodrigues(np.array([0.,-math.radians(y),0.]))[0]
            rows.append({'raw_joint_deg':q.tolist(),'T_camera_tag':C.tolist()})
        return rows

    def test_camera_scale_fits_without_using_validation_points(self):
        model=fit_joint(2,self.synthetic())
        self.assertAlmostEqual(model['endpoint_scale'],.8)
        self.assertTrue(model['validation_passed'])
        self.assertAlmostEqual(predict(model,-6.,'outward'),4.8)
        with self.assertRaises(ValueError):predict(model,-9.,'outward')
        perturbed=fit_joint(2,self.synthetic(validation_error=2))
        self.assertEqual(model['curves'],perturbed['curves'])
        self.assertFalse(perturbed['validation_passed'])

    def test_other_joint_drift_rejects_calibration(self):
        samples=self.synthetic();samples[5]['raw_joint_deg'][0]=1.
        model=fit_joint(2,samples)
        self.assertFalse(model['validation_passed'])

    def test_stall_and_unselected_motion_prevent_command(self):
        for elapsed,moved in [(1.,0.),(.05,1.1)]:
            arm=Mock();arm.read.return_value=[0.,moved,0.,0.,0.,0.]
            w=SimpleNamespace(camera_check=Mock())
            with patch('calibrate_joint_motion.time.monotonic',return_value=100.):
                runner=Runner(w,arm,[0.]*6,{i:(-10,10) for i in range(6)},Path('/tmp'))
            runner.joint=0
            with patch('calibrate_joint_motion.time.monotonic',return_value=100.+elapsed):
                with self.assertRaises(RuntimeError):runner.tick()
            arm.command_group.assert_not_called()

    def test_dense_full_rotation_validates_unseen_positions(self):
        samples=[]
        for x in DENSE_OFFSETS:
            q=[0.]*6;q[4]=-x
            # A curved camera rotation response cannot be represented by one
            # fixed axis; all three components must be validated.
            v=np.radians([.03*x*x,.8*x,.02*x*x])
            T=np.eye(4);T[:3,:3]=cv2.Rodrigues(-v)[0]
            samples.append({'raw_joint_deg':q,'T_camera_tag':T.tolist()})
        model=fit_dense_joint(4,samples)
        self.assertTrue(model['validation_passed'])
        self.assertLess(model['max_validation_error_deg'],.05)
        R=predict_rotation(model,-3.,'outward')
        self.assertAlmostEqual(float(np.linalg.det(R)),1.)
        with self.assertRaises(ValueError):predict_rotation(model,-9.,'outward')
        T=np.array(samples[11]['T_camera_tag'])
        T[:3,:3]=T[:3,:3]@cv2.Rodrigues(np.radians([3.,0,0]))[0]
        samples[11]['T_camera_tag']=T.tolist()
        bad=fit_dense_joint(4,samples)
        self.assertFalse(bad['validation_passed'])
        self.assertEqual(model['curves'],bad['curves'])

    def test_report_rejects_mismatched_geometry_and_nonfinite_values(self):
        model=fit_joint(2,self.synthetic())
        data={'geometry_fingerprint':'same','camera_is_reference':True,'global_mapping_changed':False,'joints':[model]}
        with tempfile.TemporaryDirectory() as folder:
            p=Path(folder)/'report.json';p.write_text(json.dumps(data))
            self.assertTrue(load_profile(p,'same')['joints'][0]['validation_passed'])
            with self.assertRaises(ValueError):load_profile(p,'different')
            data['joints'][0]['endpoint_scale']=float('nan');p.write_text(json.dumps(data))
            with self.assertRaises(ValueError):load_profile(p,'same')


if __name__=='__main__':unittest.main()
