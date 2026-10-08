from copy import deepcopy
import json
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from arm_geometry import Geometry
from printer_target_plan import (bracketed_pose, register_observation,
    require_fresh, resolve_scene, reference_change, preview)


class TargetPlanTests(unittest.TestCase):
    def setUp(self):
        self.g = Geometry()
        self.q = np.array([1.2599951, .5850691, 1.6840857, .2242678, .0569354, -.0684954])
        self.X = np.array(json.loads(Path('calibration/stereo_sweep_handeye_20261004.json').read_text())['T_wrist_camera'])
        self.P = np.array([[-.3088438, -.95111258, .00062819, .05941167],
            [-.55681634, .18134392, .81059851, .03844215],
            [-.77108430, .24999852, -.58560192, .23855605], [0, 0, 0, 1]])
        self.rail = dict(connected=True, state='Idle', error=None, x_mm=0.,
                         commands=5, status_time=10.1, last_motion_time=9.)
        self.history = [(t, np.degrees(self.q).tolist()) for t in [9.85, 9.95, 10.05, 10.15]]
        self.plate = dict(valid=True, observation_time=10., method='complete_stereo_well_grid',
            well_count=96, T_camera_b_plate=self.P.tolist(), bed_reference=dict(
                valid=True, observation_time=10., tag=dict(id=18, family='36h11', size_m=.018),
                T_camera_b_tag=np.eye(4).tolist()))
        self.obs = register_observation(self.plate, self.history, self.g, self.X, self.rail, 10.15, 's')
        names = [('bed','bed'),('holder','bed'),('plate','plate'),('frame','base'),
                 ('gantry','base'),('syringe','base'),('bench','base')]
        self.scene = dict(registered=True, revision='test', rail_x_mm=0.,
            geometry_fingerprint=self.g.fingerprint, uncertainty_m=.001,
            obstacles=[dict(name=n, frame=f, size_m=[.01]*3,
                            T_frame_obstacle=np.eye(4).tolist()) for n,f in names])

    def test_registration_requires_bracketed_stationary_capture_and_stopped_rail(self):
        for history in (self.history[:2], self.history[2:], self.history[::-1],
                        [*self.history[:-1], (10.15, [0.]*6)]):
            with self.assertRaises(ValueError): bracketed_pose(history, 10.)
        for changes in ({'state':'Jog'}, {'last_motion_time':10.01}, {'x_mm':float('nan')}):
            with self.assertRaises(ValueError): register_observation(
                self.plate, self.history, self.g, self.X, {**self.rail, **changes}, 10.15, 's')
        with self.assertRaises(ValueError): register_observation(
            {**self.plate, 'method':'bed_offset'}, self.history, self.g, self.X, self.rail, 10.15, 's')

    def test_mismatched_stereo_pair_wrong_tag_and_stale_data_rejected(self):
        for bed in ({**self.plate['bed_reference'], 'observation_time':9.9},
                    {**self.plate['bed_reference'], 'valid':False},
                    {**self.plate['bed_reference'], 'tag':dict(id=0)}):
            with self.assertRaises(ValueError): register_observation(
                {**self.plate, 'bed_reference':bed}, self.history, self.g, self.X, self.rail, 10.15, 's')
        for now, session in ((10.6,'s'), (9.9,'s'), (float('nan'),'s'), (10.15,'new')):
            with self.assertRaises(ValueError): require_fresh(self.obs, now, session, self.rail, self.g.fingerprint)
        for changes in ({'commands':6}, {'x_mm':.2}, {'state':'Jog'}, {'status_time':9.}):
            with self.assertRaises(ValueError): require_fresh(self.obs, 10.15, 's', {**self.rail, **changes}, self.g.fingerprint)

    def test_bed_and_plate_move_independently_fixed_gantry_does_not(self):
        before = resolve_scene(self.scene, self.obs)
        moved = deepcopy(self.obs)
        moved['T_base_bed'][1][3] += .03
        moved['T_base_plate'][0][3] += .01
        after = resolve_scene(self.scene, moved)
        for a,b in zip(before,after):
            delta = np.array(b['T_base_obstacle'])[:3,3]-np.array(a['T_base_obstacle'])[:3,3]
            expected = [0,.03,0] if a['name'] in ('bed','holder') else ([.01,0,0] if a['name']=='plate' else [0,0,0])
            np.testing.assert_allclose(delta, expected, atol=1e-10)
        self.assertTrue(reference_change(self.obs,moved)['requires_replan'])
        flipped = deepcopy(self.obs)
        T = np.array(flipped['T_base_plate']); T[:3,:3] = T[:3,:3]@np.diag([-1,-1,1])
        flipped['T_base_plate'] = T.tolist()
        self.assertFalse(reference_change(self.obs,flipped)['requires_replan'])

    def test_empty_unregistered_wrong_frame_or_wrong_rail_scene_rejected(self):
        for change in ({'registered':False}, {'obstacles':[]}, {'rail_x_mm':1.}):
            with self.assertRaises(ValueError): resolve_scene({**self.scene, **change},self.obs)
        bad = deepcopy(self.scene);bad['obstacles'][4]['frame']='bed'
        with self.assertRaises(ValueError): resolve_scene(bad,self.obs)

    def test_planner_resolves_changed_target_and_never_authorizes_motion(self):
        tips = [[-.00764567,.00084545,.1555204],[.08689201,.0008138,.15233901]]
        tool = dict(registered=True, revision='test', camera_and_cables_included=True,
                    parts_camera_b=[{}], tips_camera_b_m=tips, downstream_reach_bounds_m=[1.]*6)
        with patch('printer_target_plan.stock_path_clearance', return_value={'sampled_stock_arm_clear':True}), \
             patch('printer_target_plan.camera_parts_path_clearance', return_value={'sampled_parts_clear':True}):
            a = preview(self.g,self.q,self.X,self.obs,self.scene,tool,now=10.15,session='s',rail=self.rail)
            moved = deepcopy(self.obs);moved['T_base_plate'][1][3] += .005
            b = preview(self.g,self.q,self.X,moved,self.scene,tool,now=10.15,session='s',rail=self.rail)
        self.assertNotEqual(a['waypoints_raw_deg'][-1],b['waypoints_raw_deg'][-1])
        for candidate,obs in ((a,self.obs),(b,moved)):
            final = self.g.transform(np.radians(candidate['waypoints_raw_deg'][-1]))@self.X
            point = np.linalg.solve(np.array(obs['T_base_plate']), final@np.r_[np.mean(tips,axis=0),1])[:3]
            np.testing.assert_allclose(point,[.025,0,.065],atol=.0002)
            self.assertFalse(candidate['motion_ready'])


if __name__ == '__main__': unittest.main()
