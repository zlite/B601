from copy import deepcopy
import tempfile
import time
from pathlib import Path
import unittest
from unittest.mock import Mock, patch

import numpy as np

from printer_teach import TeachWorkbench


class TargetPreviewTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory(); self.addCleanup(self.tmp.cleanup)
        self.w = TeachWorkbench(Path('calibration/leader_pairing_20261002T202428971124Z.json'),
                                Path(self.tmp.name)/'recording')
        now = time.monotonic()
        self.w.rail.state = 'Idle';self.w.rail.connected = True
        self.w.rail.status_time = now;self.w.rail.x_mm = 0.;self.w.rail.commands = 0
        self.obs = dict(valid=True, session='camera', observation_time=now,
                        geometry_fingerprint=self.w.geometry.fingerprint,
                        rail_x_mm=0., rail_revision=0, arm_raw_rad=[0.]*6,
                        T_base_bed=np.eye(4).tolist(), T_base_plate=np.eye(4).tolist(),
                        T_bed_plate=np.eye(4).tolist())

    def feed(self, observation):
        row = dict(valid=True, observation_time=observation['observation_time'],
                   session='camera', registration=observation)
        self.w.publish('printer_plate', **row)
        self.w.target_preview.observe(row)

    def test_cached_frame_cannot_supply_three_independent_observations(self):
        for _ in range(3):self.feed(self.obs)
        self.assertEqual(len(self.w.target_preview.history),1)
        with self.assertRaisesRegex(ValueError,'three stable'): self.w.target_preview.current()
        for dt in (-.2,-.1,0.):
            self.feed({**self.obs,'observation_time':self.obs['observation_time']+dt})
        self.assertEqual(self.w.target_preview.current()['session'],'camera')
        self.w.rail.commands += 1
        self.assertFalse(self.w.target_preview.snapshot()['preview_ready'])

    def test_independent_plate_movement_resets_stability_and_missing_grid_blocks(self):
        for dt in (-.2,-.1,0.):
            self.feed({**self.obs,'observation_time':self.obs['observation_time']+dt})
        moved = deepcopy(self.obs);moved['observation_time'] += .01;moved['T_base_plate'][1][3] = .02
        self.feed(moved)
        self.assertEqual(len(self.w.target_preview.history),1)
        self.w.target_preview.observe({'valid':False})
        self.assertFalse(self.w.target_preview.history)

    def test_capture_requests_do_not_call_motor_or_rail_actions(self):
        self.w.rail.action = Mock()
        with patch.object(self.w.demo,'action') as demo:
            self.w.action({'action':'target_capture'})
        self.assertTrue((self.w.printer_capture_root/'request.json').exists())
        demo.assert_not_called();self.w.rail.action.assert_not_called()
        self.assertFalse(self.w.target_preview.snapshot()['motion_ready'])

    def test_stale_snapshot_removes_base_registration(self):
        self.feed({**self.obs,'observation_time':0.})
        source = self.w.snapshot()['sources']['printer_plate']
        self.assertFalse(source['valid']);self.assertNotIn('registration',source)

    def test_preview_is_discarded_when_plate_moves_or_camera_expires(self):
        target = self.w.target_preview
        for dt in (-.2,-.1,0.):
            self.feed({**self.obs,'observation_time':self.obs['observation_time']+dt})
        target.planned_observation = deepcopy(self.obs)
        target.result = {'path':'old'}
        self.assertIsNotNone(target.snapshot()['result'])
        moved = deepcopy(self.obs);moved['T_base_plate'][0][3] += .01
        moved['observation_time'] += .01
        self.feed(moved)
        self.assertIsNone(target.snapshot()['result'])

    def test_unregistered_scene_blocks_preview_without_enabling_any_demo(self):
        self.w.publish('follower',angles=[0.]*6)
        for dt in (-.2,-.1,0.):
            self.feed({**self.obs,'observation_time':self.obs['observation_time']+dt})
        self.w.action({'action':'target_preview'})
        self.w.target_preview.thread.join(timeout=3)
        status = self.w.target_preview.snapshot()
        self.assertIn('Register the printer',status['message'])
        self.assertFalse(status['busy']);self.assertFalse(status['motion_ready'])
        self.assertIsNone(status['result']);self.assertFalse(self.w.demo.busy())


if __name__ == '__main__': unittest.main()
