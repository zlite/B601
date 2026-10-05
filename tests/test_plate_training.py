import json,tempfile,unittest
from pathlib import Path
from plate_training import holder_successes,require_ten_holder_cycles,require_current_workspace

class TrainingEvidenceTests(unittest.TestCase):
    def test_relocation_marker_blocks_old_workspace_routes(self):
        with tempfile.TemporaryDirectory() as directory:
            marker=Path(directory)/'workspace.json'
            require_current_workspace(marker)
            marker.write_text('{"reason":"relocation"}')
            with self.assertRaisesRegex(ValueError,'relocation'):require_current_workspace(marker)

    def test_gate_rejects_incomplete_or_assisted_cycles(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            for i in range(10):
                folder=root/f'20261005T21{i:010d}Z';folder.mkdir()
                (folder/'report.json').write_text(json.dumps({'returned_to_rest':True,'motors_disabled_verified':True,'approach':'left'}))
                (folder/'pickup_report.json').write_text(json.dumps({'lift_following_verified':True,'released_verified':True,'jaw_open_verified':True,'model_lift_height_m':.02}))
            self.assertEqual(len(require_ten_holder_cycles(root)),10)
            (folder/'shutdown_verification.json').write_text(json.dumps({'physically_supported_shutdown':True}))
            self.assertEqual(len(holder_successes(root)),9)
            with self.assertRaisesRegex(ValueError,'have 9'):require_ten_holder_cycles(root)
