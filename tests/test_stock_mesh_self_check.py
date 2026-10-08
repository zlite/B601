import tempfile
import unittest
from pathlib import Path

import numpy as np
try:
    import trimesh
    from scripts.stock_mesh_self_check import check
    CAD_AVAILABLE = True
except ImportError:
    CAD_AVAILABLE = False


@unittest.skipUnless(CAD_AVAILABLE, 'Requires the isolated .venv-cad environment')
class StockMeshSelfCheckTests(unittest.TestCase):
    def test_aabb_pruning_preserves_analytic_cube_results(self):
        # Exact surface separation of equal, axis-aligned cubes along X is
        # max(abs(center difference)-side, 0). Test every nonadjacent pair.
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'cube.stl'
            trimesh.creation.box(extents=[.01]*3).export(path)
            names = ['base_link', *[f'link{i}' for i in range(1, 7)]]
            centers = np.random.default_rng(91).uniform(0, .2, (40, 7))
            centers[0] = np.arange(7)*.1
            centers[1, 4] = centers[1, 2]+.012
            centers[2, 4] = centers[2, 2]+.005
            rows = []
            for sample in centers:
                transforms = {}
                for name, x in zip(names, sample):
                    t = np.eye(4)
                    t[0, 3] = x
                    transforms[name] = t.tolist()
                rows.append({'transforms': transforms})
            data = {'meshes': [{'name': n, 'mesh': str(path), 'scale': [1, 1, 1]}
                               for n in names], 'rows': rows}
            result = check(data)
            for sample, row in zip(centers, result['rows']):
                expected = {(names[i], names[j]): max(abs(sample[i]-sample[j])-.01, 0)
                            for i in range(7) for j in range(i+2, 7)
                            if abs(sample[i]-sample[j])-.01 < .005}
                actual = {(p['a'], p['b']): p for p in row['below_margin']}
                self.assertEqual(set(actual), set(expected))
                for pair, distance in expected.items():
                    self.assertAlmostEqual(actual[pair]['distance_m'], distance, places=7)
                    self.assertEqual(actual[pair]['intersects'], distance == 0)
            self.assertFalse(result['motion_ready'])

    def test_rejects_incomplete_link_set(self):
        with self.assertRaises(ValueError):
            check({'meshes': [], 'rows': [{}]})

    def test_fixed_gripper_base_excludes_own_body_and_adjacent_joint_only(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'cube.stl'
            trimesh.creation.box(extents=[.01]*3).export(path)
            names = ['base_link', *[f'link{i}' for i in range(1, 7)], 'end_link']
            transforms = {}
            for i, name in enumerate(names):
                t = np.eye(4)
                t[0, 3] = .4 if i >= 4 else i*.1
                transforms[name] = t.tolist()
            result = check({'meshes': [{'name': n, 'mesh': str(path), 'scale': [1]*3}
                                       for n in names], 'rows': [{'transforms': transforms}]})
            pairs = {(r['a'], r['b']) for r in result['rows'][0]['below_margin']}
            self.assertIn(('link4', 'end_link'), pairs)
            self.assertNotIn(('link5', 'end_link'), pairs)
            self.assertNotIn(('link6', 'end_link'), pairs)

    def test_rejects_reduced_margin(self):
        with self.assertRaises(ValueError):
            check({}, margin_m=.001)


if __name__ == '__main__':
    unittest.main()
