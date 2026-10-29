from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace as Obj
import unittest

import numpy as np
import yaml

PYTHON_DIR = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PYTHON_DIR))
sys.path.insert(0, str(PYTHON_DIR / 'slam'))
from common.ros_cloud import project_scan, cloud_xyz
from map_io import save_map


class ProjectionTests(unittest.TestCase):
    def test_only_nearest_obstacle_return_is_used_and_unknown_bins_stay_unknown(self):
        xyz = [[3, 0, .1], [2, 0, .2], [0, 4, .1], [1, 0, -.44],
               [1, 0, 2], [0, 0, 0], [float('nan'), 2, 0]]
        scan = project_scan(xyz)
        self.assertEqual(scan[180], 2)
        self.assertEqual(scan[270], 4)
        self.assertEqual(np.isfinite(scan).sum(), 2)
        self.assertTrue(np.isinf(scan[0]))

    def test_angle_wrap_and_invalid_bounds(self):
        scan = project_scan([[-2, 0, 0], [-3, -.001, 0]])
        self.assertEqual(scan[0], 2)
        with self.assertRaises(ValueError):
            project_scan([], min_height=1, max_height=0)

    def test_legacy_cloud_drops_invalid_zero_returns(self):
        msg = Obj(_type='sensor_msgs/PointCloud',
                  points=[Obj(x=0, y=0, z=0), Obj(x=2, y=1, z=.1), Obj(x=float('nan'), y=0, z=0)])
        xyz = cloud_xyz(msg)
        self.assertEqual(xyz.shape, (1, 3))
        np.testing.assert_allclose(xyz[0], [2, 1, .1])

    def test_map_export_preserves_unknown_and_flips_rows_for_pgm(self):
        grid = Obj(data=[0, 100, -1, 0],
                   info=Obj(height=2, width=2, resolution=.05,
                            origin=Obj(position=Obj(x=-1, y=-2), orientation=Obj(x=0, y=0, z=0, w=1))))
        with tempfile.TemporaryDirectory() as directory:
            pgm, metadata = save_map(grid, directory)
            self.assertEqual(pgm.read_bytes().split(b'255\n', 1)[1], bytes([205, 254, 254, 0]))
            data = yaml.safe_load(metadata.read_text())
            self.assertEqual(data['origin'], [-1., -2., 0.])
            self.assertEqual(data['resolution'], .05)
            self.assertEqual(data['image'], 'map.pgm')


if __name__ == '__main__':
    unittest.main()
