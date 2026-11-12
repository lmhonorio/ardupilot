"""The spawned geometry worker preserves measurements and rejects missing walls."""
from concurrent.futures import ProcessPoolExecutor
import multiprocessing
import unittest
import numpy as np
from test_docking_geometry3d import surfaces
from mission_manager.geometry_worker import fit_and_sweep
from docking_lidar.geometria import pontos_3d,estimar


class GeometryWorker(unittest.TestCase):
    def test_spawned_worker_preserves_pose_sweep_and_rejects_single_wall(self):
        xyz=surfaces((.60,.03,.37),-.18)
        with ProcessPoolExecutor(max_workers=1,mp_context=multiprocessing.get_context('spawn')) as pool:
            pose,points=pool.submit(fit_and_sweep,xyz,[.65,.02,-.14],[1,1,1],.15,1200).result(timeout=15)
            self.assertAlmostEqual(pose['forward'],.60,delta=.02)
            self.assertAlmostEqual(pose['right'],-.03,delta=.02)
            self.assertAlmostEqual(pose['normal_yaw'],-.18,delta=.02)
            self.assertEqual(pose,estimar(xyz,[.65,.02,-.14],corrected=True,max_points=1200))
            np.testing.assert_array_equal(points,pontos_3d(xyz,corrected=True))
            with self.assertRaises(ValueError):
                pool.submit(fit_and_sweep,surfaces(walls=(0,)),[4,.1,.12],[1,1,1],.15,1200).result(timeout=15)


if __name__=='__main__':unittest.main()
