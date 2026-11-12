import unittest
import numpy as np
from common.cloud3d import voxel_points


class Voxels(unittest.TestCase):
    def test_first_point_and_order_match_row_unique_for_signed_and_large_cells(self):
        rng=np.random.default_rng(31)
        base=rng.uniform(-1.e4,1.e4,(10000,3))
        samples=np.concatenate([base,base.copy(),[[0,0,0],[-.001,0,0],[np.nan,0,0],[np.inf,1,0]]])
        for array in (samples,np.asfortranarray(samples),samples[::-1]):
            clean=array[np.isfinite(array).all(axis=1)]
            for size in (.015,.06,.2):
                _,idx=np.unique(np.floor(clean/size).astype(np.int64),axis=0,return_index=True)
                np.testing.assert_array_equal(voxel_points(array,size),clean[np.sort(idx)])
        self.assertEqual(voxel_points(np.empty((0,3)),.015).shape,(0,3))


if __name__=='__main__':unittest.main()
