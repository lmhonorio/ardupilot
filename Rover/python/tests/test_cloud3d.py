import pathlib,sys,unittest,math
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]))
from common.cloud3d import LocalCloud,corrected_mid360,rotation_xyzw,voxel_points

class Cloud3DTests(unittest.TestCase):
    def test_gazebo_range_offset_and_invalid_returns(self):
        x=corrected_mid360([[.9,0,0],[0,1.9,0],[0,0,0],[float('nan'),0,0]])
        np.testing.assert_allclose(x,[[1,0,0],[0,2,0]])
    def test_accumulate_in_world_then_transform_current_body(self):
        buf=LocalCloud(1.,.001)
        world=np.array([[3.,.2,.4],[4.,.7,.9]])
        buf.add(1.,world[:1],np.zeros(3),np.eye(3))
        yaw=.1;r=rotation_xyzw([0,0,math.sin(yaw/2),math.cos(yaw/2)])
        p=np.array([.1,.02,0]);result=buf.add(1.1,world[1:],p,r)
        np.testing.assert_allclose(result,(world-p)@r)
        self.assertGreater(np.ptp(result[:,2]),.4)  # Z survives; no scan projection
    def test_old_geometry_expires(self):
        b=LocalCloud(.3,.001)
        b.add(1.,[[1,0,0]],np.zeros(3),np.eye(3))
        x=b.add(1.5,[[2,0,1]],np.zeros(3),np.eye(3))
        np.testing.assert_allclose(x,[[2,0,1]])
    def test_jump_and_clock_reset_invalidate_buffer(self):
        for stamp,p in [(0.9,np.zeros(3)),(1.1,np.array([10,0,0]))]:
            b=LocalCloud();b.add(1.,[[1,0,0]],np.zeros(3),np.eye(3))
            with self.assertRaises(ValueError):b.add(stamp,[[2,0,0]],p,np.eye(3))
            self.assertFalse(b.history)
    def test_xyz_voxels_keep_separate_heights(self):
        x=voxel_points([[0,0,0],[0,0,.5],[.001,0,0]],.04)
        self.assertEqual(len(x),2)

if __name__=='__main__':unittest.main()
