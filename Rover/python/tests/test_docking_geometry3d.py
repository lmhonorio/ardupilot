import math,pathlib,sys,unittest
import numpy as np
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'docking_lidar'))
from geometria import estimar,OFFSET
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'docking'))
from controle_saida import validar_inicio,alinhamento


def surfaces(center=(4.,.1,.35),yaw=.12,walls=(0,1,2,3),n=220):
    rng=np.random.RandomState(5);pts=[]
    for wall in walls:
        xyz=rng.uniform(-.49,.49,(n,3))
        axis=0 if wall==0 else (1 if wall<3 else 2)
        xyz[:,axis]=-.5 if wall==2 else .5
        c,s=math.cos(yaw),math.sin(yaw);r=np.array([[c,-s,0],[s,c,0],[0,0,1]])
        pts.append(xyz@r.T+np.asarray(center))
    return np.concatenate(pts)-OFFSET

class Geometry3D(unittest.TestCase):
    def test_full_box_height_and_near_entry_pose(self):
        p=estimar(surfaces((.60,.03,.37),-.18),[.65,.02,-.14],corrected=True)
        self.assertAlmostEqual(p['forward'],.6,delta=.02)
        self.assertAlmostEqual(p['up'],.37,delta=.02)
        self.assertAlmostEqual(p['normal_yaw'],-.18,delta=.02)
        self.assertTrue(p['height_observed'])
    def test_far_container_with_points_outside_old_height_slice(self):
        xyz=surfaces();height=xyz[:,2]+OFFSET[2];xyz=xyz[(height<.18)|(height>.78)]
        p=estimar(xyz,[4.05,.08,.1],corrected=True)
        self.assertAlmostEqual(p['forward'],4.,delta=.03)
        self.assertEqual(p['source'],'container_surfaces_3d')
    def test_floor_or_single_wall_is_not_a_station(self):
        for walls in [(0,),(3,)]:
            with self.assertRaises(ValueError):estimar(surfaces(walls=walls),[4,.1,.12],corrected=True)
    def test_sparse_individual_scans_become_observable_together(self):
        xyz=surfaces(n=60);batches=np.array_split(xyz,12)
        for b in batches:
            with self.assertRaises(ValueError):estimar(b,[4,.1,.12],corrected=True)
        p=estimar(np.concatenate(batches),[4,.1,.12],corrected=True)
        self.assertAlmostEqual(p['right'],-.1,delta=.02)
    def test_reported_pose_is_approach_not_valid_exit(self):
        p={'forward':.592174205,'right':.140047613,'normal_yaw':-.184263426}
        self.assertAlmostEqual(alinhamento(p)[0],.6078,delta=.002)
        with self.assertRaises(ValueError):validar_inicio(p)

if __name__=='__main__':unittest.main()
