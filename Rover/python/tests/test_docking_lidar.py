import math
from pathlib import Path
import sys
import unittest
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking_lidar'))
from geometria import estimar, OFFSET
from referencia import prior_estacao,recuo_lio


def nuvem(visible=(0,1,2),center=(1.5,.2),yaw=.15):
    rng=np.random.RandomState(17);pts=[]
    for wall in visible:
        u=rng.uniform(-.49,.49,400);z=rng.uniform(.20,.75,400)
        x=np.full_like(u,.5) if wall==0 else u
        y=u if wall==0 else np.full_like(u,.5 if wall==1 else -.5)
        c,s=math.cos(yaw),math.sin(yaw)
        pts.extend(np.column_stack((center[0]+c*x-s*y,center[1]+s*x+c*y,z)))
    real=np.asarray(pts)-OFFSET
    d=np.linalg.norm(real,axis=1)
    return real*np.maximum(0,1-.1/d)[:,None]

class LidarDock(unittest.TestCase):
    def test_ajuste_em_u_com_vies_simulado(self):
        pose=estimar(nuvem(),[1.55,.16,.12])
        self.assertAlmostEqual(pose['forward'],1.5,delta=.025)
        self.assertAlmostEqual(pose['right'],-.2,delta=.025)
        self.assertAlmostEqual(pose['normal_yaw'],.15,delta=.03)
        self.assertLess(pose['rms'],.02)
    def test_parede_unica_nao_e_pose_completa(self):
        with self.assertRaises(ValueError):estimar(nuvem((0,)),[1.5,.2,.15])
    def test_referencia_gps_frente_leste(self):
        s={'latitude':-3.12315,'longitude':-41.76445,'azimute_entrada_graus':90}
        f,l,y=prior_estacao(-3.12315,-41.76446,90,s)
        self.assertGreater(f,1);self.assertAlmostEqual(l,0,delta=.01);self.assertAlmostEqual(y,0,delta=.001)
        f,l,y=prior_estacao(-3.12316,-41.76445,90,s)
        self.assertGreater(l,1);self.assertAlmostEqual(f,0,delta=.01)
    def test_recuo_e_correcao_de_lateral_e_yaw(self):
        back,side,err,angular=recuo_lio([0,0,0],[-2,.1,0],0,0)
        self.assertAlmostEqual(back,2);self.assertAlmostEqual(side,.1)
        self.assertLess(angular,0)
        _,_,_,angular=recuo_lio([0,0,0],[-2,0,0],0,.1)
        self.assertGreater(angular,0)

if __name__=='__main__': unittest.main()
