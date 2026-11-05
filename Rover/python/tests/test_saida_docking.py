import math
import pathlib
import sys
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'docking'))
from controle_saida import validar_inicio,recuo,alinhamento,Giro180

class Saida(unittest.TestCase):
    def test_recua_dois_metros_em_modelo_cinematico(self):
        pose={'forward':0.,'right':.05,'normal_yaw':0.}
        initial=validar_inicio(pose)
        for _ in range(1000):
            speed,yaw,distance,done=recuo(pose,initial)
            if done:break
            angle=yaw*.1;f=pose['forward']-speed*.1;r=pose['right']
            pose.update(forward=math.cos(angle)*f+math.sin(angle)*r,
                        right=-math.sin(angle)*f+math.cos(angle)*r,
                        normal_yaw=pose['normal_yaw']+angle)
        self.assertTrue(done)
        self.assertGreaterEqual(distance,1.96)
        self.assertLess(abs(alinhamento(pose)[1]),.05)
        self.assertEqual(speed,0)

    def test_fora_do_dock_e_desalinhamento_sao_rejeitados(self):
        for p in [{'forward':.5,'right':0,'normal_yaw':0},
                  {'forward':0,'right':.4,'normal_yaw':0},
                  {'forward':0,'right':0,'normal_yaw':.5}]:
            with self.assertRaises(ValueError):validar_inicio(p)
        with self.assertRaises(ValueError):recuo({'forward':1,'right':.3,'normal_yaw':0},0)

    def test_giro_de_180_com_cruzamento_de_pi(self):
        yaw=2.5; turn=Giro180(yaw)
        for _ in range(1000):
            rate,done=turn.step(yaw)
            if done:break
            yaw=(yaw+rate*.1+math.pi)%(2*math.pi)-math.pi
        self.assertTrue(done)
        self.assertLess(abs(turn.progresso-math.pi),math.radians(3.1))

    def test_rumo_com_salto_invalido(self):
        with self.assertRaises(ValueError):Giro180(0).step(1.5)

if __name__=='__main__':unittest.main()
