"""Desarmamento so e solicitado apos HOLD e parada confirmada."""
import pathlib
import sys
import types
import unittest
from unittest.mock import patch
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'docking'))
import entrar_container

class LinkFake:
    def __init__(self,speed):
        self.speed=speed; self.latest={}; self.last_seen={}; self.actions=[]
        self.conn=types.SimpleNamespace(mode_mapping=lambda:{'HOLD':4})
    def mode(self,name): self.actions.append(name)
    def recv(self,timeout):
        now=entrar_container.time.monotonic()
        self.latest={'VFR_HUD':types.SimpleNamespace(groundspeed=self.speed),
                     'ATTITUDE':types.SimpleNamespace(yawspeed=0),
                     'HEARTBEAT':types.SimpleNamespace(custom_mode=4,base_mode=128)}
        self.last_seen={k:now for k in self.latest}
    def disarm(self): self.actions.append('DISARM')

class Desarmar(unittest.TestCase):
    def run_case(self, speed):
        clock=iter(i*.25 for i in range(2000)); link=LinkFake(speed)
        with patch.object(entrar_container.time,'monotonic',side_effect=lambda:next(clock)):
            if speed:
                with self.assertRaises(RuntimeError): entrar_container.concluir_desarmando(link)
            else: entrar_container.concluir_desarmando(link)
        return link.actions
    def test_somente_desarma_apos_parar(self):
        self.assertEqual(self.run_case(0),['HOLD','DISARM'])
        self.assertEqual(self.run_case(.2),['HOLD'])

    def test_nao_desarma_se_pose_final_for_invalida(self):
        clock=iter(i*.25 for i in range(2000)); link=LinkFake(0)
        with patch.object(entrar_container.time,'monotonic',side_effect=lambda:next(clock)):
            with self.assertRaises(RuntimeError):
                entrar_container.concluir_desarmando(link,lambda:{'forward':.1,'right':0,'normal_yaw':0})
        self.assertEqual(link.actions,['HOLD'])

    def test_pose_final_perto_do_centro_desarma(self):
        clock=iter(i*.25 for i in range(2000)); link=LinkFake(0)
        with patch.object(entrar_container.time,'monotonic',side_effect=lambda:next(clock)):
            entrar_container.concluir_desarmando(link,lambda:{'forward':.04,'right':.03,'normal_yaw':.02})
        self.assertEqual(link.actions,['HOLD','DISARM'])

if __name__=='__main__': unittest.main()
