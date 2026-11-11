"""Validacao da ativacao sem enviar comandos a um veiculo."""
import pathlib
import sys
import types
import unittest
from unittest.mock import patch

sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'docking'))
from aguardar_docking import DOCK_LAT, DOCK_LON, distance_m, trigger_state
import aguardar_docking
from pymavlink import mavutil

class Ativacao(unittest.TestCase):
    def setUp(self):
        self.now=100.0
        self.messages={
            'HEARTBEAT':types.SimpleNamespace(type=mavutil.mavlink.MAV_TYPE_GROUND_ROVER,base_mode=128,custom_mode=15),
            'GLOBAL_POSITION_INT':types.SimpleNamespace(lat=round(DOCK_LAT*1e7),lon=round(DOCK_LON*1e7)),
            'GPS_RAW_INT':types.SimpleNamespace(fix_type=3),
            'SIMSTATE':types.SimpleNamespace(),
        }
        self.seen={key:self.now for key in self.messages}
        self.stations=[dict(nome='teste',latitude=DOCK_LAT,longitude=DOCK_LON,qr='qr',distancia_alvo_parede_m=.5)]

    def ready(self,blocked=False):
        return trigger_state(self.messages,self.seen,self.now,blocked,self.stations)[0]

    def test_distance_and_mode(self):
        self.assertTrue(self.ready())
        self.messages['HEARTBEAT'].custom_mode=10
        self.assertFalse(self.ready())
        self.messages['HEARTBEAT'].custom_mode=15
        self.messages['GLOBAL_POSITION_INT'].lat=round((DOCK_LAT+8.5/111319.49)*1e7)
        self.assertGreater(distance_m(self.messages['GLOBAL_POSITION_INT'].lat/1e7,DOCK_LON),8)
        self.assertFalse(self.ready())

    def test_stale_and_fix_and_armed_and_duplicate(self):
        self.assertFalse(self.ready(blocked=True))
        for key in self.seen:
            self.seen[key]=self.now-4
            self.assertFalse(self.ready(),key)
            self.seen[key]=self.now
        self.messages['GPS_RAW_INT'].fix_type=2
        self.assertFalse(self.ready())
        self.messages['GPS_RAW_INT'].fix_type=3
        self.messages['HEARTBEAT'].base_mode=0
        self.assertFalse(self.ready())

    def test_sem_missao_e_distancia_entre_cinco_e_oito_metros(self):
        self.messages['GLOBAL_POSITION_INT'].lat=round((DOCK_LAT+7/111319.49)*1e7)
        self.assertTrue(self.ready())
        self.assertNotIn('MISSION_CURRENT',self.messages)

    def test_monitor_hands_off_and_reopens_port(self):
        messages=[]
        for kind, obj in self.messages.items():
            obj.get_type=lambda kind=kind:kind
            obj.get_srcSystem=lambda:1
            obj.get_srcComponent=lambda:1
            messages.append(obj)
        class FakeMav:
            def heartbeat_send(self,*args): pass
            def command_long_send(self,*args): pass
            def mission_request_list_send(self,*args): pass
        class FakeConn:
            def __init__(self, values): self.values=iter(values); self.mav=FakeMav(); self.closed=False
            def recv_match(self,**kwargs):
                try: return next(self.values)
                except StopIteration: raise KeyboardInterrupt()
            def close(self): self.closed=True
        first=FakeConn(messages); second=FakeConn([])
        child=types.SimpleNamespace(wait=lambda:0)
        args=types.SimpleNamespace(mavproxy='udpin:127.0.0.1:14699',sysid=1,velocidade=.08,timeout=90,estacoes='fake.txt',namespace='/rover_test/Instance2')
        def spawn(cmd):
            self.assertTrue(first.closed)
            self.assertIn('--apenas-guided',cmd)
            self.assertIn('--uma-vez',cmd)
            self.assertEqual(cmd[cmd.index('--namespace')+1],args.namespace)
            self.assertEqual(cmd[cmd.index('--estacoes')+1],args.estacoes)
            self.assertEqual(cmd[cmd.index('--estacao')+1],'teste')
            self.assertEqual(cmd[cmd.index('--latitude-estacao')+1],str(DOCK_LAT))
            self.assertEqual(cmd[cmd.index('--longitude-estacao')+1],str(DOCK_LON))
            return child
        with patch.object(aguardar_docking.mavutil,'mavlink_connection',side_effect=[first,second]), \
             patch.object(aguardar_docking,'load_stations',return_value=self.stations), \
             patch.object(aguardar_docking.subprocess,'Popen',side_effect=spawn):
            self.assertEqual(aguardar_docking.monitorar(args),0)
        self.assertTrue(second.closed)

if __name__=='__main__': unittest.main()
