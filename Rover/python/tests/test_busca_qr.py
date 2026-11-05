import pathlib
import sys
import types
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'docking'))
from busca_qr import yaw_rate_para_estacao

class BuscaQR(unittest.TestCase):
    def setUp(self):
        self.pos=types.SimpleNamespace(lat=0,lon=10000000,hdg=0)
        self.gps=types.SimpleNamespace(fix_type=3)
    def rate(self,latitude=0,longitude=1.0001,now=100,pos_seen=99,gps_seen=99):
        return yaw_rate_para_estacao(self.pos,self.gps,now,pos_seen,gps_seen,latitude,longitude)
    def test_gira_para_direita_e_esquerda(self):
        self.assertGreater(self.rate(),0)
        self.assertLess(self.rate(longitude=.9999),0)
        self.assertLessEqual(abs(self.rate()),.15)
    def test_ja_aponta_e_wrap_angular(self):
        self.pos.hdg=9000
        self.assertEqual(self.rate(),0)
        self.pos.hdg=35000
        self.assertGreater(self.rate(latitude=.0001,longitude=1),0)
    def test_dados_invalidos_e_alvo_proximo(self):
        self.assertIsNone(self.rate(now=104))
        self.pos.hdg=65535
        self.assertIsNone(self.rate())
        self.pos.hdg=0
        self.gps.fix_type=2
        self.assertIsNone(self.rate())
        self.gps.fix_type=3
        self.assertIsNone(self.rate(latitude=0,longitude=1.000001))

if __name__=='__main__': unittest.main()
