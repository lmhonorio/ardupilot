import pathlib
import math
import sys
import types
import unittest
sys.path.insert(0,str(pathlib.Path(__file__).resolve().parents[1]/'docking'))
from entrar_container import QRPerdido, taxa_busca

class BuscaControle(unittest.TestCase):
    def test_giro_somente_qr_perdido_e_gps_atual(self):
        link=types.SimpleNamespace(latest={
            'GLOBAL_POSITION_INT':types.SimpleNamespace(lat=0,lon=10000000,hdg=0),
            'GPS_RAW_INT':types.SimpleNamespace(fix_type=3)},
            last_seen={'GLOBAL_POSITION_INT':100,'GPS_RAW_INT':100})
        self.assertGreater(taxa_busca(QRPerdido('qr'),link,100,0,1.0001),0)
        self.assertIsNone(taxa_busca(RuntimeError('imagem desatualizada'),link,100,0,1.0001))
        self.assertIsNone(taxa_busca(QRPerdido('qr'),link,104,0,1.0001))

    def test_centers_visible_candidate_once_gps_heading_is_aligned(self):
        link=types.SimpleNamespace(latest={
            'GLOBAL_POSITION_INT':types.SimpleNamespace(lat=0,lon=10000000,hdg=9000),
            'GPS_RAW_INT':types.SimpleNamespace(fix_type=3)},
            last_seen={'GLOBAL_POSITION_INT':100,'GPS_RAW_INT':100})
        exc=QRPerdido('candidato nao decodificado',math.radians(-6))
        self.assertLess(taxa_busca(exc,link,100,0,1.0001),0)
        self.assertIsNone(taxa_busca(exc,link,104,0,1.0001))

if __name__=='__main__': unittest.main()
