"""Short undecoded-QR approach must remain bounded and stop on lost inputs."""
import math
from pathlib import Path
import sys
import types
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking'))
from busca_qr import AproximacaoBusca


class ApproachTests(unittest.TestCase):
    def setUp(self):
        self.search=AproximacaoBusca()
        self.pos=types.SimpleNamespace(lat=0,lon=10000000,hdg=0)

    def step(self,t,error=0,distance=4,bearing=0,motion=True,allowed=True):
        return self.search.step(t,(math.radians(error),distance),self.pos,
                                None if bearing is None else math.radians(bearing),motion,allowed)[0]

    def test_align_strictly_below_five_degrees_and_settle(self):
        self.assertEqual(self.step(0,error=5),0)
        self.assertEqual(self.step(1,error=4.9),0)
        self.assertEqual(self.step(1.5,error=4.9),0)
        self.assertEqual(self.step(2,error=4.9),.04)
        self.assertEqual(self.step(2.1,error=-5),0)

    def test_missing_or_invalid_inputs_stop_immediately(self):
        self.step(0);self.assertGreater(self.step(1),0)
        self.assertEqual(self.step(1.1,bearing=None),0)
        self.assertEqual(self.step(2,bearing=6),0)
        self.assertEqual(self.step(3,motion=False),0)
        self.assertEqual(self.search.step(4,None,self.pos,0,True), (0.,'GPS/rumo indisponivel para aproximacao'))
        self.assertEqual(self.step(5,distance=2),0)
        self.assertEqual(self.step(6,allowed=False),0)

    def test_distance_budget_does_not_reset_on_reacquisition(self):
        self.step(0);self.assertGreater(self.step(1),0)
        self.pos.lat=40  # 0.445 m, already beyond the 0.40 m search budget.
        self.assertEqual(self.step(2),0)
        self.assertTrue(self.search.exhausted)
        self.step(3,allowed=False)
        self.pos.lat=0
        self.assertEqual(self.step(4),0)
        self.assertEqual(self.step(6),0)

    def test_total_command_time_budget_survives_loss_and_pause(self):
        self.step(0);self.step(1)
        self.step(7,bearing=None)  # 6 seconds commanded forward.
        self.step(100);self.assertGreater(self.step(101),0)
        self.assertEqual(self.step(107),0)
        self.assertTrue(self.search.exhausted)

    def test_alignment_must_be_reconfirmed_after_losing_candidate(self):
        self.step(0);self.step(1)
        self.assertEqual(self.step(1.1,bearing=None),0)
        self.assertEqual(self.step(2),0)
        self.assertEqual(self.step(2.4),0)
        self.assertGreater(self.step(3),0)

if __name__=='__main__':unittest.main()
