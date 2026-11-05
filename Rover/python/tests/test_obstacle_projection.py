import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'obstacle_avoidance'))
from projection import project, mav_distances

class ProjectionTests(unittest.TestCase):
    def test_cardinal_axes_and_closest(self):
        points=[[2,0,0],[1,0,0],[0,2,0],[-3,0,0],[0,-4,0]]
        r=project(points,offset=(0,0,.4))
        self.assertAlmostEqual(r[0],1); self.assertAlmostEqual(r[18],2)
        self.assertAlmostEqual(r[36],3); self.assertAlmostEqual(r[54],4)
        m=mav_distances(r); self.assertEqual((m[0],m[18],m[36],m[54]),(100,400,300,200))
        self.assertEqual(m[1],65535)
    def test_invalid_self_ground_high_and_close(self):
        r=project([[0,0,0],[float('nan'),1,1],[.1,.1,0],[1,0,-.7],[0,1,1],[.4,0,0]],offset=(0,0,0))
        self.assertAlmostEqual(r[0],.5); self.assertTrue(np.isnan(r[18])); self.assertEqual(np.isfinite(r).sum(),1)
    def test_nearest_and_wraparound(self):
        r=project([[2,.01,0],[1,-.01,0]],offset=(0,0,0))
        self.assertLess(r[0],1.01); self.assertEqual(np.isfinite(r).sum(),1)
    def test_nonfinite_never_becomes_free(self):
        m=mav_distances([float('nan')]*72); self.assertEqual(m,[65535]*72)
