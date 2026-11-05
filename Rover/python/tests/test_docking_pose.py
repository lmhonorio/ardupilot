import math
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking'))
from relative_pose import solve,CODE_SIZE_M

class DockPoseTests(unittest.TestCase):
    def test_frontal_target_and_left_right(self):
        fx=1000.; K=np.array([[fx,0,640],[0,fx,360],[0,0,1]])
        width=fx*CODE_SIZE_M/2.5
        corners=np.array([[640-width/2,360-width/2],[640+width/2,360-width/2],[640+width/2,360+width/2],[640-width/2,360+width/2]])
        pose=solve(corners,K,tilt_rad=0,wall_offset=.55)
        self.assertAlmostEqual(pose['forward'],2.,delta=.02)
        self.assertAlmostEqual(pose['right'],0,delta=.01)
        shifted=solve(corners+[80,0],K,tilt_rad=0,wall_offset=.55)
        self.assertGreater(shifted['right'],0)
    def test_invalid_focal_is_rejected(self):
        with self.assertRaises(ValueError):solve(np.zeros((4,2)),np.zeros((3,3)))
