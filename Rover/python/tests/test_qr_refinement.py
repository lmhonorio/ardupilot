"""Dense QR registration must correct corners without accepting another pattern."""
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking'))
from qr_refinement import QRRefiner


class RefinementTests(unittest.TestCase):
    def test_perspective_marker_recovers_perturbed_corners(self):
        refiner=QRRefiner('ROVER_DOCKING_STATION_01')
        truth=np.float32([[220,90],[356,80],[361,225],[212,230]])
        H=cv2.getPerspectiveTransform(refiner.corners,truth)
        image=cv2.warpPerspective(np.uint8(refiner.template*255),H,(640,360),borderValue=255)
        noisy=truth+np.float32([[1.,0],[0,-1.],[-1.5,1.],[.5,0]])
        refined,correlation=refiner.refine(image,noisy)
        self.assertGreater(correlation,.95)
        before=np.sqrt(np.mean((noisy-truth)**2))
        after=np.sqrt(np.mean((refined.reshape(4,2)-truth)**2))
        self.assertLess(after,before*.5)

    def test_missing_pattern_does_not_create_a_pose(self):
        refiner=QRRefiner('ROVER_DOCKING_STATION_01')
        corners=np.float32([[220,90],[356,80],[361,225],[212,230]])
        result,_=refiner.refine(np.full((360,640),180,np.uint8),corners)
        np.testing.assert_array_equal(result,corners)
