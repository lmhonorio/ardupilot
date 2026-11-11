"""Regression: the visible 32 px QR from Harmonic could not be decoded directly."""
from pathlib import Path
import sys
import unittest
import cv2
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.qr_detector import QRDetector


class QRTests(unittest.TestCase):
    def test_small_rendered_station_code_and_original_pixel_coordinates(self):
        image=cv2.imread(str(Path(__file__).with_name('fixtures')/'qr_small_harmonic.png'))
        self.assertIsNotNone(image)
        text,points,_=QRDetector().detectAndDecode(image)
        self.assertEqual(text,'ROVER_DOCKING_STATION_01')
        p=points.reshape(4,2)
        # Pixel coordinates must refer to the 640x360 original used by solvePnP,
        # not the upscaled crop; otherwise distance/heading would be incorrect.
        self.assertTrue(np.all((p[:,0]>230)&(p[:,0]<275)))
        self.assertTrue(np.all((p[:,1]>295)&(p[:,1]<342)))
        self.assertTrue(29 < np.ptp(p[:,0]) < 36)

    def test_no_code_never_becomes_station(self):
        image=np.full((360,640,3),180,np.uint8)
        text,points,_=QRDetector().detectAndDecode(image)
        self.assertEqual(text,'')
        self.assertIsNone(points)

    def test_another_code_is_not_relabelled_as_station(self):
        code=cv2.QRCodeEncoder_create().encode('OUTRA_ESTACAO')
        image=cv2.resize(np.pad(code,4,constant_values=255),None,fx=8,fy=8,
                         interpolation=cv2.INTER_NEAREST)
        text,points,_=QRDetector().detectAndDecode(image)
        self.assertEqual(text,'OUTRA_ESTACAO')
        self.assertIsNotNone(points)

if __name__=='__main__':unittest.main()
