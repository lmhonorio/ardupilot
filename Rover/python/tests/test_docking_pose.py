import math
import json
import xml.etree.ElementTree as ET
import sys
import unittest
from pathlib import Path
import numpy as np
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.docking_stations import load_stations
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking'))
from relative_pose import solve,CODE_SIZE_M,project_marker

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

    def test_measured_pan_and_tilt_preserve_body_pose(self):
        K=np.array([[554.,0,320],[0,554.,180],[0,0,1.]])
        for pan in (-.7,0.,.7):
            marker=np.array([2.,.4,.4]);heading=.25
            points,_=project_marker(marker,heading,K,np.zeros(5),pan, -.3)
            p=solve(points,K,tilt_rad=-.3,pan_rad=pan,heading_prior=heading)
            self.assertAlmostEqual(p['marker_forward'],2.,places=4)
            self.assertAlmostEqual(p['marker_right'],-.4,places=4)
            self.assertAlmostEqual(p['normal_yaw'],heading,places=4)

    def test_registered_target_offset_matches_harmonic_qr_plane(self):
        rover=Path(__file__).resolve().parents[2]
        model=rover/'ros2/assets/models/docking_station/model.sdf'
        if not model.exists():self.skipTest('Assets Harmonic nao preparados')
        pose=ET.parse(model).find(".//visual[@name='qr_code']/pose").text
        qr_x=float(pose.split()[0])
        station=load_stations(rover/'estacoes_recarga.txt')[0]
        self.assertAlmostEqual(station['distancia_alvo_parede_m'],qr_x)
