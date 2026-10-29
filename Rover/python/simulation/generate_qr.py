#!/usr/bin/env python3.8
"""Regenerate the wall texture using the OpenCV already installed in Python 3.8."""
import argparse
from pathlib import Path
import cv2
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('text', nargs='?', default='ROVER_PAREDE_VERDE_01')
args = parser.parse_args()
qr = cv2.QRCodeEncoder_create().encode(args.text)
qr = np.pad(qr, 4, constant_values=255)
qr = cv2.resize(qr, None, fx=16, fy=16, interpolation=cv2.INTER_NEAREST)
if cv2.QRCodeDetector().detectAndDecode(qr)[0] != args.text:
    parser.error('Nao foi possivel verificar o QR gerado.')
output = Path(__file__).resolve().parents[2] / 'gazebo_simple/models/wall_qr/materials/textures/qr.png'
if not cv2.imwrite(str(output), qr):
    raise OSError('Nao foi possivel salvar a textura.')
print(output, args.text)
