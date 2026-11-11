#!/usr/bin/env python3
"""Analisa uma nuvem MID360 com semente manual; nunca envia comandos MAVLink."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import argparse,math,sys,time
from pathlib import Path
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common import ros2_runtime as ros
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common.ros_cloud import subscribe_cloud,cloud_xyz
from geometria import estimar
p=argparse.ArgumentParser(description=__doc__)
p.add_argument('--frente',type=float,required=True,help='centro aproximado em metros para frente')
p.add_argument('--esquerda',type=float,required=True,help='centro aproximado em metros para esquerda')
p.add_argument('--angulo',type=float,default=0,help='eixo da estacao em graus no referencial FLU')
p.add_argument('--cloud',default='/rover_argo_N1/Instance1/livox/lidar')
a=p.parse_args()
ros.init_node('docking_lidar_diagnostico',anonymous=True,disable_signals=True)
received=[]
sub=subscribe_cloud(ros,a.cloud,lambda m:received.append(m),15)
deadline=time.monotonic()+5
while not received and time.monotonic()<deadline:time.sleep(.1)
if not received:print('Nenhuma nuvem recebida');sys.exit(1)
m=received[-1]
try:
 if m.header.frame_id!='mid360_sensor':raise ValueError('Frame inesperado: '+m.header.frame_id)
 print('Pose MID360:',estimar(cloud_xyz(m),[a.frente,a.esquerda,math.radians(a.angulo)],corrected=True))
except ValueError as e:print('Pose nao confirmada:',e);sys.exit(1)
finally:sub.unregister()
