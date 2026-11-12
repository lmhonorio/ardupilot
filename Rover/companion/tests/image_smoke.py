"""Run inside the image: native ROS types, MAVLink and QR decoder availability."""
import cv2
import numpy as np
from pathlib import Path
import subprocess
from pymavlink import mavutil
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rover_mission_interfaces.srv import StartMission
from cv_bridge import CvBridge
from mission_manager.files import load_mission
from mission_manager.manager import Manager
from docking_lidar.geometria import estimar

text='ROVER_DOCKING_STATION_01'
qr=cv2.QRCodeEncoder_create().encode(text)
qr=cv2.copyMakeBorder(qr,4,4,4,4,cv2.BORDER_CONSTANT,value=255)
qr=cv2.resize(qr,(400,400),interpolation=cv2.INTER_NEAREST)
decoded,_,_=cv2.QRCodeDetector().detectAndDecode(qr)
assert decoded==text,decoded
image=CvBridge().cv2_to_imgmsg(qr,encoding='mono8')
assert image.height==400
p=CustomPoint(offset_time=100000000,x=1.,y=0.,z=0.,reflectivity=1,tag=0,line=0)
m=CustomMsg(point_num=1,points=[p]);assert m.point_num==1
assert StartMission.Request(mission_file='/missions/example.plan').mission_file
root=Path('/opt/rover/Rover/ros2')
for binary in ('lib/fast_lio/fastlio_mapping','lib/livox_ros_driver2/livox_ros_driver2_node',
               'lib/rover_companion_bridge/sensor_ingress','lib/rover_companion_bridge/simulation_adapter'):
    binary=root/'install'/binary
    assert binary.is_file(),binary
    linkage=subprocess.run(['ldd',str(binary)],capture_output=True,text=True,check=True).stdout
    assert 'not found' not in linkage,linkage
for name in ('mat_pre.txt','mat_out.txt','dbg.txt','pos_log.txt'):
    assert (root/'deps/FAST_LIO/Log'/name).resolve()==Path('/dev/null')
print('PASS: ROS interfaces, cv_bridge, QR, pymavlink, docking/manager imports; OpenCV',cv2.__version__)
print('PASS: native FAST-LIO/Livox binaries, shared libraries and debug stream handles')
