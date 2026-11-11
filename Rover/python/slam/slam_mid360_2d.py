#!/usr/bin/env python3
"""ROS 2: MID360 height slice + FAST-LIO continuous TF -> slam_toolbox 2D."""
import argparse
from pathlib import Path
import signal
import subprocess
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from common import ros2_runtime as ros
from common.ros_cloud import cloud_xyz,project_scan,subscribe_cloud


def main():
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--topic',default='/rover_argo_N1/Instance1/livox/lidar')
    p.add_argument('--duration',type=float,default=0)
    p.add_argument('--headless',action='store_true',help='Compatibilidade: visualize em RViz2 separadamente')
    a=p.parse_args()
    if a.duration<0:p.error('--duration deve ser >=0')
    from sensor_msgs.msg import LaserScan
    import numpy as np
    ros.init_node('mid360_scan_2d')
    pub=ros.Publisher('/rover_slam/scan',LaserScan,queue_size=3)
    def receive(msg):
        if not -.05<=ros.age(msg.header.stamp)<=.5:return
        if msg.header.frame_id!='mid360_sensor':raise ValueError('Frame MID360 inesperado')
        scan=LaserScan();scan.header=msg.header
        scan.angle_min=-float(np.pi);scan.angle_increment=2*float(np.pi)/360
        scan.angle_max=scan.angle_min+359*scan.angle_increment
        scan.range_min=.35;scan.range_max=30.;scan.scan_time=.1
        scan.ranges=project_scan(cloud_xyz(msg)).tolist();pub.publish(scan)
    sub=subscribe_cloud(ros,a.topic,receive)
    config=Path(__file__).resolve().parents[2]/'ros2/config/slam2d.yaml'
    proc=subprocess.Popen(['ros2','launch','slam_toolbox','online_async_launch.py',
                           'use_sim_time:=true','slam_params_file:='+str(config)])
    print('Requer FAST-LIO ativo: camera_init -> body -> base_link. Mapa: /map',flush=True)
    start=time.monotonic()
    try:
        while not ros.is_shutdown():
            if proc.poll() is not None:return proc.returncode or 1
            if a.duration and time.monotonic()-start>=a.duration:break
            time.sleep(.1)
    except KeyboardInterrupt:pass
    finally:
        sub.unregister()
        if proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try:proc.wait(timeout=8)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
        ros.signal_shutdown()
    return 0

if __name__=='__main__':sys.exit(main())
