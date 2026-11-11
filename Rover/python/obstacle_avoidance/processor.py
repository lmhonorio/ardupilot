#!/usr/bin/python3
"""Publish nearest returns as 72-bin ROS LaserScan, without guessing clear space."""
import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import argparse, math, sys, time, threading
from pathlib import Path
import numpy as np
import yaml
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.ros_cloud import cloud_xyz, subscribe_cloud
from projection import project


def main():
    p=argparse.ArgumentParser(); p.add_argument('--config',default=str(Path(__file__).with_name('config.yaml')))
    args=p.parse_args(); c=yaml.safe_load(Path(args.config).read_text())['processing']
    from common import ros2_runtime as ros
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from visualization_msgs.msg import Marker
    from geometry_msgs.msg import Point
    ros.init_node('olhe_obstacle_processor')
    scanpub=ros.Publisher(c['scan_topic'],LaserScan,queue_size=1)
    markerpub=ros.Publisher('/olhe/obstacles/markers',Marker,queue_size=1)
    status=ros.Publisher('/olhe/obstacles/status',String,queue_size=1)
    lock=threading.Lock(); state={'message':None,'wall':0,'stamp':0}
    def callback(msg):
        with lock:
            state.update(message=msg,wall=time.monotonic(),stamp=ros.seconds(msg.header.stamp))
    sub=subscribe_cloud(ros,c['cloud_topic'],callback,30)
    period=1.0/c['rate_hz']; last_stamp=None
    while not ros.is_shutdown():
        with lock: msg,wall,stamp=state['message'],state['wall'],state['stamp']
        lag = ros.age(msg.header.stamp) if msg else 0
        if msg is None or time.monotonic()-wall > c.get('wall_timeout_s', .5) or stamp <= 0 or lag < -.05 or lag > c['temporal_window_s']:
            status.publish('STALE_LIDAR'); time.sleep(period); continue
        if last_stamp is not None and stamp < last_stamp:
            status.publish('CLOCK_RESET_RESTART'); ros.logerr('Relogio ROS retrocedeu; reinicie o processador'); break
        last_stamp=stamp
        try:
            ranges=project(cloud_xyz(msg),c['lidar_offset_m'],(c['min_height_base_m'],c['max_height_base_m']),
                           (c['self_half_length_m'],c['self_half_width_m']),c['min_distance_m'],c['max_distance_m'])
        except (ValueError,TypeError) as e:
            status.publish('INVALID_CLOUD'); ros.logwarn_throttle(5,str(e)); time.sleep(period); continue
        scan=LaserScan(); scan.header.stamp=msg.header.stamp; scan.header.frame_id=c['frame_id']
        scan.angle_min=0.; scan.angle_increment=2*math.pi/72; scan.angle_max=71*scan.angle_increment
        scan.range_min=c['min_distance_m']; scan.range_max=c['max_distance_m']; scan.ranges=ranges.tolist()
        scanpub.publish(scan)
        m=Marker(); m.header=scan.header; m.ns='sectors'; m.id=0; m.type=Marker.LINE_LIST; m.action=Marker.ADD
        m.pose.orientation.w=1.; m.scale.x=.025; m.color.r=1.; m.color.g=.4; m.color.a=1.; m.lifetime=ros.Duration(.3)
        for i,d in enumerate(ranges):
            if np.isfinite(d):
                a=i*scan.angle_increment; m.points.extend([Point(x=0.,y=0.,z=0.),Point(x=float(d*math.cos(a)),y=float(d*math.sin(a)),z=0.)])
        markerpub.publish(m); status.publish('OBSERVED_SECTORS=%d/72'%np.isfinite(ranges).sum())
        time.sleep(period)
    sub.unregister()
if __name__=='__main__': main()
