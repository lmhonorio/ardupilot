"""ROS integration: delayed scans invalidate docking, not the persistent map."""
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import numpy as np
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
from builtin_interfaces.msg import Time
from std_msgs.msg import Header


def main():
    rclpy.init();node=rclpy.create_node('map_retention_test')
    clock=node.create_publisher(Clock,'/clock',10)
    cloud=node.create_publisher(PointCloud2,'/fast_lio/cloud_registered',10)
    odom=node.create_publisher(Odometry,'/fast_lio/odometry',10)
    maps=[];locals_=[]
    node.create_subscription(PointCloud2,'/rover_slam_3d/map',maps.append,
                             QoSProfile(depth=10,durability=DurabilityPolicy.TRANSIENT_LOCAL))
    node.create_subscription(PointCloud2,'/rover_slam_3d/local_cloud',locals_.append,10)
    script=Path(__file__).resolve().parents[2]/'python/slam/slam_mid360_3d.py'
    proc=subprocess.Popen([sys.executable,str(script),'--wall-timeout','3','--output-dir','/tmp/rover-map-test'],
                          start_new_session=True)
    def wait(check,timeout=5):
        end=time.monotonic()+timeout
        while time.monotonic()<end:
            rclpy.spin_once(node,timeout_sec=.02)
            if check():return
        raise AssertionError('Expected map/local-cloud update did not arrive')
    def stamp(value):
        ns=round(value*1e9);return Time(sec=ns//10**9,nanosec=ns%10**9)
    def send(t,now,points):
        end=time.monotonic()+.3
        while time.monotonic()<end:
            clock.publish(Clock(clock=stamp(now)))
            rclpy.spin_once(node,timeout_sec=.02)
        h=Header(stamp=stamp(t),frame_id='camera_init')
        o=Odometry(header=h,child_frame_id='body');o.pose.pose.orientation.w=1.
        for i in range(6):o.pose.covariance[i*7]=.001
        odom.publish(o);cloud.publish(point_cloud2.create_cloud_xyz32(h,points))
    a=np.array([[1+i*.1,0.,.2] for i in range(20)],np.float32);b=a+np.array([0,2,0],np.float32)
    try:
        wait(lambda:clock.get_subscription_count() and cloud.get_subscription_count() and odom.get_subscription_count(),20)
        send(100,100,a);wait(lambda:maps and maps[-1].width==20)
        first=len(maps);local_count=len(locals_)
        send(100.1,100.9,a);wait(lambda:len(locals_)>local_count and locals_[-1].width==0)
        assert all(m.width for m in maps[first:]),'Delayed scan erased accumulated map'
        send(100.95,100.95,b);wait(lambda:maps[-1].header.stamp==stamp(100.95))
        assert maps[-1].width==40,'Map lost points measured before delay'
        send(101.1,101.1,b);wait(lambda:locals_[-1].header.stamp==stamp(101.1))
        send(90,90,b);wait(lambda:maps[-1].width==0)
        assert locals_[-1].width==0
        print('PASS: stale scan clears local geometry; map retains both scans; clock reversal clears both',flush=True)
    finally:
        os.killpg(proc.pid,signal.SIGINT)
        try:proc.wait(timeout=5)
        except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
        node.destroy_node();rclpy.shutdown()


if __name__=='__main__':main()
