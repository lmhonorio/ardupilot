"""Two independent ROS participants: verify full-size cloud transport in Docker."""
import argparse
import hashlib
import os
from pathlib import Path
import subprocess
import sys
import time
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField

p=argparse.ArgumentParser();p.add_argument('--publisher',action='store_true');a=p.parse_args()
rclpy.init();node=rclpy.create_node('large_cloud_'+('writer' if a.publisher else 'reader'))
topic='/companion_test/full_cloud';size=720*64*32;payload=bytes(range(256))*(size//256)
checksum=hashlib.sha256(payload).digest()
try:
    if a.publisher:
        pub=node.create_publisher(PointCloud2,topic,qos_profile_sensor_data)
        deadline=time.monotonic()+10
        while not pub.get_subscription_count() and time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
        if not pub.get_subscription_count():raise RuntimeError('No reader')
        msg=PointCloud2(height=64,width=720,point_step=32,row_step=720*32,
                       fields=[PointField(name='x',offset=0,datatype=7,count=1)],data=payload)
        msg.header.frame_id='transport_test'
        for i in range(20):
            msg.header.stamp.sec=i+1;pub.publish(msg);rclpy.spin_once(node,timeout_sec=.01);time.sleep(.15)
        time.sleep(1)
    else:
        seen=set()
        def receive(msg):
            assert len(msg.data)==size and hashlib.sha256(msg.data).digest()==checksum
            assert msg.header.frame_id=='transport_test' and msg.width==720 and msg.height==64
            seen.add(msg.header.stamp.sec)
        sub=node.create_subscription(PointCloud2,topic,receive,qos_profile_sensor_data)
        proc=subprocess.Popen([sys.executable,str(Path(__file__).resolve()),'--publisher'])
        end=time.monotonic()+20
        try:
            while time.monotonic()<end and len(seen)<20:rclpy.spin_once(node,timeout_sec=.1)
        finally:
            try:proc.wait(timeout=5)
            except subprocess.TimeoutExpired:proc.terminate();proc.wait(timeout=5)
        print('Full 1.4 MiB clouds received intact: %d/20'%len(seen),flush=True)
        if len(seen)<18:raise RuntimeError('Full cloud transport lost too many messages')
finally:
    node.destroy_node()
    if rclpy.ok():rclpy.shutdown()
