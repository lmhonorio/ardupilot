#!/usr/bin/env python3
"""Read-only delivery/age check, callable on the Raspberry or this PC."""
import argparse
import json
import time
import rclpy
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu,PointCloud2
from rosgraph_msgs.msg import Clock
from nav_msgs.msg import Odometry

p=argparse.ArgumentParser();p.add_argument('--duration',type=float,default=15);p.add_argument('--namespace',default='/rover_argo_N1/Instance1');p.add_argument('--cloud-topic');a=p.parse_args()
rclpy.init();n=rclpy.create_node('companion_network_probe',parameter_overrides=[Parameter('use_sim_time',value=True)])
counts={k:0 for k in ('clock','imu','cloud','odom','local_cloud')};ages={k:[] for k in ('cloud','odom','local_cloud')};sizes=[];empty={k:0 for k in ('cloud','local_cloud')}
def receive(k,msg):
    counts[k]+=1
    if k in empty and msg.width*msg.height==0:
        empty[k]+=1
        return
    if k in ages and n.get_clock().now().nanoseconds>0:
        ages[k].append((n.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9)
    if k=='cloud':sizes.append(msg.width*msg.height)
for topic,typ,name in [('/clock',Clock,'clock'),(a.namespace+'/livox/imu',Imu,'imu'),(a.cloud_topic or a.namespace+'/livox/lidar',PointCloud2,'cloud'),('/fast_lio/odometry',Odometry,'odom'),('/rover_slam_3d/local_cloud',PointCloud2,'local_cloud')]:
    n.create_subscription(typ,topic,lambda m,k=name:receive(k,m),qos_profile_sensor_data)
end=time.monotonic()+a.duration
while time.monotonic()<end:rclpy.spin_once(n,timeout_sec=.1)
result={'duration_wall_s':a.duration,'counts':counts,'empty_clouds':empty,'points_per_cloud':max(sizes) if sizes else 0,
        'age_s':{k:{'min':min(v),'max':max(v),'mean':sum(v)/len(v)} if v else None for k,v in ages.items()}}
print(json.dumps(result,indent=2));n.destroy_node();rclpy.shutdown()
