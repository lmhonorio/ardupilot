#!/usr/bin/env python3
"""Isolated real DDS test of docking camera, joints and calibration changes.

Run with ROS_DOMAIN_ID=93; never opens a MAVLink socket or sends commands.
"""
from pathlib import Path
import sys
import time
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))

import numpy as np
import rclpy
from builtin_interfaces.msg import Time
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import Image, CameraInfo, JointState
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from common import ros2_runtime as ros
from common.docking_camera import DockingCamera
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'docking_lidar'))
from sensores import Sensores


def main():
    ros.init_node('docking_io_test')
    peer=rclpy.create_node('docking_io_source')
    ns='/test_rover/Instance7'
    camera=DockingCamera(ns)
    joints=ros.JointFeedback(ns+'/joint_states')
    clock=peer.create_publisher(Clock,'/clock',10)
    image_pub=peer.create_publisher(Image,ns+'/camera/image_raw',qos_profile_sensor_data)
    info_pub=peer.create_publisher(CameraInfo,ns+'/camera/camera_info',qos_profile_sensor_data)
    joint_pub=peer.create_publisher(JointState,ns+'/joint_states',qos_profile_sensor_data)
    cloud_pub=peer.create_publisher(PointCloud2,ns+'/local_cloud',qos_profile_sensor_data)
    odom_pub=peer.create_publisher(Odometry,ns+'/odometry',qos_profile_sensor_data)
    lidar=Sensores(ns+'/local_cloud',ns+'/odometry')
    cloud=point_cloud2.create_cloud_xyz32(Header(frame_id='body',stamp=Time(sec=42)),
                                        [[1.,0.,0.],[1.,1.,0.],[2.,0.,0.]])
    odom=Odometry(header=Header(frame_id='camera_init',stamp=Time(sec=42)),child_frame_id='body')
    odom.pose.pose.orientation.w=1.
    odom.pose.covariance[0]=odom.pose.covariance[7]=odom.pose.covariance[14]=.01
    image=CvBridge().cv2_to_imgmsg(np.zeros((360,640,3),dtype=np.uint8),'bgr8')
    image.header.frame_id='camera_optical';image.header.stamp=Time(sec=42)
    info=CameraInfo(width=640,height=360,k=[300.,0.,320.,0.,300.,180.,0.,0.,1.],d=[0.]*5,
                    distortion_model='plumb_bob')
    info.header=image.header
    joint=JointState(name=['pt_support_s23_pan_joint','pt_support_s23_tilt_joint'],position=[0.,-.3])
    joint.header.stamp=image.header.stamp

    def publish():
        clock.publish(Clock(clock=Time(sec=42,nanosec=100000000)))
        image_pub.publish(image);info_pub.publish(info);joint_pub.publish(joint)
        cloud_pub.publish(cloud);odom_pub.publish(odom)
        rclpy.spin_once(peer,timeout_sec=.03)

    try:
        deadline=time.monotonic()+8
        while time.monotonic()<deadline:
            publish()
            try:
                frame,_,_=camera.snapshot()
                assert frame.shape==(360,640,3)
                assert joints.position('pt_support_s23_tilt_joint')==-.3
                assert lidar.fresh_cloud().width==3
                position,yaw=lidar.lio()
                assert np.all(position==0.) and yaw==0.
                break
            except (RuntimeError,ValueError):continue
        else:raise AssertionError('ROS 2 camera/joints did not arrive')
        print('PASS: best-effort Image + CameraInfo + JointState, alternate namespace, /clock')
        print('PASS: best-effort local PointCloud2 + odometry using configurable topics')
        info.width=1280
        deadline=time.monotonic()+3
        while time.monotonic()<deadline:
            publish()
            try:camera.snapshot()
            except RuntimeError as exc:
                if 'Resolucao' in str(exc):break
        else:raise AssertionError('Stale calibration was not rejected')
        print('PASS: rejects CameraInfo from a different resolution over real DDS')
    finally:
        camera.close();joints.close();lidar.close();peer.destroy_node();ros.signal_shutdown('test complete')

if __name__=='__main__':main()
