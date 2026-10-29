#!/usr/bin/python3
"""Integration test with an isolated ROS master and synthetic moving LiDAR.

Does not start Gazebo, connect to MAVLink, or move any rover.
"""
import json
import math
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import xmlrpc.client

import numpy as np


def room_scan(x, y, yaw):
    angles = np.linspace(-math.pi, math.pi, 720, endpoint=False)
    directions = np.c_[np.cos(angles + yaw), np.sin(angles + yaw)]
    intersections = []
    # Asymmetric room, with a short interior wall. No pose is sent to SLAM.
    segments = [((-8, -6), (9, -6)), ((9, -6), (9, 7)),
                ((9, 7), (-8, 7)), ((-8, 7), (-8, -6)), ((3, 2), (6, 2))]
    origin = np.array([x, y])
    for start, end in segments:
        start, end = np.array(start), np.array(end)
        v = end - start
        denominator = directions[:, 0]*v[1] - directions[:, 1]*v[0]
        delta = start - origin
        with np.errstate(divide='ignore', invalid='ignore'):
            distance = (delta[0]*v[1] - delta[1]*v[0]) / denominator
            position = (delta[0]*directions[:, 1] - delta[1]*directions[:, 0]) / denominator
        distance[(distance <= 0) | (position < 0) | (position > 1)] = np.inf
        intersections.append(distance)
    ranges = np.min(intersections, axis=0)
    return np.c_[ranges*np.cos(angles), ranges*np.sin(angles), np.full(len(angles), .1)]


def main():
    python_dir = Path(__file__).resolve().parents[1]
    output = python_dir / 'run/validation'
    output.mkdir(parents=True, exist_ok=True)
    probe = socket.socket()
    if probe.connect_ex(('127.0.0.1', 11321)) == 0:
        raise RuntimeError('Porta de teste 11321 ja ocupada; encerre apenas o teste anterior')
    probe.close()
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:11321'
    core_log = (output/'roscore.log').open('w')
    core = subprocess.Popen(['/opt/ros/noetic/bin/roscore', '-p', '11321'],
                            stdout=core_log, stderr=subprocess.STDOUT, start_new_session=True)
    slam = None
    slam_log = (output/'slam_driver.log').open('w')
    result = {}
    try:
        for _ in range(100):
            try:
                if xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI']).getPid('/validation')[0] == 1:
                    break
            except OSError:
                pass
            time.sleep(.1)
        else:
            raise RuntimeError('ROS master de teste nao iniciou')
        import rospy
        from geometry_msgs.msg import Point32
        from sensor_msgs.msg import PointCloud, PointCloud2
        from sensor_msgs import point_cloud2
        from std_msgs.msg import Header
        rospy.init_node('slam_validation', anonymous=True, disable_signals=True)
        topic = '/synthetic_mid360'
        publisher = rospy.Publisher(topic, PointCloud, queue_size=1, latch=True)
        publisher2 = rospy.Publisher('/synthetic_cloud2', PointCloud2, queue_size=1, latch=True)
        stamp = rospy.Time.now()
        cloud2 = point_cloud2.create_cloud_xyz32(Header(stamp=stamp, frame_id='synthetic_lidar'),
                                                [[1, 2, .1], [0, 0, 0], [3, -2, .2]])
        publisher2.publish(cloud2)
        reader = subprocess.run(['/usr/bin/python3.8', str(python_dir/'lidar/read_ros_lidar.py'),
                                 '--topic', '/synthetic_cloud2', '--snapshot', str(output/'cloud2.npz')],
                                capture_output=True, text=True, timeout=15)
        assert reader.returncode == 0, (reader.stdout, reader.stderr)
        with np.load(output/'cloud2.npz') as saved:
            assert saved['xyz'].shape == (2, 3)
        slam_output = output / 'slam'
        slam = subprocess.Popen(['/usr/bin/python3.8', str(python_dir/'slam/slam_mid360_2d.py'),
                                 '--topic', topic, '--headless', '--duration', '15', '--map-size', '512',
                                 '--output-dir', str(slam_output), '--timeout', '10'],
                                stdout=slam_log, stderr=subprocess.STDOUT, start_new_session=True)
        started = time.monotonic()
        last_truth = (0., 0., 0.)
        while slam.poll() is None and time.monotonic() - started < 25:
            elapsed = time.monotonic() - started
            fraction = min(max((elapsed - 4) / 8, 0), 1)
            last_truth = (.8*fraction, .2*fraction, .12*fraction)
            xyz = room_scan(*last_truth)
            # Partial, changing angular coverage approximates a sparse scan.
            offset = int(elapsed*10) % 2
            xyz = xyz[offset::2]
            message = PointCloud()
            message.header.stamp = rospy.Time.now()
            message.header.frame_id = 'synthetic_lidar'
            message.points = [Point32(*map(float, row)) for row in xyz]
            publisher.publish(message)
            time.sleep(.1)
        assert slam.wait(timeout=5) == 0, (output/'slam_driver.log').read_text()[-4000:]
        status = json.loads((slam_output/'status.json').read_text())
        assert status['map_saved'] and status['scans'] >= 20 and status['pose'] is not None, status
        estimate = status['pose']
        translation_error = math.hypot(estimate[0]-last_truth[0], estimate[1]-last_truth[1])
        yaw_error = abs((estimate[2]-last_truth[2]+math.pi) % (2*math.pi)-math.pi)
        assert translation_error < .3 and yaw_error < .15, (estimate, last_truth)
        import cv2
        image = cv2.imread(str(slam_output/'map.pgm'), cv2.IMREAD_GRAYSCALE)
        assert np.any(image == 0) and np.any(image == 254) and np.any(image == 205)
        result = {'pointcloud2_reader': 'passed', 'map_saved': True, 'scans': status['scans'],
                  'estimated_pose': estimate, 'synthetic_truth': last_truth,
                  'translation_error_m': translation_error, 'yaw_error_rad': yaw_error,
                  'validation_source': 'synthetic LiDAR only; no ground truth supplied to SLAM'}
        (output/'validation.json').write_text(json.dumps(result, indent=2))
        print(json.dumps(result, indent=2))
    finally:
        if slam is not None and slam.poll() is None:
            os.killpg(slam.pid, signal.SIGINT)
            try:
                slam.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(slam.pid, signal.SIGKILL); slam.wait()
        try:
            rospy.signal_shutdown('validation finished')
        except NameError:
            pass
        os.killpg(core.pid, signal.SIGINT)
        try:
            core.wait(timeout=8)
        except subprocess.TimeoutExpired:
            os.killpg(core.pid, signal.SIGTERM); core.wait(timeout=5)
        core_log.close(); slam_log.close()


if __name__ == '__main__':
    main()
