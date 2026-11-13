#!/usr/bin/env python3
"""Valida PointCloud2 e IMU fisicas no dominio 21, sem comandos ao rover."""
import argparse
import json
import math
import time

import numpy as np
import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu, PointCloud2


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--seconds', type=float, default=20)
    args = parser.parse_args()
    if not 5 <= args.seconds <= 120:
        parser.error('--seconds deve estar entre 5 e 120')
    rclpy.init()
    node = rclpy.create_node('mid360_hardware_test')
    qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    cloud = []; imu = []; points = []; norms = []; steps = []; errors = []
    frames = set(); fields = set(); sampled = 0; finite = 0; valid = 0
    previous = None

    def on_cloud(msg):
        nonlocal previous, sampled, finite, valid
        cloud.append(time.monotonic()); points.append(msg.width*msg.height)
        frames.add(msg.header.frame_id)
        fields.update(f.name for f in msg.fields)
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec/1e9
        if previous is not None: steps.append(stamp-previous)
        previous = stamp
        xyz = {f.name: f for f in msg.fields if f.name in ('x', 'y', 'z')}
        if (set(xyz) != {'x', 'y', 'z'} or any(f.datatype != 7 or f.count != 1 for f in xyz.values())
                or msg.point_step < 12 or msg.row_step < msg.width*msg.point_step
                or len(msg.data) != msg.row_step*msg.height
                or any(f.offset+4 > msg.point_step for f in xyz.values())):
            errors.append('Layout PointCloud2 invalido'); return
        dtype = np.dtype({'names': ['x', 'y', 'z'], 'formats': [('>' if msg.is_bigendian else '<')+'f4']*3,
                          'offsets': [xyz[k].offset for k in ('x', 'y', 'z')], 'itemsize': msg.point_step})
        a = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                       strides=(msg.row_step, msg.point_step))
        good = np.isfinite(a['x']) & np.isfinite(a['y']) & np.isfinite(a['z'])
        sampled += a.size; finite += int(good.sum())
        valid += int((good & (a['x']**2+a['y']**2+a['z']**2 > .01)).sum())

    def on_imu(msg):
        imu.append(time.monotonic())
        a = msg.linear_acceleration
        norms.append(math.sqrt(a.x*a.x+a.y*a.y+a.z*a.z))

    node.create_subscription(PointCloud2, '/mid360/points', on_cloud, qos)
    node.create_subscription(Imu, '/mid360/imu', on_imu, qos)
    start = time.monotonic()
    try:
        while time.monotonic()-start < args.seconds:
            rclpy.spin_once(node, timeout_sec=.1)
        def rate(stamps):
            return (len(stamps)-1)/(stamps[-1]-stamps[0]) if len(stamps)>1 else 0
        result = dict(duration_s=args.seconds, cloud_messages=len(cloud), cloud_hz=rate(cloud),
                      imu_messages=len(imu), imu_hz=rate(imu),
                      points_min=min(points, default=0), points_max=max(points, default=0),
                      inspected_points=sampled, finite_points=finite, nonzero_points=valid,
                      frame_ids=sorted(frames), fields=sorted(fields), errors=errors,
                      imu_acceleration_norm_min=min(norms, default=0),
                      imu_acceleration_norm_max=max(norms, default=0),
                      cloud_stamp_step_min=min(steps, default=0), cloud_stamp_step_max=max(steps, default=0))
        result['passed'] = (len(cloud)>=10 and len(imu)>=50 and not errors and finite==sampled and valid>=100
                            and bool(steps) and min(steps)>0 and all(math.isfinite(n) for n in norms)
                            and time.monotonic()-cloud[-1]<2 and time.monotonic()-imu[-1]<2)
        print(json.dumps(result, indent=2), flush=True)
        return 0 if result['passed'] else 1
    finally:
        node.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
