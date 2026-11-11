#!/usr/bin/python3
"""Run official FAST-LIO2 on the Gazebo MID360 and forward its pose to Lua."""
import argparse
from datetime import datetime
import math
import os
from pathlib import Path
import secrets
import signal
import struct
import subprocess
import sys
import threading
import time

import numpy as np
PYTHON_DIR = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PYTHON_DIR))
sys.path.insert(0, '/home/grin/scanner_ws/devel/lib/python3/dist-packages')
from common.ros_cloud import cloud_xyz, subscribe_cloud
from install_fast_lio import BINARY as FAST_LIO_BINARY

PACKET = struct.Struct('<4sIIQ7fB')
DATA_TYPE, SOURCE_SYSID, SOURCE_COMPID = 201, 245, 197
OFFSET_NS = 100_000_000  # all points were acquired at the same simulator instant
BASE_TO_IMU = np.array([-.003, 0., .434])


def rotation(q):
    w, x, y, z = q
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def pose_payload(position, quaternion, first_rotation, stamp_us, session, sequence):
    position, quaternion = np.asarray(position, dtype=float), np.asarray(quaternion, dtype=float)
    norm = np.linalg.norm(quaternion)
    if not np.isfinite(position).all() or not np.isfinite(quaternion).all() or not .9 < norm < 1.1:
        raise ValueError('Posicao/quaternion FAST-LIO invalido')
    quaternion = quaternion / norm
    r = rotation(quaternion)
    first_rotation = r if first_rotation is None else first_rotation
    # FAST-LIO estimates the IMU origin; translate to the initial rover base.
    base = position + (first_rotation - r).dot(BASE_TO_IMU)
    w, x, y, z = quaternion
    # Both local and body axes: ROS FLU -> local/body FRD.
    payload = PACKET.pack(b'FLIO', session, sequence, stamp_us,
                          base[0], -base[1], -base[2], w, x, -y, -z, 165)
    return payload, first_rotation


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/rover_argo_N1/Instance1')
    parser.add_argument('--cloud-topic')
    parser.add_argument('--imu-topic')
    parser.add_argument('--mavproxy', default='udpin:127.0.0.1:14556')
    parser.add_argument('--no-mavlink', action='store_true')
    parser.add_argument('--duration', type=float, default=0, help='Tempo de teste em segundos; 0 = continuo')
    parser.add_argument('--max-lag', type=float, default=.5, help='Idade maxima de uma pose em segundos ROS')
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if not args.mavproxy.startswith('udpin:') or args.duration < 0 or args.max_lag <= 0:
        parser.error('Use udpin:IP:PORTA de MAVProxy, duration >= 0 e max-lag > 0')
    backend = FAST_LIO_BINARY
    if not backend.is_file():
        print('FAST-LIO2 nao encontrado. Execute python/slam/fast_lio/install_fast_lio.py', file=sys.stderr); return 1
    import rospy, rosnode, rosparam
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import Imu
    from livox_ros_driver.msg import CustomMsg, CustomPoint
    try:
        rospy.get_master().getPid()
    except Exception as exc:
        print('Inicie o roslaunch primeiro:', exc, file=sys.stderr); return 1
    rospy.init_node('fast_lio_companion', anonymous=True)
    if '/fast_lio/laserMapping' in rosnode.get_node_names():
        print('FAST-LIO ja esta rodando em /fast_lio', file=sys.stderr); return 1
    ns = '/' + args.namespace.strip('/')
    cloud_topic = args.cloud_topic or ns + '/livox/lidar'
    imu_topic = args.imu_topic or ns + '/livox/imu'
    output = args.output_dir or PYTHON_DIR / 'run/fast_lio' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=True)
    params = {
        'common': {'lid_topic': '/fast_lio/lidar', 'imu_topic': imu_topic, 'time_sync_en': False},
        'preprocess': {'lidar_type': 1, 'scan_line': 1, 'blind': .35, 'time_scale': .001},
        'mapping': {'acc_cov': .1, 'gyr_cov': .1, 'b_acc_cov': .0001, 'b_gyr_cov': .0001,
                    'det_range': 50., 'extrinsic_est_en': False, 'extrinsic_T': [0., 0., 0.],
                    'extrinsic_R': [1., 0., 0., 0., 1., 0., 0., 0., 1.]},
        'publish': {'path_en': True, 'scan_publish_en': True,
                    'dense_publish_en': False, 'scan_bodyframe_pub_en': False,
                    'scan_effect_pub_en': False},
        'feature_extract_enable': False, 'point_filter_num': 1, 'max_iteration': 3,
        'filter_size_surf': .15, 'filter_size_map': .15, 'cube_side_length': 200.,
        'ivox_grid_resolution': .25, 'ivox_nearby_type': 18, 'esti_plane_threshold': .1,
        'runtime_pos_log_enable': False, 'path_save_en': False,
        'pcd_save': {'pcd_save_en': False, 'interval': -1}}
    rosparam.upload_params('/fast_lio', params)
    cloud_pub = rospy.Publisher('/fast_lio/lidar', CustomMsg, queue_size=1)
    lock = threading.RLock()
    state = {'clouds': 0, 'imu': 0, 'poses': 0, 'sent': 0, 'dropped': 0,
             'last_stamp': 0., 'first_rotation': None, 'session': secrets.randbits(32),
             'last_pose': None, 'clock_error': False}
    mav = None
    if not args.no_mavlink:
        os.environ['MAVLINK20'] = '1'
        from pymavlink import mavutil
        mav = mavutil.mavlink_connection(args.mavproxy, source_system=SOURCE_SYSID,
                                         source_component=SOURCE_COMPID, dialect='ardupilotmega')

    def receive_cloud(msg):
        try:
            xyz = cloud_xyz(msg)
            distance = np.linalg.norm(xyz, axis=1)
            xyz = xyz[(distance >= .35) & (distance <= 50)]
            now_ns = msg.header.stamp.to_nsec()
            if len(xyz) < 10 or now_ns <= OFFSET_NS:
                return
            out = CustomMsg()
            # Gazebo evaluates the entire batch at the message timestamp.
            # Represent every return at that same instant, without invented scan timing.
            out.header.stamp = rospy.Time.from_sec((now_ns - OFFSET_NS) / 1e9)
            out.header.frame_id = msg.header.frame_id
            out.timebase = now_ns - OFFSET_NS
            out.point_num = len(xyz)
            out.points = [CustomPoint(offset_time=OFFSET_NS, x=float(x), y=float(y), z=float(z),
                                      reflectivity=1, tag=0, line=0) for x, y, z in xyz]
            cloud_pub.publish(out)
            with lock:
                state['clouds'] += 1
        except (ValueError, TypeError, OverflowError) as exc:
            rospy.logwarn_throttle(5, 'MID360 invalido: %s', exc)

    def receive_imu(msg):
        with lock:
            state['imu'] += 1

    def receive_pose(msg):
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        stamp = msg.header.stamp.to_sec()
        lag = (rospy.Time.now() - msg.header.stamp).to_sec()
        with lock:
            if state['last_stamp'] and stamp < state['last_stamp']:
                state['clock_error'] = True
                return
            if stamp <= 0 or lag < -.1 or lag > args.max_lag:
                state['dropped'] += 1
                return
            try:
                packet, state['first_rotation'] = pose_payload(
                    [p.x, p.y, p.z], [q.w, q.x, q.y, q.z], state['first_rotation'],
                    msg.header.stamp.to_nsec() // 1000, state['session'], state['poses'] + 1)
            except ValueError:
                state['dropped'] += 1
                return
            state['last_stamp'] = stamp
            state['poses'] += 1
            values = PACKET.unpack(packet)
            state['last_pose'] = list(values[4:11])
            if mav is not None and mav.clients:
                mav.mav.data64_send(DATA_TYPE, len(packet), list(packet.ljust(64, b'\0')))
                state['sent'] += 1

    subscribers = [rospy.Subscriber(imu_topic, Imu, receive_imu, queue_size=50),
                   rospy.Subscriber('/fast_lio/odometry', Odometry, receive_pose, queue_size=2)]
    env = os.environ.copy()
    env['ROS_NAMESPACE'] = '/fast_lio'
    env['LD_LIBRARY_PATH'] = '/home/grin/scanner_ws/devel/lib:' + env.get('LD_LIBRARY_PATH', '')
    command = [str(backend), '__name:=laserMapping', '/Odometry:=/fast_lio/odometry',
               '/path:=/fast_lio/path', '/cloud_registered:=/fast_lio/cloud_registered',
               '/cloud_registered_body:=/fast_lio/cloud_registered_body',
               '/cloud_registered_effect_world:=/fast_lio/cloud_effect']
    log = (output/'fast_lio.log').open('w')
    proc = None
    result = 0
    try:
        proc = subprocess.Popen(command, env=env, cwd=str(output), stdout=log, stderr=subprocess.STDOUT)
        subscribers.append(subscribe_cloud(rospy, cloud_topic, receive_cloud, 30))
        print('MID360:', cloud_topic, '| IMU:', imu_topic, flush=True)
        print('Odometria: /fast_lio/odometry | Lua via MAVProxy:', args.mavproxy, flush=True)
        print('Mantenha o rover parado durante a inicializacao da IMU. Log:', output, flush=True)
        start, report_at = time.monotonic(), 0
        while not rospy.is_shutdown():
            now = time.monotonic()
            if proc.poll() is not None:
                raise RuntimeError('FAST-LIO encerrou; confira %s' % (output/'fast_lio.log'))
            with lock:
                if state['clock_error']:
                    raise RuntimeError('Relogio ROS voltou; reinicie FAST-LIO e o receptor Lua')
                if mav is not None:
                    for _ in range(100):
                        if mav.recv_match(blocking=False) is None: break
                summary = (state['clouds'], state['imu'], state['poses'], state['sent'],
                           state['dropped'], state['last_pose'])
            if now >= report_at:
                print('nuvens=%d imu=%d poses=%d enviadas=%d descartadas=%d pose_FRD=%s' % summary, flush=True)
                report_at = now + 5
            if args.duration and now-start >= args.duration:
                if not state['poses']:
                    print('Nenhuma pose valida foi recebida durante o teste.', file=sys.stderr)
                    result = 1
                break
            time.sleep(.05)
    except (RuntimeError, TimeoutError, ValueError, OSError) as exc:
        print(str(exc), file=sys.stderr); result = 1
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subscribers: sub.unregister()
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            try: proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                proc.terminate()
                try: proc.wait(timeout=3)
                except subprocess.TimeoutExpired: proc.kill(); proc.wait()
        if mav is not None: mav.close()
        log.close()
        with lock:
            saved = {k: v for k, v in state.items() if k != 'first_rotation'}
        import json
        (output/'status.json').write_text(json.dumps(saved, indent=2))
        rospy.signal_shutdown('FAST-LIO encerrado')
    return result


if __name__ == '__main__':
    sys.exit(main())
