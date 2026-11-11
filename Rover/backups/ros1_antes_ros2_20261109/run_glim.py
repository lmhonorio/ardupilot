#!/usr/bin/python3.8
"""GLIM CPU/ROS1 for the Gazebo MID360; no MAVLink or vehicle commands."""
import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import threading
import time

import numpy as np

HERE = Path(__file__).resolve().parent
PYTHON_DIR = HERE.parents[1]
sys.path.insert(0, str(PYTHON_DIR))
from common.ros_cloud import cloud_xyz, subscribe_cloud
from common.cloud3d import corrected_mid360
from install_glim import ROOT, PREFIX


def prepare_config(output, imu_topic):
    destination = output / 'config'
    shutil.copytree(HERE/'config', destination)
    path = destination/'config_ros.json'
    data = json.loads(path.read_text())
    data['glim_ros']['imu_topic'] = imu_topic
    path.write_text(json.dumps(data, indent=2)+'\n')
    return destination


def make_cloud(message):
    """Gazebo ray correction, XYZ preserved; all rays share the actual scan time."""
    from sensor_msgs.msg import PointCloud2, PointField
    xyz = corrected_mid360(cloud_xyz(message)).astype('<f4')
    values = np.zeros((len(xyz), 5), dtype='<f4')
    values[:, :3] = xyz
    values[:, 3] = 1.  # simulator has no reflectivity
    cloud = PointCloud2()
    cloud.header.stamp = message.header.stamp
    cloud.header.frame_id = 'glim_lidar'
    cloud.height = 1
    cloud.width = len(xyz)
    cloud.fields = [PointField(name=name, offset=4*i, datatype=PointField.FLOAT32, count=1)
                    for i, name in enumerate(('x', 'y', 'z', 'intensity', 'time'))]
    cloud.is_bigendian = False
    cloud.point_step = 20
    cloud.row_step = 20*len(xyz)
    cloud.is_dense = True
    cloud.data = values.tobytes()
    return cloud


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/rover_argo_N1/Instance1')
    parser.add_argument('--cloud-topic')
    parser.add_argument('--imu-topic')
    parser.add_argument('--duration', type=float, default=0)
    parser.add_argument('--output-dir', type=Path)
    args = parser.parse_args()
    if args.duration < 0:
        parser.error('--duration deve ser >= 0')
    binary = PREFIX/'lib/glim_ros/glim_rosnode'
    if not binary.exists():
        parser.error('Instale primeiro: /usr/bin/python3.8 python/slam/glim/install_glim.py')
    import rospy
    import rosnode
    from nav_msgs.msg import Odometry
    from sensor_msgs.msg import PointCloud2
    from std_msgs.msg import String
    rospy.init_node('glim_mid360_adapter', anonymous=True)
    if '/glim_rosnode' in rosnode.get_node_names():
        parser.error('GLIM ja esta rodando; encerre a instancia anterior')
    output = (args.output_dir or PYTHON_DIR/'run/glim'/datetime.now().strftime('%Y%m%d_%H%M%S_%f')).resolve()
    output.mkdir(parents=True, exist_ok=True)
    ns = '/'+args.namespace.strip('/')
    config = prepare_config(output, args.imu_topic or ns+'/livox/imu')
    pub = rospy.Publisher('/glim/input/points', PointCloud2, queue_size=2)
    status_pub = rospy.Publisher('/glim/status', String, queue_size=1, latch=True)
    lock = threading.RLock()
    state = dict(clouds=0, points=0, poses=0, maps=0, dropped=0, last_stamp=0.,
                 last_pose_stamp=0., pose_age=None, clock_reset=False, error='aguardando sensores')

    def receive(msg):
        stamp = msg.header.stamp.to_sec()
        with lock:
            if state['last_stamp'] and stamp < state['last_stamp']:
                state['clock_reset'] = True
                return
            lag = (rospy.Time.now()-msg.header.stamp).to_sec()
            if stamp <= 0 or lag < -.05 or lag > .5 or stamp <= state['last_stamp']:
                state['dropped'] += 1
                return
            state['last_stamp'] = stamp
        try:
            cloud = make_cloud(msg)
            if cloud.width < 20:
                with lock:
                    state['dropped'] += 1
                return
            with lock:
                state['clouds'] += 1
                state['points'] = cloud.width
            pub.publish(cloud)
        except (ValueError, TypeError) as exc:
            rospy.logwarn_throttle(5, 'Nuvem GLIM invalida: %s', exc)

    def pose(msg):
        with lock:
            p = msg.pose.pose.position
            q = msg.pose.pose.orientation
            if not np.isfinite([p.x, p.y, p.z, q.x, q.y, q.z, q.w]).all():
                state['error'] = 'odometria nao finita'
                return
            state['poses'] += 1
            state['last_pose_stamp'] = msg.header.stamp.to_sec()

    def mapped(msg):
        with lock:
            state['maps'] += 1

    # Subscribe before starting GLIM: upstream only publishes when there are consumers.
    subscribers = [rospy.Subscriber('/glim_ros/odom', Odometry, pose, queue_size=2),
                   rospy.Subscriber('/glim_ros/map', PointCloud2, mapped, queue_size=1)]
    env = os.environ.copy()
    env['LD_LIBRARY_PATH'] = str(PREFIX/'lib')+':/opt/ros/noetic/lib:/usr/lib/x86_64-linux-gnu'
    env['ROS_PACKAGE_PATH'] = str(PREFIX/'share')+':'+env.get('ROS_PACKAGE_PATH', '')
    env['OMP_NUM_THREADS'] = '2'
    log = (output/'glim.log').open('w')
    proc = None
    result = 0
    try:
        proc = subprocess.Popen([str(binary), '__name:=glim_rosnode', '_config_path:='+str(config),
                                 '_dump_path:='+str(output/'dump')], env=env, cwd=str(output),
                                stdout=log, stderr=subprocess.STDOUT)
        subscribers.append(subscribe_cloud(rospy, args.cloud_topic or ns+'/livox/lidar', receive, 30))
        print('GLIM CPU: /glim_ros/map | /glim_ros/odom | /glim_ros/pose', flush=True)
        print('Mantenha o rover parado por pelo menos 3 s para inicializar. Log:', output, flush=True)
        start = time.monotonic()
        report = 0
        while not rospy.is_shutdown():
            if proc.poll() is not None:
                raise RuntimeError('GLIM encerrou; consulte '+str(output/'glim.log'))
            with lock:
                if state['clock_reset']:
                    raise RuntimeError('Relogio reiniciado; reinicie GLIM para criar um novo mapa')
                state['pose_age'] = rospy.Time.now().to_sec()-state['last_pose_stamp'] if state['poses'] else None
                state['error'] = '' if state['poses'] and -.05 <= state['pose_age'] < 1. else 'sem odometria recente'
                if time.monotonic() >= report:
                    status_pub.publish(json.dumps(state))
                    print(json.dumps(state), flush=True)
                    report = time.monotonic()+5
            if args.duration and time.monotonic()-start >= args.duration:
                if not state['poses'] or state['error']:
                    result = 1
                break
            time.sleep(.1)
    except (RuntimeError, TimeoutError, OSError) as exc:
        print(str(exc), file=sys.stderr)
        result = 1
    except KeyboardInterrupt:
        pass
    finally:
        for sub in subscribers:
            sub.unregister()
        if proc is not None and proc.poll() is None:
            proc.send_signal(signal.SIGINT)
            # GLIM drains mapping queues and saves its graph during shutdown.
            try:
                proc.wait(timeout=45)
            except subprocess.TimeoutExpired:
                print('GLIM nao concluiu exportacao em 45 s; encerrando.', file=sys.stderr)
                proc.terminate()
                try:
                    proc.wait(timeout=5)
                except subprocess.TimeoutExpired:
                    proc.kill()
                    proc.wait()
                result = 1
        log.close()
        (output/'status.json').write_text(json.dumps(state, indent=2)+'\n')
        rospy.signal_shutdown('GLIM encerrado')
    return result


if __name__ == '__main__':
    sys.exit(main())
