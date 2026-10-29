#!/usr/bin/python3
"""MID360 -> height slice -> LaserScan -> Hector 2D SLAM, without Gazebo ground truth."""
import argparse
from collections import deque
import csv
from datetime import datetime
import json
import math
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.ros_cloud import cloud_xyz, project_scan, subscribe_cloud
from install_backend import find_backend, check_backend
from map_io import occupancy_image, save_map


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/rover_argo_N1/Instance1')
    parser.add_argument('--topic', help='Topico MID360 PointCloud ou PointCloud2')
    parser.add_argument('--slam-namespace', default='/rover_slam')
    parser.add_argument('--headless', action='store_true', help='Nao abrir janela do mapa')
    parser.add_argument('--output-dir', type=Path)
    parser.add_argument('--resolution', type=float, default=.1)
    parser.add_argument('--map-size', type=int, default=1024, help='Lado do mapa em celulas')
    parser.add_argument('--bins', type=int, default=360)
    parser.add_argument('--min-height', type=float, default=-.2, help='Altura no referencial do LiDAR (m)')
    parser.add_argument('--max-height', type=float, default=.8)
    parser.add_argument('--min-range', type=float, default=.35)
    parser.add_argument('--max-range', type=float, default=30)
    parser.add_argument('--min-hits', type=int, default=10)
    parser.add_argument('--accumulate', type=int, default=5,
                        help='Nuvens consecutivas por scan; MID360 esparso: padrao 5')
    parser.add_argument('--lidar-x', type=float, default=-.003, help='Sensor em relacao ao base_link (m)')
    parser.add_argument('--lidar-y', type=float, default=0)
    parser.add_argument('--lidar-z', type=float, default=.434)
    parser.add_argument('--timeout', type=float, default=30, help='Tempo sem scans validos; 0 espera indefinidamente')
    parser.add_argument('--duration', type=float, default=0, help='Encerrar apos N segundos de parede; 0 = continuo')
    args = parser.parse_args()
    if (not .01 <= args.resolution <= .5 or not 128 <= args.map_size <= 4096
            or args.bins < 36 or not 0 < args.min_range < args.max_range
            or not args.min_height < args.max_height or not 3 <= args.min_hits <= args.bins
            or not 1 <= args.accumulate <= 20 or args.timeout < 0 or args.duration < 0):
        parser.error('Limites de mapa/projecao/timeout invalidos')
    return args


def main():
    args = arguments()
    backend = find_backend()
    if backend is None:
        print('Hector ausente. Execute /usr/bin/python3.8 python/slam/install_backend.py', file=sys.stderr)
        return 1
    check_backend(backend)
    import rospy
    import tf2_ros
    from geometry_msgs.msg import TransformStamped, PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    try:
        rospy.get_master().getPid()
    except Exception as exc:
        print('Inicie o roslaunch primeiro:', exc, file=sys.stderr)
        return 1
    rospy.init_node('mid360_slam_reader', anonymous=True)
    ns = '/' + args.slam_namespace.strip('/')
    frame_prefix = ns.strip('/').replace('/', '_')
    base_frame, lidar_frame, map_frame = [frame_prefix + x for x in ('_base', '_lidar', '_map')]
    node_name = ns + '/hector_mapping'
    import rosnode
    if node_name in rosnode.get_node_names():
        print('SLAM ja iniciado em %s; use outro --slam-namespace ou encerre o anterior' % ns, file=sys.stderr)
        return 1
    topic = args.topic or '/' + args.namespace.strip('/') + '/livox/lidar'
    output = args.output_dir or Path(__file__).resolve().parents[1] / 'run/slam' / datetime.now().strftime('%Y%m%d_%H%M%S_%f')
    output.mkdir(parents=True, exist_ok=True)
    lock = threading.Lock()
    state = {'map': None, 'pose': None, 'clouds': 0, 'scans': 0, 'hits': 0,
             'received': time.monotonic(), 'last_stamp': None, 'source_frame': None,
             'clock_error': False, 'closed': False}
    scan_pub = rospy.Publisher(ns + '/scan', LaserScan, queue_size=1)
    trajectory = (output / 'trajectory.csv').open('w', newline='')
    writer = csv.writer(trajectory)
    writer.writerow(['stamp_s', 'x_m', 'y_m', 'yaw_rad'])
    history = deque(maxlen=args.accumulate)

    def receive_map(message):
        with lock:
            state['map'] = message

    def receive_pose(message):
        p, q = message.pose.position, message.pose.orientation
        yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        with lock:
            state['pose'] = (p.x, p.y, yaw)
            writer.writerow([message.header.stamp.to_sec(), p.x, p.y, yaw])
            trajectory.flush()

    def receive_cloud(message):
        try:
            xyz = cloud_xyz(message)
            history.append(xyz)
            if len(history) < args.accumulate:
                return
            xyz = np.concatenate(list(history), axis=0)
            ranges = project_scan(xyz, args.bins, args.min_height, args.max_height,
                                  args.min_range, args.max_range)
        except (ValueError, TypeError) as exc:
            rospy.logwarn_throttle(5, 'Projecao MID360 falhou: %s', exc)
            return
        stamp = message.header.stamp.to_sec()
        hits = int(np.isfinite(ranges).sum())
        with lock:
            state['clouds'] += 1
            state['hits'] = hits
            state['source_frame'] = message.header.frame_id
            if state['last_stamp'] is not None and stamp < state['last_stamp']:
                state['clock_error'] = True
                return
            state['last_stamp'] = stamp
        if hits < args.min_hits:
            rospy.logwarn_throttle(5, 'MID360: apenas %d bins com obstaculos; minimo %d', hits, args.min_hits)
            return
        scan = LaserScan()
        scan.header.stamp, scan.header.frame_id = message.header.stamp, lidar_frame
        scan.angle_increment = 2 * math.pi / args.bins
        scan.angle_min = -math.pi + scan.angle_increment / 2
        scan.angle_max = scan.angle_min + (args.bins - 1) * scan.angle_increment
        scan.range_min, scan.range_max = args.min_range, args.max_range
        # The plugin does not supply per-point times: do not invent motion deskew.
        scan.time_increment = 0
        scan.scan_time = 0.1
        scan.ranges = ranges.tolist()
        scan_pub.publish(scan)
        with lock:
            state['scans'] += 1
            state['received'] = time.monotonic()

    transform = TransformStamped()
    transform.header.stamp = rospy.Time.now()
    transform.header.frame_id, transform.child_frame_id = base_frame, lidar_frame
    transform.transform.translation.x = args.lidar_x
    transform.transform.translation.y = args.lidar_y
    transform.transform.translation.z = args.lidar_z
    transform.transform.rotation.w = 1
    static_tf = tf2_ros.StaticTransformBroadcaster()
    static_tf.sendTransform(transform)
    subscriptions = [rospy.Subscriber(ns + '/map', OccupancyGrid, receive_map, queue_size=1),
                     rospy.Subscriber(ns + '/slam_out_pose', PoseStamped, receive_pose, queue_size=1)]
    params = {'map_frame': map_frame, 'base_frame': base_frame, 'odom_frame': base_frame,
              'scan_topic': ns + '/scan', 'use_tf_scan_transformation': True,
              'use_tf_pose_start_estimate': False, 'map_with_known_poses': False,
              'pub_map_odom_transform': True, 'pub_map_scanmatch_transform': False,
              'pub_odometry': True, 'advertise_map_service': True,
              'map_resolution': args.resolution, 'map_size': args.map_size,
              'map_multi_res_levels': 3, 'map_pub_period': 1.0,
              'map_update_distance_thresh': .15, 'map_update_angle_thresh': .05,
              'laser_min_dist': args.min_range, 'laser_max_dist': args.max_range,
              'scan_subscriber_queue_size': 2}
    command = [str(backend), '__name:=hector_mapping', '__ns:=' + ns]
    command += ['_%s:=%s' % (k, str(v).lower() if isinstance(v, bool) else v) for k, v in params.items()]
    log = (output / 'hector.log').open('w')
    process = None
    plt = fig = None
    result = 0

    def save_current():
        with lock:
            grid = state['map']
        if grid is None or not any(value >= 0 for value in grid.data):
            return False
        paths = save_map(grid, output)
        print('Mapa salvo:', paths[1], flush=True)
        return True

    try:
        process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
        subscriptions.append(subscribe_cloud(rospy, topic, receive_cloud, args.timeout))
        print('MID360: %s\nMapa: %s/map | frame: %s\nSaida: %s' % (topic, ns, map_frame, output), flush=True)
        print('S salva mapa; Q/fechar janela/Ctrl+C salva e encerra. Nenhum comando de movimento e enviado.', flush=True)
        if not args.headless:
            import matplotlib.pyplot as plt
            fig, ax = plt.subplots(figsize=(10, 8))
            fig.canvas.manager.set_window_title('MID360 — SLAM 2D')
            image_artist = ax.imshow(np.full((1, 1), .8), origin='lower', cmap='gray', vmin=0, vmax=1)
            pose_artist, = ax.plot([], [], 'ro')
            ax.set(xlabel='X (m)', ylabel='Y (m)', title='Aguardando mapa do MID360')

            def key(event):
                if event.key in ('q', 'escape'):
                    state['closed'] = True
                elif event.key == 's':
                    save_current()
            fig.canvas.mpl_connect('key_press_event', key)
            fig.canvas.mpl_connect('close_event', lambda event: state.update(closed=True))
        started = time.monotonic()
        report_at = 0
        last_grid = None
        while not rospy.is_shutdown() and not state['closed']:
            now = time.monotonic()
            if process.poll() is not None:
                raise RuntimeError('Hector encerrou; veja %s' % (output / 'hector.log'))
            with lock:
                grid, pose = state['map'], state['pose']
                scans, clouds, hits = state['scans'], state['clouds'], state['hits']
                received, clock_error = state['received'], state['clock_error']
            if clock_error:
                raise RuntimeError('Relogio ROS voltou no tempo; reinicie o SLAM apos resetar o Gazebo')
            if args.timeout and now - received > args.timeout:
                raise TimeoutError('Sem scans validos por %.1f s; verifique topico, altura e obstaculos' % args.timeout)
            if args.duration and now - started >= args.duration:
                break
            if now >= report_at:
                print('Nuvens=%d scans=%d bins_validos=%d pose=%s' % (clouds, scans, hits, pose), flush=True)
                report_at = now + 5
            if fig is not None:
                if grid is not None and grid is not last_grid:
                    info = grid.info
                    image_artist.set_data(occupancy_image(grid) / 255.)
                    x, y = info.origin.position.x, info.origin.position.y
                    extent = [x, x + info.width*info.resolution, y, y + info.height*info.resolution]
                    image_artist.set_extent(extent)
                    ax.set_xlim(extent[:2]); ax.set_ylim(extent[2:])
                    ax.set_title('%s/map | %.2f m/celula' % (ns, info.resolution))
                    last_grid = grid
                if pose:
                    pose_artist.set_data([pose[0]], [pose[1]])
                plt.pause(.1)
            else:
                time.sleep(.1)
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        result = 1
    except KeyboardInterrupt:
        pass
    finally:
        for subscription in subscriptions:
            subscription.unregister()
        saved = save_current()
        if not saved:
            print('Nenhum mapa com celulas observadas foi produzido.', file=sys.stderr)
            result = 1
        summary = {'map_saved': saved, 'input_topic': topic, 'map_topic': ns+'/map',
                   'map_frame': map_frame, 'base_frame': base_frame, 'lidar_frame': lidar_frame,
                   'source_frame': state['source_frame'], 'clouds': state['clouds'], 'scans': state['scans'],
                   'pose': state['pose'], 'params': vars(args), 'backend': str(backend)}
        (output/'status.json').write_text(json.dumps(summary, indent=2, default=str))
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                process.terminate()
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    process.kill(); process.wait()
        log.close(); trajectory.close()
        if plt is not None:
            plt.close('all')
        rospy.signal_shutdown('SLAM encerrado')
    return result


if __name__ == '__main__':
    sys.exit(main())
