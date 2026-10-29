#!/usr/bin/python3
"""Read and display the simulated MID360 ROS cloud; never commands the rover."""
import argparse
import json
from pathlib import Path
import sys
import threading
import time

import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from common.ros_cloud import cloud_xyz, subscribe_cloud


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--namespace', default='/rover_argo_N1/Instance1')
    parser.add_argument('--topic', help='Topico PointCloud ou PointCloud2')
    parser.add_argument('--list', action='store_true')
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--snapshot', type=Path, help='Salvar primeiro frame valido em NPZ e sair')
    parser.add_argument('--save', type=Path, help='Salvar primeiro frame em PNG e sair')
    parser.add_argument('--timeout', type=float, default=30)
    parser.add_argument('--max-points', type=int, default=10000)
    parser.add_argument('--min-range', type=float, default=.1)
    parser.add_argument('--max-range', type=float, default=40, help='Alcance exibido/salvo em metros')
    parser.add_argument('--z-scale', type=float, default=3, help='Ampliacao visual de Z; 1 = escala fisica')
    args = parser.parse_args()
    if args.timeout < 0 or args.max_points < 100 or not 0 <= args.min_range < args.max_range or args.z_scale <= 0:
        parser.error('timeout >= 0; max-points >= 100; 0 <= min-range < max-range')
    if args.snapshot and args.snapshot.suffix.lower() != '.npz':
        parser.error('--snapshot deve terminar em .npz')
    if args.save and args.save.suffix.lower() != '.png':
        parser.error('--save deve terminar em .png')
    import rospy
    try:
        rospy.get_master().getPid()
    except Exception as exc:
        print('Inicie o roslaunch primeiro:', exc, file=sys.stderr)
        return 1
    rospy.init_node('rover_lidar_reader', anonymous=True)
    if args.list:
        for topic, kind in rospy.get_published_topics('/'):
            if kind in ('sensor_msgs/PointCloud', 'sensor_msgs/PointCloud2'):
                print(topic, kind)
        return 0
    topic = args.topic or '/' + args.namespace.strip('/') + '/livox/lidar'
    latest = {'xyz': None, 'count': 0, 'time': time.monotonic(), 'metadata': {}}
    lock = threading.Lock()

    def receive(message):
        try:
            xyz = cloud_xyz(message)
            distances = np.linalg.norm(xyz, axis=1)
            xyz = xyz[(distances >= args.min_range) & (distances <= args.max_range)]
        except (ValueError, TypeError) as exc:
            rospy.logwarn_throttle(5, 'Nuvem invalida: %s', exc)
            return
        if not len(xyz):
            return
        metadata = {'frame_id': message.header.frame_id, 'sec': message.header.stamp.secs,
                    'nsec': message.header.stamp.nsecs, 'topic': topic, 'type': message._type,
                    'min_range': args.min_range, 'max_range': args.max_range}
        with lock:
            latest.update(xyz=xyz, count=latest['count'] + 1, time=time.monotonic(), metadata=metadata)

    subscriber = None
    plt = fig = None
    print('Lendo', topic, flush=True)
    try:
        subscriber = subscribe_cloud(rospy, topic, receive, args.timeout)
        if (not args.headless and not args.snapshot) or args.save:
            import matplotlib
            if args.save:
                matplotlib.use('Agg')
            import matplotlib.pyplot as plt
            from matplotlib.ticker import MaxNLocator
            fig = plt.figure(figsize=(10, 7))
            fig.canvas.manager.set_window_title('MID360 — nuvem ROS')
            ax = fig.add_subplot(111, projection='3d')
        last_drawn = -1
        report_at = 0
        while not rospy.is_shutdown():
            with lock:
                xyz, count, received, meta = latest['xyz'], latest['count'], latest['time'], latest['metadata']
            now = time.monotonic()
            if args.timeout and now - received > args.timeout:
                raise TimeoutError('Sem nuvem valida por %.1f s; confira topico e Gazebo' % args.timeout)
            if xyz is not None:
                if args.snapshot:
                    args.snapshot.parent.mkdir(parents=True, exist_ok=True)
                    np.savez_compressed(str(args.snapshot), xyz=xyz, metadata=json.dumps(meta))
                    print('Nuvem salva:', args.snapshot.resolve())
                    return 0
                if now >= report_at:
                    print('Frames=%d pontos=%d frame=%s alcance=%.2f..%.2f m' %
                          (count, len(xyz), meta['frame_id'], np.linalg.norm(xyz, axis=1).min(),
                           np.linalg.norm(xyz, axis=1).max()), flush=True)
                    report_at = now + 2
                if fig is not None and count != last_drawn:
                    elevation, azimuth = ax.elev, ax.azim
                    ax.clear()
                    sampled = xyz[::max(1, int(np.ceil(len(xyz) / args.max_points)))]
                    ax.scatter(sampled[:, 0], sampled[:, 1], sampled[:, 2], c=sampled[:, 2],
                               cmap='viridis', s=2)
                    ax.scatter([0], [0], [0], color='red', marker='x')
                    ax.set(xlabel='X (m)', ylabel='Y (m)',
                           zlabel='Z (m; visual x%g)' % args.z_scale, title=meta['frame_id'])
                    low, high = np.minimum(xyz.min(axis=0), 0), np.maximum(xyz.max(axis=0), 0)
                    span = np.maximum(high - low, 1)
                    for setter, center, extent in zip((ax.set_xlim, ax.set_ylim, ax.set_zlim), (low + high) / 2, span):
                        setter(center - extent * .55, center + extent * .55)
                    ax.set_box_aspect(span * [1, 1, args.z_scale])
                    ax.zaxis.set_major_locator(MaxNLocator(4))
                    ax.view_init(elev=elevation, azim=azimuth)
                    last_drawn = count
                    if args.save:
                        args.save.parent.mkdir(parents=True, exist_ok=True)
                        fig.savefig(str(args.save), dpi=150)
                        print('Imagem salva:', args.save.resolve())
                        return 0
            if fig is not None:
                if not plt.fignum_exists(fig.number):
                    return 0
                plt.pause(.1)
            else:
                time.sleep(.05)
        return 0
    except (RuntimeError, TimeoutError, ValueError) as exc:
        print(str(exc), file=sys.stderr)
        return 1
    finally:
        if subscriber:
            subscriber.unregister()
        if plt is not None:
            plt.close('all')
        rospy.signal_shutdown('Leitor LiDAR encerrado')


if __name__ == '__main__':
    sys.exit(main())
