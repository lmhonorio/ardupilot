"""Read ROS PointCloud/PointCloud2 and project valid obstacle returns to 2D."""
import math
import time

import numpy as np


def cloud_xyz(message):
    if message._type == 'sensor_msgs/PointCloud':
        xyz = np.array([(p.x, p.y, p.z) for p in message.points], dtype=np.float32).reshape(-1, 3)
    elif message._type == 'sensor_msgs/PointCloud2':
        from sensor_msgs import point_cloud2
        xyz = np.array(list(point_cloud2.read_points(
            message, field_names=('x', 'y', 'z'), skip_nans=True)), dtype=np.float32).reshape(-1, 3)
    else:
        raise ValueError('Expected PointCloud or PointCloud2, got %s' % message._type)
    return xyz[np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 0.001)]


def subscribe_cloud(rospy, topic, callback, timeout=30):
    """Discover the message type before subscribing, without guessing its format."""
    from sensor_msgs.msg import PointCloud, PointCloud2
    types = {'sensor_msgs/PointCloud': PointCloud, 'sensor_msgs/PointCloud2': PointCloud2}
    deadline = time.monotonic() + timeout if timeout else math.inf
    while not rospy.is_shutdown():
        published = dict(rospy.get_published_topics('/'))
        if topic in published:
            if published[topic] not in types:
                raise ValueError('%s has unsupported type %s' % (topic, published[topic]))
            return rospy.Subscriber(topic, types[published[topic]], callback,
                                    queue_size=1, buff_size=32 * 1024 * 1024)
        if time.monotonic() >= deadline:
            raise TimeoutError('Topico LiDAR nao anunciado: %s; habilite mid360_enabled:=true' % topic)
        time.sleep(0.1)
    raise RuntimeError('ROS encerrado durante descoberta do LiDAR')


def project_scan(xyz, bins=360, min_height=-0.2, max_height=0.8,
                 min_range=0.35, max_range=30.0):
    """Nearest horizontal return per bin; unobserved bins stay inf, never occupied/free guesses.

    Heights are in the LiDAR's local frame. The simulated sensor is ~0.434 m
    above the rover reference: the default slice excludes flat ground returns.
    """
    if bins < 36 or not min_height < max_height or not 0 < min_range < max_range:
        raise ValueError('Invalid projection bounds')
    xyz = np.asarray(xyz, dtype=float).reshape(-1, 3)
    distances = np.hypot(xyz[:, 0], xyz[:, 1])
    valid = (np.isfinite(xyz).all(axis=1) & (xyz[:, 2] >= min_height)
             & (xyz[:, 2] <= max_height) & (distances >= min_range)
             & (distances <= max_range))
    angles = np.arctan2(xyz[valid, 1], xyz[valid, 0])
    indices = np.floor((angles + np.pi) * bins / (2 * np.pi)).astype(int) % bins
    ranges = np.full(bins, np.inf, dtype=np.float32)
    np.minimum.at(ranges, indices, distances[valid])
    return ranges
