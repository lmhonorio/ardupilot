"""Bounded visualization map of registered XYZ; no localization on the PC."""
from collections import OrderedDict
import time

import numpy as np
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import PointCloud2, PointField


class MapView:
    def __init__(self, node, voxel=.1, maximum=300000):
        self.voxel = voxel
        self.maximum = maximum
        self.cells = OrderedDict()
        self.publisher = node.create_publisher(PointCloud2, '/lio_pi/map_pc', qos_profile_sensor_data)
        self.last = 0.0

    def add(self, raw):
        cloud = deserialize_message(raw, PointCloud2)
        fields = {f.name: f for f in cloud.fields}
        if cloud.header.frame_id != 'camera_init':
            raise ValueError('Nuvem registrada fora do frame camera_init')
        if any(fields[k].datatype != PointField.FLOAT32 for k in ('x', 'y', 'z')):
            raise ValueError('XYZ deve ser FLOAT32')
        dtype = np.dtype({'names': ['x', 'y', 'z'],
                          'formats': [('>' if cloud.is_bigendian else '<')+'f4']*3,
                          'offsets': [fields[k].offset for k in ('x', 'y', 'z')],
                          'itemsize': cloud.point_step})
        values = np.ndarray((cloud.height, cloud.width), dtype=dtype, buffer=cloud.data,
                            strides=(cloud.row_step, cloud.point_step))
        points = np.column_stack([values[k].ravel() for k in ('x', 'y', 'z')])
        points = points[np.isfinite(points).all(axis=1)]
        keys = np.floor(points/self.voxel).astype(np.int64)
        keys, indices = np.unique(keys, axis=0, return_index=True)
        for key, point in zip(keys, points[indices]):
            key = tuple(key)
            self.cells[key] = tuple(point)
            self.cells.move_to_end(key)
        while len(self.cells) > self.maximum:
            self.cells.popitem(last=False)
        if time.monotonic()-self.last < 1:
            return
        self.last = time.monotonic()
        result = PointCloud2()
        result.header = cloud.header
        result.height = 1
        result.width = len(self.cells)
        result.fields = [PointField(name=k, offset=i*4, datatype=PointField.FLOAT32, count=1)
                         for i, k in enumerate(('x', 'y', 'z'))]
        result.point_step = 12
        result.row_step = result.width*12
        result.is_dense = True
        result.data = np.asarray(list(self.cells.values()), dtype='<f4').tobytes()
        self.publisher.publish(result)
