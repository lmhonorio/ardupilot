"""Check the simulator/GLIM contract, especially frame and point timing."""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import numpy as np
from geometry_msgs.msg import Point32
from sensor_msgs.msg import PointCloud
from sensor_msgs_py import point_cloud2
from builtin_interfaces.msg import Time

HERE = Path(__file__).resolve().parents[1] / 'slam/glim'
sys.path.insert(0, str(HERE))
from run_glim import make_cloud, prepare_config


class GlimAdapterTests(unittest.TestCase):
    def test_preserves_xyz_and_scan_time_without_invented_sweep(self):
        msg = PointCloud()
        msg.header.stamp = Time(sec=12, nanosec=345678901)
        msg.points = [Point32(x=.9, y=0., z=0.), Point32(x=0., y=0., z=1.9),
                      Point32(x=0., y=0., z=0.), Point32(x=float('nan'), y=0., z=0.)]
        result = make_cloud(msg)
        xyzit = point_cloud2.read_points_numpy(result)
        self.assertEqual(result.header.stamp, msg.header.stamp)
        self.assertEqual(result.header.frame_id, 'glim_lidar')
        np.testing.assert_allclose(xyzit[:, :3], [[.9, 0, 0], [0, 0, 1.9]], atol=1e-6)
        np.testing.assert_array_equal(xyzit[:, 4], 0)

    def test_config_cpu_loop_closure_and_gazebo_imu_units(self):
        with tempfile.TemporaryDirectory() as folder:
            dest = prepare_config(Path(folder), '/test/imu')
            config = json.loads((dest/'config.json').read_text())['global']
            self.assertEqual(config['config_odometry'], 'config_odometry_cpu.json')
            self.assertEqual(config['config_global_mapping'], 'config_global_mapping_pose_graph.json')
            ros = json.loads((dest/'config_ros.json').read_text())['glim_ros']
            self.assertEqual(ros['imu_topic'], '/test/imu')
            self.assertEqual(ros['acc_scale'], 1.)  # Gazebo already publishes m/s²
            self.assertTrue(ros['enable_global_mapping'])
            sensors = json.loads((dest/'config_sensors.json').read_text())['sensors']
            self.assertTrue(sensors['global_shutter_lidar'])
            self.assertEqual(sensors['T_lidar_imu'], [0, 0, 0, 0, 0, 0, 1])
            self.assertNotEqual(ros['odom_frame_id'], 'camera_init')
            self.assertNotEqual(ros['imu_frame_id'], 'body')



if __name__ == '__main__':
    unittest.main()
