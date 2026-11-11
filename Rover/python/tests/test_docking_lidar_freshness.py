"""A ROS topic may remain registered after gzserver dies."""
import pathlib
import sys
import threading
import time
import types
import unittest
from unittest.mock import patch

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parents[1] / 'docking_lidar'))
from sensores import Sensores


class Freshness(unittest.TestCase):
    def sensor(self):
        s = Sensores.__new__(Sensores)
        s.lock = threading.Lock()
        s.use_sim_time = True
        s.clock_wall = time.monotonic()
        s.clock_value = 10.
        s.cloud_wall = time.monotonic()
        s.topic = '/livox/lidar'
        s.sub = types.SimpleNamespace(get_num_connections=lambda: 1)
        s.cloud = types.SimpleNamespace(header=types.SimpleNamespace(
            frame_id='mid360_sensor', stamp=10.))
        return s

    def test_registered_topic_but_dead_clock(self):
        s = self.sensor()
        s.clock_wall -= 3
        with self.assertRaisesRegex(ValueError, '/clock sem avancar'):
            s.fresh_cloud()

    def test_clock_running_but_no_cloud(self):
        s = self.sensor()
        s.cloud = None
        with self.assertRaisesRegex(ValueError, 'MID360 sem mensagens.*conexoes: 1'):
            s.fresh_cloud()

    def test_frozen_clock_messages_do_not_refresh_progress(self):
        s = self.sensor()
        s.clock_wall -= 3
        s.clock_callback(types.SimpleNamespace(clock=types.SimpleNamespace(sec=10,nanosec=0)))
        with self.assertRaisesRegex(ValueError, '/clock sem avancar'):
            s.fresh_cloud()

    def test_gazebo_restart_discards_old_pose_reference(self):
        s = self.sensor()
        s.odom = object()
        s.odom_wall = time.monotonic()
        s.clock_callback(types.SimpleNamespace(clock=types.SimpleNamespace(sec=1,nanosec=0)))
        self.assertIsNone(s.cloud)
        self.assertIsNone(s.odom)
        with self.assertRaisesRegex(ValueError, 'Relogio /clock reiniciado'):
            s.fresh_cloud()
        with self.assertRaisesRegex(ValueError, 'Relogio /clock reiniciado'):
            s.lio()

    def test_fresh_cloud_and_frame_validation(self):
        s = self.sensor()
        class Now:
            def __sub__(self, stamp):
                return types.SimpleNamespace(to_sec=lambda: .05)
        with patch('sensores.ros.age', return_value=.05):
            self.assertIs(s.fresh_cloud(), s.cloud)
            s.cloud.header.frame_id = 'wrong_frame'
            with self.assertRaisesRegex(ValueError, 'Frame MID360 inesperado'):
                s.fresh_cloud()

    def test_recent_arrival_with_old_or_future_stamp_is_rejected(self):
        s = self.sensor()
        for age in (2., -.2):
            class Now:
                def __sub__(self, stamp):
                    return types.SimpleNamespace(to_sec=lambda: age)
            with patch('sensores.ros.age', return_value=age):
                with self.assertRaisesRegex(ValueError, 'Timestamp da nuvem invalido'):
                    s.fresh_cloud()


if __name__ == '__main__':
    unittest.main()
