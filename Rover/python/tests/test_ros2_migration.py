"""Transport-independent checks for ROS 2 docking timestamps and asset conversion."""
import importlib.util
import math
from pathlib import Path
import sys
import types
import unittest
import xml.etree.ElementTree as ET

PYTHON=Path(__file__).resolve().parents[1]
sys.path.insert(0,str(PYTHON))
from common import ros2_runtime as ros

class MigrationTests(unittest.TestCase):
    def test_stamp_precision(self):
        stamp=types.SimpleNamespace(sec=123,nanosec=456789123)
        self.assertEqual(ros.nanoseconds(stamp),123456789123)
        self.assertAlmostEqual(ros.seconds(stamp),123.456789123)

    def test_age_uses_ros_clock_not_wall_clock(self):
        original=ros._node
        ros._node=types.SimpleNamespace(get_clock=lambda:types.SimpleNamespace(now=lambda:types.SimpleNamespace(nanoseconds=100100000000)))
        try:
            self.assertAlmostEqual(ros.age(types.SimpleNamespace(sec=100,nanosec=0)),.1)
            self.assertAlmostEqual(ros.age(types.SimpleNamespace(sec=101,nanosec=0)),-.9)
        finally:ros._node=original

    def test_wall_and_ros_freshness_reject_stale_joint_feedback(self):
        feedback=ros.JointFeedback.__new__(ros.JointFeedback)
        feedback._data=None;feedback._wall=0
        with self.assertRaisesRegex(RuntimeError,'ausente/desatualizado'):
            feedback.position('pan')

    def test_pid_limits_are_ordered(self):
        path=PYTHON.parent/'ros2/generated/rover.urdf'
        if not path.exists():self.skipTest('Assets nao gerados')
        robot=ET.parse(path)
        for control in robot.findall('.//control'):
            self.assertLess(float(control.findtext('cmd_min')),float(control.findtext('cmd_max')))

    def test_autopilot_imu_specific_force_points_up_in_frd(self):
        path=PYTHON.parent/'ros2/generated/rover.urdf'
        if not path.exists():self.skipTest('Assets nao gerados')
        robot=ET.parse(path)
        sensor=robot.find(".//sensor[@name='imu_sensor']")
        pose=list(map(float,sensor.findtext('pose').split()))
        # Stationary Gazebo sensor reads +g along world up. Aircraft FRD must
        # receive -g on Z; otherwise EKF tilt initialization is upside-down.
        self.assertAlmostEqual(9.8*math.cos(pose[3])*math.cos(pose[4]),-9.8)

if __name__=='__main__':unittest.main()
