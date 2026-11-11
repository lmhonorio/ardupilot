#!/usr/bin/env python3
"""ROS 2 integration entry point; run in the isolated validation container.

Starts Gazebo, SITL, sensors and SLAM. Does not arm unless --drive is explicit.
The former ROS 1 synthetic test is retained in backups/ros1_antes_ros2_20261109.
"""
import runpy
from pathlib import Path

if __name__ == '__main__':
    runpy.run_path(str(Path(__file__).resolve().parents[1] / 'simulation/ros2/validate_stack.py'),
                   run_name='__main__')
