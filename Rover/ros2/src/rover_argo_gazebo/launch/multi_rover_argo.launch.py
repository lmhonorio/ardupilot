"""ROS 2 counterpart of multi_rover_argo.launch."""
import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    root=Path(os.environ.get('ROVER_ROS2_ROOT',str(Path.home()/'ardupilot/Rover/ros2')))
    return LaunchDescription([
        DeclareLaunchArgument('world_name',default_value='gravel_plane'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(str(root/'launch/simulation.launch.py')),
                                launch_arguments={'world':LaunchConfiguration('world_name')}.items())])
