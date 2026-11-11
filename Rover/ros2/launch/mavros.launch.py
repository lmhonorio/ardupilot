"""MAVROS with the same simulation clock as the sensors and SLAM."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share=Path(get_package_share_directory('mavros'))/'launch'
    root=Path(__file__).resolve().parents[1]
    return LaunchDescription([
        DeclareLaunchArgument('fcu_url',default_value='udp://:14555@'),
        DeclareLaunchArgument('namespace',default_value='/rover_argo_N1/Instance1'),
        DeclareLaunchArgument('tgt_system',default_value='1'),
        Node(package='mavros',executable='mavros_node',namespace=LaunchConfiguration('namespace'),
             parameters=[str(share/'apm_pluginlists.yaml'),str(share/'apm_config.yaml'),
                         str(root/'config/mavros_sim.yaml'),
                         {'fcu_url':LaunchConfiguration('fcu_url'),'gcs_url':'',
                          'tgt_system':ParameterValue(LaunchConfiguration('tgt_system'),value_type=int),
                          'tgt_component':1,'fcu_protocol':'v2.0'}],output='screen')])
