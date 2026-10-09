"""Run the saved zones against an already running localization and Nav2."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('obstacle_zone_tools')
    return LaunchDescription([
        DeclareLaunchArgument('zones_file', default_value='~/obstacle_zones.yaml'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(share, 'config', 'switch_params.yaml')),
        DeclareLaunchArgument('robot_frame', default_value='base_link'),
        DeclareLaunchArgument('exit_margin', default_value='0.0'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='obstacle_zone_tools', executable='obstacle_zone_runner',
             name='obstacle_zone_node', output='screen', parameters=[
                 LaunchConfiguration('params_file'), {
                     'zones_file': LaunchConfiguration('zones_file'), 'frame_id': 'map',
                     'robot_frame': LaunchConfiguration('robot_frame'),
                     'exit_margin': ParameterValue(LaunchConfiguration('exit_margin'), value_type=float),
                     'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 }]),
    ])
