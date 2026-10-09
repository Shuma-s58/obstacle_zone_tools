"""Load the map and start the rectangle editor and RViz."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('obstacle_zone_tools')
    clock = ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)
    return LaunchDescription([
        DeclareLaunchArgument('map', description='Absolute path to the map YAML'),
        DeclareLaunchArgument('zones_file', default_value='~/obstacle_zones.yaml'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        Node(package='nav2_map_server', executable='map_server',
             name='obstacle_zone_map_server', output='screen', parameters=[{
                 'yaml_filename': LaunchConfiguration('map'),
                 'frame_id': 'map', 'topic_name': 'map', 'use_sim_time': clock,
             }]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='obstacle_zone_map_lifecycle_manager', output='screen', parameters=[{
                 'autostart': True, 'node_names': ['obstacle_zone_map_server'],
                 'use_sim_time': clock,
             }]),
        Node(package='obstacle_zone_tools', executable='obstacle_zone_editor',
             name='obstacle_zone_node', output='screen', parameters=[{
                 'zones_file': LaunchConfiguration('zones_file'),
                 'frame_id': 'map', 'use_sim_time': clock,
             }]),
        Node(package='rviz2', executable='rviz2', name='obstacle_zone_rviz',
             output='screen', arguments=['-d', os.path.join(share, 'rviz', 'edit.rviz')],
             parameters=[{'use_sim_time': clock}]),
    ])
