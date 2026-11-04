"""Upstream Nav2 planning only. Test TF represents an explicit start, not the dog."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = str(Path(get_package_share_directory('dog_navigation')) / 'config/dog_planner.yaml')
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('start_x', default_value='0.6'),
        DeclareLaunchArgument('start_y', default_value='0.0'),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='dog_planning_start_tf', namespace='dog_nav',
             arguments=['--x', LaunchConfiguration('start_x'), '--y', LaunchConfiguration('start_y'),
                        '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0',
                        '--frame-id', 'dog_lio_map', '--child-frame-id', 'dog_planning_start']),
        Node(package='nav2_map_server', executable='map_server',
             name='dog_map_server', namespace='dog_nav', output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'),
                          'frame_id': 'dog_lio_map', 'use_sim_time': False}]),
        Node(package='nav2_planner', executable='planner_server',
             name='dog_planner_server', namespace='dog_nav', output='screen', parameters=[config]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='dog_planning_lifecycle_manager', namespace='dog_nav', output='screen',
             parameters=[{'autostart': True, 'use_sim_time': False,
                          'node_names': ['dog_map_server', 'dog_planner_server']}]),
    ])
