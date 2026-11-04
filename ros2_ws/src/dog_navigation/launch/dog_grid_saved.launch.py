"""Load a saved occupancy grid with upstream Nav2 map_server; no planning/control."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', description='Existing map_saver YAML file'),
        Node(package='nav2_map_server', executable='map_server', name='dog_map_server',
             namespace='dog_grid', output='screen',
             parameters=[{'yaml_filename': LaunchConfiguration('map_yaml'),
                          'frame_id': 'dog_lio_map', 'use_sim_time': False}]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='dog_grid_lifecycle_manager', namespace='dog_grid', output='screen',
             parameters=[{'autostart': True, 'node_names': ['dog_map_server'],
                          'use_sim_time': False}]),
    ])
