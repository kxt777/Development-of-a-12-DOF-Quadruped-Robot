"""Configure upstream octomap_server's 2D projection; no footprint or inflation.

Input is FAST-LIO's deskewed cloud in the IMU frame, with its matching TF.
The height band is relative to dog_lio_map, NOT measured height above ground.
The same launch works with live FAST-LIO or offline bag replay.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('resolution', default_value='0.05'),
        DeclareLaunchArgument('min_z', default_value='0.10',
                              description='Projection lower Z in initial LIO frame, meters'),
        DeclareLaunchArgument('max_z', default_value='1.50',
                              description='Projection upper Z in initial LIO frame, meters'),
        Node(package='octomap_server', executable='octomap_server_node',
             name='dog_octomap', namespace='dog_grid', output='screen',
             parameters=[{
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
                 'frame_id': 'dog_lio_map', 'base_frame_id': 'dog_lidar_imu',
                 'resolution': ParameterValue(LaunchConfiguration('resolution'), value_type=float),
                 'occupancy_min_z': ParameterValue(LaunchConfiguration('min_z'), value_type=float),
                 'occupancy_max_z': ParameterValue(LaunchConfiguration('max_z'), value_type=float),
                 'filter_ground_plane': False, 'filter_speckles': False,
                 'publish_free_space': True, 'latch': True,
                 'incremental_2D_projection': False,
             }],
             remappings=[('cloud_in', '/dog_mapping/cloud_body'),
                         ('projected_map', '/dog_grid/map')]),
    ])
