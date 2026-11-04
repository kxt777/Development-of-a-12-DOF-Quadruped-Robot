"""Sensor-only launch. No motor nodes, static mounting guesses, or navigation goals."""
import json
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start_driver(context):
    config = Path(LaunchConfiguration('config_path').perform(context)).expanduser().resolve()
    data = json.loads(config.read_text())
    if not any(key in data for key in ('MID360', 'Mid360s')) or not data.get('lidar_configs'):
        raise ValueError('Expected a MID360 or MID360S SDK2 configuration')
    fmt = LaunchConfiguration('format').perform(context)
    if fmt not in ('pointcloud2', 'custom'):
        raise ValueError('format must be pointcloud2 or custom')
    return [Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='dog_lidar', output='screen',
        parameters=[{
            'xfer_format': 0 if fmt == 'pointcloud2' else 1,
            'multi_topic': 0, 'data_src': 0, 'publish_freq': 10.0,
            'output_data_type': 0, 'frame_id': 'dog_lidar_link',
            'user_config_path': str(config),
            'cmdline_input_bd_code': 'livox0000000001',
            'lvx_file_path': '',
        }],
        remappings=[
            ('livox/lidar', '/dog_lidar/points' if fmt == 'pointcloud2' else '/dog_lidar/custom'),
            ('livox/imu', '/dog_lidar/imu'),
        ],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('config_path', description='Absolute path to verified SDK2 network JSON'),
        DeclareLaunchArgument('format', default_value='pointcloud2',
                              description='pointcloud2 for RViz/audit; custom retains Livox per-point timing for LIO'),
        OpaqueFunction(function=start_driver),
    ])
