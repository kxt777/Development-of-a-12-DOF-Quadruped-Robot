"""Inspect a saved real map in RViz; never launches control or localization."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('pcd_path', description='Existing saved map PCD'),
        DeclareLaunchArgument('show_rviz', default_value='true'),
        Node(package='dog_navigation', executable='dog_saved_map', name='dog_saved_map',
             arguments=['--pcd', LaunchConfiguration('pcd_path')], output='screen'),
        Node(package='rviz2', executable='rviz2', name='dog_mapping_view',
             arguments=['-d', PathJoinSubstitution([FindPackageShare('dog_navigation'),
                                                   'config', 'dog_mapping.rviz'])],
             condition=IfCondition(LaunchConfiguration('show_rviz')), output='screen'),
    ])
