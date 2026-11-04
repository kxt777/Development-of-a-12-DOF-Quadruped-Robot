from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('deployment_config'),
        DeclareLaunchArgument('allow_actuation',default_value='false'),
        DeclareLaunchArgument('imu_port',default_value='/dev/serial/by-id/usb-DM-Tech_DM-IMU-L1_DMIMU20250212-if00'),
        Node(package='dm_imu',executable='dm_imu_node',name='dog_imu',output='screen',
             parameters=[{'port':LaunchConfiguration('imu_port'),'baudrate':921600,'slave_id':13,
                          'crc_mode':'legacy_v1','imu_topic':'/dog_imu/data'}]),
        Node(package='dog_ik_walk',executable='dog_ik_walk_node',name='dog_ik_walk_imu',output='screen',
             parameters=[{'deployment_config':LaunchConfiguration('deployment_config'),
                          'allow_actuation':ParameterValue(LaunchConfiguration('allow_actuation'),value_type=bool)}])])
