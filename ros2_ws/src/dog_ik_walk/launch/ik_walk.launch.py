from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('deployment_config'),
        DeclareLaunchArgument('allow_actuation',default_value='false'),
        Node(package='dog_ik_walk',executable='dog_ik_walk_node',name='dog_ik_walk',output='screen',
             parameters=[{'deployment_config':LaunchConfiguration('deployment_config'),
                          'allow_actuation':ParameterValue(LaunchConfiguration('allow_actuation'),value_type=bool)}])])
