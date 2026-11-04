"""Live sensing + localization + Nav2; starting this never enables motors."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription,OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def start(context):
    mounting=LaunchConfiguration('mounting_config').perform(context)
    mount=json.loads(Path(mounting).read_text())
    if not (mount.get('calibrated') or (mount.get('orientation_confirmed') and mount.get('translation_assumption_accepted'))):
        raise ValueError('Confirm the mounting convention before live navigation')
    share=Path(get_package_share_directory('dog_navigation'))/'launch'
    def include(file,args):return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(share/file)),launch_arguments=args.items())
    return [include('dog_lidar.launch.py',{'config_path':LaunchConfiguration('lidar_config'),'format':'custom'}),
            include('dog_mapping.launch.py',{'world_frame':'dog_odom','publish_tf':'false','save_enabled':'false',
                'map_path':'/tmp/dog_navigation_unsaved_map.pcd'}),
            include('dog_navigation.launch.py',{'live_sensors':'true','mounting_config':mounting,
                'map_yaml':LaunchConfiguration('map_yaml'),'allow_motor_output':LaunchConfiguration('allow_motor_output')})]


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('mounting_config'),DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('lidar_config'),DeclareLaunchArgument('allow_motor_output',default_value='false'),
        OpaqueFunction(function=start)])
