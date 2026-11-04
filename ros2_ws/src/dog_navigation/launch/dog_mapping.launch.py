"""FAST-LIO2 using the verified MID-360 internal IMU. Sensor processing only."""
from pathlib import Path
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def start_mapping(context):
    output = Path(LaunchConfiguration('map_path').perform(context)).expanduser().resolve()
    if output.exists():
        raise ValueError('Use a new map_path to preserve previous maps')
    output.parent.mkdir(parents=True, exist_ok=True)
    # Internal extrinsics: hku-mars/FAST_LIO config/mid360.yaml.
    # Device broadcast verified type 9 (MID-360), serial 47MDM710020256.
    # This is lidar origin in IMU frame, not the mounting transform to the dog.
    params = {
        'world_frame': LaunchConfiguration('world_frame'),
        'publish.tf_en': ParameterValue(LaunchConfiguration('publish_tf'), value_type=bool),
        'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool),
        'common.lid_topic': '/dog_lidar/custom',
        'common.imu_topic': '/dog_lidar/imu',
        'common.time_sync_en': False,
        'common.time_offset_lidar_to_imu': 0.0,
        'mapping.extrinsic_T': [-0.011, -0.02329, 0.04412],
        'mapping.extrinsic_R': [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
        'mapping.extrinsic_est_en': False,
        # Algorithm initial settings retained from upstream MID360 config.
        # These are filtering/estimation choices, not hardware ratings.
        'feature_extract_enable': False, 'point_filter_num': 3, 'max_iteration': 3,
        'filter_size_surf': 0.5, 'filter_size_map': 0.5, 'cube_side_length': 1000.0,
        'mapping.acc_cov': 0.1, 'mapping.gyr_cov': 0.1,
        'mapping.b_acc_cov': 0.0001, 'mapping.b_gyr_cov': 0.0001,
        'mapping.fov_degree': 360.0, 'mapping.det_range': 100.0,
        'preprocess.lidar_type': 1, 'preprocess.scan_line': 4,
        'preprocess.blind': 0.5, 'preprocess.timestamp_unit': 3, 'preprocess.scan_rate': 10,
        'publish.path_en': True, 'publish.map_en': ParameterValue(LaunchConfiguration('save_enabled'),value_type=bool),
        'publish.effect_map_en': False, 'publish.scan_publish_en': True,
        'publish.dense_publish_en': True, 'publish.scan_bodyframe_pub_en': True,
        'runtime_pos_log_enable': False, 'pcd_save.pcd_save_en': ParameterValue(LaunchConfiguration('save_enabled'),value_type=bool),
        'pcd_save.interval': -1, 'map_file_path': str(output),
    }
    return [Node(
        package='fast_lio', executable='fastlio_mapping', name='dog_fastlio',
        output='screen', parameters=[params],
        remappings=[('/cloud_registered', '/dog_mapping/cloud_registered'),
                    ('/cloud_registered_body', '/dog_mapping/cloud_body'),
                    ('/cloud_effected', '/dog_mapping/effective_cloud'),
                    ('/Laser_map', '/dog_mapping/map_cloud'),
                    ('/Odometry', '/dog_mapping/odometry'),
                    ('/path', '/dog_mapping/trajectory'),
                    ('map_save', '/dog_mapping/save_map')],
    )]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('world_frame', default_value='dog_lio_map'),
        DeclareLaunchArgument('publish_tf', default_value='true'),
        DeclareLaunchArgument('save_enabled', default_value='true'),
        DeclareLaunchArgument('map_path', description='New PCD file path for the explicit save-map service'),
        OpaqueFunction(function=start_mapping),
    ])
