"""One monitor-only entry for sensing, mapping, perception and Nav2 preview."""
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from dog_bringup.configuration import PROFILES, project_root, read_site, resolve, create_run, prepare_deployment
from dog_bringup.preflight import check, acquire_locks


def start(context):
    mode = LaunchConfiguration('mode').perform(context)
    if mode not in PROFILES:
        raise ValueError('mode must be one of: ' + ', '.join(PROFILES))
    root = project_root(LaunchConfiguration('project_root').perform(context))
    site, wanted = read_site(root), PROFILES[mode]
    acquire_locks(root, wanted)
    check(wanted, site['vision_port'])
    run = create_run(root, mode, LaunchConfiguration('run_dir').perform(context))
    nav_share = Path(get_package_share_directory('dog_navigation')) / 'launch'

    def include(file, values):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(str(nav_share / file)),
                                        launch_arguments={k: str(v) for k, v in values.items()}.items())

    actions = [LogInfo(msg=f'dog_final {mode}: {run}; motor writes disabled')]
    if 'imu' in wanted:
        actions.append(Node(package='dm_imu', executable='dm_imu_node', name='dog_imu', output='screen',
                            parameters=[{'port': site['imu_port'], 'baudrate': 921600, 'slave_id': 13,
                                         'crc_mode': 'legacy_v1', 'imu_topic': '/dog_imu/data'}]))
    if 'controller' in wanted:
        config = prepare_deployment(root, run, site)
        actions.append(Node(package='dog_ik_walk', executable='dog_ik_walk_node', name='dog_ik_walk_imu',
                            output='screen', parameters=[{'deployment_config': str(config),
                                                         'allow_actuation': False}]))
    if 'lidar' in wanted:
        actions.append(include('dog_lidar.launch.py', {'config_path': resolve(root, site['lidar_config']),
                                                      'format': 'custom'}))
    if 'mapping' in wanted:
        actions.append(include('dog_mapping.launch.py', {
            'map_path': run / 'map.pcd', 'save_enabled': 'true' if 'grid' in wanted else 'false',
            'world_frame': 'dog_odom' if 'navigation' in wanted else 'dog_lio_map',
            'publish_tf': 'false' if 'navigation' in wanted else 'true'}))
    if 'grid' in wanted:
        actions.append(include('dog_grid_projection.launch.py', {'resolution': site['grid_resolution'],
            'min_z': site['grid_min_z'], 'max_z': site['grid_max_z']}))
    if 'vision' in wanted:
        driver = Path(get_package_share_directory('depthai_ros_driver_v3')) / 'launch/driver.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(driver)),
            launch_arguments={'name': 'dog_oak', 'namespace': '/dog_vision', 'parent_frame': 'dog_oak_mount',
                              'params_file': str(resolve(root, site['camera_config'])), 'use_rviz': 'false'}.items()))
        python = root / '.venv/bin/python'
        if not python.is_file():
            python = Path(site['vision_python'])
        if not python.is_file():
            raise FileNotFoundError('Vision Python missing; see config/site.json and ./dog doctor')
        actions.append(Node(package='dog_perception', executable='dog_yolo', name='dog_custom_yolo',
            namespace='/dog_vision', output='screen', prefix=str(python),
            arguments=['--model', str(resolve(root, site['model'])), '--directory', str(run / 'vision'),
                       '--port', str(site['vision_port'])]))
    if 'plan' in wanted:
        map_yaml = LaunchConfiguration('map_yaml').perform(context) or str(resolve(root, site['map_yaml']))
        actions.append(include('dog_planning_offline.launch.py', {'map_yaml': map_yaml,
            'start_x': LaunchConfiguration('start_x').perform(context),
            'start_y': LaunchConfiguration('start_y').perform(context)}))
    if 'navigation' in wanted:
        map_yaml = LaunchConfiguration('map_yaml').perform(context) or str(resolve(root, site['map_yaml']))
        actions.append(include('dog_navigation.launch.py', {'map_yaml': map_yaml, 'live_sensors': 'true',
            'mounting_config': resolve(root, site['mounting_config']), 'allow_motor_output': 'false'}))
    if 'model' in wanted:
        file = Path(get_package_share_directory('dog_description')) / 'launch/display.launch.py'
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(str(file))))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mode', default_value='monitor', choices=list(PROFILES)),
        DeclareLaunchArgument('project_root', default_value=str(project_root())),
        DeclareLaunchArgument('run_dir', default_value='', description='NEW output directory; never reused'),
        DeclareLaunchArgument('map_yaml', default_value='', description='Optional saved map override'),
        DeclareLaunchArgument('start_x', default_value='0.6', description='Explicit fictional planning start X'),
        DeclareLaunchArgument('start_y', default_value='0.0', description='Explicit fictional planning start Y'),
        OpaqueFunction(function=start),
    ])
