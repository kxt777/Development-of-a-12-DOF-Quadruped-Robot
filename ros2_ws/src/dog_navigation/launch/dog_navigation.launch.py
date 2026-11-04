"""Saved-map Nav2. Defaults to preview; never enables motors or sends a goal."""
import json
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node,SetParameter
from launch_ros.parameter_descriptions import ParameterValue


def start(context):
    share=Path(get_package_share_directory('dog_navigation'))
    config=str(share/'config/dog_navigation.yaml')
    live=LaunchConfiguration('live_sensors').perform(context).lower()=='true'
    dynamic=live or LaunchConfiguration('dynamic_obstacles').perform(context).lower()=='true'
    allow=LaunchConfiguration('allow_motor_output').perform(context).lower()=='true'
    mounting=LaunchConfiguration('mounting_config').perform(context)
    mount=json.loads(Path(mounting).read_text()) if mounting else {}
    if allow and not live:raise ValueError('Motor output requires live sensors')
    mounting_ready=bool(mount.get('calibrated') or (mount.get('orientation_confirmed') and mount.get('translation_assumption_accepted')))
    if live and not mounting_ready:raise ValueError('Confirm the mounting convention before live navigation')
    nodes=[]
    # Costmap overrides are full keyed YAML loaded after the common defaults.
    costmap={}
    if dynamic:
        for name in ('dog_global_costmap','dog_local_costmap'):
            costmap[f'/dog_nav/{name}/{name}']={'ros__parameters':{
                'plugins':['static_layer','obstacle_layer','inflation_layer'],
                'obstacle_layer':{'plugin':'nav2_costmap_2d::ObstacleLayer','observation_sources':'scan',
                    'scan':{'topic':'/dog_nav/scan','data_type':'LaserScan','clearing':True,'marking':True,
                            'obstacle_max_range':10.,'raytrace_max_range':12.,'inf_is_valid':True}},
                'inflation_layer':{'plugin':'nav2_costmap_2d::InflationLayer','inflation_radius':.15,'cost_scaling_factor':10.}}}
            if mount.get('footprint_xy_m'):
                costmap[f'/dog_nav/{name}/{name}']['ros__parameters']['footprint']=json.dumps(mount['footprint_xy_m'])
    import tempfile,yaml
    # Launch parameter artifact is retained with the run for inspection.
    extra=tempfile.NamedTemporaryFile(mode='w',prefix='dog_nav_costmap_',suffix='.yaml',delete=False)
    yaml.safe_dump(costmap,extra);extra.close()
    common={'namespace':'dog_nav','output':'screen'}
    nodes.append(Node(package='nav2_map_server',executable='map_server',name='dog_map_server',
        parameters=[{'yaml_filename':LaunchConfiguration('map_yaml'),'frame_id':'dog_lio_map'}],**common))
    for package,executable,name in [('nav2_planner','planner_server','dog_planner_server'),
                                    ('nav2_controller','controller_server','dog_controller_server')]:
        nodes.append(Node(package=package,executable=executable,name=name,parameters=[config,extra.name],**common))
    nodes.append(Node(package='nav2_bt_navigator',executable='bt_navigator',name='dog_bt_navigator',
        parameters=[config,{'default_nav_through_poses_bt_xml':str(share/'config/dog_through_poses.xml'),
                            'default_nav_to_pose_bt_xml':str(share/'config/dog_to_pose.xml')}],**common))
    nodes.append(Node(package='nav2_behaviors',executable='behavior_server',name='dog_behavior_server',
        parameters=[config],**common))
    lifecycle=['dog_map_server','dog_planner_server','dog_controller_server','dog_behavior_server','dog_bt_navigator']
    if live:
        nodes.append(Node(package='dog_navigation',executable='dog_nav_alignment',name='dog_nav_alignment',**common))
        nodes.append(Node(package='dog_navigation',executable='dog_nav_odometry',name='dog_nav_odometry',
            parameters=[{'mounting_config':mounting}],**common))
        nodes.append(Node(package='pointcloud_to_laserscan',executable='pointcloud_to_laserscan_node',
            name='dog_cloud_to_scan',parameters=[{'target_frame':'dog_base_link',
                'min_height':float(mount['scan_min_height_m']),'max_height':float(mount['scan_max_height_m']),
                'angle_min':-3.14159265359,'angle_max':3.14159265359,'angle_increment':.00872664626,
                'scan_time':.1,'range_min':.3,'range_max':12.,'use_inf':True,'transform_tolerance':.1}],
            remappings=[('cloud_in','/dog_mapping/cloud_body'),('scan','/dog_nav/scan')],**common))
    nodes.append(Node(package='nav2_lifecycle_manager',executable='lifecycle_manager',
        name='dog_navigation_lifecycle_manager',parameters=[{'autostart':True,'node_names':lifecycle}],**common))
    nodes.append(Node(package='dog_navigation',executable='dog_nav_velocity_bridge',name='dog_nav_velocity_bridge',
        parameters=[{'allow_motor_output':allow,'mounting_configured':mounting_ready,
                     'footprint_json':json.dumps(mount.get('footprint_xy_m') or [])}],**common))
    return nodes


def generate_launch_description():
    return LaunchDescription([DeclareLaunchArgument('map_yaml'),
        DeclareLaunchArgument('use_sim_time',default_value='false'),
        SetParameter(name='use_sim_time',value=ParameterValue(LaunchConfiguration('use_sim_time'),value_type=bool)),
        DeclareLaunchArgument('live_sensors',default_value='false'),
        DeclareLaunchArgument('dynamic_obstacles',default_value='false'),
        DeclareLaunchArgument('allow_motor_output',default_value='false'),
        DeclareLaunchArgument('mounting_config',default_value=''),OpaqueFunction(function=start)])
