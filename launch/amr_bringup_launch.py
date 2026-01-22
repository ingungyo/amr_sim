#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def check_param_file(context, *args, **kwargs):
    params_path = LaunchConfiguration('params_file').perform(context)
    if not os.path.isfile(params_path):
        raise FileNotFoundError(f"[Launch] Parameter file not found: {params_path}")
    return []

def get_robot_model_prefix(context, *args, **kwargs):
    """Get amr_model prefix based on robot_type"""
    robot_type_value = LaunchConfiguration('robot_type').perform(context)
    
    if robot_type_value == "HAMR":
        return 'hamr/'
    elif robot_type_value == "AMMR_SJ":
        return 'ammr_sj/'
    elif robot_type_value == "AMMR20":
        return 'ammr20/'
    else:
        # Default
        return 'bcr_bot/'


def generate_launch_description():
    amr_bringup_dir = get_package_share_directory('amr_sim')

    amr_model = 'ammr_sj/' # default = 'default/' ex)ammr = 'ammr/'

    robot_description_dir = PathJoinSubstitution([amr_bringup_dir, 'description', amr_model])
    amr_bringup_launch_file_dir = os.path.join(amr_bringup_dir, "launch")
    
    # Robot type configuration (추가된 인자)
    robot_type = LaunchConfiguration('robot_type')
    
    # 기존 파라미터 파일명들 (amr_model 기반)
    core_param_filename = amr_model + 'amr_core_param.yaml'
    docking_param_filename = amr_model + 'amr_docking_param.yaml'
    interface_param_filename = amr_model + 'amr_interface_param.yaml'
    front_lidar_param_filename = amr_model + 'amr_sick_picoscan_front.launch'
    rear_lidar_param_filename = amr_model + 'amr_sick_picoscan_rear.launch'
    lidar_merger_param_filename = amr_model + 'amr_dual_laser_merger_param.yaml'
    pointcloud_merger_param_filename = amr_model + 'amr_pointcloud_merger_param.yaml'
    robot_description_filename = 'ammr_sj.xacro'

    navigation_param_filename = amr_model + 'amr_navigation.yaml'
    navigation_map_filename = 'test_sh.yaml'
    
    # Interface에서 사용할 파라미터 파일 경로들 (LaunchConfiguration)
    nav_param_file = LaunchConfiguration('nav_param_file')
    slam_param_file = LaunchConfiguration('slam_param_file')

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map')

    use_localization = LaunchConfiguration('use_localization')
    use_amcl = LaunchConfiguration('use_amcl')  

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='Top-level namespace',
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation (Gazebo) clock if true',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(amr_bringup_dir, 'param', navigation_param_filename),
        description='Full path to param file to load',
    )

    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(amr_bringup_dir, 'data', 'maps', navigation_map_filename),
        description='Full path to map file to load',
    )

    declare_use_localization_cmd = DeclareLaunchArgument(
        'use_localization', default_value='true',
        description='Use localization if true',
    )

    declare_use_amcl_cmd = DeclareLaunchArgument(
        'use_amcl', default_value='true',
        description='Use AMCL if true',
    )

    declare_robot_type_cmd = DeclareLaunchArgument(
        'robot_type', default_value='AMMR_SJ',
        description='Robot type (HAMR, AMMR_SJ, AMMR20)',
    )

    declare_core_param_filename_cmd = DeclareLaunchArgument(
        'core_param_filename', default_value=core_param_filename,
        description='Core parameter filename',
    )

    declare_docking_param_filename_cmd = DeclareLaunchArgument(
        'docking_param_filename', default_value=docking_param_filename,
        description='Docking parameter filename',
    )

    declare_interface_param_filename_cmd = DeclareLaunchArgument(
        'interface_param_filename', default_value=interface_param_filename,
        description='Interface parameter filename',
    )

    declare_front_lidar_param_filename_cmd = DeclareLaunchArgument(
        'front_lidar_param_filename', default_value=front_lidar_param_filename,
        description='Front lidar parameter filename',
    )

    declare_rear_lidar_param_filename_cmd = DeclareLaunchArgument(
        'rear_lidar_param_filename', default_value=rear_lidar_param_filename,
        description='Rear lidar parameter filename',
    )

    declare_lidar_merger_param_filename_cmd = DeclareLaunchArgument(
        'lidar_merger_param_filename', default_value=lidar_merger_param_filename,
        description='Lidar merger parameter filename',
    )

    declare_pointcloud_merger_param_filename_cmd = DeclareLaunchArgument(
        'pointcloud_merger_param_filename', default_value=pointcloud_merger_param_filename,
        description='Pointcloud merger parameter filename',
    )

    declare_robot_description_filename_cmd = DeclareLaunchArgument(
        'robot_description_filename', default_value=robot_description_filename,
        description='Robot description filename',
    )

    declare_navigation_param_filename_cmd = DeclareLaunchArgument(
        'navigation_param_filename', default_value=navigation_param_filename,
        description='Navigation parameter filename',
    )

    declare_nav_param_file_cmd = DeclareLaunchArgument(
        'nav_param_file', 
        default_value=os.path.join(amr_bringup_dir, 'param', navigation_param_filename),
        description='Navigation parameter file path for interface',
    )

    declare_slam_param_file_cmd = DeclareLaunchArgument(
        'slam_param_file', 
        default_value=os.path.join(amr_bringup_dir, 'param', amr_model + 'slam_params.yaml'),
        description='SLAM parameter file path for interface',
    )

    check_param_file_action = OpaqueFunction(function=check_param_file)

    amr_core_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_bringup_launch_file_dir, "amr_core_bringup_launch.py")
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'amr_bringup_dir': amr_bringup_dir,
            'robot_type': robot_type,
            'core_param_filename': LaunchConfiguration('core_param_filename'),
            'docking_marker_param_filename': LaunchConfiguration('docking_param_filename'),
            'interface_param_filename': LaunchConfiguration('interface_param_filename'),
            'front_lidar_param_filename': LaunchConfiguration('front_lidar_param_filename'),
            'rear_lidar_param_filename': LaunchConfiguration('rear_lidar_param_filename'),
            'lidar_merger_param_filename': LaunchConfiguration('lidar_merger_param_filename'),
            'pointcloud_merger_param_filename': LaunchConfiguration('pointcloud_merger_param_filename'),
            'robot_description_dir': robot_description_dir,
            'robot_description_filename': LaunchConfiguration('robot_description_filename'),
            'nav_param_file': nav_param_file,
            'slam_param_file': slam_param_file,
            'use_state_publisher': 'false',
            'use_dual_lidar': 'false',
            'use_lidar_merger': 'true',
            'use_pointcloud_merger': 'false',
            'use_core': 'false',
            'use_docking': 'false',
            'use_interface': 'false',
        }.items(),
    )

    amr_nav_bringup_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(amr_bringup_launch_file_dir, "amr_nav_bringup_launch.py")
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'namespace': namespace,
            'map': map_file,
            'params_file': params_file,
            'use_localization': use_localization,
            'use_amcl': use_amcl,
        }.items(),
    )

    ld = LaunchDescription()

    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_map_cmd)
    ld.add_action(declare_use_localization_cmd)
    ld.add_action(declare_use_amcl_cmd)
    ld.add_action(declare_robot_type_cmd)
    ld.add_action(declare_core_param_filename_cmd)
    ld.add_action(declare_docking_param_filename_cmd)
    ld.add_action(declare_interface_param_filename_cmd)
    ld.add_action(declare_front_lidar_param_filename_cmd)
    ld.add_action(declare_rear_lidar_param_filename_cmd)
    ld.add_action(declare_lidar_merger_param_filename_cmd)
    ld.add_action(declare_pointcloud_merger_param_filename_cmd)
    ld.add_action(declare_robot_description_filename_cmd)
    ld.add_action(declare_navigation_param_filename_cmd)
    ld.add_action(declare_nav_param_file_cmd)
    ld.add_action(declare_slam_param_file_cmd)
    ld.add_action(check_param_file_action)
    ld.add_action(amr_core_bringup_include)
    ld.add_action(amr_nav_bringup_include)

    return ld
