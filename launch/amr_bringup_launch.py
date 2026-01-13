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


def generate_launch_description():
    amr_bringup_dir = get_package_share_directory('amr_sim')

    robot_description_dir = PathJoinSubstitution([amr_bringup_dir, 'description', 'hamr'])
    amr_bringup_launch_file_dir = os.path.join(amr_bringup_dir, "launch")
    amr_model = 'bcr_bot/' # default = 'default/' ex)ammr = 'ammr/'
    core_param_filename = amr_model + 'amr_core_param.yaml'
    docking_param_filename = amr_model + 'amr_docking_param.yaml'
    interface_param_filename = amr_model + 'amr_interface_param.yaml'
    front_lidar_param_filename = amr_model + 'amr_sick_picoscan_front.launch'
    rear_lidar_param_filename = amr_model + 'amr_sick_picoscan_rear.launch'
    lidar_merger_param_filename = amr_model + 'amr_lidar_merger_param.yaml'
    pointcloud_merger_param_filename = amr_model + 'amr_pointcloud_merger_param.yaml'
    robot_description_filename = 'bcr_bot.xacro'

    navigation_param_filename = amr_model + 'nav2_params.yaml'
    navigation_map_filename = 'test_sh.yaml'

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
        'use_amcl', default_value='false',
        description='Use AMCL if true',
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
            'core_param_filename': core_param_filename,
            'docking_marker_param_filename': docking_param_filename,
            'interface_param_filename': interface_param_filename,
            'front_lidar_param_filename': front_lidar_param_filename,
            'rear_lidar_param_filename': rear_lidar_param_filename,
            'lidar_merger_param_filename': lidar_merger_param_filename,
            'pointcloud_merger_param_filename': pointcloud_merger_param_filename,
            'robot_description_dir': robot_description_dir,
            'robot_description_filename': robot_description_filename,
            'use_state_publisher': 'false',
            'use_dual_lidar': 'false',
            'use_lidar_merger': 'false',
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
    ld.add_action(check_param_file_action)
    ld.add_action(amr_core_bringup_include)
    ld.add_action(amr_nav_bringup_include)

    return ld
