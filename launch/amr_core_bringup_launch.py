import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    default_amr_bringup_dir = get_package_share_directory('amr_bringup')
    amr_core_dir = get_package_share_directory('amr_core')
    amr_docking_marker_dir = get_package_share_directory('amr_docking_marker')
    amr_interface_dir = get_package_share_directory('amr_interface')
    amr_lidar_merger_dir = get_package_share_directory('amr_lidar_merger')
    amr_pointcloud_merger_dir = get_package_share_directory('amr_pointcloud_merger')

    amr_bringup_dir = LaunchConfiguration('amr_bringup_dir')

    default_param_dir = PathJoinSubstitution([amr_bringup_dir, 'param'])
    default_description_dir = PathJoinSubstitution([amr_bringup_dir, 'description', 'urdf'])

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    param_dir = LaunchConfiguration('param_dir')
    core_param_filename = LaunchConfiguration('core_param_filename')
    docking_param_filename = LaunchConfiguration('docking_marker_param_filename')
    interface_param_filename = LaunchConfiguration('interface_param_filename')
    front_lidar_param_filename = LaunchConfiguration('front_lidar_param_filename')
    rear_lidar_param_filename = LaunchConfiguration('rear_lidar_param_filename')
    lidar_merger_param_filename = LaunchConfiguration('lidar_merger_param_filename')
    pointcloud_merger_param_filename = LaunchConfiguration('pointcloud_merger_param_filename')

    robot_description_dir = LaunchConfiguration('robot_description_dir')
    robot_description_filename = LaunchConfiguration('robot_description_filename')
    prefix = LaunchConfiguration('prefix')

    use_state_publisher = LaunchConfiguration('use_state_publisher')
    use_dual_lidar = LaunchConfiguration('use_dual_lidar')
    use_lidar_merger = LaunchConfiguration('use_lidar_merger')
    use_core = LaunchConfiguration('use_core')
    use_docking = LaunchConfiguration('use_docking')
    use_interface = LaunchConfiguration('use_interface')
    use_pointcloud_merger = LaunchConfiguration('use_pointcloud_merger')
    robot_type = LaunchConfiguration('robot_type')
    nav_param_file = LaunchConfiguration('nav_param_file')
    slam_param_file = LaunchConfiguration('slam_param_file')


    declare_amr_bringup_dir_cmd = DeclareLaunchArgument(
        'amr_bringup_dir', default_value=default_amr_bringup_dir,
        description="Base directory of amr_bringup package."
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (sim time)'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='ROS namespace'
    )

    declare_param_dir_cmd = DeclareLaunchArgument(
        'param_dir', default_value=default_param_dir,
        description='Absolute path to the parameter YAML file'
    )

    declare_core_param_filename_cmd = DeclareLaunchArgument(
        'core_param_filename', default_value='amr_core_param.yaml',
        description='Param YAML filename inside the param/ folder'
    )

    declare_docking_param_filename_cmd = DeclareLaunchArgument(
        'docking_marker_param_filename', default_value='amr_docking_param.yaml',
        description='Docking param YAML filename inside the param/ folder'
    )

    declare_interface_param_filename_cmd = DeclareLaunchArgument(
        'interface_param_filename', default_value='amr_interface_param.yaml',
        description='Interface param YAML filename inside the param/ folder'
    )

    declare_front_lidar_param_filename_cmd = DeclareLaunchArgument(
        'front_lidar_param_filename', default_value='amr_sick_picoscan_front.launch',
        description='Front lidar launch filename inside the param/ folder'
    )

    declare_rear_lidar_param_filename_cmd = DeclareLaunchArgument(
        'rear_lidar_param_filename', default_value='amr_sick_picoscan_rear.launch',
        description='Rear lidar launch filename inside the param/ folder'
    )

    declare_lidar_merger_param_filename_cmd = DeclareLaunchArgument(
        'lidar_merger_param_filename', default_value='amr_lidar_merger_param.yaml',
        description='Lidar merger param YAML filename inside the param/ folder'
    )

    declare_pointcloud_merger_param_filename_cmd = DeclareLaunchArgument(
        'pointcloud_merger_param_filename', default_value='amr_pointcloud_merger_param.yaml',
        description='point_cloud merger param YAML filename inside the param/ folder'
    )

    declare_robot_description_filename_cmd = DeclareLaunchArgument(
        'robot_description_filename', default_value='ammr.urdf.xacro',
        description='urdf filename'
    )

    declare_robot_description_dir_cmd = DeclareLaunchArgument(
        'robot_description_dir', default_value=default_description_dir,
        description='Absolute path to the parameter urdf file'
    )

    declare_prefix_cmd = DeclareLaunchArgument(
        'prefix', default_value='',
        description='Optional joint name prefix (for multi-robot scenarios)'
    )

    declare_use_state_publisher_cmd = DeclareLaunchArgument(
        'use_state_publisher', default_value='true',
        description='Whether to launch robot_state_publisher group'
    )

    declare_use_dual_lidar_cmd = DeclareLaunchArgument(
        'use_dual_lidar', default_value='true',
        description='Whether to launch dual lidar bringup'
    )

    declare_use_lidar_merger_cmd = DeclareLaunchArgument(
        'use_lidar_merger', default_value='true',
        description='Whether to launch lidar merger'
    )

    declare_use_core_cmd = DeclareLaunchArgument(
        'use_core', default_value='true',
        description='Whether to launch core nodes'
    )

    declare_use_docking_cmd = DeclareLaunchArgument(
        'use_docking', default_value='true',
        description='Whether to launch docking marker nodes'
    )

    declare_use_interface_cmd = DeclareLaunchArgument(
        'use_interface', default_value='true',
        description='Whether to launch interface nodes'
    )

    declare_use_pointcloud_merger_cmd = DeclareLaunchArgument(
        'use_pointcloud_merger', default_value='false',
        description='Whether to launch pointcloud merger nodes'
    )

    declare_robot_type_cmd = DeclareLaunchArgument(
        'robot_type', default_value='HAMR',
        description='Robot type (HAMR, AMMR_SJ, AMMR20)'
    )

    declare_nav_param_file_cmd = DeclareLaunchArgument(
        'nav_param_file', default_value='',
        description='Navigation parameter file path'
    )

    declare_slam_param_file_cmd = DeclareLaunchArgument(
        'slam_param_file', default_value='',
        description='SLAM parameter file path'
    )

    state_publisher_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([amr_bringup_dir, 'launch',
                                  'amr_state_publisher_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'prefix': prefix,
            'use_sim_time': use_sim_time,
            'robot_description_dir': robot_description_dir,
            'robot_description_filename': robot_description_filename,
        }.items(),
        condition=IfCondition(use_state_publisher)
    )

    dual_lidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([amr_bringup_dir, 'launch',
                                  'amr_sick_dual_lidar_bringup_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'param_dir': param_dir,
            'front_param_filename': front_lidar_param_filename,
            'rear_param_filename':  rear_lidar_param_filename,
        }.items(),
        condition=IfCondition(use_dual_lidar)
    )

    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([amr_core_dir, 'launch',
                                  'core_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'param_dir': param_dir,
            'param_filename': core_param_filename,
        }.items(),
        condition=IfCondition(use_core)
    )

    docking_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([amr_docking_marker_dir, 'launch',
                                  'docking_marker_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'param_dir': param_dir,
            'param_filename': docking_param_filename,
        }.items(),
        condition=IfCondition(use_docking)
    )

    interface_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([amr_interface_dir, 'launch',
                                  'interface_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'param_dir': param_dir,
            'param_filename': interface_param_filename,
            'robot_type': robot_type,
            'nav_param_file': nav_param_file,
            'slam_param_file': slam_param_file,
        }.items(),
        condition=IfCondition(use_interface)
    )

    lidar_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([amr_lidar_merger_dir, 'launch',
                                  'dual_laser_merger_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'param_dir': param_dir,
            'param_filename': lidar_merger_param_filename,
        }.items(),
        condition=IfCondition(use_lidar_merger)
    )

    pointcloud_merger_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([amr_pointcloud_merger_dir, 'launch',
                                  'amr_pointcloud_merger_launch.py'])
        ),
        launch_arguments={
            'namespace': namespace,
            'use_sim_time': use_sim_time,
            'param_dir': param_dir,
            'param_filename': pointcloud_merger_param_filename,
        }.items(),
        condition=IfCondition(use_pointcloud_merger)
    )

    bringup_group = GroupAction([
        state_publisher_launch,
        dual_lidar_launch,
        lidar_merger_launch,
        core_launch,
        docking_launch,
        interface_launch,
        pointcloud_merger_launch,
    ])

    ld = LaunchDescription()

    ld.add_action(declare_amr_bringup_dir_cmd)

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_param_dir_cmd)
    ld.add_action(declare_core_param_filename_cmd)
    ld.add_action(declare_docking_param_filename_cmd)
    ld.add_action(declare_interface_param_filename_cmd)
    ld.add_action(declare_front_lidar_param_filename_cmd)
    ld.add_action(declare_rear_lidar_param_filename_cmd)
    ld.add_action(declare_lidar_merger_param_filename_cmd)
    ld.add_action(declare_pointcloud_merger_param_filename_cmd)
    ld.add_action(declare_robot_description_dir_cmd)
    ld.add_action(declare_robot_description_filename_cmd)
    ld.add_action(declare_prefix_cmd)


    ld.add_action(declare_use_state_publisher_cmd)
    ld.add_action(declare_use_dual_lidar_cmd)
    ld.add_action(declare_use_lidar_merger_cmd)
    ld.add_action(declare_use_core_cmd)
    ld.add_action(declare_use_docking_cmd)
    ld.add_action(declare_use_interface_cmd)
    ld.add_action(declare_use_pointcloud_merger_cmd)
    ld.add_action(declare_robot_type_cmd)
    ld.add_action(declare_nav_param_file_cmd)
    ld.add_action(declare_slam_param_file_cmd)

    ld.add_action(bringup_group)

    return ld