#!/usr/bin/python3

import os

from os.path import join
from xacro import parse, process_doc

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

def get_xacro_to_doc(xacro_file_path, mappings):
    doc = parse(open(xacro_file_path))
    process_doc(doc, mappings=mappings)
    return doc

def generate_launch_description():
   
    sim_pkg_dir = get_package_share_directory("amr_sim")
    # swerve_controller_dir = get_package_share_directory("swerve_controller")
    # swerve_controller_dir = get_package_share_directory("zinger_swerve_controller_cpp")

    use_sim_time = LaunchConfiguration("use_sim_time")
    namespace = LaunchConfiguration("namespace")
    robot_model = LaunchConfiguration("robot_model")
    robot_name = LaunchConfiguration("robot_name")

    description_filename = LaunchConfiguration("description_filename")
    description_dir = LaunchConfiguration("description_dir")
    description_file = PathJoinSubstitution([description_dir, description_filename])

    ros_gz_bridge_param_dir = LaunchConfiguration("ros_gz_bridge_param_dir")
    ros_gz_bridge_param_filename = LaunchConfiguration("ros_gz_bridge_param_filename")
    ros_gz_bridge_param_file = PathJoinSubstitution([ros_gz_bridge_param_dir, ros_gz_bridge_param_filename])

    init_pose_x = LaunchConfiguration("init_pose_x")
    init_pose_y = LaunchConfiguration("init_pose_y")
    init_pose_z = LaunchConfiguration("init_pose_z")
    init_roll   = LaunchConfiguration("init_roll")
    init_pitch  = LaunchConfiguration("init_pitch")
    init_yaw    = LaunchConfiguration("init_yaw")

    odometry_source = LaunchConfiguration("odometry_source")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use /clock (sim time)",
    )

    declare_namespace = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace",
    )

    declare_robot_model = DeclareLaunchArgument(
        "robot_model",
        default_value="hamr30",
        description="Robot model",
    )

    declare_robot_name = DeclareLaunchArgument(
        "robot_name",
        default_value="robot",
        description="Robot name (for Gazebo entity)",
    )

    declare_description_dir = DeclareLaunchArgument(
        "description_dir",
        default_value=join(sim_pkg_dir, "description", "hamr30"),
        description="Directory path to robot description (URDF/XACRO)",
    )

    declare_description_filename = DeclareLaunchArgument(
        "description_filename",
        default_value="hamr30.xacro",
        description="Robot description filename",
    )

    declare_ros_gz_bridge_param_dir = DeclareLaunchArgument(
        "ros_gz_bridge_param_dir",
        default_value=join(sim_pkg_dir, "param", "hamr30"),
        description="Directory path to robot description (URDF/XACRO)",
    )

    declare_ros_gz_bridge_param_filename = DeclareLaunchArgument(
        "ros_gz_bridge_param_filename",
        default_value="ros_gz_bridge_param.yaml",
        description="ros gz bridge param filename",
    )

    declare_init_pose_x = DeclareLaunchArgument(
        "init_pose_x",
        default_value="0.0",
        description="Initial pose X",
    )
    declare_init_pose_y = DeclareLaunchArgument(
        "init_pose_y",
        default_value="0.0",
        description="Initial pose Y",
    )
    declare_init_pose_z = DeclareLaunchArgument(
        "init_pose_z",
        default_value="0.5",
        description="Initial pose Z",
    )
    declare_init_roll = DeclareLaunchArgument(
        "init_roll",
        default_value="0.0",
        description="Initial roll (rad)",
    )
    declare_init_pitch = DeclareLaunchArgument(
        "init_pitch",
        default_value="0.0",
        description="Initial pitch (rad)",
    )
    declare_init_yaw = DeclareLaunchArgument(
        "init_yaw",
        default_value="0.0",
        description="Initial yaw (rad)",
    )

    declare_odom_source = DeclareLaunchArgument(
        "odometry_source",
        default_value="encoders",
        description="odometry source for xacro (e.g., 'world', 'encoder', 'slam')",
    )

    robot_description_param = ParameterValue(
        Command([
            "xacro ",
            description_file,
            " odometry_source:=", odometry_source,
            " sim_ign:=false",
        ]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        namespace=namespace,
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"robot_description": robot_description_param},
        ],
    )

    gz_spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        namespace=namespace,
        output="screen",
        arguments=[
            "-topic", "robot_description",
            "-name", robot_name,
            "-allow_renaming", "true",
            "-z", init_pose_z,
            "-x", init_pose_x,
            "-y", init_pose_y,
            "-R", init_roll,
            "-P", init_pitch,
            "-Y", init_yaw,
        ],
    )

    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        namespace=namespace,
        parameters=[{
            'config_file': ros_gz_bridge_param_file
        }]
    )

    ##########################TODO##########################################
    ## select controller ###################################################
    # controller_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([sim_pkg_dir, 'launch',
    #                               'swerve_controllers.launch.py'])
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #     }.items()
    # )

    # swerve_controller_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([swerve_controller_dir, 'launch',
    #                               'swerve_controller_cpp.launch.py'])
    #     ),
    #     launch_arguments={
    #         'use_sim_time': use_sim_time,
    #         'param_file': PathJoinSubstitution([sim_pkg_dir, 'param', robot_model, 'swerve_controller.yaml'])
    #     }.items()
    # )
    ########################################################################



    # transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments = ["--x", "0.0",
    #                 "--y", "0.0",
    #                 "--z", "0.0",
    #                 "--yaw", "0.0",
    #                 "--pitch", "0.0",
    #                 "--roll", "0.0",
    #                 "--frame-id", "kinect_camera",
    #                 "--child-frame-id", "bcr_bot/base_footprint/kinect_camera"]
    # )

    # transform_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments = ["--x", "0.0",
    #                 "--y", "0.0",
    #                 "--z", "0.0",
    #                 "--yaw", "0.0",
    #                 "--pitch", "0.0",
    #                 "--roll", "0.0"]
    # )

    lidar_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0",
            "0", "0", "0","1",
            "gpu_lidar",
            "bcr_bot/base_footprint/gpu_lidar"
        ],
        parameters=[{'use_sim_time': True}],
        name="static_tf_gpu_lidar",
        output="screen",
    )

    imu_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0", "0", "0",
            "0", "0", "0","1",
            "imu_frame",
            "bcr_bot/base_footprint/imu_sensor"
        ],
        parameters=[{'use_sim_time': True}],
        name="static_tf_imu",
        output="screen",
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time)
    ld.add_action(declare_namespace)
    ld.add_action(declare_robot_model)
    ld.add_action(declare_robot_name)

    ld.add_action(declare_description_dir)
    ld.add_action(declare_description_filename)
    ld.add_action(declare_ros_gz_bridge_param_dir)
    ld.add_action(declare_ros_gz_bridge_param_filename)

    ld.add_action(declare_init_pose_x)
    ld.add_action(declare_init_pose_y)
    ld.add_action(declare_init_pose_z)
    ld.add_action(declare_init_roll)
    ld.add_action(declare_init_pitch)
    ld.add_action(declare_init_yaw)

    ld.add_action(declare_odom_source)

    ld.add_action(robot_state_publisher)
    ld.add_action(gz_spawn_entity)
    ld.add_action(gz_ros2_bridge)
    # ld.add_action(controller_launch)
    # ld.add_action(swerve_controller_launch)
    return ld


    # return LaunchDescription([
    #     DeclareLaunchArgument("position_x", default_value="0.0"),
    #     DeclareLaunchArgument("position_y", default_value="0.0"),
    #     DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
    #     DeclareLaunchArgument("odometry_source", default_value="world"),
    #     robot_state_publisher,
    #     gz_spawn_entity,
    #     # transform_publisher,
    #     # lidar_transform_publisher,
    #     # imu_transform_publisher,
    #     gz_ros2_bridge
    # ])