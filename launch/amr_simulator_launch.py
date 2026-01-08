#!/usr/bin/python3

import os
from os.path import join
import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, AppendEnvironmentVariable, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PythonExpression, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import AppendEnvironmentVariable

def launch_setup(context, *args, **kwargs):
    sim_pkg_dir = get_package_share_directory('amr_sim')

    param_file_path = LaunchConfiguration('param_file').perform(context)
    with open(param_file_path, 'r') as f:
        data = yaml.safe_load(f) or {}

    sim_param = (data.get('sim') or {}).get('ros__parameters') or {}

    world_pkg = sim_param.get('world_pkg', 'amr_sim')
    world_name = sim_param.get('world_name', 'samhyun')

    world_pkg_dir = get_package_share_directory(world_pkg)
    world_dir = os.path.join(world_pkg_dir, 'worlds', world_name)
    world_filename = world_name + '.sdf'
    world_file = os.path.join(world_dir, world_filename)

    robots = sim_param.get('robots', [])

    actions = []

    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"{world_file} -r",
        }.items(),
    )
    actions.append(gz_sim_launch)

    robot_spawn_launch_path = os.path.join(sim_pkg_dir, 'launch', 'robot_spawn_launch.py')

    for robot in robots:
        model = robot.get('model', 'hamr30')
        name = robot.get('name', 'robot')
        namespace = robot.get('namespace', name)

        desc_pkg = robot.get('description_pkg', 'amr_sim')
        ros_gz_bridge_param = robot.get('ros_gz_bridge_param', 'rod_gz_bridge_param')
        controller_name = robot.get('controller_name', 'custom_controller')

        desc_pkg_dir = get_package_share_directory(desc_pkg)
        description_dir = os.path.join(desc_pkg_dir, 'description', model)
        description_filename = model + '.xacro'

        ros_gz_bridge_param_dir = os.path.join(desc_pkg_dir, 'param', model)
        ros_gz_bridge_param_filename = ros_gz_bridge_param + '.yaml'

        initial_pose = robot.get('initial_pose', [0.0]*6)
        while len(initial_pose) < 6:
            initial_pose.append(0.0)

        spawn_robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(robot_spawn_launch_path),
            launch_arguments={
                "robot_model": model,
                "robot_name": name,
                "namespace": namespace,
                "description_dir": description_dir,
                "description_filename": description_filename,
                "ros_gz_bridge_param_dir": ros_gz_bridge_param_dir,
                "ros_gz_bridge_param_filename": ros_gz_bridge_param_filename,
                "controller_name": controller_name,
                "init_pose_x": str(initial_pose[0]),
                "init_pose_y": str(initial_pose[1]),
                "init_pose_z": str(initial_pose[2]),
                "init_roll":  str(initial_pose[3]),
                "init_pitch": str(initial_pose[4]),
                "init_yaw":   str(initial_pose[5]),
            }.items(),
        )
        actions.append(spawn_robot_launch)

    return actions


def generate_launch_description():
    sim_pkg_dir = get_package_share_directory('amr_sim')

    default_param_file = PathJoinSubstitution(
        [sim_pkg_dir, 'param', 'robot_spawn_param.yaml']
    )

    declare_param_file_cmd = DeclareLaunchArgument(
        'param_file',
        default_value=default_param_file,
        description='Simulation parameter YAML file'
    )

    ld = LaunchDescription()
    ld.add_action(declare_param_file_cmd)

    ld.add_action(
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=join(sim_pkg_dir, "worlds"),
        )
    )
    ld.add_action(
        AppendEnvironmentVariable(
            name='IGN_GAZEBO_RESOURCE_PATH',
            value=join(sim_pkg_dir, "models"),
        )
    )

    ld.add_action(OpaqueFunction(function=launch_setup))

    return ld
