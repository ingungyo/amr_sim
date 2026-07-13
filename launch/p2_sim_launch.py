"""STMF P2 Gazebo 시뮬 — world + diff 로봇 스폰 + bridge. (M1, model-agnostic)

robot_spawn_launch.py는 swerve 하드코딩이라, diff 로봇용 자체완결 스폰만 담는다.
경로는 **model 이름으로 유도**(amr_sim 컨벤션): description/<model>/<model>.xacro,
param/<model>/ros_gz_bridge_param.yaml. 로봇 바꾸려면 model:=<name>만.

  ros2 launch amr_sim p2_sim_launch.py                 # model=p2bot
  ros2 launch amr_sim p2_sim_launch.py model:=<other>  # description/<other>/<other>.xacro + param/<other>/...
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _spawn(context, *args, **kwargs):
    amr = get_package_share_directory('amr_sim')
    gz_sim = get_package_share_directory('ros_gz_sim')

    def cfg(n):
        return LaunchConfiguration(n).perform(context)

    model = cfg('model')
    xacro_file = os.path.join(amr, 'description', model, f'{model}.xacro')
    bridge_yaml = os.path.join(amr, 'param', model, 'ros_gz_bridge_param.yaml')

    gz = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f"{cfg('world')} -r"}.items())

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher', output='screen',
        parameters=[{'use_sim_time': True, 'robot_description': robot_description}])
    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', model,
                   '-x', cfg('x'), '-y', cfg('y'), '-z', '0.15', '-Y', cfg('yaw'),
                   '-allow_renaming', 'true'])
    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_yaml, 'use_sim_time': True}])
    return [gz, rsp, spawn, bridge]


def generate_launch_description():
    amr = get_package_share_directory('amr_sim')
    return LaunchDescription([
        DeclareLaunchArgument(
            'model', default_value='p2bot',
            description='robot model — description/<model>/<model>.xacro + param/<model>/*.yaml'),
        DeclareLaunchArgument(
            'world', default_value=os.path.join(amr, 'worlds', 'p2', 'p2.sdf'),
            description='world sdf 경로'),
        DeclareLaunchArgument('x', default_value='13.0', description='스폰 x (home)'),
        DeclareLaunchArgument('y', default_value='6.5', description='스폰 y (home)'),
        DeclareLaunchArgument('yaw', default_value='0.0', description='스폰 yaw'),
        OpaqueFunction(function=_spawn),
    ])
