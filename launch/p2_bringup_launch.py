"""STMF P2 Gazebo sanity — 한 방: world + diff 로봇 + Nav2(AMCL,diff) + RViz. (M2, model-agnostic)

경로는 model 이름으로 유도(amr_sim 컨벤션): nav param = param/<model>/nav2_params.yaml.
map은 world 자산(모델 무관): data/maps/<map>.

  ros2 launch amr_sim p2_bringup_launch.py                    # model=p2bot, map=p2_map
  ros2 launch amr_sim p2_bringup_launch.py use_nav:=false     # sim만(M1)
  ros2 launch amr_sim p2_bringup_launch.py model:=<other> map:=<m>.yaml
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, ExecuteProcess,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _bringup(context, *args, **kwargs):
    amr = get_package_share_directory('amr_sim')

    def cfg(n):
        return LaunchConfiguration(n).perform(context)

    model = cfg('model')
    nav_params = os.path.join(amr, 'param', model, 'nav2_params.yaml')   # param/<model>/
    map_file = os.path.join(amr, 'data', 'maps', cfg('map'))
    x, y = cfg('x'), cfg('y')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(amr, 'launch', 'p2_sim_launch.py')),
        launch_arguments={'model': model, 'x': x, 'y': y, 'yaw': cfg('yaw')}.items())

    out = [sim]
    if cfg('use_nav').lower() in ('true', '1', 'yes'):
        out.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(amr, 'launch', 'amr_nav_bringup_launch.py')),
            launch_arguments={
                'use_sim_time': 'true', 'params_file': nav_params, 'map': map_file,
                'use_amcl': 'true', 'autostart': 'true',
            }.items()))
        # AMCL 초기 pose 자동 발행(스폰 위치) — sim/nav 뜬 뒤
        out.append(TimerAction(period=8.0, actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', '/initialpose',
                 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 f'{{header: {{frame_id: "map"}}, pose: {{pose: {{position: '
                 f'{{x: {x}, y: {y}, z: 0.0}}, orientation: {{z: 0.0, w: 1.0}}}}}}}}'],
            output='screen')]))
    if cfg('use_rviz').lower() in ('true', '1', 'yes'):
        out.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
                        parameters=[{'use_sim_time': True}]))
    return out


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('model', default_value='p2bot',
                              description='robot model → param/<model>/nav2_params.yaml 등'),
        DeclareLaunchArgument('map', default_value='p2_map.yaml',
                              description='data/maps/<map> (world 자산, model 무관)'),
        DeclareLaunchArgument('x', default_value='13.0', description='스폰/초기 x (home)'),
        DeclareLaunchArgument('y', default_value='6.5', description='스폰/초기 y (home)'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('use_nav', default_value='true', description='Nav2 포함(false=sim만)'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        OpaqueFunction(function=_bringup),
    ])
