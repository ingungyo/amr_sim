"""STMF P2 멀티로봇 Nav2 — 로봇별 namespaced Nav2 + 단일 RViz용 tf relay. (멀티 M2)

각 로봇(robots_file의 robots): amr_nav_bringup을 namespace로 include(검증된 nav2 체인, namespace tf)
 + 프레임을 <ns>/로 프리픽스한 per-robot params 주입 + /<ns>/tf → 전역 /tf relay(단일 RViz)
 + 초기 pose 자동 발행. map은 공유(p2_map).

  ros2 launch amr_sim p2_multi_nav_launch.py        # p2_multi_sim 먼저 띄운 상태에서
전제: p2_multi_sim_launch.py 로 로봇들이 이미 스폰돼 있어야 함(별도 터미널).
"""
import os
import copy
import math

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction, ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

_FRAME_KEYS = {"base_frame_id", "odom_frame_id", "robot_base_frame", "odom_frame",
               "local_frame"}   # local_frame: behavior_server(spin/backup) recovery용
_NAV_TMP = "/tmp/p2_nav_params"


def _prefix_frames(node, ns):
    """params 트리를 순회하며 프레임/절대 odom 토픽을 <ns>/ 프리픽스. map/scan(상대)은 그대로."""
    if isinstance(node, dict):
        for k, v in node.items():
            if isinstance(v, (dict, list)):
                _prefix_frames(v, ns)
            elif isinstance(v, str):
                if k in _FRAME_KEYS and v in ("base_footprint", "base_link", "odom"):
                    node[k] = f"{ns}/{v}"
                elif k == "global_frame" and v == "odom":       # local costmap
                    node[k] = f"{ns}/odom"
                elif k == "odom_topic" and v.startswith("/"):
                    node[k] = f"/{ns}{v}"
                # costmap의 obstacle_layer scan topic이 상대경로 'scan'이면
                # sub-namespace(/{ns}/local_costmap/scan)로 잘못 해석됨 → 절대경로로 고정.
                elif k == "topic" and v == "scan":
                    node[k] = f"/{ns}/scan"
    elif isinstance(node, list):
        for x in node:
            _prefix_frames(x, ns)


def _make_params(base_params, ns):
    d = copy.deepcopy(yaml.safe_load(open(base_params)))
    _prefix_frames(d, ns)
    os.makedirs(_NAV_TMP, exist_ok=True)
    out = os.path.join(_NAV_TMP, f"{ns}.yaml")
    with open(out, "w") as f:
        yaml.safe_dump(d, f)
    return out


def _setup(context, *args, **kwargs):
    amr = get_package_share_directory('amr_sim')
    spawn_file = LaunchConfiguration('robots_file').perform(context)
    map_yaml = LaunchConfiguration('map').perform(context)
    sim = (yaml.safe_load(open(spawn_file)).get('sim') or {}).get('ros__parameters') or {}
    robots = sim.get('robots', [])

    actions = []
    for r in robots:
        ns = r['name']; model = r.get('model', 'p2bot')
        pose = (r.get('initial_pose', [0.0] * 6) + [0.0] * 6)[:6]
        base_params = os.path.join(amr, 'param', model, 'nav2_params.yaml')
        params = _make_params(base_params, ns)

        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(amr, 'launch', 'amr_nav_bringup_launch.py')),
            launch_arguments={
                'namespace': ns, 'use_namespace': 'true',
                'use_sim_time': 'true', 'params_file': params,
                'map': map_yaml, 'use_amcl': 'true', 'autostart': 'true',
            }.items()))

        # /<ns>/tf → 전역 /tf relay (프레임이 <ns>/로 프리픽스돼 충돌 없음, 단일 RViz용)
        actions.append(Node(package='topic_tools', executable='relay',
                            name=f'tfrelay_{ns}', output='screen',
                            arguments=[f'/{ns}/tf', '/tf']))
        actions.append(Node(package='topic_tools', executable='relay',
                            name=f'tfsrelay_{ns}', output='screen',
                            arguments=[f'/{ns}/tf_static', '/tf_static']))

        # RViz "Goal <ns>" 툴(/<ns>/goal_pose) → Nav2 액션 relay (단일 RViz 클릭 제어)
        actions.append(Node(package='stmf_tools', executable='p2_goal_relay',
                            name=f'goalrelay_{ns}', output='screen',
                            parameters=[{'namespace': ns}]))

        # AMCL 초기 pose (namespace) — 스폰 위치·방향, nav 뜬 뒤
        # 스폰 yaw(pose[5])로 orientation 계산 — robot1(yaw=π) 등 방향 정합 필수
        qz = math.sin(pose[5] / 2.0)
        qw = math.cos(pose[5] / 2.0)
        actions.append(TimerAction(period=10.0, actions=[ExecuteProcess(
            cmd=['ros2', 'topic', 'pub', '--once', f'/{ns}/initialpose',
                 'geometry_msgs/msg/PoseWithCovarianceStamped',
                 f'{{header: {{frame_id: "map"}}, pose: {{pose: {{position: '
                 f'{{x: {pose[0]}, y: {pose[1]}, z: 0.0}}, '
                 f'orientation: {{z: {qz}, w: {qw}}}}}}}}}'],
            output='screen')]))

    if LaunchConfiguration('use_rviz').perform(context).lower() in ('true', '1', 'yes'):
        actions.append(Node(
            package='rviz2', executable='rviz2', name='rviz2', output='screen',
            arguments=['-d', os.path.join(amr, 'rviz', 'p2_multi.rviz')],
            parameters=[{'use_sim_time': True}]))
    return actions


def generate_launch_description():
    amr = get_package_share_directory('amr_sim')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_file', default_value=os.path.join(amr, 'param', 'p2_multi_spawn.yaml')),
        DeclareLaunchArgument(
            'map', default_value=os.path.join(amr, 'data', 'maps', 'p2_map.yaml')),
        DeclareLaunchArgument('use_rviz', default_value='true',
                              description='단일 RViz(p2_multi.rviz) 실행'),
        OpaqueFunction(function=_setup),
    ])
