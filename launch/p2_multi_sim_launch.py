"""STMF P2 멀티로봇 Gazebo 스폰 — robots 리스트 순회, 로봇별 namespace. (멀티 M1)

각 로봇: rsp(namespace + frame_prefix=<ns>/ , tf 전역) + gz create + parameter_bridge(프리픽스 토픽).
tf는 전역 /tf에 프리픽스 프레임으로 모임(단일 RViz용). /clock은 전역 1회만 브리지.

  ros2 launch amr_sim p2_multi_sim_launch.py
  ros2 launch amr_sim p2_multi_sim_launch.py robots_file:=<path>
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

import yaml

_BRIDGE_DIR = "/tmp/p2_bridges"


def _bridge_yaml(ns):
    """로봇별 gz↔ros bridge. gz topic=/{ns}/X, ros topic=/{ns}/X (tf만 ros /tf 전역)."""
    p = f"/{ns}"
    e = []

    def add(ros_t, gz_t, rtype, gtype, direction):
        e.append(f'- ros_topic_name: "{ros_t}"\n  gz_topic_name: "{gz_t}"\n'
                 f'  ros_type_name: "{rtype}"\n  gz_type_name: "{gtype}"\n'
                 f'  direction: "{direction}"\n')
    add(f"{p}/cmd_vel", f"{p}/cmd_vel", "geometry_msgs/msg/Twist", "gz.msgs.Twist", "BIDIRECTIONAL")
    add(f"{p}/odom", f"{p}/odom", "nav_msgs/msg/Odometry", "gz.msgs.Odometry", "GZ_TO_ROS")
    # tf는 로봇별 네임스페이스 토픽(/{ns}/tf) — nav2 체인(namespace tf)과 정합. 단일 RViz는 relay로 전역화.
    add(f"{p}/tf", f"{p}/tf", "tf2_msgs/msg/TFMessage", "gz.msgs.Pose_V", "GZ_TO_ROS")
    add(f"{p}/scan", f"{p}/scan", "sensor_msgs/msg/LaserScan", "gz.msgs.LaserScan", "GZ_TO_ROS")
    add(f"{p}/imu", f"{p}/imu", "sensor_msgs/msg/Imu", "gz.msgs.IMU", "GZ_TO_ROS")
    add(f"{p}/joint_states", f"{p}/joint_states", "sensor_msgs/msg/JointState", "gz.msgs.Model", "GZ_TO_ROS")
    return "".join(e)


def _setup(context, *args, **kwargs):
    amr = get_package_share_directory('amr_sim')
    gz_sim = get_package_share_directory('ros_gz_sim')

    spawn_file = LaunchConfiguration('robots_file').perform(context)
    sim = (yaml.safe_load(open(spawn_file)).get('sim') or {}).get('ros__parameters') or {}
    world_name = sim.get('world_name', 'p2')
    world = os.path.join(amr, 'worlds', world_name, f'{world_name}.sdf')
    robots = sim.get('robots', [])

    os.makedirs(_BRIDGE_DIR, exist_ok=True)
    actions = [IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'{world} -r'}.items())]

    # 전역 clock bridge (1회)
    actions.append(Node(
        package='ros_gz_bridge', executable='parameter_bridge', name='clock_bridge', output='screen',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock']))

    for r in robots:
        ns = r['name']; model = r.get('model', 'p2bot')
        pose = (r.get('initial_pose', [0.0] * 6) + [0.0] * 6)[:6]
        xacro_file = os.path.join(amr, 'description', model, f'{model}.xacro')

        rd = ParameterValue(
            Command(['xacro ', xacro_file, ' namespace:=', ns]), value_type=str)
        actions.append(Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            namespace=ns, name='robot_state_publisher', output='screen',
            parameters=[{'use_sim_time': True, 'robot_description': rd}],
            remappings=[('/tf', 'tf'), ('/tf_static', 'tf_static')]))   # → /{ns}/tf (nav2 정합)

        actions.append(Node(
            package='ros_gz_sim', executable='create', namespace=ns, output='screen',
            arguments=['-topic', 'robot_description', '-name', ns,
                       '-x', str(pose[0]), '-y', str(pose[1]), '-z', str(pose[2]),
                       '-R', str(pose[3]), '-P', str(pose[4]), '-Y', str(pose[5]),
                       '-allow_renaming', 'true']))

        bfile = os.path.join(_BRIDGE_DIR, f'{ns}.yaml')
        with open(bfile, 'w') as f:
            f.write(_bridge_yaml(ns))
        actions.append(Node(
            package='ros_gz_bridge', executable='parameter_bridge',
            namespace=ns, name='bridge', output='screen',
            parameters=[{'config_file': bfile, 'use_sim_time': True}]))

    return actions


def generate_launch_description():
    amr = get_package_share_directory('amr_sim')
    return LaunchDescription([
        DeclareLaunchArgument(
            'robots_file', default_value=os.path.join(amr, 'param', 'p2_multi_spawn.yaml'),
            description='멀티로봇 정의(sim.ros__parameters.robots)'),
        OpaqueFunction(function=_setup),
    ])
