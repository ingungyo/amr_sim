from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'),
    DeclareLaunchArgument(
        'param_file',
        default_value=PathJoinSubstitution([
            get_package_share_directory('amr_sim'),
            'param',
            'ammr_sj',
            'amr_swerve_contorller.yaml'
        ]),
        description='Path to the parameter YAML file'),
]


def generate_launch_description():
    node = Node(
        package='amr_sim',
        executable='swerve_controller_node',
        name='swerve_controller',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
            LaunchConfiguration('param_file')
        ],
    )

    return LaunchDescription(ARGUMENTS + [node])

