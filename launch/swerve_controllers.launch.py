from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch.substitutions.launch_configuration import LaunchConfiguration


ARGUMENTS = [
    DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        choices=['true', 'false'],
        description='use_sim_time'
    ),
    DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="Start robot with fake hardware mirroring command to its states.",
    ),
    DeclareLaunchArgument(
        "fake_sensor_commands",
        default_value="false",
        description="Enable fake command interfaces for sensors used for simple simulations. Used only if 'use_fake_hardware' parameter is true.",
    ),
]

def generate_launch_description():
    is_simulation = LaunchConfiguration("use_sim_time")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    fake_sensor_commands = LaunchConfiguration("fake_sensor_commands")

    ld = LaunchDescription(ARGUMENTS)

    from launch_ros.actions import Node

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen'
    )
    ld.add_action(joint_state_broadcaster_spawner)

    postion_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drive_module_steering_angle_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay creating the position trajectory controller until the joint_state_broadcast node has been started so that
    # the position trajectory controller can get the different TF frames from the broadcaster
    delay_steering_angle_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[postion_trajectory_controller_spawner],
        )
    )
    ld.add_action(delay_steering_angle_controller_spawner_after_joint_state_broadcaster_spawner)


    velocity_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['drive_module_velocity_controller', '--controller-manager', '/controller_manager'],
        output='screen'
    )

    # Delay creating the velocity controller until the joint_state_broadcast node has been started so that
    # the velocity controller can get the different TF frames from the broadcaster
    delay_velocity_controller_spawner_after_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[velocity_controller_spawner],
        )
    )
    ld.add_action(delay_velocity_controller_spawner_after_joint_state_broadcaster_spawner)

    return ld