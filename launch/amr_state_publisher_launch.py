from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
)
from launch.substitutions import PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    amr_bringup_dir = get_package_share_directory('amr_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    namespace = LaunchConfiguration('namespace')
    description_filename = LaunchConfiguration('robot_description_filename')
    description_dir = LaunchConfiguration('robot_description_dir')
    prefix = LaunchConfiguration('prefix')
    jsp_gui = LaunchConfiguration('jsp_gui')
    default_param_dir = PathJoinSubstitution([amr_bringup_dir, 'description', 'urdf'])

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use /clock (sim time)'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='',
        description='ROS namespace'
    )

    declare_description_filename_cmd = DeclareLaunchArgument(
        'robot_description_filename', default_value='ammr.urdf.xacro',
        description='urdf filename'
    )

    declare_description_dir_cmd = DeclareLaunchArgument(
        'robot_description_dir',
        default_value=default_param_dir,
        description='Absolute path to the parameter urdf file'
    )

    declare_prefix_arg_cmd = DeclareLaunchArgument(
        'prefix', default_value='',
        description='Optional joint name prefix (e.g., for multi-robot scenarios)'
    )

    declare_jsp_gui_arg_cmd = DeclareLaunchArgument(
        'jsp_gui', default_value='false',
        description='If true, use joint_state_publisher_gui'
    )
    
    robot_description_filepath = PathJoinSubstitution([description_dir, description_filename])

    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ', robot_description_filepath
    ])
    robot_description = ParameterValue(robot_description_content, value_type=str)

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        namespace=namespace,
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
            'frame_prefix': PythonExpression(["'", prefix, "/'"]) ##멀티로봇 테스트 필요
        }]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        namespace=namespace,  
        name='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(jsp_gui)
    )

    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        namespace=namespace,
        name='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(jsp_gui)
    )

    ld = LaunchDescription()

    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_description_filename_cmd)
    ld.add_action(declare_description_dir_cmd)
    ld.add_action(declare_prefix_arg_cmd)
    ld.add_action(declare_jsp_gui_arg_cmd)

    # ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    ld.add_action(joint_state_publisher_gui_node)

    return ld

