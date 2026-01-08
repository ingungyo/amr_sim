import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import launch_ros.actions

def generate_launch_description():
    # Get package directories
    amr_sim_dir = get_package_share_directory('amr_sim')
    amr_hdl_localization_dir = get_package_share_directory('amr_hdl_localization')
    amr_pointcloud_converter_dir = get_package_share_directory('amr_pointcloud_converter')
    amr_hdl_global_localization_dir = get_package_share_directory('amr_hdl_global_localization')

    robot_name = 'hamr30'
    
    # Default parameter file path
    # default_param_file = PathJoinSubstitution([amr_hdl_localization_dir, 'param', 'hdl_localization_params.yaml'])
    default_param_file = PathJoinSubstitution([amr_sim_dir, 'param', robot_name, 'hdl_localization_simulation_params.yaml'])
    
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Declare launch arguments
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use /clock (sim time)'
    )
    
    # Map file configuration - read filename from YAML and construct full path
    # param_file_path = os.path.join(amr_hdl_localization_dir, 'param', 'hdl_localization_simulation_params.yaml')
    param_file_path = os.path.join(amr_sim_dir, 'param', robot_name, 'hdl_localization_simulation_params.yaml')
    with open(param_file_path, 'r') as f:
        param_data = yaml.safe_load(f)
    
    # Extract filename from YAML (assuming it's just the filename now)
    map_filename = param_data['HdlLocalizationNodelet']['ros__parameters']['globalmap_pcd']
    
    # Construct full path
    # default_map_path = os.path.join(amr_hdl_localization_dir, 'data', map_filename)
    default_map_path = os.path.join(amr_sim_dir, 'data', 'pcd', map_filename)

    # Point cloud converter launch
    pointcloud_converter_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([amr_pointcloud_converter_dir, 'launch', 'pcl_converter.launch.py'])
        ),
        launch_arguments={
            'hdl_localization_param_file': param_file_path,
        }.items()
    )

    # EKF node
    ekf_node = Node(
        name='ekf_param_file',
        package='robot_localization',
        executable='ekf_node',
        # parameters=[PathJoinSubstitution([amr_hdl_localization_dir, 'param', 'robot_localization_ekf.yaml'])],
        parameters=[PathJoinSubstitution([amr_sim_dir, 'param', robot_name, 'hdl_localization_simulation_ekf.yaml'])],
        output='screen'
    )

    # Global localization service node
    global_localization_node = Node(
        package='amr_hdl_global_localization',
        executable='amr_hdl_global_localization_node',
        name='GlobalLocalizationNode',
        namespace='hdl_global_localization',
        parameters=[
            PathJoinSubstitution([amr_hdl_global_localization_dir, 'config', 'general_config_fpfh.yaml']),  # FPFH_RANSAC 엔진 사용
            PathJoinSubstitution([amr_hdl_global_localization_dir, 'config', 'bbs_config.yaml']),
            PathJoinSubstitution([amr_hdl_global_localization_dir, 'config', 'fpfh_config_improved.yaml']),  # 개선된 FPFH 설정
            PathJoinSubstitution([amr_hdl_global_localization_dir, 'config', 'ransac_config_improved.yaml']),  # 개선된 RANSAC 설정
            PathJoinSubstitution([amr_hdl_global_localization_dir, 'config', 'teaser_config.yaml']),
            {'globalmap_pcd': default_map_path}
        ],
        output='screen'
    )
    # Composable node container for HDL localization
    hdl_localization_container = ComposableNodeContainer(
        name='container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='amr_hdl_localization',
                plugin='hdl_localization::GlobalmapServerNodelet',
                name='GlobalmapServerNodelet',
                parameters=[param_file_path, {'globalmap_pcd': default_map_path, 'use_sim_time': use_sim_time}]
            ),
            ComposableNode(
                package='amr_hdl_localization',
                plugin='hdl_localization::HdlLocalizationNodelet',
                name='HdlLocalizationNodelet',
                parameters=[param_file_path, {'globalmap_pcd': default_map_path, 'use_sim_time': use_sim_time}]
            )
        ],
        output='screen'
    )

    # RViz node
    rviz_node = Node(
        name='rviz',
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([amr_hdl_localization_dir, 'rviz', 'hdl_localization_ros2.rviz'])],
        output='screen'
    )

    # Optional lidar TF node (commented out)
    static_tf_node = Node(
        name='lidar_tf',
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=['0.0', '0.0', '0.0', '0', '0', '0', '1', 'odom', 'base_link'],
        output='screen'
    )

    # Group all actions
    hdl_localization_group = GroupAction([
        # pointcloud_converter_launch,
        # ekf_node,  # EKF node enabled to publish odom -> base_footprint transform
        # static_tf_node,  # Uncomment if needed
        global_localization_node,  # Global localization service node
        hdl_localization_container,
        # rviz_node,
    ])

    # Create launch description
    ld = LaunchDescription()
    # Add launch arguments
    ld.add_action(declare_use_sim_time_cmd)
    # Add group action
    ld.add_action(hdl_localization_group)
    # Add global parameters
    ld.add_action(launch_ros.actions.SetParameter(name='use_sim_time', value=True))

    return ld    