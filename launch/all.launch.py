import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ld = LaunchDescription()

    # 1) Simulator
    sim_pkg = get_package_share_directory('amr_sim')
    sim_action = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_pkg, 'launch', 'amr_simulator_launch.py')
        )
    )
    ld.add_action(sim_action)

    # 2) Core interface 
    core_pkg = get_package_share_directory('amr_sim')
    core_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(core_pkg, 'launch', 'amr_core_sim_launch.py')
        )
    )
    ld.add_action(
        TimerAction(
            period=10.0,      # 2초 대기
            actions=[core_launch]
        )
    )

    # 4) RViz 
    rviz_pkg = get_package_share_directory('caselab_rviz_plugin')
    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(rviz_pkg, 'launch', 'test.launch.py')
        )
    )
    ld.add_action(
        TimerAction(
            period=6.0,      
            actions=[rviz_launch]
        )
    )

    web_pkg = get_package_share_directory('webserver_interface_ros2')
    web_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(web_pkg, 'launch', 'webserver_interface_launch.py')
        )
    )
    
    # ld.add_action(
    #     TimerAction(
    #         period=5.0,     
    #         actions=[web_launch]
    #     )
    # )

    return ld
