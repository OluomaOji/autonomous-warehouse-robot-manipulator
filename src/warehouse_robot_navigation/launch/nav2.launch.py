"""
NAV2 STACK : Path Planning and Controller
"""
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python import get_package_share_directory


def generate_launch_description():
    # Get the path planning directory
    pkg_nav = get_package_share_directory('warehouse_robot_navigation')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    # Declare Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description="Use simulation time"
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )

    amcl_and_map_server = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'map_server.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'map': LaunchConfiguration('map')
        }.items()
    )

    # Nav2 Bringup - Use bringup_launch.py which doesn't start map_server/amcl
    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': nav2_params,
            'use_composition': 'False',
            'use_respawn': 'False',
            'map':LaunchConfiguration('map')
        }.items()
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        amcl_and_map_server,
        nav2_bringup
    ])