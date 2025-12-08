"""
Gazebo Launch File: Launch file to start Gazebo with the warehouse world
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Get package share directory
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_warehouse_gazebo = FindPackageShare('warehouse_robot_gazebo').find('warehouse_robot_gazebo')
    
    # Path to world file
    world_file_path = os.path.join(pkg_warehouse_gazebo, 'worlds', 'empty_warehouse.world')
    
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    world = DeclareLaunchArgument(
        'world',
        default_value=world_file_path,
        description='Full path to world file to load'
    )
    
    # Gazebo server (runs physics)
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'verbose': 'true'
        }.items()
    )
    
    # Gazebo client (GUI)
    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )
    
    return LaunchDescription([
        use_sim_time,
        world,
        gzserver,
        gzclient
    ])