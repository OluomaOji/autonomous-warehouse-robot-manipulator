""""
Warehouse World Launch File: Launch file to spawn the warehouse world and robot in Gazebo"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    # Package directory
    pkg_warehouse_gazebo = FindPackageShare('warehouse_robot_gazebo').find('warehouse_robot_gazebo')
    
    # Include spawn_robot launch (which includes everything)
    spawn_robot = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_warehouse_gazebo, 'launch', 'spawn_robot.launch.py')
        ])
    )
    
    return LaunchDescription([
        spawn_robot
    ])