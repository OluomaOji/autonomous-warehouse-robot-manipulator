"""
Manipulation launch file for the warehouse robot in Gazebo with Nav2 localization.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Package paths
    pkg_description = get_package_share_directory('warehouse_robot_description')
    pkg_manipulation = get_package_share_directory('warehouse_robot_manipulation')
    pkg_gazebo = get_package_share_directory('warehouse_robot_gazebo')
    pkg_navigation = get_package_share_directory('warehouse_robot_navigation')

    # Gazebo launch
    gazebo_launch_file = os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')

    # Nav2 localization launch
    nav2_launch_file = os.path.join(pkg_navigation, 'launch', 'full_navigation.launch.py')
    map_yaml_path = os.path.join(pkg_navigation, 'maps', 'warehouse_map.yaml')

    return LaunchDescription([

        # 1️⃣ Launch Gazebo with robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gazebo_launch_file),
        ),

        # 2️⃣ Load controllers
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'joint_state_broadcaster'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'arm_controller'],
            output='screen'
        ),
        ExecuteProcess(
            cmd=['ros2', 'control', 'load_controller', '--set-state', 'active', 'gripper_controller'],
            output='screen'
        ),

        # 3️⃣ Launch Nav2 localization using your map
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={'map': map_yaml_path}.items()
        ),

        # 4 Launch your pick_and_place.py node (MoveIt handled inside Python)
        Node(
            package='warehouse_robot_manipulation',
            executable='pick_and_place_node.py',
            name='pick_and_place_node',
            output='screen'
        )
    ])
