"""
SLAM TOOLBOX - LIVE MAPPING
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Get package paths
    slam_pkg_path = get_package_share_directory("warehouse_robot_navigation")
    slam_params_directory = os.path.join(slam_pkg_path,'config','slam_params.yaml')

    # Declare Launch Arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation if true'
    )

    slam_node = Node(
        name="slam_toolbox",
        package ="slam_toolbox",
        executable="async_slam_toolbox_node",
        output="screen",
        parameters=[slam_params_directory,{'use_sim_time':LaunchConfiguration('use_sim_time')}]
    )

    return LaunchDescription([
        use_sim_time,
        slam_node
    ])

