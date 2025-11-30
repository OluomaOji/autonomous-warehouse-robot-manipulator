import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    # Get package share directory
    pkg_share = FindPackageShare('warehouse_robot_description').find('warehouse_robot_description')

    # Paths to files
    urdf_model_path = os.path.join(pkg_share, 'urdf','robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_share,'rviz','robot_view.rviz')

    # Declare Launches
    urdf_model = DeclareLaunchArgument(
        name="urdf_model",
        default_value=urdf_model_path,
        description="Path to the urdf file"
    )

    rviz_config = DeclareLaunchArgument(
        name="rviz_config",
        default_value=rviz_config_path,
        description="Path to the Rviz file"
    )

    # Process the URDF file
    robot_description_content = ParameterValue(
        Command(['xacro'])
    )