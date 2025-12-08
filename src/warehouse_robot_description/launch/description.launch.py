import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package paths
    pkg_path = get_package_share_directory('warehouse_robot_description')
    urdf_file = os.path.join(pkg_path, 'urdf', 'robot.urdf.xacro')
    rviz_config_path = os.path.join(pkg_path, 'rviz', 'robot_view.rviz')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    # Declare Launch Argument
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description="Use simulation time"
    )

    # Use xacro to generate robot_description
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str
    )


    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
             'use_sim_time': use_sim_time
             }]
    )

    # Joint State Publisher GUI
    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui"
    )

    # RViz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path]
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher_gui,
        rviz
    ])
