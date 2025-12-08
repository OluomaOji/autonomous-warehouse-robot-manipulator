"""
Warehouse Robot Spawn Launch File: Launch file to spawn the warehouse robot in Gazebo
"""

import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package directories
    pkg_warehouse_description = FindPackageShare('warehouse_robot_description').find('warehouse_robot_description')
    pkg_warehouse_gazebo = FindPackageShare('warehouse_robot_gazebo').find('warehouse_robot_gazebo')
    
    # Paths
    default_urdf_path = os.path.join(pkg_warehouse_description, 'urdf', 'robot.urdf.xacro')
    default_world_path = os.path.join(pkg_warehouse_gazebo, 'worlds', 'empty_warehouse.world')
    
    # Launch arguments
    urdf_model = DeclareLaunchArgument(
        name='urdf_model',
        default_value=default_urdf_path,
        description='Absolute path to robot URDF file'
    )

     
    # Declare launch arguments
    use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    world = DeclareLaunchArgument(
        name='world',
        default_value=default_world_path,
        description='Full path to world file'
    )
    
    x_pose = DeclareLaunchArgument(
        name='x_pose',
        default_value='3.0',
        description='X position to spawn robot'
    )
    
    y_pose = DeclareLaunchArgument(
        name='y_pose',
        default_value='3.0',
        description='Y position to spawn robot'
    )
    
    z_pose = DeclareLaunchArgument(
        name='z_pose',
        default_value='0.2',
        description='Z position to spawn robot'
    )
    
    # Include robot description launch
    robot_description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_warehouse_description, 'launch', 'description.launch.py')
        ]),
        launch_arguments={
            'urdf_model': LaunchConfiguration('urdf_model')
        }.items()
    )
    
    # Include Gazebo launch
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(pkg_warehouse_gazebo, 'launch', 'gazebo.launch.py')
        ]),
        launch_arguments={
            'world': LaunchConfiguration('world')
        }.items()
    )
    
    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'warehouse_robot',
            '-x', LaunchConfiguration('x_pose'),
            '-y', LaunchConfiguration('y_pose'),
            '-z', LaunchConfiguration('z_pose')
        ],
        output='screen'
    )
    
    return LaunchDescription([
        use_sim_time,
        urdf_model,
        world,
        x_pose,
        y_pose,
        z_pose,
        robot_description,
        gazebo,
        spawn_entity
    ])