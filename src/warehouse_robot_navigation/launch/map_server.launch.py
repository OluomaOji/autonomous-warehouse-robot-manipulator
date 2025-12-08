"""
MAP SERVER : LOAD SAVED MAP AND LOCALIZATION
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python import get_package_share_directory

def generate_launch_description():
    # Load the Saved map directory
    map_pkg = get_package_share_directory('warehouse_robot_navigation')
    map_dir = os.path.join(map_pkg,'maps','warehouse_map.yaml')

    # Launch COnfigurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map', default=map_dir)

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation clock')
    
    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=map_dir,
        description='Full path to map file')

    # Map Server Node
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time,
                     'yaml_filename': map_yaml_file}]
    )


    # AMCL Node for Localization
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}]
    )

    # Nav2 Lifecycle Manager for Map Server
    nav2_lifecycle_manager = Node(
        name="lifecycle_manager_localization",
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        output = 'screen',
        parameters=[
            {'use_sim_time':use_sim_time},
            {'autostart':True},
            {'node_names':['map_server','amcl']}
        ]
        
    )
    return LaunchDescription([
        declare_use_sim_time,
        declare_map_yaml,
        map_server,
        amcl_node,
        nav2_lifecycle_manager
    ])