"""
Full Navigation Launch File - No Duplicate Nodes
This properly launches Gazebo, URDF, SLAM/Nav2 without conflicts
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    IncludeLaunchDescription, 
    GroupAction
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    pkg_nav = get_package_share_directory('warehouse_robot_navigation')
    pkg_desc = get_package_share_directory('warehouse_robot_description')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_warehouse_gazebo = get_package_share_directory('warehouse_robot_gazebo')

    # Paths
    urdf_file = os.path.join(pkg_desc, 'urdf', 'robot.urdf.xacro')
    world_file = os.path.join(pkg_warehouse_gazebo, 'worlds', 'empty_warehouse.world')
    rviz_config = os.path.join(pkg_nav, 'rviz', 'robot_view.rviz')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')

    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='true',
        description='Run SLAM or localization mode'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(pkg_nav, 'maps', 'warehouse_map.yaml'),
        description='Full path to map file'
    )

    # ========================================
    # 1. ROBOT DESCRIPTION (URDF)
    # ========================================
    robot_description = ParameterValue(
        Command([FindExecutable(name='xacro'), ' ', urdf_file]),
        value_type=str
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time
        }]
    )

    static_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_base_footprint',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ========================================
    # 2. GAZEBO
    # ========================================
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
        ),
        launch_arguments={'world': world_file, 'verbose': 'true'}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
        )
    )

    # Spawn robot in Gazebo
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'warehouse_robot',
            '-x', '3.0',
            '-y', '3.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    # ========================================
    # 3. SLAM TOOLBOX
    # ========================================
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'slam.launch.py')
        ),
        launch_arguments=[('use_sim_time', use_sim_time)]
    )

    # ========================================
    # 4. MAP SERVER + AMCL (Localization)
    # ========================================
    map_server_and_amcl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'map_server.launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('map', map_yaml_file)
        ]
    )   

    # ========================================
    # 5. NAV2 NAVIGATION STACK
    # ========================================
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'nav2.launch.py')
        ),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('map', map_yaml_file),
            ('slam', slam)
        ]
    )

    # ========================================
    # 6. RVIZ (Only ONE instance)
    # ========================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ========================================
    # 7. TELEOP
    # ========================================
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # ========================================
    # CONDITIONAL GROUPS
    # ========================================
    slam_group = GroupAction(
        actions=[slam_toolbox],
        condition=IfCondition(slam)
    )

    localization_group = GroupAction(
        actions=[map_server_and_amcl, nav2],
        condition=UnlessCondition(slam)
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim_time,
        declare_slam,
        declare_map_yaml,

        # Core components (always run)
        robot_state_publisher,  # ONE robot_state_publisher
        static_base_footprint,
        gzserver,
        gzclient,
        spawn_entity,

        # Conditional: SLAM or Localization
        slam_group,
        localization_group,

        # Visualization and control
        rviz_node,  # ONE RViz
        teleop_node
    ])