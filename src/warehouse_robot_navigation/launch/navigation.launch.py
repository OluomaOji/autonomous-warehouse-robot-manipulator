"""
UNIFIED NAVIGATION LAUNCH FILE
Handles everything: Robot State, Gazebo, SLAM/Localization, Nav2, RViz, Teleop
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.conditions import IfCondition, UnlessCondition


def generate_launch_description():

    # Package directories
    pkg_nav = get_package_share_directory('warehouse_robot_navigation')
    pkg_desc = get_package_share_directory('warehouse_robot_description')
    pkg_gazebo = get_package_share_directory('warehouse_robot_gazebo')

    # Config files
    nav2_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    map_yaml_file_path = os.path.join(pkg_nav, 'maps', 'warehouse_map.yaml')

    # Launch Configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    slam = LaunchConfiguration('slam')
    map_yaml_file = LaunchConfiguration('map')
    autostart = LaunchConfiguration('autostart')

    # Declare Launch Arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation clock'
    )

    declare_slam = DeclareLaunchArgument(
        'slam',
        default_value='True',
        description='Whether to run SLAM (True) or localization (False)'
    )

    declare_map_yaml = DeclareLaunchArgument(
        'map',
        default_value=map_yaml_file_path,
        description='Full path to map yaml file'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically start lifecycle nodes'
    )

    # =============================================================================
    # 1. ROBOT STATE PUBLISHER (URDF + TF)
    # =============================================================================
    robot_state_publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_desc, 'launch', 'description.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # =============================================================================
    # 2. GAZEBO WORLD AND ROBOT SPAWN
    # =============================================================================
    gazebo_world = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'warehouse_world.launch.py')
        )
    )

    # =============================================================================
    # 3. SLAM TOOLBOX (only when slam:=True)
    # =============================================================================
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav, 'launch', 'slam.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # =============================================================================
    # 4. MAP SERVER (only when slam:=False)
    # =============================================================================
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'yaml_filename': map_yaml_file
        }]
    )

    # =============================================================================
    # 5. AMCL LOCALIZATION (only when slam:=False)
    # =============================================================================
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[nav2_params]
    )

    # =============================================================================
    # 6. LIFECYCLE MANAGER FOR LOCALIZATION (only when slam:=False)
    # =============================================================================
    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': ['map_server', 'amcl']
        }]
    )

    # =============================================================================
    # 7. NAV2 NAVIGATION STACK (only when slam:=False)
    # =============================================================================
    
    # Planner Server
    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Controller Server
    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Behavior Tree Navigator
    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[nav2_params]
    )

    # Smoother Server
    smoother_server = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=[nav2_params]
    )

    # Velocity Smoother
    velocity_smoother = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[nav2_params]
    )

    # Waypoint Follower
    waypoint_follower = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[nav2_params]
    )

    # Lifecycle Manager for Navigation
    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
                'smoother_server',
                'velocity_smoother',
                'waypoint_follower'
            ]
        }]
    )

    # =============================================================================
    # 8. RVIZ
    # =============================================================================
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_nav, 'rviz', 'robot_view.rviz')],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # =============================================================================
    # 9. TELEOP
    # =============================================================================
    teleop_node = Node(
        package='teleop_twist_keyboard',
        executable='teleop_twist_keyboard',
        name='teleop_twist_keyboard',
        output='screen',
        prefix='xterm -e',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # =============================================================================
    # CONDITIONAL GROUPS
    # =============================================================================
    
    # SLAM MODE (slam:=True)
    slam_group = GroupAction(
        actions=[slam_toolbox],
        condition=IfCondition(slam)
    )

    # LOCALIZATION + NAVIGATION MODE (slam:=False)
    localization_group = GroupAction(
        actions=[
            map_server,
            amcl_node,
            lifecycle_manager_localization,
            planner_server,
            controller_server,
            bt_navigator,
            smoother_server,
            velocity_smoother,
            waypoint_follower,
            lifecycle_manager_navigation
        ],
        condition=UnlessCondition(slam)
    )

    # =============================================================================
    # LAUNCH DESCRIPTION
    # =============================================================================
    return LaunchDescription([
        # Declare arguments
        declare_use_sim_time,
        declare_slam,
        declare_map_yaml,
        declare_autostart,

        # Always launch these
        robot_state_publisher,
        gazebo_world,
        rviz_node,
        teleop_node,

        # Conditional: SLAM or Localization+Navigation
        slam_group,
        localization_group
    ])