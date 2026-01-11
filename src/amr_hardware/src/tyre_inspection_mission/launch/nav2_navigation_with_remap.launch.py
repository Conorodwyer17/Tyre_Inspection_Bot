#!/usr/bin/env python3
"""
Nav2 Navigation Launch with cmd_vel Remapping

This is a custom wrapper around nav2_bringup's navigation_launch.py that
remaps Nav2's controller cmd_vel output from /cmd_vel to /cmd_vel/nav2.

CRITICAL: This is needed because Nav2's controller_server publishes directly
to /cmd_vel, which conflicts with our cmd_vel_multiplexer. By remapping to
/cmd_vel/nav2, the multiplexer can handle it as Priority 3.

This wrapper launches Nav2's navigation stack manually (instead of using
IncludeLaunchDescription) so we can add remapping to the controller_server node.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    ugv_nav_pkg = get_package_share_directory('ugv_nav')
    
    # Get default params file path
    default_params_file = os.path.join(ugv_nav_pkg, 'param', 'slam_nav.yaml')
    
    # Declare launch arguments (same as nav2_bringup navigation_launch.py)
    namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='false', description='Whether to apply a namespace')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='False', description='Use sim time')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='True', description='Automatically startup nav2 stack')
    params_file_arg = DeclareLaunchArgument('params_file', default_value=default_params_file, description='Full path to params file')
    use_composition_arg = DeclareLaunchArgument('use_composition', default_value='False', description='Use composed bringup')
    container_name_arg = DeclareLaunchArgument('container_name', default_value='nav2_container', description='Container name')
    use_respawn_arg = DeclareLaunchArgument('use_respawn', default_value='False', description='Respawn nodes on crash')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='Log level')
    
    # Get launch arguments
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file_config = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Standard remappings
    remappings = [('/tf', 'tf'), ('/tf_static', 'tf_static')]
    
    # CRITICAL: Add remapping for controller_server's cmd_vel output
    # Remap from /cmd_vel to /cmd_vel/nav2 so multiplexer can handle it as Priority 3
    # CRITICAL FIX: Use absolute topic names ('/cmd_vel' not 'cmd_vel') because Nav2 publishes to absolute topic
    controller_remappings = remappings + [('/cmd_vel', '/cmd_vel/nav2')]
    behavior_remappings = remappings + [('/cmd_vel', '/cmd_vel/nav2')]
    velocity_smoother_remappings = remappings + [('/cmd_vel', '/cmd_vel/nav2')]
    
    # Configure params - use params_file LaunchConfiguration directly
    # nav2_bringup's navigation_launch.py uses this same approach
    # Note: params_file_config is a LaunchConfiguration, so it will use the value
    # passed via launch_arguments, or the default if not provided
    # The main launch always passes nav2_params, so this should work
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file_config,  # LaunchConfiguration - uses value from launch_arguments or default
            root_key=namespace,
            param_rewrites={'autostart': autostart},
            convert_types=True,
        ),
        allow_substs=True,
    )
    
    stdout_linebuf_envvar = SetEnvironmentVariable(
        'RCUTILS_LOGGING_BUFFERED_STREAM', '1'
    )
    
    # Lifecycle nodes - must match nodes launched below
    # Note: velocity_smoother and collision_monitor are not lifecycle nodes,
    # they're regular nodes that subscribe/publish
    lifecycle_nodes = [
        'controller_server',
        'smoother_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
    ]
    
    # Load nodes individually (not using composition for now)
    # CRITICAL: SetParameter is not available in ROS 2 Humble launch.actions
    # Instead, we pass use_sim_time via parameters to each node
    load_nodes = GroupAction(
        condition=IfCondition(PythonExpression(['not ', use_composition])),
        actions=[
            # CRITICAL: Controller server with cmd_vel remapping
            Node(
                package='nav2_controller',
                executable='controller_server',
                name='controller_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],  # Set use_sim_time via parameters
                arguments=['--ros-args', '--log-level', log_level],
                remappings=controller_remappings,  # Remaps cmd_vel to cmd_vel/nav2
            ),
            # Smoother server (no remapping needed)
            Node(
                package='nav2_smoother',
                executable='smoother_server',
                name='smoother_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # Planner server (no remapping needed)
            Node(
                package='nav2_planner',
                executable='planner_server',
                name='planner_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # Behavior server with cmd_vel remapping
            Node(
                package='nav2_behaviors',
                executable='behavior_server',
                name='behavior_server',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=behavior_remappings,  # Remaps cmd_vel to cmd_vel/nav2
            ),
            # BT Navigator
            Node(
                package='nav2_bt_navigator',
                executable='bt_navigator',
                name='bt_navigator',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # Waypoint follower
            Node(
                package='nav2_waypoint_follower',
                executable='waypoint_follower',
                name='waypoint_follower',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # Velocity smoother - also needs cmd_vel remapping
            Node(
                package='nav2_velocity_smoother',
                executable='velocity_smoother',
                name='velocity_smoother',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=velocity_smoother_remappings,  # Remaps cmd_vel to cmd_vel/nav2
            ),
            # Collision monitor (no remapping needed)
            Node(
                package='nav2_collision_monitor',
                executable='collision_monitor',
                name='collision_monitor',
                output='screen',
                respawn=use_respawn,
                respawn_delay=2.0,
                parameters=[configured_params, {'use_sim_time': use_sim_time}],
                arguments=['--ros-args', '--log-level', log_level],
                remappings=remappings,
            ),
            # Lifecycle manager (manages lifecycle nodes only)
            Node(
                package='nav2_lifecycle_manager',
                executable='lifecycle_manager',
                name='lifecycle_manager_navigation',
                output='screen',
                arguments=['--ros-args', '--log-level', log_level],
                parameters=[
                    {'use_sim_time': use_sim_time},
                    {'autostart': autostart},
                    {'node_names': lifecycle_nodes},
                ],
            ),
        ],
    )
    
    ld = LaunchDescription()
    ld.add_action(stdout_linebuf_envvar)
    ld.add_action(namespace_arg)
    ld.add_action(use_namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(autostart_arg)
    ld.add_action(params_file_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(container_name_arg)
    ld.add_action(use_respawn_arg)
    ld.add_action(log_level_arg)
    ld.add_action(PushRosNamespace(condition=IfCondition(use_namespace), namespace=namespace))
    ld.add_action(load_nodes)
    
    return ld
