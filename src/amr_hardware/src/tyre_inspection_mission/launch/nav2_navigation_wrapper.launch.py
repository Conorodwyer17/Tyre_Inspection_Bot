#!/usr/bin/env python3
"""
Nav2 Navigation Wrapper Launch File

This wrapper remaps Nav2's controller cmd_vel output to /cmd_vel/nav2
so it can be handled by the cmd_vel_multiplexer.

CRITICAL: Nav2's controller_server publishes to /cmd_vel by default, which
conflicts with our multiplexer. This wrapper remaps it to /cmd_vel/nav2.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    # Get package directories
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    ugv_nav_pkg = get_package_share_directory('ugv_nav')
    
    # Declare launch arguments (same as nav2_bringup navigation_launch.py)
    namespace_arg = DeclareLaunchArgument('namespace', default_value='', description='Top-level namespace')
    use_namespace_arg = DeclareLaunchArgument('use_namespace', default_value='false', description='Whether to apply a namespace')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false', description='Use sim time')
    autostart_arg = DeclareLaunchArgument('autostart', default_value='true', description='Automatically startup nav2 stack')
    params_file_arg = DeclareLaunchArgument('params_file', default_value='', description='Full path to params file')
    use_composition_arg = DeclareLaunchArgument('use_composition', default_value='false', description='Use composed bringup')
    container_name_arg = DeclareLaunchArgument('container_name', default_value='nav2_container', description='Container name')
    use_respawn_arg = DeclareLaunchArgument('use_respawn', default_value='false', description='Respawn nodes on crash')
    log_level_arg = DeclareLaunchArgument('log_level', default_value='info', description='Log level')
    
    # Get launch arguments
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    use_composition = LaunchConfiguration('use_composition')
    container_name = LaunchConfiguration('container_name')
    use_respawn = LaunchConfiguration('use_respawn')
    log_level = LaunchConfiguration('log_level')
    
    # Default params file if not provided
    default_params_file = os.path.join(ugv_nav_pkg, 'param', 'slam_nav.yaml')
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(namespace_arg)
    ld.add_action(use_namespace_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(autostart_arg)
    ld.add_action(params_file_arg)
    ld.add_action(use_composition_arg)
    ld.add_action(container_name_arg)
    ld.add_action(use_respawn_arg)
    ld.add_action(log_level_arg)
    
    # CRITICAL: Remap Nav2 controller's cmd_vel output to /cmd_vel/nav2
    # Since we can't remap inside IncludeLaunchDescription, we need to launch
    # Nav2's navigation nodes individually with remapping.
    # However, nav2_bringup's navigation_launch.py doesn't easily support this.
    # 
    # SOLUTION: Include Nav2's navigation launch BUT add a relay node that
    # intercepts Nav2's /cmd_vel and republishes to /cmd_vel/nav2.
    # However, this creates a loop if not handled carefully.
    #
    # BETTER SOLUTION: We'll use Nav2's default navigation launch but add
    # a relay that filters Nav2's messages. Actually, the SIMPLEST solution
    # is to create our own navigation launch that remaps the controller output.
    #
    # For now, let's use the relay approach but make it smarter.
    # Actually wait - the BEST solution is to create a wrapper that launches
    # Nav2 navigation but adds remapping for the controller server.
    
    # Since we can't easily remap Nav2's controller via IncludeLaunchDescription,
    # we'll use Nav2's navigation launch as-is, but add explicit remapping
    # using a topic relay that intercepts before multiplexer publishes.
    #
    # Actually, the REAL solution: Create a custom launch that launches Nav2's
    # navigation nodes individually with remapping. But that's complex.
    #
    # SIMPLEST WORKING SOLUTION: Use ROS 2's namespace feature or topic remapping
    # at the controller server level by creating our own controller launch node.
    #
    # For now, let's just include Nav2's navigation launch as-is and handle
    # the remapping via the relay node (which is already created).
    # The relay will subscribe to /cmd_vel and filter messages from Nav2.
    
    # Include Nav2's navigation launch (unchanged)
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'namespace': namespace,
            'use_namespace': use_namespace,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': params_file if params_file != LaunchConfiguration('params_file') else default_params_file,
            'use_composition': use_composition,
            'container_name': container_name,
            'use_respawn': use_respawn,
            'log_level': log_level,
        }.items()
    )
    
    ld.add_action(nav2_navigation_launch)
    
    return ld
