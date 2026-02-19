# Launch Nav2 with Aurora: odom, scan, and map from Aurora SDK.
# No SLAM, no AMCL - Aurora provides localization and map.

import os
import sys

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, GroupAction, SetEnvironmentVariable, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import SetRemap, Node

def generate_launch_description():
    # Force Fast DDS to use UDP only (avoids SHM port conflicts when multiple ROS processes run).
    # For more reliable Nav2 lifecycle bringup, use Cyclone in *all* terminals:
    #   export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp  (install: ros-humble-rmw-cyclonedds-cpp)
    set_fastdds_udp = SetEnvironmentVariable(name='FASTDDS_BUILTIN_TRANSPORTS', value='UDPv4')

    bringup_dir = get_package_share_directory('nav2_bringup')
    ugv_nav_dir = get_package_share_directory('ugv_nav')
    launch_dir = os.path.join(bringup_dir, 'launch')

    use_rviz = LaunchConfiguration('use_rviz', default='false')
    aurora_ip = LaunchConfiguration('aurora_ip', default='192.168.11.1')

    params_file = os.path.join(ugv_nav_dir, 'param', 'nav_aurora.yaml')

    # So Nav2 global costmap gets Aurora's map (Aurora publishes to slamware_map)
    nav_group = GroupAction(
        actions=[
            SetRemap(src='/map', dst='/slamware_ros_sdk_server_node/map'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(launch_dir, 'navigation_launch.py')
                ),
                launch_arguments={
                    'namespace': '',
                    'use_sim_time': 'false',
                    'autostart': 'false',  # Manual lifecycle script; composition needs a container we don't provide
                    'params_file': params_file,
                    'use_composition': 'False',
                    'use_respawn': 'False',
                    'container_name': 'nav2_container',
                }.items(),
            ),
        ]
    )

    # TF: Nav2 uses global_frame "map"; Aurora SDK uses "slamware_map". Publish map->slamware_map (identity)
    # so the map frame exists. Aurora must be running (slamware_map->odom->base_link).
    map_to_slamware_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_slamware_map',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'map', '--child-frame-id', 'slamware_map'],
    )
    # Nav2 expects base_footprint; Aurora/URDF use base_link.
    base_footprint_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_base_footprint',
        arguments=['--x', '0', '--y', '0', '--z', '0', '--yaw', '0', '--pitch', '0', '--roll', '0', '--frame-id', 'base_link', '--child-frame-id', 'base_footprint'],
    )

    # Delay Nav2 bringup so TF and services are ready (Aurora must publish map, odom, scan first).
    nav_group_delayed = TimerAction(period=15.0, actions=[nav_group])

    # Manual lifecycle STARTUP at 120s so all Nav2 nodes are fully initialized before configure/activate.
    ugv_nav_prefix = get_package_prefix('ugv_nav')
    startup_script = os.path.join(ugv_nav_prefix, 'lib', 'ugv_nav', 'nav_lifecycle_startup.py')
    lifecycle_startup = ExecuteProcess(
        cmd=[sys.executable, startup_script],
        name='nav_lifecycle_startup',
        output='screen',
    )
    lifecycle_startup_delayed = TimerAction(period=120.0, actions=[lifecycle_startup])

    ld = LaunchDescription()
    ld.add_action(set_fastdds_udp)
    ld.add_action(DeclareLaunchArgument('use_rviz', default_value='false', description='Launch RViz'))
    ld.add_action(DeclareLaunchArgument('aurora_ip', default_value='192.168.11.1', description='Aurora IP (for reference)'))
    ld.add_action(map_to_slamware_tf)
    ld.add_action(base_footprint_tf)
    ld.add_action(nav_group_delayed)
    ld.add_action(lifecycle_startup_delayed)
    return ld
