#!/usr/bin/env python3
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
    # Declare launch argument for LiDAR port
    # Check which port exists
    default_port = '/dev/ttyACM1' if os.path.exists('/dev/ttyACM1') else '/dev/ttyACM0'
    port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=default_port,
        description='Serial port for LiDAR (e.g., /dev/ttyACM0 or /dev/ttyACM1)'
    )

    LDLIDAR_MODEL = os.environ.get('LDLIDAR_MODEL', 'ld19')
    ldlidar_launch_file = LDLIDAR_MODEL + '.launch.py'
    
    laser_bringup_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ldlidar'), 'launch/'),
             ldlidar_launch_file]
        ),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port')
        }.items()
    )

    ld = LaunchDescription()
    ld.add_action(port_arg)
    ld.add_action(laser_bringup_launch)

    return ld
