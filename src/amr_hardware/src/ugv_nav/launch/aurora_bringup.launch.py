# Copyright 2024 UGV Tire Inspection
# Launch SLAMTEC Aurora SDK node (slamware_ros_sdk) with configurable IP.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    ip_address_arg = DeclareLaunchArgument(
        'ip_address',
        default_value='192.168.11.1',
        description='IP address of the SLAMTEC Aurora device',
    )

    slamware_node = Node(
        package='slamware_ros_sdk',
        executable='slamware_ros_sdk_server_node',
        name='slamware_ros_sdk_server_node',
        output='both',
        parameters=[{
            'ip_address': LaunchConfiguration('ip_address'),
            'angle_compensate': True,
            'map_frame': 'slamware_map',
            'robot_frame': 'base_link',
            'odom_frame': 'odom',
            'laser_frame': 'laser',
            'imu_frame': 'imu_link',
            'camera_left': 'camera_left',
            'camera_right': 'camera_right',
            'robot_pose_pub_period': 0.05,
            'scan_pub_period': 0.1,
            'map_pub_period': 0.2,
            'imu_raw_data_period': 0.005,
            'ladar_data_clockwise': True,
            'no_preview_image': False,
            'raw_image_on': False,
        }],
        remappings=[
            ('scan', 'scan'),
            ('odom', 'odom'),
            ('map', 'slamware_map'),
            ('map_metadata', 'map_metadata'),
        ],
    )

    # TF: map -> odom (Aurora provides odom relative to map)
    odom2map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom2map',
        arguments=['0', '0', '0', '0', '0', '0', '1', 'slamware_map', 'odom'],
    )
    # Camera and IMU frames (adjust if your URDF differs)
    leftcam2base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='leftcam2base',
        arguments=['0.0418', '0.03', '0', '-0.5', '0.5', '-0.5', '0.5', 'base_link', 'camera_left'],
    )
    rightcam2leftcam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='rightcam2Leftcam',
        arguments=['0.06', '0', '0', '0', '0', '0', '1', 'camera_left', 'camera_right'],
    )
    imu2leftcam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu2Leftcam',
        arguments=['0.03', '0', '0', '0', '0', '-0.7071068', '0.7071068', 'camera_left', 'imu_link'],
    )

    return LaunchDescription([
        ip_address_arg,
        slamware_node,
        odom2map,
        leftcam2base,
        rightcam2leftcam,
        imu2leftcam,
    ])
