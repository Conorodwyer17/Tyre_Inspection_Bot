#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

'''
Parameter Description:
---
- Set laser scan directon: 
  1. Set counterclockwise, example: {'laser_scan_dir': True}
  2. Set clockwise,        example: {'laser_scan_dir': False}
- Angle crop setting, Mask data within the set angle range:
  1. Enable angle crop fuction:
    1.1. enable angle crop,  example: {'enable_angle_crop_func': True}
    1.2. disable angle crop, example: {'enable_angle_crop_func': False}
  2. Angle cropping interval setting:
  - The distance and intensity data within the set angle range will be set to 0.
  - angle >= 'angle_crop_min' and angle <= 'angle_crop_max' which is [angle_crop_min, angle_crop_max], unit is degress.
    example:
      {'angle_crop_min': 135.0}
      {'angle_crop_max': 225.0}
      which is [135.0, 225.0], angle unit is degress.
'''

def generate_launch_description():
  # Declare launch argument for LiDAR port (default: /dev/ttyACM1, fallback to /dev/ttyACM0)
  # Check which port exists
  default_port = '/dev/ttyACM1' if os.path.exists('/dev/ttyACM1') else '/dev/ttyACM0'
  port_arg = DeclareLaunchArgument(
    'lidar_port',
    default_value=default_port,
    description='Serial port for LiDAR (e.g., /dev/ttyACM0 or /dev/ttyACM1)'
  )

  # LDROBOT LiDAR publisher node
  ldlidar_node = Node(
      package='ldlidar',
      executable='ldlidar_node',
      name='LD19',
      output='screen',
      parameters=[
        {'product_name': 'LDLiDAR_LD19'},
        {'topic_name': 'scan'},
        {'frame_id': 'base_lidar_link'},
        {'port_name': LaunchConfiguration('lidar_port')},
        {'port_baudrate': 230400},
        {'laser_scan_dir': True},
        {'enable_angle_crop_func': True},
        {'angle_crop_min': 225.0},
        {'angle_crop_max': 315.0}
      ]
  )

  # base_link to base_laser tf node
  base_footprint_to_laser_tf_node = Node(
    package='tf2_ros',
    executable='static_transform_publisher',
    name='base_footprint_to_base_laser_ld19',
    arguments=['0','0','0','0','0','0','base_footprint','base_lidar_link']
  )


  # Define LaunchDescription variable
  ld = LaunchDescription()

  ld.add_action(port_arg)
  ld.add_action(ldlidar_node)
 # ld.add_action(base_footprint_to_laser_tf_node)

  return ld
