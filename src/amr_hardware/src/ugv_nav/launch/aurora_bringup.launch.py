"""
Launch the SLAMTEC Aurora ROS 2 SDK node.

Brings up the Aurora driver so we get LiDAR scan, 6DOF odom, map, point cloud,
left/right/depth images, and IMU. Topics are under slamware_ros_sdk_server_node;
TF includes map, base_link, laser, imu_link, camera_left, camera_right.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource, XMLLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Aurora ROS2 SDK package name
    aurora_package = 'slamware_ros_sdk'
    
    # Aurora SDK launch file (XML format)
    aurora_launch_file = 'slamware_ros_sdk_server_and_view.xml'
    
    # Check if Aurora package exists
    try:
        aurora_dir = get_package_share_directory(aurora_package)
        aurora_launch_path = os.path.join(aurora_dir, 'launch', aurora_launch_file)
        
        if not os.path.exists(aurora_launch_path):
            # Try alternative path
            aurora_launch_path = os.path.join(aurora_dir, aurora_launch_file)
            if not os.path.exists(aurora_launch_path):
                raise FileNotFoundError(f"Aurora launch file not found: {aurora_launch_path}")
        
        # Include Aurora driver launch
        # Default IP: 192.168.11.1 (Aurora AP mode)
        # Note: Aurora SDK uses XML launch files
        aurora_launch = IncludeLaunchDescription(
            XMLLaunchDescriptionSource(aurora_launch_path),
            launch_arguments={
                'ip_address': LaunchConfiguration('ip_address', default='192.168.11.1'),
            }.items()
        )
        
        return LaunchDescription([
            DeclareLaunchArgument(
                'ip_address',
                default_value='192.168.11.1',
                description='IP address of Aurora device (default: 192.168.11.1 for AP mode)'
            ),
            aurora_launch,
        ])
        
    except Exception as e:
        # If Aurora package not found, return a warning launch description
        from launch.actions import LogInfo
        
        return LaunchDescription([
            LogInfo(msg=f'[WARNING] Aurora ROS2 SDK not found: {e}'),
            LogInfo(msg='Please install SLAMTEC Aurora ROS2 SDK from: https://www.slamtec.com/en/aurora'),
            LogInfo(msg='Expected package: slamware_ros_sdk'),
            LogInfo(msg='Expected launch file: slamware_ros_sdk_server_and_view.xml'),
            LogInfo(msg='After installation, ensure LD_LIBRARY_PATH includes aurora_remote_public library path'),
        ])
