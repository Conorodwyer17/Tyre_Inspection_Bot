"""
Aurora SLAM Device Launch File

This launch file integrates SLAMTEC Aurora ROS2 SDK.
Based on official Aurora ROS2 SDK documentation.

Aurora SDK Topics (with namespace prefix by default):
- /slamware_ros_sdk_server_node/scan (sensor_msgs/LaserScan) - LiDAR data
- /slamware_ros_sdk_server_node/odom (nav_msgs/Odometry) - 6DOF odometry
- /slamware_ros_sdk_server_node/robot_pose (geometry_msgs/PoseStamped) - Robot pose
- /slamware_ros_sdk_server_node/map (nav_msgs/OccupancyGrid) - Map data
- /slamware_ros_sdk_server_node/point_cloud (sensor_msgs/PointCloud2) - Point cloud
- /slamware_ros_sdk_server_node/left_image_raw (sensor_msgs/Image) - Left camera
- /slamware_ros_sdk_server_node/right_image_raw (sensor_msgs/Image) - Right camera
- /slamware_ros_sdk_server_node/depth_image_raw (sensor_msgs/Image) - Depth image
- /slamware_ros_sdk_server_node/imu_raw_data (sensor_msgs/Imu) - IMU data

Aurora TF frames:
- map (map frame)
- base_link (robot base frame, configurable via robot_frame parameter)
- laser (LiDAR frame, configurable via laser_frame parameter)
- imu_link (IMU frame, configurable via imu_frame parameter)
- camera_left, camera_right (camera frames)
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
