# Launch Aurora SDK bridge + health monitor.
# Run slamware_ros_sdk_server_node separately (e.g. from ugv_nav).
# Use for tire inspection mission.
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bridge_share = get_package_share_directory("aurora_sdk_bridge")
    bridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bridge_share, "launch", "aurora_sdk_bridge.launch.py")
        ),
    )
    health_node = Node(
        package="aurora_interface",
        executable="aurora_health_monitor",
        name="aurora_health_monitor",
        output="screen",
    )
    return LaunchDescription([bridge_launch, health_node])
