# Launch Aurora SDK Bridge (depth pipeline for classic Aurora)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("aurora_sdk_bridge")
    calib_default = os.path.join(pkg, "config", "equidistant_calibration.yaml")
    # Prefer Phase C verified calibration if present
    try:
        pkg_seg = get_package_share_directory("segmentation_3d")
        calib_verified = os.path.join(pkg_seg, "config", "stereo_calibration_verified.yaml")
        calib = calib_verified if os.path.isfile(calib_verified) else calib_default
    except Exception:
        calib = calib_default

    calib_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=calib,
        description="Path to equidistant stereo calibration YAML",
    )

    bridge = Node(
        package="aurora_sdk_bridge",
        executable="aurora_sdk_bridge_node",
        name="aurora_sdk_bridge",
        output="screen",
        parameters=[{
            "calibration_file": LaunchConfiguration("calibration_file"),
            "left_image_topic": "/slamware_ros_sdk_server_node/left_image_raw",
            "right_image_topic": "/slamware_ros_sdk_server_node/right_image_raw",
        }],
    )

    return LaunchDescription([calib_arg, bridge])
