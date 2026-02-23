# Launch Aurora SDK Bridge (depth pipeline for classic Aurora)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg = get_package_share_directory("aurora_sdk_bridge")
    # Optional: use bridge only if calibration YAML exists. Default: use Aurora native (use_bridge:=false).
    calib_default = os.path.join(pkg, "config", "equidistant_calibration.yaml")
    calib_bridge_verified = os.path.join(pkg, "config", "stereo_calibration_verified.yaml")
    if os.path.isfile(calib_bridge_verified):
        calib = calib_bridge_verified
    elif os.path.isfile(calib_default):
        calib = calib_default
    else:
        try:
            pkg_seg = get_package_share_directory("segmentation_3d")
            calib_seg = os.path.join(pkg_seg, "config", "stereo_calibration_verified.yaml")
            calib = calib_seg if os.path.isfile(calib_seg) else calib_default
        except Exception:
            calib = calib_default

    calib_arg = DeclareLaunchArgument(
        "calibration_file",
        default_value=calib,
        description="Path to equidistant stereo calibration YAML",
    )

    # Production stereo params (num_disparities=160, block_size=11) aligned with PRODUCTION_CONFIG.yaml
    stereo_params_file = os.path.join(pkg, "config", "stereo_production_params.yaml")
    if not os.path.isfile(stereo_params_file):
        stereo_params_file = None

    bridge_params = [{
        "calibration_file": LaunchConfiguration("calibration_file"),
        "left_image_topic": "/slamware_ros_sdk_server_node/left_image_raw",
        "right_image_topic": "/slamware_ros_sdk_server_node/right_image_raw",
    }]
    if stereo_params_file:
        bridge_params.append(stereo_params_file)

    bridge = Node(
        package="aurora_sdk_bridge",
        executable="aurora_sdk_bridge_node",
        name="aurora_sdk_bridge",
        output="screen",
        parameters=bridge_params,
    )

    return LaunchDescription([calib_arg, bridge])
