from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    trucks_file = LaunchConfiguration("trucks_file")
    detection_topic = LaunchConfiguration("detection_topic")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "trucks_file",
                default_value="",
                description="Path to trucks YAML (falls back to package config).",
            ),
            DeclareLaunchArgument(
                "detection_topic",
                default_value="/darknet_ros_3d/bounding_boxes",
                description="Topic for gb_visual_detection_3d_msgs/BoundingBoxes3d (Aurora detection output).",
            ),
            DeclareLaunchArgument(
                "use_dynamic_detection",
                default_value="true",
                description="Use dynamic vehicle detection from bounding boxes (true) or YAML file (false).",
            ),
            DeclareLaunchArgument(
                "vehicle_labels",
                default_value="car,truck",
                description="Comma-separated list of vehicle class names to detect (e.g., 'car,truck').",
            ),
            DeclareLaunchArgument(
                "tire_label",
                default_value="tire",
                description="Tire class name for detection.",
            ),
            Node(
                package="inspection_manager",
                executable="inspection_manager_node",
                name="inspection_manager",
                output="screen",
                parameters=[
                    {"trucks_file": trucks_file},
                    {"detection_topic": detection_topic},
                    {"use_dynamic_detection": LaunchConfiguration("use_dynamic_detection")},
                    {"vehicle_labels": LaunchConfiguration("vehicle_labels")},
                    {"tire_label": LaunchConfiguration("tire_label")},
                ],
            ),
            Node(
                package="inspection_manager",
                executable="photo_capture_service",
                name="photo_capture_service",
                output="screen",
                parameters=[
                    {"camera_topic": "/slamware_ros_sdk_server_node/left_image_raw"},
                    {"save_directory": "~/ugv_ws/tire_inspection_photos"},
                    {"image_format": "jpg"},
                ],
            ),
        ]
    )

