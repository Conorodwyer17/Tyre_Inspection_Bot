"""Mission launch skeleton (live or replay mode)."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("bag", default_value=""),
            Node(
                package="inspection_manager",
                executable="manager_node",
                name="inspection_manager_refactor",
                output="screen",
                parameters=[{"bag_path": LaunchConfiguration("bag")}],
            ),
        ]
    )

