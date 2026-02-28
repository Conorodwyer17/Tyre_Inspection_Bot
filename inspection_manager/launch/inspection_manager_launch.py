"""Launch skeleton for refactored inspection manager."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="inspection_manager",
                executable="manager_node",
                name="inspection_manager_refactor",
                output="screen",
            )
        ]
    )

