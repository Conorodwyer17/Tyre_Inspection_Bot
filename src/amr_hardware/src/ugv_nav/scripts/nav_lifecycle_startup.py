#!/usr/bin/env python3
# Call Nav2 lifecycle_manager manage_nodes STARTUP once.
# Do not retry: if the first call partially succeeds, nodes end up CONFIGURED/ACTIVE,
# and a second STARTUP would send CONFIGURE again and fail with "transition not registered".

import sys

import rclpy
from rclpy.node import Node
from nav2_msgs.srv import ManageLifecycleNodes


def main():
    rclpy.init()
    node = Node('nav_lifecycle_startup')
    client = node.create_client(
        ManageLifecycleNodes,
        '/lifecycle_manager_navigation/manage_nodes'
    )
    if not client.wait_for_service(timeout_sec=60.0):
        node.get_logger().error('manage_nodes service not available')
        rclpy.shutdown()
        return 1

    req = ManageLifecycleNodes.Request()
    req.command = ManageLifecycleNodes.Request.STARTUP

    node.get_logger().info('Calling manage_nodes STARTUP (single attempt)')
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future, timeout_sec=180.0)
    if not future.done():
        node.get_logger().error('Service call timed out')
        rclpy.shutdown()
        return 1
    try:
        resp = future.result()
        if resp.success:
            node.get_logger().info('Nav2 lifecycle startup succeeded')
            rclpy.shutdown()
            return 0
        node.get_logger().error('manage_nodes returned success=false')
    except Exception as e:
        node.get_logger().error(f'Service call failed: {e}')
    rclpy.shutdown()
    return 1


if __name__ == '__main__':
    sys.exit(main())
