from nav2_msgs.action import NavigateToPose


def send_nav_goal(node, pose, done_cb) -> bool:
    """Send a NavigateToPose goal. Returns True if goal was sent, False if Nav2 unavailable."""
    if not node.nav_client.wait_for_server(timeout_sec=1.0):
        node.get_logger().error("Nav2 action server not available.")
        return False

    goal_msg = NavigateToPose.Goal()
    goal_msg.pose = pose
    node.pending_goal_handle = node.nav_client.send_goal_async(goal_msg)
    node.pending_goal_handle.add_done_callback(done_cb)
    return True
