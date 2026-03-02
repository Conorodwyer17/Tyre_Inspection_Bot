#!/usr/bin/env python3
"""
cmd_vel Safety Watchdog — Optional industrial safety layer.

Subscribes to cmd_vel (or configurable topic). If no messages received within
timeout_s, publishes zero Twist to ensure robot stops. Pattern from acc-qcar2
qcar_watchdog_node.

Usage: ros2 run ugv_nav cmd_vel_watchdog_node.py
  (or add to launch with remaps)

Params:
  - input_topic: topic to monitor (default /cmd_vel)
  - output_topic: topic to publish to (default /cmd_vel — requires remap in launch)
  - timeout: seconds without input before publishing STOP (default 0.5)

For integration: remap so controller -> watchdog_in, watchdog_out -> velocity_smoother.
Or run standalone monitoring input_topic; publishes to output_topic only when timeout.
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelWatchdog(Node):
    """Publishes STOP when no cmd_vel received within timeout."""

    def __init__(self):
        super().__init__("cmd_vel_watchdog")
        self.declare_parameter("input_topic", "/cmd_vel")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("timeout", 0.5)
        self.declare_parameter("watchdog_rate_hz", 20.0)

        in_topic = self.get_parameter("input_topic").value
        out_topic = self.get_parameter("output_topic").value
        self.timeout = self.get_parameter("timeout").value
        rate_hz = self.get_parameter("watchdog_rate_hz").value

        self._last_cmd_time = self.get_clock().now()
        self._stop_published = False

        self.sub = self.create_subscription(Twist, in_topic, self._cmd_cb, 10)
        self.pub = self.create_publisher(Twist, out_topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._watchdog_cb)

        self.get_logger().info(
            f"cmd_vel watchdog: input={in_topic} output={out_topic} timeout={self.timeout}s"
        )

    def _cmd_cb(self, msg: Twist):
        self._last_cmd_time = self.get_clock().now()
        self._stop_published = False
        self.pub.publish(msg)  # Forward when received

    def _watchdog_cb(self):
        now = self.get_clock().now()
        dt = (now.nanoseconds - self._last_cmd_time.nanoseconds) * 1e-9
        if dt > self.timeout:
            if not self._stop_published:
                self.get_logger().warn_throttle(5.0, f"cmd_vel timeout ({dt:.1f}s); publishing STOP")
                self._stop_published = True
            stop = Twist()
            stop.linear.x = 0.0
            stop.linear.y = 0.0
            stop.linear.z = 0.0
            stop.angular.x = 0.0
            stop.angular.y = 0.0
            stop.angular.z = 0.0
            self.pub.publish(stop)


def main():
    rclpy.init()
    node = CmdVelWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
