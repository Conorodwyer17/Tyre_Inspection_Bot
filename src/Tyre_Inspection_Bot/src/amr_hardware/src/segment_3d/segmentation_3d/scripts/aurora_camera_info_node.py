#!/usr/bin/env python3
"""Publishes sensor_msgs/CameraInfo for Aurora depth/left camera."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header


class AuroraCameraInfoNode(Node):
    def __init__(self):
        super().__init__("aurora_camera_info")
        self.declare_parameter("image_width", 640)
        self.declare_parameter("image_height", 480)
        self.declare_parameter("frame_id", "camera_depth_optical_frame")
        self.declare_parameter("fx", 525.0)
        self.declare_parameter("fy", 525.0)
        self.declare_parameter("cx", 319.5)
        self.declare_parameter("cy", 239.5)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")

        self.width = self.get_parameter("image_width").value
        self.height = self.get_parameter("image_height").value
        self.frame_id = self.get_parameter("frame_id").value
        fx = self.get_parameter("fx").value
        fy = self.get_parameter("fy").value
        cx = self.get_parameter("cx").value
        cy = self.get_parameter("cy").value
        topic = self.get_parameter("camera_info_topic").value

        self.pub = self.create_publisher(CameraInfo, topic, 10)
        rate_hz = self.get_parameter("publish_rate").value
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.get_logger().info(
            f"Publishing Aurora camera_info to {topic}: {self.width}x{self.height} frame={self.frame_id}"
        )

    def _publish(self):
        msg = CameraInfo()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height = self.height
        msg.width = self.width
        msg.distortion_model = "plumb_bob"
        msg.d = self.D
        msg.k = self.K
        msg.r = self.R
        msg.p = self.P
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = AuroraCameraInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
