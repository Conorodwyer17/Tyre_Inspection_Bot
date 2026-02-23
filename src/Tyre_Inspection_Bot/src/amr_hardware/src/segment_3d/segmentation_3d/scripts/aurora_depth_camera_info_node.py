#!/usr/bin/env python3
"""
Publishes sensor_msgs/CameraInfo for Aurora firmware 2.11 depth_image_raw.

Depth resolution: 224×416 (height×width). Intrinsics loaded from YAML.
Required for depth_to_registered_pointcloud to produce correct point cloud geometry.
"""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
import yaml


class AuroraDepthCameraInfoNode(Node):
    def __init__(self):
        super().__init__("aurora_depth_camera_info")
        self.declare_parameter("intrinsics_file", "")
        self.declare_parameter("camera_info_topic", "/camera/depth/camera_info")
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("frame_id", "camera_depth_optical_frame")

        intrinsics_file = self.get_parameter("intrinsics_file").value
        topic = self.get_parameter("camera_info_topic").value
        rate_hz = self.get_parameter("publish_rate").value
        self.frame_id = self.get_parameter("frame_id").value

        # Load intrinsics from YAML or use defaults for 416×224
        if intrinsics_file and os.path.isfile(intrinsics_file):
            with open(intrinsics_file) as f:
                cfg = yaml.safe_load(f)
            # Support nested key (aurora_depth_camera_info) or flat
            c = cfg.get("aurora_depth_camera_info", cfg)
            self.width = int(c.get("image_width", 416))
            self.height = int(c.get("image_height", 224))
            fx = float(c.get("fx", 180.0))
            fy = float(c.get("fy", 180.0))
            cx = float(c.get("cx", 208.0))
            cy = float(c.get("cy", 112.0))
            self.get_logger().info(
                f"Loaded intrinsics from {intrinsics_file}: {self.width}x{self.height} fx={fx:.1f}"
            )
        else:
            # Default for Aurora 2.11 depth (224×416)
            self.width = 416
            self.height = 224
            fx = fy = 180.0
            cx, cy = 208.0, 112.0
            if intrinsics_file:
                self.get_logger().warn(f"Intrinsics file not found: {intrinsics_file}, using defaults")

        self.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
        self.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

        self.pub = self.create_publisher(CameraInfo, topic, 10)
        self.timer = self.create_timer(1.0 / rate_hz, self._publish)

        self.get_logger().info(
            f"Publishing Aurora depth CameraInfo to {topic}: {self.width}x{self.height} frame={self.frame_id}"
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
    node = AuroraDepthCameraInfoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
