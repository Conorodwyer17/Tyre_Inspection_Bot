#!/usr/bin/env python3
"""Aurora health monitor: checks odom, scan, map, stereo, TF; publishes status."""
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy
from std_msgs.msg import Bool, String
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import OccupancyGrid
import tf2_ros
import json
import time


class AuroraHealthMonitor(Node):
    def __init__(self):
        super().__init__("aurora_health_monitor")
        self.declare_parameter("odom_topic", "/slamware_ros_sdk_server_node/odom")
        self.declare_parameter("scan_topic", "/slamware_ros_sdk_server_node/scan")
        self.declare_parameter("map_topic", "/map")
        self.declare_parameter("left_topic", "/slamware_ros_sdk_server_node/left_image_raw")
        self.declare_parameter("right_topic", "/slamware_ros_sdk_server_node/right_image_raw")
        self.declare_parameter("world_frame", "slamware_map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("timeout_s", 2.0)

        self.last_odom = None
        self.last_scan = None
        self.last_map = None
        self.last_left = None
        self.last_right = None
        self.tf_valid = False
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        qos_be = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT)
        qos_rel = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=5,
                             reliability=QoSReliabilityPolicy.RELIABLE)

        odom_t = self.get_parameter("odom_topic").value
        scan_t = self.get_parameter("scan_topic").value
        map_t = self.get_parameter("map_topic").value
        left_t = self.get_parameter("left_topic").value
        right_t = self.get_parameter("right_topic").value

        def _ts(attr):
            return lambda m: setattr(self, attr, time.time())
        self.create_subscription(Odometry, odom_t, _ts("last_odom"), qos_be)
        self.create_subscription(LaserScan, scan_t, _ts("last_scan"), qos_be)
        self.create_subscription(OccupancyGrid, map_t, _ts("last_map"), qos_rel)
        self.create_subscription(Image, left_t, _ts("last_left"), qos_be)
        self.create_subscription(Image, right_t, _ts("last_right"), qos_be)

        self.healthy_pub = self.create_publisher(Bool, "aurora_interface/healthy", 10)
        self.diag_pub = self.create_publisher(String, "aurora_interface/diagnostics", 10)
        self.timer = self.create_timer(0.5, self._check)

    def _check(self):
        t = time.time()
        timeout = self.get_parameter("timeout_s").value
        wf = self.get_parameter("world_frame").value
        bf = self.get_parameter("base_frame").value

        try:
            to = rclpy.duration.Duration(seconds=0.5)
            self.tf_buffer.lookup_transform(wf, bf, rclpy.time.Time(), timeout=to)
            self.tf_valid = True
        except Exception:
            self.tf_valid = False

        def ok(last):
            return last is not None and (t - last) < timeout

        odom_ok = ok(self.last_odom)
        scan_ok = ok(self.last_scan)
        map_ok = ok(self.last_map)
        left_ok = ok(self.last_left)
        right_ok = ok(self.last_right)

        healthy = odom_ok and scan_ok and map_ok and left_ok and right_ok and self.tf_valid

        self.healthy_pub.publish(Bool(data=healthy))
        diag = {
            "odom": odom_ok,
            "scan": scan_ok,
            "map": map_ok,
            "left_image": left_ok,
            "right_image": right_ok,
            "tf": self.tf_valid,
            "healthy": healthy,
        }
        msg = String()
        msg.data = json.dumps(diag)
        self.diag_pub.publish(msg)


def main():
    rclpy.init()
    node = AuroraHealthMonitor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
