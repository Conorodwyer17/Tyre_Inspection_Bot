#!/usr/bin/env python3
"""
Capture one PointCloud2 message and write PLY to file.
Usage: python3 scripts/ros_pc_to_ply.py --topic /camera/depth/points --out path/to/ros_pointcloud.ply
"""
import argparse
import sys
import struct

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--topic", default="/camera/depth/points", help="PointCloud2 topic")
    parser.add_argument("--out", default="/tmp/ros_pointcloud.ply", help="Output PLY path")
    parser.add_argument("--timeout", type=float, default=30.0, help="Seconds to wait for one message")
    args = parser.parse_args()

    rclpy.init()
    node = Node("ros_pc_to_ply")
    qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)
    msg = None

    def cb(m):
        nonlocal msg
        msg = m

    sub = node.create_subscription(PointCloud2, args.topic, cb, qos)
    import time
    t0 = time.monotonic()
    while rclpy.ok() and msg is None and (time.monotonic() - t0) < args.timeout:
        rclpy.spin_once(node, timeout_sec=0.5)
    node.destroy_node()
    rclpy.shutdown()

    if msg is None:
        print("No message received", file=sys.stderr)
        return 1

    points = list(point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True))
    with open(args.out, "w") as f:
        f.write("ply\nformat ascii 1.0\nelement vertex %d\nproperty float x\nproperty float y\nproperty float z\nend_header\n" % len(points))
        for p in points:
            f.write("%f %f %f\n" % (p[0], p[1], p[2]))
    print("Wrote %d points to %s" % (len(points), args.out))
    return 0


if __name__ == "__main__":
    sys.exit(main())
