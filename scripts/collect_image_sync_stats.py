#!/usr/bin/env python3
"""
Collect left/right image timestamp deltas for stereo sync tuning.
Outputs mean, std, min, max (and optional JSON file).
Usage:
  python3 scripts/collect_image_sync_stats.py --left-topic /slamware_ros_sdk_server_node/left_image_raw --right-topic /slamware_ros_sdk_server_node/right_image_raw --samples 100
"""
import argparse
import json
import os
import sys

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer


def main():
    parser = argparse.ArgumentParser(description="Collect L/R image sync stats")
    parser.add_argument("--left-topic", default="/slamware_ros_sdk_server_node/left_image_raw", help="Left image topic")
    parser.add_argument("--right-topic", default="/slamware_ros_sdk_server_node/right_image_raw", help="Right image topic")
    parser.add_argument("--samples", type=int, default=100, help="Number of pairs to collect")
    parser.add_argument("-o", "--output", help="Write JSON to file")
    args = parser.parse_args()

    rclpy.init()
    node = Node("collect_image_sync_stats")
    deltas_ms = []

    def cb(lmsg, rmsg):
        lt = lmsg.header.stamp.sec + lmsg.header.stamp.nanosec * 1e-9
        rt = rmsg.header.stamp.sec + rmsg.header.stamp.nanosec * 1e-9
        deltas_ms.append(abs(lt - rt) * 1000)

    qos = rclpy.qos.QoSProfile(
        reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
        history=rclpy.qos.HistoryPolicy.KEEP_LAST,
        depth=10,
    )
    sub_l = Subscriber(node, Image, args.left_topic, qos_profile=qos)
    sub_r = Subscriber(node, Image, args.right_topic, qos_profile=qos)
    sync = ApproximateTimeSynchronizer([sub_l, sub_r], queue_size=10, slop=0.2)
    sync.registerCallback(cb)

    node.get_logger().info("Collecting %d pairs from %s and %s ..." % (args.samples, args.left_topic, args.right_topic))
    while rclpy.ok() and len(deltas_ms) < args.samples:
        rclpy.spin_once(node, timeout_sec=0.5)
        if len(deltas_ms) % 25 == 0 and len(deltas_ms) > 0:
            node.get_logger().info("  %d/%d" % (len(deltas_ms), args.samples))

    if not deltas_ms:
        node.get_logger().error("No pairs collected")
        node.destroy_node()
        rclpy.shutdown()
        return 1

    import numpy as np
    arr = np.array(deltas_ms)
    stats = {
        "n_pairs": len(deltas_ms),
        "mean_ms": float(np.mean(arr)),
        "std_ms": float(np.std(arr)),
        "min_ms": float(np.min(arr)),
        "max_ms": float(np.max(arr)),
    }
    node.get_logger().info("mean=%.2f ms std=%.2f min=%.2f max=%.2f" % (stats["mean_ms"], stats["std_ms"], stats["min_ms"], stats["max_ms"]))
    print("mean_ms=%.2f std_ms=%.2f min_ms=%.2f max_ms=%.2f" % (stats["mean_ms"], stats["std_ms"], stats["min_ms"], stats["max_ms"]))

    if args.output:
        os.makedirs(os.path.dirname(args.output) or ".", exist_ok=True)
        with open(args.output, "w") as f:
            json.dump(stats, f, indent=2)
        node.get_logger().info("Wrote %s" % args.output)

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
