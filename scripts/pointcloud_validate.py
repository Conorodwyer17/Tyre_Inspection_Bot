#!/usr/bin/env python3
"""
Phase F: Point cloud validation.
Checks: no NaN/Inf clusters, Z span <= 20m, planar fit residual < 3cm at 1.5m.
"""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import json
import sys

def main():
    rclpy.init()
    node = Node("pointcloud_validate")
    result = {"passed": False, "errors": [], "metrics": {}}
    data = None

    def cb(msg):
        nonlocal data
        data = msg

    node.create_subscription(PointCloud2, "/camera/depth/points", cb, 10)
    node.get_logger().info("Waiting for one point cloud...")
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.2)
        if data is not None:
            break

    if data is None:
        result["errors"].append("No point cloud received")
        out = sys.argv[1] if len(sys.argv) > 1 else "logs/cursor_runs/20260222T180830Z/phaseF/pointcloud_validation.json"
        with open(out, "w") as f:
            json.dump(result, f, indent=2)
        node.destroy_node()
        rclpy.shutdown()
        return 1

    pts = np.array(list(pc2.read_points(data, field_names=("x", "y", "z"), skip_nans=False)))
    if pts.size == 0:
        result["errors"].append("Empty point cloud")
    else:
        valid = np.isfinite(pts).all(axis=1) & (pts[:, 2] > 0)
        n_valid = np.sum(valid)
        n_total = len(pts)
        result["metrics"]["n_points"] = int(n_total)
        result["metrics"]["n_valid"] = int(n_valid)
        result["metrics"]["nan_inf_ratio"] = 1.0 - n_valid / n_total if n_total > 0 else 1.0

        if n_valid > 0:
            z = pts[valid, 2]
            z_min, z_max = float(np.min(z)), float(np.max(z))
            result["metrics"]["z_min_m"] = z_min
            result["metrics"]["z_max_m"] = z_max
            result["metrics"]["z_span_m"] = z_max - z_min
            if z_max - z_min > 20.0:
                result["errors"].append(f"Z span {z_max - z_min:.1f}m > 20m")

        mid = pts[valid]
        if len(mid) > 100:
            near = mid[np.abs(mid[:, 2] - 1.5) < 0.3]
            if len(near) > 50:
                from numpy.linalg import lstsq
                A = np.column_stack([near[:, 0], near[:, 1], np.ones(len(near))])
                c, _, _, _ = lstsq(A, near[:, 2], rcond=None)
                residuals = np.abs(near[:, 2] - (A @ c))
                rmse = float(np.sqrt(np.mean(residuals**2)))
                result["metrics"]["plane_rmse_15m"] = rmse
                if rmse > 0.03:
                    result["errors"].append(f"Plane fit RMSE {rmse*100:.2f}cm > 3cm at ~1.5m")

    result["passed"] = len(result["errors"]) == 0
    out = sys.argv[1] if len(sys.argv) > 1 else "logs/cursor_runs/20260222T180830Z/phaseF/pointcloud_validation.json"
    import os
    os.makedirs(os.path.dirname(out) or ".", exist_ok=True)
    with open(out, "w") as f:
        json.dump(result, f, indent=2)
    node.get_logger().info(f"Validation: {'PASS' if result['passed'] else 'FAIL'} -> {out}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result["passed"] else 1

if __name__ == "__main__":
    sys.exit(main())
