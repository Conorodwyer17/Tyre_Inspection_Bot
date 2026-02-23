#!/usr/bin/env python3
"""
Aurora semantic fusion — vehicle detection from native semantic_segmentation + depth.

COCO80 labels: bicycle(2), car(3), motorcycle(4), bus(6), truck(8).
Resizes semantic (480x640) to depth (224x416) via nearest-neighbor.
Computes mask centroid, samples depth, projects to 3D, transforms to slamware_map.
Publishes BoundingBoxes3d (vehicles only) for inspection_manager.
"""
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
import numpy as np
import tf2_ros
from geometry_msgs.msg import PointStamped, TransformStamped
import cv2
import yaml


# COCO80 label IDs for vehicles
VEHICLE_LABEL_IDS = {2, 3, 4, 6, 8}  # bicycle, car, motorcycle, bus, truck
VEHICLE_LABEL_NAMES = {2: "bicycle", 3: "car", 4: "motorcycle", 6: "bus", 8: "truck"}


class AuroraSemanticFusionNode(Node):
    def __init__(self):
        super().__init__("aurora_semantic_fusion")
        self.declare_parameter("semantic_topic", "/slamware_ros_sdk_server_node/semantic_labels")
        self.declare_parameter("depth_topic", "/slamware_ros_sdk_server_node/depth_image_raw")
        self.declare_parameter("output_topic", "/aurora_semantic/vehicle_bounding_boxes")
        self.declare_parameter("target_frame", "slamware_map")
        self.declare_parameter("camera_frame", "camera_depth_optical_frame")
        self.declare_parameter("min_depth_m", 0.3)
        self.declare_parameter("max_depth_m", 15.0)
        self.declare_parameter("min_mask_area", 100)
        self.declare_parameter("intrinsics_file", "")  # Load from aurora_depth_intrinsics.yaml when set
        self.declare_parameter("fx", 180.0)
        self.declare_parameter("fy", 180.0)
        self.declare_parameter("cx", 208.0)
        self.declare_parameter("cy", 112.0)

        # Load intrinsics from file or use params (must match aurora_depth_camera_info for correct geometry)
        intrinsics_file = self.get_parameter("intrinsics_file").value
        if intrinsics_file and os.path.isfile(intrinsics_file):
            with open(intrinsics_file) as f:
                cfg = yaml.safe_load(f)
            c = cfg.get("aurora_depth_camera_info", cfg)
            fx = float(c.get("fx", 180.0))
            fy = float(c.get("fy", 180.0))
            cx = float(c.get("cx", 208.0))
            cy = float(c.get("cy", 112.0))
            self.get_logger().info(f"Loaded intrinsics from {intrinsics_file}: fx={fx:.1f} cx={cx:.1f}")
        else:
            fx = self.get_parameter("fx").value
            fy = self.get_parameter("fy").value
            cx = self.get_parameter("cx").value
            cy = self.get_parameter("cy").value

        self.semantic_topic = self.get_parameter("semantic_topic").value
        self.depth_topic = self.get_parameter("depth_topic").value
        self.output_topic = self.get_parameter("output_topic").value
        self.target_frame = self.get_parameter("target_frame").value
        self.camera_frame = self.get_parameter("camera_frame").value
        self.min_depth = self.get_parameter("min_depth_m").value
        self.max_depth = self.get_parameter("max_depth_m").value
        self.min_mask_area = self.get_parameter("min_mask_area").value
        self.fx = fx
        self.fy = fy
        self.cx = cx
        self.cy = cy

        self.bridge = CvBridge()
        self._semantic = None
        self._depth = None
        self._semantic_header = None
        self._depth_header = None
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        qos_be = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST, depth=5)

        self._sub_semantic = self.create_subscription(
            Image, self.semantic_topic, self._cb_semantic, qos_be
        )
        self._sub_depth = self.create_subscription(
            Image, self.depth_topic, self._cb_depth, qos_be
        )
        self._pub = self.create_publisher(BoundingBoxes3d, self.output_topic, 10)

        self._alignment_logged = False
        self.get_logger().info(
            f"aurora_semantic_fusion: semantic={self.semantic_topic} + depth={self.depth_topic} -> {self.output_topic}"
        )

    def _cb_semantic(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self._semantic = img
            self._semantic_header = msg.header
        except Exception as e:
            self.get_logger().warn(f"Semantic cb: {e}")

    def _cb_depth(self, msg: Image):
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            if depth.dtype == np.uint16:
                depth = depth.astype(np.float32) / 1000.0
            elif depth.dtype != np.float32:
                depth = depth.astype(np.float32)
            self._depth = depth
            self._depth_header = msg.header
        except Exception as e:
            self.get_logger().warn(f"Depth cb: {e}")

    def _run_fusion(self):
        if self._semantic is None or self._depth is None:
            return
        if self._depth_header is None:
            return

        h_d, w_d = self._depth.shape
        h_s, w_s = self._semantic.shape

        if not self._alignment_logged:
            self.get_logger().info(
                f"Alignment: semantic {w_s}x{h_s} -> depth {w_d}x{h_d}, scale=({w_d/w_s:.3f}, {h_d/h_s:.3f})"
            )
            if (h_d, w_d) != (224, 416):
                self.get_logger().warn(
                    f"Depth resolution {w_d}x{h_d} != expected Aurora 2.11 (416x224). "
                    "Verify depth topic and intrinsics."
                )
            if (h_s, w_s) != (480, 640):
                self.get_logger().warn(
                    f"Semantic resolution {w_s}x{h_s} != expected Aurora 2.11 (640x480). "
                    "Alignment may be incorrect."
                )
            self._alignment_logged = True

        # Resize semantic to depth resolution (nearest-neighbor for label IDs)
        if (h_s, w_s) != (h_d, w_d):
            semantic_resized = cv2.resize(
                self._semantic, (w_d, h_d), interpolation=cv2.INTER_NEAREST
            )
        else:
            semantic_resized = self._semantic

        # semantic_labels topic: mono8, pixel value = COCO80 class ID
        if semantic_resized.ndim == 3:
            semantic_labels = cv2.cvtColor(semantic_resized, cv2.COLOR_BGR2GRAY)
        else:
            semantic_labels = semantic_resized

        depth = self._depth
        boxes = BoundingBoxes3d()
        boxes.header = self._depth_header
        boxes.header.frame_id = self.target_frame

        for label_id in VEHICLE_LABEL_IDS:
            mask = (semantic_labels == label_id)
            if not np.any(mask):
                continue
            area = np.sum(mask)
            if area < self.min_mask_area:
                continue
            ys, xs = np.where(mask)
            cy_px = float(np.mean(ys))
            cx_px = float(np.mean(xs))
            # Sample depth at centroid (with small neighborhood median for robustness)
            r = 2
            y0, y1 = max(0, int(cy_px) - r), min(h_d, int(cy_px) + r + 1)
            x0, x1 = max(0, int(cx_px) - r), min(w_d, int(cx_px) + r + 1)
            roi = depth[y0:y1, x0:x1]
            valid = roi[np.isfinite(roi) & (roi > self.min_depth) & (roi < self.max_depth)]
            if valid.size == 0:
                continue
            z = float(np.median(valid))
            if z <= 0 or not np.isfinite(z):
                continue
            # Unproject to camera frame
            x_cam = (cx_px - self.cx) * z / self.fx
            y_cam = (cy_px - self.cy) * z / self.fy
            z_cam = z
            # Transform to target frame
            try:
                t = self._tf_buffer.lookup_transform(
                    self.target_frame,
                    self.camera_frame,
                    self._depth_header.stamp,
                    rclpy.duration.Duration(seconds=0.2),
                )
            except Exception as e:
                self.get_logger().debug(f"TF lookup failed: {e}")
                continue
            # Apply transform (quaternion to rotation matrix)
            R = np.array([
                [t.transform.rotation.w**2 + t.transform.rotation.x**2 - t.transform.rotation.y**2 - t.transform.rotation.z**2,
                 2*(t.transform.rotation.x*t.transform.rotation.y - t.transform.rotation.w*t.transform.rotation.z),
                 2*(t.transform.rotation.x*t.transform.rotation.z + t.transform.rotation.w*t.transform.rotation.y)],
                [2*(t.transform.rotation.x*t.transform.rotation.y + t.transform.rotation.w*t.transform.rotation.z),
                 t.transform.rotation.w**2 - t.transform.rotation.x**2 + t.transform.rotation.y**2 - t.transform.rotation.z**2,
                 2*(t.transform.rotation.y*t.transform.rotation.z - t.transform.rotation.w*t.transform.rotation.x)],
                [2*(t.transform.rotation.x*t.transform.rotation.z - t.transform.rotation.w*t.transform.rotation.y),
                 2*(t.transform.rotation.y*t.transform.rotation.z + t.transform.rotation.w*t.transform.rotation.x),
                 t.transform.rotation.w**2 - t.transform.rotation.x**2 - t.transform.rotation.y**2 + t.transform.rotation.z**2],
            ])
            p = np.array([x_cam, y_cam, z_cam])
            t_vec = np.array([t.transform.translation.x, t.transform.translation.y, t.transform.translation.z])
            p_map = R @ p + t_vec
            # Create minimal BoundingBox3d (center + small extent)
            box = BoundingBox3d()
            box.header = boxes.header
            box.object_name = VEHICLE_LABEL_NAMES[label_id]
            box.probability = 0.95
            extent = 0.5
            box.xmin = p_map[0] - extent
            box.xmax = p_map[0] + extent
            box.ymin = p_map[1] - extent
            box.ymax = p_map[1] + extent
            box.zmin = p_map[2] - extent
            box.zmax = p_map[2] + extent
            boxes.bounding_boxes.append(box)

        if boxes.bounding_boxes:
            self._pub.publish(boxes)


def main(args=None):
    rclpy.init(args=args)
    node = AuroraSemanticFusionNode()
    timer = node.create_timer(0.1, node._run_fusion)  # 10 Hz
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
