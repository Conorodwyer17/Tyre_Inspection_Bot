#!/usr/bin/env python3
"""
Aurora Fisheye Stereo Calibration — SLAMTEC Aurora (180° FOV, binocular fisheye, 6cm baseline).

Uses checkerboard capture + cv2.fisheye.stereoCalibrate. Exports to aurora_sdk_bridge format.

Aurora specs (from docs): Binocular fisheye, FOV 180°, 6cm baseline, 640×480.
Detection is challenging on fisheye; this script uses multiple patterns, center crop,
and auto-capture for reliability.

Usage:
  1. Start: ros2 launch ugv_nav aurora_testing.launch.py
  2. Run:   python3 scripts/aurora_stereo_calibration.py

Checkerboard: 8×8 squares = 7×7 inner corners (or use --pattern 6 for 6×6 inner).
Square size: 3 cm.

Press SPACE to capture when both see board, or use auto-capture (enabled by default).
"""

import argparse
import os
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from message_filters import Subscriber, ApproximateTimeSynchronizer
import cv2
import numpy as np
import yaml


# 8×8 board = 7×7 inner corners; 3 cm squares
# Also support 6×6 and 5×5 (use inner region of same board)
PATTERN_OPTIONS = {
    "7": (7, 7),   # full 8×8 board
    "6": (6, 6),   # inner 7×7 region
    "5": (5, 5),   # inner 6×6 region
}
SQUARE_SIZE_M = 0.03
TARGET_PAIRS = 20
MIN_PAIRS = 15
AUTO_CAPTURE_FRAMES = 20  # frames both must detect before auto-capture (~1.3 s at 15 Hz)
CENTER_CROP_FRAC = 0.75   # use center 75% for detection (less distortion)


def get_parser():
    p = argparse.ArgumentParser(description="Aurora fisheye stereo calibration (180° FOV)")
    p.add_argument("--output", "-o", default=None, help="Output YAML path")
    p.add_argument("--pairs", type=int, default=TARGET_PAIRS, help=f"Target pairs (default {TARGET_PAIRS})")
    p.add_argument("--save-dir", default=None, help="Save captured images")
    p.add_argument("--pattern", choices=list(PATTERN_OPTIONS.keys()), default="6",
                   help="Inner corners: 7=7×7, 6=6×6, 5=5×5. Smaller = easier on fisheye")
    p.add_argument("--no-auto-capture", action="store_true", help="Disable auto-capture; require SPACE/Enter")
    p.add_argument("--preview-dir", default="/tmp/aurora_calib_preview",
                   help="Save latest preview images here (for viewing when no GUI)")
    p.add_argument("--left-topic", default="/slamware_ros_sdk_server_node/left_image_raw")
    p.add_argument("--right-topic", default="/slamware_ros_sdk_server_node/right_image_raw")
    return p


class AuroraStereoCalibration(Node):
    def __init__(self, args):
        super().__init__("aurora_stereo_calibration")
        self.args = args
        self.bridge = CvBridge()
        self._left = self._right = None
        self.pairs_left = []
        self.pairs_right = []
        self._last_good_left = None
        self._last_good_right = None
        self.pattern_size = PATTERN_OPTIONS[args.pattern]
        self.square_size = SQUARE_SIZE_M
        self._both_ok_count = 0
        self._preview_count = 0

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self._sub_left = Subscriber(self, Image, args.left_topic, qos_profile=qos)
        self._sub_right = Subscriber(self, Image, args.right_topic, qos_profile=qos)
        # Increased slop for better sync across cameras
        self._sync = ApproximateTimeSynchronizer(
            [self._sub_left, self._sub_right], queue_size=10, slop=0.2
        )
        self._sync.registerCallback(self._callback)

        # Publish preview for rqt_image_view (no OpenCV GUI needed)
        self._pub_left = self.create_publisher(Image, "/aurora_calibration/preview_left", 10)
        self._pub_right = self.create_publisher(Image, "/aurora_calibration/preview_right", 10)
        self._pub_combined = self.create_publisher(Image, "/aurora_calibration/preview_combined", 10)

        if args.save_dir:
            os.makedirs(args.save_dir, exist_ok=True)
        os.makedirs(args.preview_dir, exist_ok=True)

        self.get_logger().info("=" * 60)
        self.get_logger().info("Aurora Fisheye Stereo Calibration")
        self.get_logger().info("=" * 60)
        self.get_logger().info(f"Pattern: {self.pattern_size[0]}×{self.pattern_size[1]} inner corners, {SQUARE_SIZE_M*100:.0f} cm squares")
        self.get_logger().info(f"Target: {args.pairs} pairs (min {MIN_PAIRS})")
        self.get_logger().info(f"Preview images: {args.preview_dir}/")
        self.get_logger().info("")
        self.get_logger().info("VIEW THE FEED: Run in another terminal: rqt_image_view")
        self.get_logger().info("  Then select: /aurora_calibration/preview_combined (or preview_left/right)")
        self.get_logger().info("")
        self.get_logger().info("Tips for 180° fisheye:")
        self.get_logger().info("  • Keep checkerboard in CENTER of both images (less distortion)")
        self.get_logger().info("  • Distance ~0.5–1.5 m; vary angles and positions")
        self.get_logger().info("  • Good lighting, avoid reflections")
        if not args.no_auto_capture:
            self.get_logger().info(f"  • Auto-capture when both detect for ~{AUTO_CAPTURE_FRAMES} frames")
        self.get_logger().info("")
        self.get_logger().info("Waiting for images...")

    def _callback(self, left_msg: Image, right_msg: Image):
        try:
            self._left = self.bridge.imgmsg_to_cv2(left_msg, desired_encoding="passthrough")
            self._right = self.bridge.imgmsg_to_cv2(right_msg, desired_encoding="passthrough")
            # Publish preview for rqt_image_view
            self._pub_left.publish(left_msg)
            self._pub_right.publish(right_msg)
            if self._left is not None and self._right is not None:
                left_bgr = cv2.cvtColor(self._left, cv2.COLOR_GRAY2BGR) if self._left.ndim == 2 else self._left
                right_bgr = cv2.cvtColor(self._right, cv2.COLOR_GRAY2BGR) if self._right.ndim == 2 else self._right
                combined = np.hstack([left_bgr, right_bgr])
                combined_msg = self.bridge.cv2_to_imgmsg(combined, encoding="bgr8")
                combined_msg.header = left_msg.header
                self._pub_combined.publish(combined_msg)
        except Exception as e:
            self.get_logger().warn(f"cv_bridge: {e}")

    def _center_crop(self, img, frac=CENTER_CROP_FRAC):
        """Crop to center region to reduce fisheye distortion at edges."""
        if img is None:
            return None
        h, w = img.shape[:2]
        nh, nw = int(h * frac), int(w * frac)
        y0, x0 = (h - nh) // 2, (w - nw) // 2
        return img[y0:y0+nh, x0:x0+nw]

    def find_corners(self, img, use_center_crop=True):
        """Detect checkerboard corners. Try full image first, then center crop."""
        if img is None:
            return False, None
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if img.ndim == 3 else img.copy()
        # Do NOT use CALIB_CB_FAST_CHECK — it hurts fisheye detection (Stack Overflow)
        flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE

        # Try full image first
        ret, corners = cv2.findChessboardCorners(gray, self.pattern_size, flags)
        if not ret and use_center_crop:
            cropped = self._center_crop(gray)
            if cropped is not None:
                ret, corners = cv2.findChessboardCorners(cropped, self.pattern_size, flags)
                if ret:
                    # Remap corners to full image
                    h, w = gray.shape
                    ch, cw = cropped.shape
                    y0, x0 = (h - ch) // 2, (w - cw) // 2
                    corners = corners + np.array([[[x0, y0]]], dtype=np.float32)

        if ret:
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 1e-6)
            corners = cv2.cornerSubPix(gray, corners, (5, 5), (-1, -1), criteria)
        return ret, corners

    def _capture_pair(self):
        if self._last_good_left is None or self._last_good_right is None:
            return False
        self.pairs_left.append(self._last_good_left.copy())
        self.pairs_right.append(self._last_good_right.copy())
        self._last_good_left = None
        self._last_good_right = None
        self._both_ok_count = 0
        n = len(self.pairs_left)
        self.get_logger().info("")
        self.get_logger().info(f">>> PAIR {n}/{self.args.pairs} CAPTURED <<<")
        if self.args.save_dir:
            lpath = os.path.join(self.args.save_dir, f"left_{n:03d}.png")
            rpath = os.path.join(self.args.save_dir, f"right_{n:03d}.png")
            cv2.imwrite(lpath, self.pairs_left[-1])
            cv2.imwrite(rpath, self.pairs_right[-1])
            self.get_logger().info(f"    Saved {lpath}, {rpath}")
        self.get_logger().info("")
        return True

    def run_interactive(self):
        use_gui = True
        try:
            cv2.namedWindow("Aurora Stereo Calibration", cv2.WINDOW_NORMAL)
        except cv2.error:
            use_gui = False
            self.get_logger().info("OpenCV GUI not available. Use rqt_image_view or view preview images:")
            self.get_logger().info(f"  watch -n 1 'ls -la {self.args.preview_dir}/'")

        last_log_time = 0
        last_status = None

        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.05)

            if self._left is None or self._right is None:
                t = time.time()
                if t - last_log_time > 2.0:
                    self.get_logger().info("Waiting for left/right images...")
                    last_log_time = t
                continue

            ok_l, corners_l = self.find_corners(self._left)
            ok_r, corners_r = self.find_corners(self._right)

            if ok_l and ok_r:
                self._last_good_left = self._left.copy()
                self._last_good_right = self._right.copy()
                self._both_ok_count += 1
            else:
                self._both_ok_count = 0

            n = len(self.pairs_left)
            status = f"Pairs: {n}/{self.args.pairs}"
            if ok_l and ok_r:
                status += f" [ready] (hold {AUTO_CAPTURE_FRAMES - self._both_ok_count} frames for auto)"
            else:
                status += f" [L:{'OK' if ok_l else 'no'} R:{'OK' if ok_r else 'no'}]"

            if status != last_status:
                self.get_logger().info(status)
                last_status = status

            # Save preview images for headless viewing
            self._preview_count += 1
            if self._preview_count % 15 == 0:
                try:
                    lp = os.path.join(self.args.preview_dir, "left_latest.png")
                    rp = os.path.join(self.args.preview_dir, "right_latest.png")
                    cv2.imwrite(lp, self._left)
                    cv2.imwrite(rp, self._right)
                except Exception:
                    pass

            # Auto-capture when both detect for N consecutive frames
            if not self.args.no_auto_capture and self._both_ok_count >= AUTO_CAPTURE_FRAMES:
                if self._capture_pair():
                    if len(self.pairs_left) >= self.args.pairs:
                        self.get_logger().info("All pairs captured. Running calibration...")
                        break

            if use_gui:
                left_vis = self._left.copy()
                right_vis = self._right.copy()
                if left_vis.ndim == 2:
                    left_vis = cv2.cvtColor(left_vis, cv2.COLOR_GRAY2BGR)
                if right_vis.ndim == 2:
                    right_vis = cv2.cvtColor(right_vis, cv2.COLOR_GRAY2BGR)
                if ok_l and corners_l is not None:
                    cv2.drawChessboardCorners(left_vis, self.pattern_size, corners_l, ok_l)
                if ok_r and corners_r is not None:
                    cv2.drawChessboardCorners(right_vis, self.pattern_size, corners_r, ok_r)
                combined = np.hstack([left_vis, right_vis])
                cv2.putText(combined, "SPACE=capture | ESC=quit", (10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.imshow("Aurora Stereo Calibration", combined)
                key = cv2.waitKey(100) & 0xFF
                if key == 27:
                    self.get_logger().info("Calibration cancelled")
                    break
                if key == ord(" ") and self._last_good_left is not None and self._last_good_right is not None:
                    if self._capture_pair() and len(self.pairs_left) >= self.args.pairs:
                        break
            else:
                # Headless: Enter to capture
                import select
                try:
                    r, _, _ = select.select([sys.stdin], [], [], 0.1)
                    if r:
                        line = sys.stdin.readline().strip().lower()
                        if line in ("q", "quit"):
                            break
                        if self._last_good_left is not None and self._last_good_right is not None:
                            if self._capture_pair() and len(self.pairs_left) >= self.args.pairs:
                                break
                        else:
                            self.get_logger().info("Hold board steady until [ready], then press Enter.")
                except (select.error, AttributeError):
                    pass

        if use_gui:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        return len(self.pairs_left)

    def calibrate(self):
        if len(self.pairs_left) < MIN_PAIRS:
            self.get_logger().error(f"Need at least {MIN_PAIRS} pairs, got {len(self.pairs_left)}")
            return None

        objp = np.zeros((self.pattern_size[0] * self.pattern_size[1], 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.pattern_size[0], 0:self.pattern_size[1]].T.reshape(-1, 2)
        objp *= self.square_size
        obj_points = []
        img_points_left = []
        img_points_right = []

        for left, right in zip(self.pairs_left, self.pairs_right):
            ok_l, corners_l = self.find_corners(left)
            ok_r, corners_r = self.find_corners(right)
            if ok_l and ok_r:
                obj_points.append(objp)
                img_points_left.append(corners_l)
                img_points_right.append(corners_r)

        if len(obj_points) < MIN_PAIRS:
            self.get_logger().error(f"Only {len(obj_points)} valid pairs with corners")
            return None

        h, w = self.pairs_left[0].shape[:2]
        self.get_logger().info(f"Calibrating with {len(obj_points)} pairs, {w}x{h}...")

        flags = cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_CHECK_COND | cv2.fisheye.CALIB_FIX_SKEW
        K1 = np.eye(3)
        D1 = np.zeros(4)
        K2 = np.eye(3)
        D2 = np.zeros(4)
        R = np.eye(3)
        T = np.zeros(3)

        rms, K1, D1, K2, D2, R, T = cv2.fisheye.stereoCalibrate(
            obj_points, img_points_left, img_points_right,
            K1, D1, K2, D2,
            (w, h), R, T, flags,
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6),
        )
        self.get_logger().info(f"Calibration RMS: {rms:.4f} px (threshold <= 0.6)")

        return {
            "image_width": w,
            "image_height": h,
            "baseline_m": abs(T[0]) if abs(T[0]) > 1e-6 else 0.06,
            "rms_error": float(rms),
            "left": {
                "camera_matrix": K1.flatten().tolist(),
                "dist_coeffs": D1.flatten().tolist(),
                "distortion_model": "equidistant",
            },
            "right": {
                "camera_matrix": K2.flatten().tolist(),
                "dist_coeffs": D2.flatten().tolist(),
                "distortion_model": "equidistant",
            },
            "rotation": R.flatten().tolist(),
            "translation": T.flatten().tolist(),
        }

    def save_yaml(self, data, path):
        with open(path, "w") as f:
            yaml.dump(data, f, default_flow_style=False, sort_keys=False)
        self.get_logger().info(f"Saved calibration to {path}")


def main():
    args = get_parser().parse_args()

    if args.output is None:
        ws = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        args.output = os.path.join(
            ws, "src", "aurora_sdk_bridge", "config", "equidistant_calibration.yaml"
        )

    rclpy.init()
    node = AuroraStereoCalibration(args)
    n = node.run_interactive()

    if n < MIN_PAIRS:
        node.destroy_node()
        rclpy.shutdown()
        print(f"Calibration aborted: need at least {MIN_PAIRS} pairs, got {n}")
        return 1

    pairs_left = node.pairs_left
    pairs_right = node.pairs_right
    node.destroy_node()
    rclpy.shutdown()

    rclpy.init()
    node2 = AuroraStereoCalibration(args)
    node2.pairs_left = pairs_left
    node2.pairs_right = pairs_right
    data = node2.calibrate()
    if data:
        node2.save_yaml(data, args.output)
        print("")
        print("Calibration complete! Restart aurora_testing to use new calibration.")
    node2.destroy_node()
    rclpy.shutdown()
    return 0 if data else 1


if __name__ == "__main__":
    sys.exit(main())
