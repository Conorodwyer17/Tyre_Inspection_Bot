#!/usr/bin/env python3
"""
Photo Capture Service for Tire Inspection (Phase G hardening)
Subscribes to /inspection_manager/capture_photo and saves images from Aurora camera.
Publishes capture result to /inspection_manager/capture_result (SUCCESS|FAILURE,filename,bytes).
Writes per-image metadata: sidecar JSON (map pose, timestamp, tire_class, tire_position)
and appends to manifest CSV in the save directory.
"""

import csv
import json
import os
import time
from datetime import datetime

import cv2
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String


class PhotoCaptureService(Node):
    """Service that captures photos when triggered by inspection_manager."""
    
    def __init__(self):
        super().__init__('photo_capture_service')
        
        # Parameters
        self.declare_parameter('camera_topic', '/slamware_ros_sdk_server_node/left_image_raw')
        self.declare_parameter('save_directory', '~/ugv_ws/tire_inspection_photos')
        self.declare_parameter('image_format', 'jpg')
        self.declare_parameter('capture_metadata_topic', '/inspection_manager/capture_metadata')
        self.declare_parameter('metadata_max_age_s', 2.0)
        
        camera_topic = self.get_parameter('camera_topic').value
        save_dir = os.path.expanduser(self.get_parameter('save_directory').value)
        self.image_format = self.get_parameter('image_format').value
        self._metadata_max_age = self.get_parameter('metadata_max_age_s').value
        
        # Create save directory if it doesn't exist
        os.makedirs(save_dir, exist_ok=True)
        self.save_directory = save_dir
        self._manifest_path = os.path.join(self.save_directory, 'manifest.csv')
        self._latest_metadata = None
        self._latest_metadata_time = 0.0
        
        self.get_logger().info(f"Photo capture service initialized")
        self.get_logger().info(f"  Camera topic: {camera_topic}")
        self.get_logger().info(f"  Save directory: {self.save_directory}")
        
        # Bridge for image conversion
        self.bridge = CvBridge()
        
        # Latest image storage
        self.latest_image = None
        self.image_received = False
        
        # Photo counter
        self.photo_count = 0
        
        # Subscribe to camera topic
        self.camera_sub = self.create_subscription(
            Image,
            camera_topic,
            self.camera_callback,
            10
        )
        
        # Subscribe to capture trigger
        self.capture_sub = self.create_subscription(
            Bool,
            '/inspection_manager/capture_photo',
            self.capture_callback,
            10
        )
        # Subscribe to capture metadata (pose, tire_class, etc.) from inspection_manager
        self.capture_metadata_topic = self.get_parameter('capture_metadata_topic').value
        self.metadata_sub = self.create_subscription(
            String,
            self.capture_metadata_topic,
            self._metadata_callback,
            10
        )

        # Phase G: Publish capture result for validation
        self.capture_result_pub = self.create_publisher(
            String, '/inspection_manager/capture_result', 10
        )
        
        self.get_logger().info("Photo capture service ready. Waiting for capture triggers...")
    
    def camera_callback(self, msg: Image):
        """Store latest camera image."""
        try:
            self.latest_image = msg
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")

    def _metadata_callback(self, msg: String):
        """Store latest capture metadata (published just before capture_photo trigger)."""
        try:
            self._latest_metadata = msg.data
            self._latest_metadata_time = self.get_clock().now().nanoseconds / 1e9
        except Exception as e:
            self.get_logger().warn(f"Capture metadata callback: {e}")

    def _write_metadata_for_capture(self, filepath: str, filename: str) -> bool:
        """Write sidecar JSON and append to manifest.csv if we have recent metadata. Returns True if written."""
        now = self.get_clock().now().nanoseconds / 1e9
        if self._latest_metadata is None or (now - self._latest_metadata_time) > self._metadata_max_age:
            return False
        try:
            meta = json.loads(self._latest_metadata)
        except (json.JSONDecodeError, TypeError):
            return False
        base, _ = os.path.splitext(filepath)
        json_path = base + '.json'
        with open(json_path, 'w') as f:
            json.dump(meta, f, indent=2)
        # Append to manifest CSV (create with header if new)
        file_exists = os.path.exists(self._manifest_path)
        with open(self._manifest_path, 'a', newline='') as f:
            writer = csv.writer(f)
            if not file_exists:
                writer.writerow([
                    'filename', 'frame_id', 'x', 'y', 'z', 'yaw', 'tire_class', 'tire_position',
                    'vehicle_id', 'timestamp_sec', 'timestamp_nanosec'
                ])
            writer.writerow([
                filename,
                meta.get('frame_id', ''),
                meta.get('x', ''),
                meta.get('y', ''),
                meta.get('z', ''),
                meta.get('yaw', ''),
                meta.get('tire_class', ''),
                meta.get('tire_position', ''),
                meta.get('vehicle_id', ''),
                meta.get('timestamp_sec', ''),
                meta.get('timestamp_nanosec', ''),
            ])
        self.get_logger().info(f"Metadata written: {json_path}, manifest updated")
        return True
    
    def _publish_capture_result(self, success: bool, filename: str = "", size_bytes: int = 0):
        """Phase G: Publish capture result for validation."""
        result = "SUCCESS" if success else "FAILURE"
        msg = String()
        msg.data = f"{result},{filename},{size_bytes}"
        self.capture_result_pub.publish(msg)

    def capture_callback(self, msg: Bool):
        """Handle photo capture trigger. Phase G: validate and publish result."""
        if not msg.data:
            return
        
        if not self.image_received or self.latest_image is None:
            self.get_logger().warn("Capture triggered but no camera image available yet")
            self._publish_capture_result(False, "", 0)
            return
        
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
            self.photo_count += 1
            filename = f"tire_{self.photo_count:03d}_{timestamp}.{self.image_format}"
            filepath = os.path.join(self.save_directory, filename)
            
            # Save image
            ok = cv2.imwrite(filepath, cv_image)
            if not ok:
                self.get_logger().error("cv2.imwrite returned False")
                self._publish_capture_result(False, filename, 0)
                return
            
            # Phase G: validate file exists and non-zero
            size_bytes = 0
            if os.path.exists(filepath):
                size_bytes = os.path.getsize(filepath)
            if size_bytes == 0:
                self.get_logger().error(f"Saved file is zero bytes: {filepath}")
                self._publish_capture_result(False, filename, 0)
                return
            
            # Write per-image metadata (sidecar JSON + manifest CSV) when available
            self._write_metadata_for_capture(filepath, filename)
            
            self.get_logger().info(
                f"✓ Photo captured: {filename} "
                f"(Resolution: {cv_image.shape[1]}x{cv_image.shape[0]}, {size_bytes} bytes)"
            )
            self._publish_capture_result(True, filename, size_bytes)
            
        except Exception as e:
            self.get_logger().error(f"Error capturing photo: {e}")
            self._publish_capture_result(False, "", 0)


def main(args=None):
    rclpy.init(args=args)
    node = PhotoCaptureService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
