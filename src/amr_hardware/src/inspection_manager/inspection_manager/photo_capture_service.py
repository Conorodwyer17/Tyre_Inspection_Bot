#!/usr/bin/env python3
"""
Photo Capture Service for Tire Inspection
Subscribes to /inspection_manager/capture_photo and saves images from Aurora camera
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime
import time


class PhotoCaptureService(Node):
    """Service that captures photos when triggered by inspection_manager."""
    
    def __init__(self):
        super().__init__('photo_capture_service')
        
        # Parameters
        self.declare_parameter('camera_topic', '/slamware_ros_sdk_server_node/left_image_raw')
        self.declare_parameter('save_directory', '~/ugv_ws/tire_inspection_photos')
        self.declare_parameter('image_format', 'jpg')
        
        camera_topic = self.get_parameter('camera_topic').value
        save_dir = os.path.expanduser(self.get_parameter('save_directory').value)
        self.image_format = self.get_parameter('image_format').value
        
        # Create save directory if it doesn't exist
        os.makedirs(save_dir, exist_ok=True)
        self.save_directory = save_dir
        
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
        
        self.get_logger().info("Photo capture service ready. Waiting for capture triggers...")
    
    def camera_callback(self, msg: Image):
        """Store latest camera image."""
        try:
            self.latest_image = msg
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f"Error processing camera image: {e}")
    
    def capture_callback(self, msg: Bool):
        """Handle photo capture trigger."""
        if not msg.data:
            return
        
        if not self.image_received or self.latest_image is None:
            self.get_logger().warn("Capture triggered but no camera image available yet")
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
            cv2.imwrite(filepath, cv_image)
            
            self.get_logger().info(
                f"✓ Photo captured: {filename} "
                f"(Resolution: {cv_image.shape[1]}x{cv_image.shape[0]})"
            )
            
        except Exception as e:
            self.get_logger().error(f"Error capturing photo: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = PhotoCaptureService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
