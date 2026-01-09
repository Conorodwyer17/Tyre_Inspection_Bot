#!/usr/bin/env python3
"""
Photo Capture Service

Captures images from camera and saves them to disk with metadata.
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from pathlib import Path
from datetime import datetime
import json
import tf2_ros


class PhotoCaptureService(Node):
    def __init__(self):
        super().__init__('photo_capture_service')
        
        # Parameters
        self.declare_parameter('camera_topic', '/oak/rgb/image_rect')
        self.declare_parameter('default_storage_dir', '~/tyre_inspection_photos')
        self.declare_parameter('image_format', 'jpg')
        self.declare_parameter('image_quality', 95)  # JPEG quality 0-100
        
        self.camera_topic = self.get_parameter('camera_topic').value
        self.default_storage_dir = Path(self.get_parameter('default_storage_dir').value).expanduser()
        self.image_format = self.get_parameter('image_format').value
        self.image_quality = self.get_parameter('image_quality').value
        
        # Current storage directory (can be changed by mission controller)
        self.storage_dir = self.default_storage_dir
        
        # Ensure storage directory exists
        self.storage_dir.mkdir(parents=True, exist_ok=True)
        
        # Note: Storage directory can be changed via parameters at runtime
        # For mission controller, it will create mission-specific directories
        # The service will save to self.storage_dir which defaults to default_storage_dir parameter
        
        # Subscriber for camera images
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Latest image storage
        self.latest_image = None
        self.image_lock = False
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Service
        self.capture_service = self.create_service(
            Trigger,
            '/photo_capture/capture',
            self.capture_callback
        )
        
        # TF for robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.get_logger().info(
            f"Photo capture service initialized. Listening on {self.camera_topic}"
        )
        
    def image_callback(self, msg):
        """Store latest image"""
        if not self.image_lock:
            self.latest_image = msg
            
    def capture_callback(self, request, response):
        """Service callback to capture photo"""
        if self.latest_image is None:
            response.success = False
            response.message = "No image available from camera"
            self.get_logger().error("Capture requested but no image available")
            return response
            
        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:-3]  # milliseconds
            filename = f"photo_{timestamp}.{self.image_format}"
            
            # Save image
            if self.image_format.lower() == 'jpg' or self.image_format.lower() == 'jpeg':
                encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), self.image_quality]
                success = cv2.imwrite(str(self.storage_dir / filename), cv_image, encode_param)
            else:
                success = cv2.imwrite(str(self.storage_dir / filename), cv_image)
                
            if not success:
                response.success = False
                response.message = "Failed to save image to disk"
                self.get_logger().error("Failed to save image")
                return response
                
            # Get robot pose
            robot_pose = self.get_robot_pose()
            
            # Save metadata
            metadata = {
                'filename': filename,
                'timestamp': datetime.now().isoformat(),
                'image_size': {
                    'width': cv_image.shape[1],
                    'height': cv_image.shape[0]
                },
                'robot_pose': robot_pose,
                'camera_topic': self.camera_topic,
                'storage_path': str(self.storage_dir / filename)
            }
            
            metadata_filename = filename.rsplit('.', 1)[0] + '_metadata.json'
            with open(self.storage_dir / metadata_filename, 'w') as f:
                json.dump(metadata, f, indent=2)
                
            response.success = True
            response.message = str(self.storage_dir / filename)
            self.get_logger().info(f"Photo captured: {self.storage_dir / filename}")
            
            return response
            
        except Exception as e:
            self.get_logger().error(f"Error capturing photo: {e}", exc_info=True)
            response.success = False
            response.message = f"Error: {str(e)}"
            return response
            
    def get_robot_pose(self):
        """Get current robot pose in map frame"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )
            
            return {
                'x': transform.transform.translation.x,
                'y': transform.transform.translation.y,
                'z': transform.transform.translation.z,
                'orientation': {
                    'x': transform.transform.rotation.x,
                    'y': transform.transform.rotation.y,
                    'z': transform.transform.rotation.z,
                    'w': transform.transform.rotation.w
                }
            }
        except Exception as e:
            self.get_logger().warn(f"Could not get robot pose: {e}")
            return None
            
    def set_storage_directory(self, directory):
        """Set storage directory for photos (can be called via parameter service)"""
        try:
            self.storage_dir = Path(directory).expanduser()
            self.storage_dir.mkdir(parents=True, exist_ok=True)
            self.get_logger().info(f"Storage directory set to: {self.storage_dir}")
        except Exception as e:
            self.get_logger().error(f"Error setting storage directory: {e}")
    
    def set_storage_directory_callback(self, request, response):
        """Service callback to set storage directory"""
        try:
            # Expect directory path in request message
            directory = request.message if hasattr(request, 'message') else str(self.default_storage_dir)
            self.set_storage_directory(directory)
            response.success = True
            response.message = f"Storage directory set to: {self.storage_dir}"
        except Exception as e:
            response.success = False
            response.message = f"Error: {str(e)}"
        return response


def main(args=None):
    rclpy.init(args=args)
    node = PhotoCaptureService()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Photo capture service shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
