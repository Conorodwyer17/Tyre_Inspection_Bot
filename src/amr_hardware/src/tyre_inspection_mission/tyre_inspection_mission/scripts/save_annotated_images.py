#!/usr/bin/env python3
"""
Save annotated camera images to disk for viewing.

This script subscribes to the annotated image topic and saves images
to a directory that can be accessed from Windows via SFTP/network share.

Perfect for headless systems where you can't use GUI viewers.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from pathlib import Path
from datetime import datetime
from typing import Optional


class ImageSaver(Node):
    """Save annotated images to disk for viewing."""
    
    def __init__(self):
        super().__init__('image_saver')
        
        # Parameters
        self.declare_parameter('input_topic', '/detection_visualization/image_annotated')
        self.declare_parameter('output_dir', '~/tyre_inspection_visualizations')
        self.declare_parameter('save_interval', 0.5)  # Save every 0.5 seconds (2 fps)
        self.declare_parameter('max_images', 100)  # Keep last 100 images
        self.declare_parameter('latest_filename', 'latest_annotated.jpg')
        self.declare_parameter('save_timestamped', True)
        
        input_topic = self.get_parameter('input_topic').value
        output_dir = Path(self.get_parameter('output_dir').value).expanduser()
        self.save_interval = self.get_parameter('save_interval').value
        self.max_images = self.get_parameter('max_images').value
        self.latest_filename = self.get_parameter('latest_filename').value
        self.save_timestamped = self.get_parameter('save_timestamped').value
        
        # Create output directory
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # Image tracking
        self.last_save_time = None
        self.image_count = 0
        
        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )
        
        # Timer to clean up old images
        self.cleanup_timer = self.create_timer(10.0, self.cleanup_old_images)
        
        self.get_logger().info(
            f"Image saver initialized:\n"
            f"  Input topic: {input_topic}\n"
            f"  Output directory: {self.output_dir}\n"
            f"  Save interval: {self.save_interval}s\n"
            f"  Max images to keep: {self.max_images}\n"
            f"  Latest image: {self.output_dir / self.latest_filename}\n"
            f"\n"
            f"📁 Access images from Windows:\n"
            f"  1. Use SFTP client (WinSCP, FileZilla, etc.)\n"
            f"  2. Connect to: jetson@{os.uname().nodename if hasattr(os, 'uname') else 'jetson'}\n"
            f"  3. Navigate to: {self.output_dir}\n"
            f"  4. View: {self.latest_filename} (always latest)\n"
            f"  5. Or browse timestamped images\n"
        )
    
    def image_callback(self, msg: Image):
        """Save image when interval has passed."""
        current_time = self.get_clock().now()
        
        # Check if we should save this image
        if self.last_save_time is None:
            should_save = True
        else:
            elapsed = (current_time - self.last_save_time).nanoseconds / 1e9
            should_save = elapsed >= self.save_interval
        
        if not should_save:
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Always save as "latest"
            latest_path = self.output_dir / self.latest_filename
            cv2.imwrite(str(latest_path), cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            
            # Also save timestamped version if enabled
            if self.save_timestamped:
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # milliseconds
                timestamped_path = self.output_dir / f"annotated_{timestamp}.jpg"
                cv2.imwrite(str(timestamped_path), cv_image, [cv2.IMWRITE_JPEG_QUALITY, 95])
                self.image_count += 1
            
            self.last_save_time = current_time
            
            if self.image_count % 10 == 0:  # Log every 10th image
                self.get_logger().info(
                    f"Saved {self.image_count} images. "
                    f"Latest: {self.latest_filename}"
                )
                
        except Exception as e:
            self.get_logger().error(f"Error saving image: {e}", exc_info=True)
    
    def cleanup_old_images(self):
        """Remove old timestamped images if we exceed max_images."""
        if not self.save_timestamped:
            return
        
        try:
            # Get all timestamped image files
            image_files = sorted(
                self.output_dir.glob("annotated_*.jpg"),
                key=lambda p: p.stat().st_mtime,
                reverse=True
            )
            
            # Remove oldest if we exceed max
            if len(image_files) > self.max_images:
                files_to_remove = image_files[self.max_images:]
                for file in files_to_remove:
                    file.unlink()
                    self.get_logger().debug(f"Removed old image: {file.name}")
                
                self.image_count = len(image_files) - len(files_to_remove)
                self.get_logger().info(
                    f"Cleaned up {len(files_to_remove)} old images. "
                    f"Keeping {self.image_count} latest images."
                )
                
        except Exception as e:
            self.get_logger().warn(f"Error cleaning up images: {e}")


def main(args=None):
    rclpy.init(args=args)
    
    node = ImageSaver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
