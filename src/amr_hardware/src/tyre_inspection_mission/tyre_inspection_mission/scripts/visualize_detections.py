#!/usr/bin/env python3
"""
Visualization node for camera feed with bounding box overlays.

This node subscribes to:
- Camera image topic: /oak/rgb/image_rect
- Bounding boxes topic: /darknet_ros_3d/bounding_boxes

And publishes:
- Annotated image topic: /detection_visualization/image_annotated

You can view the annotated image using:
    rqt_image_view /detection_visualization/image_annotated

Or add it to RViz2 as an Image display.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from cv_bridge import CvBridge
import cv2
import numpy as np
from typing import Optional


class DetectionVisualizer(Node):
    """Visualize camera feed with bounding box overlays."""
    
    def __init__(self):
        super().__init__('detection_visualizer')
        
        # Parameters
        self.declare_parameter('camera_topic', '/oak/rgb/image_rect')
        self.declare_parameter('bbox_topic', '/darknet_ros_3d/bounding_boxes')
        self.declare_parameter('output_topic', '/detection_visualization/image_annotated')
        self.declare_parameter('show_labels', True)
        self.declare_parameter('show_confidence', True)
        self.declare_parameter('line_thickness', 2)
        self.declare_parameter('font_scale', 0.6)
        
        self.camera_topic = self.get_parameter('camera_topic').value
        self.bbox_topic = self.get_parameter('bbox_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.show_labels = self.get_parameter('show_labels').value
        self.show_confidence = self.get_parameter('show_confidence').value
        self.line_thickness = self.get_parameter('line_thickness').value
        self.font_scale = self.get_parameter('font_scale').value
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Latest messages
        self.latest_image: Optional[Image] = None
        self.latest_bboxes: Optional[BoundingBoxes3d] = None
        
        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        self.bbox_sub = self.create_subscription(
            BoundingBoxes3d,
            self.bbox_topic,
            self.bbox_callback,
            10
        )
        
        # Publisher for annotated image
        self.annotated_image_pub = self.create_publisher(
            Image,
            self.output_topic,
            10
        )
        
        # Timer to publish annotated images
        self.timer = self.create_timer(0.1, self.publish_annotated_image)  # 10 Hz
        
        # Option to show OpenCV window (set via parameter)
        self.declare_parameter('show_window', False)
        self.show_window = self.get_parameter('show_window').value
        if self.show_window:
            cv2.namedWindow('Detection Visualization', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('Detection Visualization', 1280, 720)
        
        self.get_logger().info(
            f"Detection visualizer initialized:\n"
            f"  Camera topic: {self.camera_topic}\n"
            f"  BBox topic: {self.bbox_topic}\n"
            f"  Output topic: {self.output_topic}\n"
            f"  View with: ros2 run rqt_image_view rqt_image_view {self.output_topic}\n"
            f"  Or use RViz2: Add Image display, topic: {self.output_topic}\n"
            f"  OpenCV window: {'Enabled' if self.show_window else 'Disabled (set show_window:=True to enable)'}"
        )
    
    def image_callback(self, msg: Image):
        """Store latest camera image."""
        self.latest_image = msg
    
    def bbox_callback(self, msg: BoundingBoxes3d):
        """Store latest bounding boxes."""
        self.latest_bboxes = msg
    
    def publish_annotated_image(self):
        """Publish annotated image with bounding boxes drawn."""
        if self.latest_image is None:
            return
        
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            
            # Draw bounding boxes if available
            if self.latest_bboxes is not None and self.latest_bboxes.bounding_boxes:
                cv_image = self.draw_bounding_boxes(cv_image, self.latest_bboxes)
            
            # Convert back to ROS Image
            annotated_msg = self.bridge.cv2_to_imgmsg(cv_image, "bgr8")
            annotated_msg.header = self.latest_image.header
            
            # Publish
            self.annotated_image_pub.publish(annotated_msg)
            
            # Show OpenCV window if enabled
            if self.show_window:
                cv2.imshow('Detection Visualization', cv_image)
                cv2.waitKey(1)  # Non-blocking wait for key press
            
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}", exc_info=True)
    
    def draw_bounding_boxes(self, image: np.ndarray, bboxes_msg: BoundingBoxes3d) -> np.ndarray:
        """
        Draw bounding boxes on image.
        
        Args:
            image: OpenCV image (BGR format)
            bboxes_msg: BoundingBoxes3d message
            
        Returns:
            Annotated image
        """
        annotated = image.copy()
        
        # Color mapping for different object classes
        color_map = {
            'truck': (0, 255, 0),      # Green
            'car': (0, 255, 255),      # Yellow
            'tyre': (255, 0, 0),       # Blue
            'tire': (255, 0, 0),       # Blue (alternative spelling)
            'person': (255, 0, 255),   # Magenta
            'default': (0, 165, 255),  # Orange
        }
        
        for bbox in bboxes_msg.bounding_boxes:
            try:
                # Get object class and color
                class_name = bbox.object_name.lower() if bbox.object_name else "unknown"
                color = color_map.get(class_name, color_map['default'])
                
                # Extract 2D bounding box coordinates
                # BoundingBox3d has xmin, xmax, ymin, ymax, zmin, zmax
                x_min = int(bbox.xmin)
                y_min = int(bbox.ymin)
                x_max = int(bbox.xmax)
                y_max = int(bbox.ymax)
                
                # Validate coordinates
                h, w = image.shape[:2]
                x_min = max(0, min(x_min, w - 1))
                y_min = max(0, min(y_min, h - 1))
                x_max = max(0, min(x_max, w - 1))
                y_max = max(0, min(y_max, h - 1))
                
                if x_max <= x_min or y_max <= y_min:
                    continue  # Skip invalid boxes
                
                # Draw rectangle
                cv2.rectangle(
                    annotated,
                    (x_min, y_min),
                    (x_max, y_max),
                    color,
                    self.line_thickness
                )
                
                # Prepare label text
                label_parts = []
                if self.show_labels:
                    label_parts.append(bbox.object_name if bbox.object_name else "unknown")
                if self.show_confidence and hasattr(bbox, 'probability'):
                    label_parts.append(f"{bbox.probability:.2f}")
                
                if label_parts:
                    label = " ".join(label_parts)
                    
                    # Calculate text size for background
                    (text_width, text_height), baseline = cv2.getTextSize(
                        label,
                        cv2.FONT_HERSHEY_SIMPLEX,
                        self.font_scale,
                        1
                    )
                    
                    # Draw text background
                    cv2.rectangle(
                        annotated,
                        (x_min, y_min - text_height - baseline - 5),
                        (x_min + text_width, y_min),
                        color,
                        -1  # Filled
                    )
                    
                    # Draw text
                    cv2.putText(
                        annotated,
                        label,
                        (x_min, y_min - baseline - 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        self.font_scale,
                        (255, 255, 255),  # White text
                        1
                    )
                
                # Draw center point
                center_x = (x_min + x_max) // 2
                center_y = (y_min + y_max) // 2
                cv2.circle(annotated, (center_x, center_y), 3, color, -1)
                
            except Exception as e:
                self.get_logger().warn(f"Error drawing bounding box: {e}")
                continue
        
        return annotated


def main(args=None):
    rclpy.init(args=args)
    
    node = DetectionVisualizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup OpenCV windows
        if node.show_window:
            cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
