#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np
from sensor_msgs.msg import Image
from std_msgs.msg import String
from segmentation_msgs.msg import ObjectsSegment, ObjectSegment
from ultralytics import YOLO
import cv2
import time
import os
from pathlib import Path

class UltralyticsSegmentationNode(Node):
    def __init__(self):
        super().__init__('ultralytics_segmentation')
        
        # Declare parameters for model paths
        # Changed to yolov8n-seg.pt (nano) for faster inference - ~5-10x faster than yolov8m-seg
        # Trade-off: Slightly lower accuracy but much better for real-time operation
        self.declare_parameter("navigation_model", "yolov8n-seg.pt")  # Model 1 for navigation (lighter, faster)
        self.declare_parameter("inspection_model", "best.pt")  # Model 2 for inspection
        self.declare_parameter("mode_topic", "/segmentation_mode")
        self.declare_parameter("default_mode", "navigation")
        
        # Performance optimization parameters
        self.declare_parameter("img_size", 320)  # Image size for inference (320 is faster than 640)
        self.declare_parameter("device", "cpu")  # Device: cpu, cuda (if GPU available)
        self.declare_parameter("conf_threshold", 0.25)  # Confidence threshold
        self.declare_parameter("frame_skip", 1)  # Process every Nth frame (1 = all frames, 3 = every 3rd)
        self.declare_parameter("half", False)  # FP16 inference (requires CUDA)
        
        # Get performance parameters
        self.img_size = self.get_parameter("img_size").value
        self.device = self.get_parameter("device").value
        self.conf_threshold = self.get_parameter("conf_threshold").value
        self.frame_skip = max(1, int(self.get_parameter("frame_skip").value))  # Ensure >= 1
        self.half = self.get_parameter("half").value
        
        # Frame counter for skipping
        self.frame_counter = 0
        
        # Helper function to resolve model paths
        def resolve_model_path(model_path):
            """Resolve model file path, checking multiple locations if relative."""
            # If absolute path provided and exists, use it
            if os.path.isabs(model_path) and os.path.exists(model_path):
                return model_path
            
            # If relative path, check common locations
            if not os.path.isabs(model_path):
                # Check current working directory
                if os.path.exists(model_path):
                    return os.path.abspath(model_path)
                
                # Check workspace root (common location)
                workspace_root = os.path.expanduser("~/ugv_ws")
                workspace_path = os.path.join(workspace_root, model_path)
                if os.path.exists(workspace_path):
                    return workspace_path
                
                # Check deep_learning_models directory
                dl_models_dir = os.path.join(workspace_root, "src", "amr_hardware", "deep_learning_models")
                dl_models_path = os.path.join(dl_models_dir, model_path)
                if os.path.exists(dl_models_path):
                    return dl_models_path
            
            # If still not found, return original path (will cause error with better message)
            return model_path
        
        # Load both YOLO segmentation models
        nav_model_path = self.get_parameter("navigation_model").value
        insp_model_path = self.get_parameter("inspection_model").value
        
        # Resolve paths
        nav_model_path = resolve_model_path(nav_model_path)
        insp_model_path = resolve_model_path(insp_model_path)
        
        # Validate paths exist
        if not os.path.exists(nav_model_path):
            self.get_logger().error(f"Navigation model not found: {nav_model_path}")
            raise FileNotFoundError(f"Navigation model not found: {nav_model_path}")
        
        if not os.path.exists(insp_model_path):
            self.get_logger().error(f"Inspection model not found: {insp_model_path}")
            raise FileNotFoundError(f"Inspection model not found: {insp_model_path}")
        
        self.get_logger().info(f"Loading navigation model: {nav_model_path}")
        self.navigation_model = YOLO(nav_model_path)  # Model 1 for navigation to trucks
        
        self.get_logger().info(f"Loading inspection model: {insp_model_path}")
        self.inspection_model = YOLO(insp_model_path)  # Model 2 for inspection under truck
        
        # Current active model (default to navigation)
        default_mode = self.get_parameter("default_mode").value
        if default_mode == "inspection":
            self.segmentation_model = self.inspection_model
            self.current_mode = "inspection"
        else:
            self.segmentation_model = self.navigation_model
            self.current_mode = "navigation"
        
        self.get_logger().info(f"Initial segmentation mode: {self.current_mode}")

        # Publishers
        self.objects_segment_pub = self.create_publisher(
            ObjectsSegment, 
            "/ultralytics/segmentation/objects_segment", 
            10
        )
        self.seg_image_pub = self.create_publisher(
            Image, 
            "/ultralytics/segmentation/image", 
            10
        )

        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image,
            "/oak/rgb/image_rect",
            self.callback,
            10
        )
        
        # Subscribe to mode topic to switch between models
        mode_topic = self.get_parameter("mode_topic").value
        self.mode_sub = self.create_subscription(
            String,
            mode_topic,
            self.mode_callback,
            10
        )
        self.get_logger().info(f"Subscribed to mode topic: {mode_topic}")

    def mode_callback(self, msg: String):
        """Callback to switch between navigation and inspection models."""
        mode = msg.data.lower().strip()
        
        if mode == "navigation":
            if self.current_mode != "navigation":
                self.get_logger().info("Switching to navigation mode (Model 1)")
                self.segmentation_model = self.navigation_model
                self.current_mode = "navigation"
        elif mode == "inspection":
            if self.current_mode != "inspection":
                self.get_logger().info("Switching to inspection mode (Model 2)")
                self.segmentation_model = self.inspection_model
                self.current_mode = "inspection"
        else:
            self.get_logger().warn(f"Unknown mode received: '{mode}'. Expected 'navigation' or 'inspection'")

    def callback(self, rgb_data):
        # Frame skipping: Process every Nth frame
        self.frame_counter += 1
        if self.frame_counter % self.frame_skip != 0:
            return  # Skip this frame
        
        # Convert the RGB image to a NumPy array
        try:
            image = rnp.numpify(rgb_data)
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        original_height, original_width, _ = image.shape
        # Log at debug level to reduce verbosity (images come at high rate)
        self.get_logger().debug(f"Processing image with shape: {image.shape}")

        # Performance optimization: Resize image BEFORE inference for faster processing
        inference_start = time.time()
        
        # Resize image to target size if needed (maintaining aspect ratio)
        if original_width != self.img_size or original_height != self.img_size:
            scale = min(self.img_size / original_width, self.img_size / original_height)
            new_width = int(original_width * scale)
            new_height = int(original_height * scale)
            image_resized = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_LINEAR)
        else:
            image_resized = image

        # Apply segmentation model with optimized parameters
        seg_result = self.segmentation_model(
            image_resized,
            imgsz=self.img_size,
            conf=self.conf_threshold,
            device=self.device,
            half=self.half,
            verbose=False  # Reduce logging overhead
        )
        
        inference_time = (time.time() - inference_start) * 1000  # Convert to milliseconds
        
        # Log inference time periodically (every 10 processed frames)
        if self.frame_counter % (10 * self.frame_skip) == 0:
            self.get_logger().info(
                f"YOLO inference: {inference_time:.1f}ms (frame {self.frame_counter}, "
                f"size={self.img_size}, device={self.device}, skip={self.frame_skip})"
            )

        # Prepare the ObjectsSegment message
        objects_msg = ObjectsSegment()
        objects_msg.header = rgb_data.header  # Copy the header from the RGB image
        objects_msg.header.stamp = self.get_clock().now().to_msg()

        for index, cls in enumerate(seg_result[0].boxes.cls):
            class_index = int(cls.cpu().numpy())
            name = seg_result[0].names[class_index]

            # Ensure result.masks is not None
            if seg_result[0].masks is not None:
                mask = seg_result[0].masks.data.cpu().numpy()[index, :, :]
                # Resize mask to ORIGINAL image size (not inference size) for correct coordinates
                mask_resized = cv2.resize(mask, (original_width, original_height), interpolation=cv2.INTER_NEAREST)
                binary_mask = (mask_resized > 0.5).astype(np.uint8)

                # Get pixel indices for the mask
                y_indices, x_indices = np.where(binary_mask > 0)

                if len(x_indices) == 0 or len(y_indices) == 0:
                    self.get_logger().warn(f"No valid indices found for object: {name}")
                    continue

                # Create ObjectSegment message
                obj_msg = ObjectSegment()
                obj_msg.header = objects_msg.header
                obj_msg.class_name = name
                obj_msg.probability = float(seg_result[0].boxes.conf[index].item())  # Accessing the probability score
                obj_msg.x_indices = x_indices.tolist()
                obj_msg.y_indices = y_indices.tolist()

                # Append the object segment to the array
                objects_msg.objects.append(obj_msg)
            else:
                self.get_logger().warn("Segmentation result has no masks")

        # Publish the ObjectsSegment message
        if objects_msg.objects:
            detected_names = [obj.class_name for obj in objects_msg.objects]
            self.get_logger().info(
                f"🎯 YOLO detected: {', '.join(set(detected_names))} ({len(objects_msg.objects)} objects)"
            )
            self.get_logger().debug(f"Publishing {len(objects_msg.objects)} segmented objects")
            self.objects_segment_pub.publish(objects_msg)
        else:
            # Log when no objects detected (throttled to avoid spam)
            self.get_logger().debug(
                f"No objects detected (frame {self.frame_counter}, conf={self.conf_threshold})",
                throttle_duration_sec=5.0
            )

        # Segmentation Visualization
        if self.seg_image_pub.get_subscription_count() > 0:
            try:
                # Generate and publish the annotated segmentation image
                seg_annotated = seg_result[0].plot(show=False)
                self.seg_image_pub.publish(rnp.msgify(Image, seg_annotated, encoding="rgb8"))
                self.get_logger().debug("Segmentation image published")
            except Exception as e:
                self.get_logger().error(f"Error while publishing segmentation image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = UltralyticsSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
