#!/usr/bin/env python3
"""
License Plate Detection Module

Detects and reads license plates from vehicle images using OCR.
Calculates 3D positions using depth camera point cloud.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
import cv2
import numpy as np
import math
from typing import Optional, Tuple, List
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped
import traceback

# Try to import OCR libraries (with fallbacks)
try:
    import easyocr
    EASYOCR_AVAILABLE = True
except ImportError:
    EASYOCR_AVAILABLE = False

try:
    import pytesseract
    TESSERACT_AVAILABLE = True
except ImportError:
    TESSERACT_AVAILABLE = False


class LicensePlateDetector:
    """Detects and reads license plates from vehicle images"""
    
    def __init__(self, node=None):
        """
        Initialize license plate detector
        
        Args:
            node: Optional ROS 2 node (if None, will create one)
        """
        self._node = node
        
        # Get node for logging and parameters
        if self._node is None:
            # For standalone use, we need a node but don't initialize rclpy here
            # (it should already be initialized by the caller)
            try:
                if not rclpy.ok():
                    rclpy.init()
                self._node = Node('license_plate_detector')
                self._standalone_node = True
            except:
                # If rclpy not available, create a minimal mock node
                class MockNode:
                    def get_logger(self):
                        class MockLogger:
                            def info(self, msg): print(f"[INFO] {msg}")
                            def warn(self, msg): print(f"[WARN] {msg}")
                            def error(self, msg): print(f"[ERROR] {msg}")
                            def debug(self, msg): pass
                        return MockLogger()
                    def get_clock(self):
                        class MockClock:
                            def now(self):
                                class MockTime:
                                    def to_msg(self):
                                        from builtin_interfaces.msg import Time
                                        return Time()
                                return MockTime()
                        return MockClock()
                    def declare_parameter(self, name, default):
                        setattr(self, name, default)
                    def get_parameter(self, name):
                        class MockParam:
                            def value(self):
                                return getattr(self._node, name, None)
                        return MockParam()
                self._node = MockNode()
                self._standalone_node = True
        else:
            self._standalone_node = False
        
        # Parameters (with defaults if node doesn't have them)
        try:
            self._node.declare_parameter('navigation_frame', 'map')
            self._node.declare_parameter('ocr_method', 'easyocr')
            self._node.declare_parameter('min_confidence', 0.5)
            self._node.declare_parameter('crop_margin', 0.1)
            
            self.nav_frame = self._node.get_parameter('navigation_frame').value
            self.ocr_method = self._node.get_parameter('ocr_method').value
            self.min_confidence = self._node.get_parameter('min_confidence').value
            self.crop_margin = self._node.get_parameter('crop_margin').value
        except:
            # Fallback defaults
            self.nav_frame = 'map'
            self.ocr_method = 'easyocr'
            self.min_confidence = 0.5
            self.crop_margin = 0.1
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # TF (only if node is provided and has proper ROS 2 node interface)
        try:
            if self._node is not None and hasattr(self._node, 'get_clock'):
                self.tf_buffer = tf2_ros.Buffer()
                self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self._node)
            else:
                self.tf_buffer = None
                self.tf_listener = None
        except:
            self.tf_buffer = None
            self.tf_listener = None
        
        # OCR initialization
        self.ocr_reader = None
        self._initialize_ocr()
        
        if self._node:
            self._node.get_logger().info("License plate detector initialized")
    
    def _initialize_ocr(self):
        """Initialize OCR reader based on available libraries"""
        logger = self._node.get_logger() if self._node else None
        
        if self.ocr_method == 'easyocr' and EASYOCR_AVAILABLE:
            try:
                # Initialize EasyOCR reader (English only for license plates)
                self.ocr_reader = easyocr.Reader(['en'], gpu=True)
                if logger:
                    logger.info("✅ EasyOCR initialized successfully")
                return
            except Exception as e:
                if logger:
                    logger.warn(f"Failed to initialize EasyOCR: {e}. Trying Tesseract...")
                self.ocr_method = 'tesseract'
        
        if self.ocr_method == 'tesseract' and TESSERACT_AVAILABLE:
            try:
                # Tesseract is ready to use
                self.ocr_reader = True
                if logger:
                    logger.info("✅ Tesseract OCR available")
                return
            except Exception as e:
                if logger:
                    logger.warn(f"Failed to initialize Tesseract: {e}")
        
        # Fallback: No OCR available
        if logger:
            logger.error(
                "❌ No OCR library available! Install one of:\n"
                "  - pip install easyocr  (recommended)\n"
                "  - pip install pytesseract && apt-get install tesseract-ocr"
            )
        self.ocr_reader = None
    
    def detect_license_plate(
        self,
        vehicle_image: np.ndarray,
        vehicle_bbox_2d: Optional[Tuple[int, int, int, int]] = None,
        pointcloud: Optional[PointCloud2] = None,
        camera_frame: str = 'oak_rgb_camera_optical_frame'
    ) -> Optional[dict]:
        """
        Detect and read license plate from vehicle image
        
        Args:
            vehicle_image: Full camera image (numpy array, BGR format)
            vehicle_bbox_2d: Optional (x_min, y_min, x_max, y_max) in pixels
            pointcloud: Optional PointCloud2 for 3D position calculation
            camera_frame: Frame ID of camera
            
        Returns:
            dict with keys:
                - 'text': License plate text (str)
                - 'confidence': Detection confidence (float)
                - 'bbox_2d': Bounding box in image (x_min, y_min, x_max, y_max)
                - 'position_3d': 3D position in camera frame (Point)
                - 'position_nav': 3D position in navigation frame (PoseStamped)
            or None if detection failed
        """
        if self.ocr_reader is None:
            logger = self._node.get_logger() if self._node else None
            if logger:
                logger.warn("OCR not available, cannot detect license plate")
            return None
        
        try:
            # Crop vehicle region if bbox provided
            if vehicle_bbox_2d is not None:
                x_min, y_min, x_max, y_max = vehicle_bbox_2d
                h, w = vehicle_image.shape[:2]
                
                # Add margin
                margin_x = int((x_max - x_min) * self.crop_margin)
                margin_y = int((y_max - y_min) * self.crop_margin)
                
                x_min = max(0, x_min - margin_x)
                y_min = max(0, y_min - margin_y)
                x_max = min(w, x_max + margin_x)
                y_max = min(h, y_max + margin_y)
                
                vehicle_roi = vehicle_image[y_min:y_max, x_min:x_max]
            else:
                # Use full image
                vehicle_roi = vehicle_image
                x_min, y_min = 0, 0
            
            if vehicle_roi.size == 0:
                logger = self._node.get_logger() if self._node else None
                if logger:
                    logger.warn("Vehicle ROI is empty")
                return None
            
            # Focus on upper portion (license plate is usually at top of vehicle)
            roi_h, roi_w = vehicle_roi.shape[:2]
            # License plate is typically in upper 40% of vehicle
            license_plate_roi = vehicle_roi[0:int(roi_h * 0.4), :]
            
            # Preprocess image for OCR
            processed_image = self._preprocess_image(license_plate_roi)
            
            # Run OCR
            ocr_results = self._run_ocr(processed_image)
            
            logger = self._node.get_logger() if self._node else None
            
            if not ocr_results:
                if logger:
                    logger.debug("No text detected in license plate region")
                return None
            
            # Filter and select best license plate candidate
            best_result = self._filter_license_plate_results(ocr_results)
            
            if best_result is None:
                if logger:
                    logger.debug("No valid license plate text found")
                return None
            
            text, confidence, bbox = best_result
            
            # Adjust bbox to full image coordinates
            bbox_full = (
                x_min + bbox[0],
                y_min + bbox[1],
                x_min + bbox[2],
                y_min + bbox[3]
            )
            
            # Calculate 3D position if pointcloud available
            position_3d = None
            position_nav = None
            
            if pointcloud is not None:
                position_3d = self._calculate_3d_position(
                    bbox_full, pointcloud, camera_frame
                )
                
                if position_3d is not None:
                    position_nav = self._transform_to_nav_frame(
                        position_3d, camera_frame
                    )
            
            result = {
                'text': text,
                'confidence': confidence,
                'bbox_2d': bbox_full,
                'position_3d': position_3d,
                'position_nav': position_nav
            }
            
            if logger:
                logger.info(
                    f"✅ License plate detected: '{text}' (confidence: {confidence:.2f})"
                )
            
            return result
            
        except Exception as e:
            logger = self._node.get_logger() if self._node else None
            if logger:
                logger.error(f"Error detecting license plate: {e}\n{traceback.format_exc()}")
            return None
    
    def _preprocess_image(self, image: np.ndarray) -> np.ndarray:
        """Preprocess image for better OCR results"""
        # Convert to grayscale
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image
        
        # Enhance contrast
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        enhanced = clahe.apply(gray)
        
        # Denoise
        denoised = cv2.fastNlMeansDenoising(enhanced, h=10)
        
        # Threshold to binary
        _, binary = cv2.threshold(denoised, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        
        # Optional: Morphological operations to clean up
        kernel = np.ones((2, 2), np.uint8)
        cleaned = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel)
        
        return cleaned
    
    def _run_ocr(self, image: np.ndarray) -> List[Tuple[str, float, Tuple[int, int, int, int]]]:
        """
        Run OCR on image
        
        Returns:
            List of (text, confidence, bbox) tuples
        """
        results = []
        
        try:
            if self.ocr_method == 'easyocr' and isinstance(self.ocr_reader, easyocr.Reader):
                # EasyOCR
                ocr_results = self.ocr_reader.readtext(image)
                
                for detection in ocr_results:
                    bbox_points, text, confidence = detection
                    if confidence >= self.min_confidence:
                        # Convert bbox points to (x_min, y_min, x_max, y_max)
                        x_coords = [p[0] for p in bbox_points]
                        y_coords = [p[1] for p in bbox_points]
                        bbox = (int(min(x_coords)), int(min(y_coords)),
                                int(max(x_coords)), int(max(y_coords)))
                        results.append((text, confidence, bbox))
            
            elif self.ocr_method == 'tesseract' and self.ocr_reader:
                # Tesseract OCR
                import pytesseract
                
                # Get detailed data
                data = pytesseract.image_to_data(image, output_type=pytesseract.Output.DICT)
                
                n_boxes = len(data['text'])
                for i in range(n_boxes):
                    text = data['text'][i].strip()
                    conf = float(data['conf'][i]) / 100.0  # Convert to 0-1 range
                    
                    if text and conf >= self.min_confidence:
                        x = data['left'][i]
                        y = data['top'][i]
                        w = data['width'][i]
                        h = data['height'][i]
                        bbox = (x, y, x + w, y + h)
                        results.append((text, conf, bbox))
        
        except Exception as e:
            logger = self._node.get_logger() if self._node else None
            if logger:
                logger.error(f"OCR error: {e}")
        
        return results
    
    def _filter_license_plate_results(
        self,
        results: List[Tuple[str, float, Tuple[int, int, int, int]]]
    ) -> Optional[Tuple[str, float, Tuple[int, int, int, int]]]:
        """
        Filter OCR results to find most likely license plate
        
        License plate characteristics:
        - Usually 6-8 characters
        - Mix of letters and numbers
        - No spaces or special characters (after cleaning)
        """
        if not results:
            return None
        
        # Clean and score each result
        scored_results = []
        
        for text, confidence, bbox in results:
            # Clean text (remove spaces, special chars)
            cleaned = ''.join(c for c in text.upper() if c.isalnum())
            
            if not cleaned:
                continue
            
            # Score based on license plate characteristics
            score = confidence
            
            # Prefer 6-8 characters
            length = len(cleaned)
            if 6 <= length <= 8:
                score *= 1.2
            elif 4 <= length <= 10:
                score *= 1.0
            else:
                score *= 0.7
            
            # Prefer mix of letters and numbers
            has_letters = any(c.isalpha() for c in cleaned)
            has_numbers = any(c.isdigit() for c in cleaned)
            if has_letters and has_numbers:
                score *= 1.1
            
            scored_results.append((cleaned, score, bbox, confidence))
        
        if not scored_results:
            return None
        
        # Return best result
        best = max(scored_results, key=lambda x: x[1])
        return (best[0], best[3], best[2])  # (text, confidence, bbox)
    
    def _calculate_3d_position(
        self,
        bbox_2d: Tuple[int, int, int, int],
        pointcloud: PointCloud2,
        camera_frame: str
    ) -> Optional[Point]:
        """
        Calculate 3D position of license plate from point cloud
        
        Args:
            bbox_2d: (x_min, y_min, x_max, y_max) in image pixels
            pointcloud: PointCloud2 message
            camera_frame: Frame ID of camera
            
        Returns:
            Point with 3D coordinates in camera frame, or None
        """
        try:
            import sensor_msgs_py.point_cloud2 as pc2
            
            # Get center of bbox
            x_center = (bbox_2d[0] + bbox_2d[2]) // 2
            y_center = (bbox_2d[1] + bbox_2d[3]) // 2
            
            # Get point at center pixel
            center_points = list(pc2.read_points(
                pointcloud,
                field_names=("x", "y", "z"),
                skip_nans=True,
                uvs=[(x_center, y_center)]
            ))
            
            if center_points:
                point = center_points[0]
                if not (math.isnan(point[0]) or math.isnan(point[1]) or math.isnan(point[2])):
                    position = Point(
                        x=float(point[0]),
                        y=float(point[1]),
                        z=float(point[2])
                    )
                    return position
            
            # Fallback: sample points in bbox region
            x_min, y_min, x_max, y_max = bbox_2d
            sample_points = []
            
            # Sample grid of points in bbox
            for y in range(y_min, y_max, max(1, (y_max - y_min) // 5)):
                for x in range(x_min, x_max, max(1, (x_max - x_min) // 5)):
                    points = list(pc2.read_points(
                        pointcloud,
                        field_names=("x", "y", "z"),
                        skip_nans=True,
                        uvs=[(x, y)]
                    ))
                    if points:
                        p = points[0]
                        if not (math.isnan(p[0]) or math.isnan(p[1]) or math.isnan(p[2])):
                            sample_points.append(p)
            
            if sample_points:
                # Use median to reduce noise
                x_vals = [p[0] for p in sample_points]
                y_vals = [p[1] for p in sample_points]
                z_vals = [p[2] for p in sample_points]
                
                position = Point(
                    x=float(np.median(x_vals)),
                    y=float(np.median(y_vals)),
                    z=float(np.median(z_vals))
                )
                return position
            
            return None
            
        except Exception as e:
            logger = self._node.get_logger() if self._node else None
            if logger:
                logger.warn(f"Error calculating 3D position: {e}")
            return None
    
    def _transform_to_nav_frame(
        self,
        position_3d: Point,
        source_frame: str
    ) -> Optional[PoseStamped]:
        """
        Transform 3D position to navigation frame
        
        Args:
            position_3d: Point in camera frame
            source_frame: Frame ID of source
            
        Returns:
            PoseStamped in navigation frame, or None
        """
        try:
            # Create pose in source frame
            pose_source = PoseStamped()
            pose_source.header.frame_id = source_frame
            if self._node:
                pose_source.header.stamp = self._node.get_clock().now().to_msg()
            else:
                from rclpy.clock import Clock
                pose_source.header.stamp = Clock().now().to_msg()
            pose_source.pose.position = position_3d
            pose_source.pose.orientation.w = 1.0
            
            # Transform to navigation frame
            if self.tf_buffer is None:
                logger = self._node.get_logger() if self._node else None
                if logger:
                    logger.warn("TF buffer not available, cannot transform to navigation frame")
                return None
                
            transform = self.tf_buffer.lookup_transform(
                self.nav_frame,
                source_frame,
                rclpy.time.Time(seconds=0),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )
            
            pose_nav = do_transform_pose_stamped(pose_source, transform)
            pose_nav.header.frame_id = self.nav_frame
            pose_nav.header.stamp = transform.header.stamp
            
            return pose_nav
            
        except Exception as e:
            logger = self._node.get_logger() if self._node else None
            if logger:
                logger.warn(f"Transform failed: {e}")
            return None


def main(args=None):
    rclpy.init(args=args)
    node = LicensePlateDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('License plate detector shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
