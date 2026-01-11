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
        detected_license_plate_bbox_3d: Optional[object] = None,  # NEW: 3D bbox from YOLO detection model (BoundingBox3d)
        pointcloud: Optional[PointCloud2] = None,
        camera_frame: str = 'oak_rgb_camera_optical_frame'
    ) -> Optional[dict]:
        """
        Detect and read license plate from vehicle image (TWO-STAGE APPROACH).
        
        Stage 1 (if detected_license_plate_bbox_3d provided): Uses YOLO-detected license plate bbox
        Stage 2: Runs OCR on detected/precise region
        
        Args:
            vehicle_image: Full camera image (numpy array, BGR format)
            vehicle_bbox_2d: Optional (x_min, y_min, x_max, y_max) in pixels (vehicle bbox, for heuristic fallback)
            detected_license_plate_bbox_3d: Optional BoundingBox3d from YOLO detection model (Stage 1 - preferred)
            pointcloud: Optional PointCloud2 for 3D position calculation and 3D→2D projection
            camera_frame: Frame ID of camera
            
        Returns:
            dict with keys:
                - 'text': License plate text (str)
                - 'confidence': OCR confidence (float)
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
            h, w = vehicle_image.shape[:2]
            logger = self._node.get_logger() if self._node else None
            
            # CRITICAL FIX: TWO-STAGE LICENSE PLATE DETECTION
            # Stage 1: Use YOLO-detected license plate bbox if available (PREFERRED - precise location)
            # Stage 2: Fallback to heuristic (upper 40% of vehicle) if no detection
            
            license_plate_bbox_2d = None
            use_detected_bbox = False
            
            if detected_license_plate_bbox_3d is not None and pointcloud is not None:
                # CRITICAL: Project 3D bbox to 2D pixel coordinates using point cloud
                # This enables "camera to know what a license plate looks like" - uses detected bbox
                license_plate_bbox_2d = self._project_3d_bbox_to_2d(
                    detected_license_plate_bbox_3d, 
                    pointcloud, 
                    w, h
                )
                
                if license_plate_bbox_2d is not None:
                    use_detected_bbox = True
                    if logger:
                        logger.info(
                            f"✅ TWO-STAGE DETECTION: Using YOLO-detected license plate bbox (Stage 1 → Stage 2): "
                            f"2D bbox={license_plate_bbox_2d} pixels (projected from 3D bbox)"
                        )
                else:
                    if logger:
                        logger.warn(
                            "⚠️ Failed to project 3D license plate bbox to 2D pixels. "
                            "Falling back to heuristic approach."
                        )
            
            # Initialize offsets for bbox coordinate adjustment
            x_min_offset = 0
            y_min_offset = 0
            
            if use_detected_bbox and license_plate_bbox_2d is not None:
                # TWO-STAGE APPROACH: Use detected license plate bbox directly (precise, efficient)
                # This is PREFERRED - camera "knows what a license plate looks like" via YOLO detection
                lp_x_min, lp_y_min, lp_x_max, lp_y_max = license_plate_bbox_2d
                
                # Ensure bbox is within image bounds
                lp_x_min = max(0, int(lp_x_min))
                lp_y_min = max(0, int(lp_y_min))
                lp_x_max = min(w, int(lp_x_max))
                lp_y_max = min(h, int(lp_y_max))
                
                # Validate bbox size (should be reasonable for license plate)
                if lp_x_max <= lp_x_min or lp_y_max <= lp_y_min:
                    if logger:
                        logger.warn("Invalid license plate bbox (zero or negative size). Using heuristic fallback.")
                    use_detected_bbox = False
                else:
                    # Add small margin for OCR (license plate bbox might be tight)
                    margin = 5  # pixels - small margin for OCR
                    lp_x_min = max(0, lp_x_min - margin)
                    lp_y_min = max(0, lp_y_min - margin)
                    lp_x_max = min(w, lp_x_max + margin)
                    lp_y_max = min(h, lp_y_max + margin)
                    
                    # Crop to detected license plate region (much smaller and more accurate than heuristic)
                    license_plate_roi = vehicle_image[lp_y_min:lp_y_max, lp_x_min:lp_x_max]
                    x_min_offset, y_min_offset = lp_x_min, lp_y_min
                    
                    if logger:
                        logger.info(
                            f"✅ TWO-STAGE DETECTION: Using YOLO-detected license plate region: "
                            f"bbox=({lp_x_min},{lp_y_min},{lp_x_max},{lp_y_max})px, "
                            f"size={lp_x_max-lp_x_min}x{lp_y_max-lp_y_min}px"
                        )
            
            if not use_detected_bbox or license_plate_bbox_2d is None:
                # FALLBACK: Heuristic approach (upper 40% of vehicle region)
                # This is used if YOLO detection failed or 3D→2D projection failed
                if vehicle_bbox_2d is not None:
                    x_min, y_min, x_max, y_max = vehicle_bbox_2d
                    
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
                    if logger:
                        logger.warn("Vehicle ROI is empty")
                    return None
                
                # FALLBACK: Focus on upper portion (license plate is usually at top of vehicle)
                # This is the old heuristic approach - less reliable
                roi_h, roi_w = vehicle_roi.shape[:2]
                # License plate is typically in upper 40% of vehicle
                license_plate_roi = vehicle_roi[0:int(roi_h * 0.4), :]
                x_min_offset, y_min_offset = x_min, y_min
                
                if logger:
                    logger.info(
                        "⚠️ Using heuristic fallback (upper 40% of vehicle region). "
                        "License plate detection model did not find plate or projection failed. "
                        "This is less reliable than two-stage detection."
                    )
            
            # Validate license plate ROI is not empty
            if license_plate_roi.size == 0:
                if logger:
                    logger.warn("License plate ROI is empty after cropping")
                return None
            
            # Preprocess image for OCR
            processed_image = self._preprocess_image(license_plate_roi)
            
            # Run OCR with higher confidence threshold (industry standard: 70%)
            ocr_results = self._run_ocr(processed_image)
            
            if not ocr_results:
                if logger:
                    logger.debug("No text detected in license plate region")
                return None
            
            # CRITICAL FIX: Filter with higher confidence thresholds (industry standard: 70-80%)
            min_char_conf = self._node.get_parameter('ocr_min_char_confidence').value if self._node and self._node.has_parameter('ocr_min_char_confidence') else 0.7
            min_global_conf = self._node.get_parameter('ocr_min_global_confidence').value if self._node and self._node.has_parameter('ocr_min_global_confidence') else 0.8
            
            # Filter and select best license plate candidate with higher confidence thresholds
            best_result = self._filter_license_plate_results(ocr_results, min_char_confidence=min_char_conf, min_global_confidence=min_global_conf)
            
            if best_result is None:
                if logger:
                    logger.debug("No valid license plate text found")
                return None
            
            text, confidence, bbox = best_result
            
            # Adjust OCR bbox to full image coordinates (OCR bbox is relative to license_plate_roi)
            bbox_full = (
                x_min_offset + bbox[0],
                y_min_offset + bbox[1],
                x_min_offset + bbox[2],
                y_min_offset + bbox[3]
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
                
                # CRITICAL FIX: Use higher confidence threshold (70% instead of 50%)
                min_conf = self._node.get_parameter('ocr_min_char_confidence').value if self._node and self._node.has_parameter('ocr_min_char_confidence') else 0.7
                
                for detection in ocr_results:
                    bbox_points, text, confidence = detection
                    if confidence >= min_conf:  # Use higher threshold (70%)
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
                    
                    # CRITICAL FIX: Use higher confidence threshold (70% instead of 50%)
                    min_conf = self._node.get_parameter('ocr_min_char_confidence').value if self._node and self._node.has_parameter('ocr_min_char_confidence') else 0.7
                    if text and conf >= min_conf:
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
        results: List[Tuple[str, float, Tuple[int, int, int, int]]],
        min_char_confidence: float = 0.7,  # NEW: Per-character confidence threshold (70% - industry standard)
        min_global_confidence: float = 0.8  # NEW: Overall license plate confidence threshold (80% - industry standard)
    ) -> Optional[Tuple[str, float, Tuple[int, int, int, int]]]:
        """
        Filter OCR results to find most likely license plate (with higher confidence thresholds).
        
        CRITICAL FIX: Uses industry-standard confidence thresholds (70-80%) instead of 50%.
        This significantly reduces false positives from stickers, signs, or other text.
        
        License plate characteristics:
        - Usually 6-8 characters
        - Mix of letters and numbers
        - No spaces or special characters (after cleaning)
        - High confidence per character (70%+) and overall (80%+)
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
            
            # CRITICAL FIX: Reject results below minimum character confidence threshold (70%)
            if confidence < min_char_confidence:
                continue  # Reject low-confidence results early
            
            # Score based on license plate characteristics
            score = confidence
            
            # Prefer 6-8 characters (typical license plate length)
            length = len(cleaned)
            if 6 <= length <= 8:
                score *= 1.2
            elif 4 <= length <= 10:
                score *= 1.0
            else:
                score *= 0.7  # Penalize unusual lengths
            
            # Prefer mix of letters and numbers (typical license plate format)
            has_letters = any(c.isalpha() for c in cleaned)
            has_numbers = any(c.isdigit() for c in cleaned)
            if has_letters and has_numbers:
                score *= 1.1  # Bonus for typical format
            
            scored_results.append((cleaned, score, bbox, confidence))
        
        if not scored_results:
            return None
        
        # Return best result ONLY if it meets global confidence threshold (80%)
        best = max(scored_results, key=lambda x: x[1])
        best_score = best[1]
        best_confidence = best[3]
        
        # CRITICAL FIX: Final check - overall confidence must meet global threshold
        if best_score < min_global_confidence or best_confidence < min_char_confidence:
            logger = self._node.get_logger() if self._node else None
            if logger:
                logger.debug(
                    f"License plate candidate rejected: "
                    f"score={best_score:.2f} < {min_global_confidence:.2f} or "
                    f"confidence={best_confidence:.2f} < {min_char_confidence:.2f}. "
                    f"Text was: '{best[0]}'"
                )
            return None  # Reject if below global threshold
        
        return (best[0], best[3], best[2])  # (text, confidence, bbox)
    
    def _project_3d_bbox_to_2d(
        self,
        bbox_3d: object,  # BoundingBox3d message
        pointcloud: PointCloud2,
        image_width: int,
        image_height: int
    ) -> Optional[Tuple[int, int, int, int]]:
        """
        Project 3D bounding box to 2D pixel coordinates using point cloud.
        
        This enables two-stage license plate detection: YOLO detects plate in 3D space,
        then we project to 2D pixels for OCR.
        
        CRITICAL: This uses point cloud to find pixels corresponding to 3D bbox region.
        Proper implementation would use camera calibration/projection matrix, but this
        approach uses point cloud sampling to estimate 2D region.
        
        Args:
            bbox_3d: BoundingBox3d message with license plate detection
            pointcloud: PointCloud2 message for 3D→2D projection
            image_width: Width of image in pixels
            image_height: Height of image in pixels
            
        Returns:
            (x_min, y_min, x_max, y_max) in pixels, or None if projection fails
        """
        try:
            import sensor_msgs_py.point_cloud2 as pc2
            
            # Get 3D bbox bounds
            x_min_3d = bbox_3d.xmin
            x_max_3d = bbox_3d.xmax
            y_min_3d = bbox_3d.ymin
            y_max_3d = bbox_3d.ymax
            z_min_3d = bbox_3d.zmin
            z_max_3d = bbox_3d.zmax
            
            # Get center of 3D bbox
            center_x_3d = (x_min_3d + x_max_3d) / 2.0
            center_y_3d = (y_min_3d + y_max_3d) / 2.0
            center_z_3d = (z_min_3d + z_max_3d) / 2.0
            
            # Calculate 3D bbox size (for validation and estimation)
            bbox_width_3d = abs(x_max_3d - x_min_3d)
            bbox_height_3d = abs(y_max_3d - y_min_3d)
            bbox_depth_3d = abs(z_max_3d - z_min_3d)
            
            # Validate bbox size (license plates should be small: 0.2-1.0m wide, 0.08-0.3m tall)
            if (bbox_width_3d < 0.15 or bbox_width_3d > 1.5 or
                bbox_height_3d < 0.05 or bbox_height_3d > 0.5):
                logger = self._node.get_logger() if self._node else None
                if logger:
                    logger.debug(
                        f"3D bbox size invalid for license plate: "
                        f"{bbox_width_3d:.3f}x{bbox_height_3d:.3f}m (expected ~0.3-0.6m x 0.1-0.2m)"
                    )
                return None
            
            # APPROACH: Sample point cloud points that fall within 3D bbox region
            # Find corresponding pixel coordinates for these points
            # Point cloud structure: organized by row (height) and column (width), matching image pixels
            # Each point in point cloud corresponds to a pixel in the image
            
            matching_pixels = []  # List of (x, y) pixel coordinates in bbox region
            
            # Sample point cloud and find points within 3D bbox
            # Point cloud is typically organized: point_cloud[y * width + x] = point at pixel (x, y)
            try:
                # Get point cloud dimensions
                pc_height = pointcloud.height
                pc_width = pointcloud.width
                
                if pc_height > 0 and pc_width > 0:
                    # Organized point cloud - can map directly to pixels
                    for y in range(0, min(pc_height, image_height), max(1, pc_height // 100)):  # Sample every N rows
                        for x in range(0, min(pc_width, image_width), max(1, pc_width // 100)):  # Sample every N columns
                            # Get point at this pixel location
                            idx = y * pc_width + x
                            points = list(pc2.read_points(
                                pointcloud,
                                field_names=("x", "y", "z"),
                                skip_nans=True,
                                uvs=[(x, y)]
                            ))
                            
                            if points:
                                px, py, pz = points[0]
                                # Check if point is within 3D bbox region (with small tolerance)
                                tolerance = 0.05  # 5cm tolerance
                                if (x_min_3d - tolerance <= px <= x_max_3d + tolerance and
                                    y_min_3d - tolerance <= py <= y_max_3d + tolerance and
                                    z_min_3d - tolerance <= pz <= z_max_3d + tolerance):
                                    matching_pixels.append((x, y))
                
                # If we found matching pixels, use them to determine 2D bbox
                if matching_pixels:
                    x_coords = [p[0] for p in matching_pixels]
                    y_coords = [p[1] for p in matching_pixels]
                    x_min_pixel = max(0, int(min(x_coords)))
                    y_min_pixel = max(0, int(min(y_coords)))
                    x_max_pixel = min(image_width, int(max(x_coords)))
                    y_max_pixel = min(image_height, int(max(y_coords)))
                    
                    logger = self._node.get_logger() if self._node else None
                    if logger:
                        logger.info(
                            f"✅ Projected 3D license plate bbox to 2D using point cloud: "
                            f"found {len(matching_pixels)} matching pixels, "
                            f"2D bbox=({x_min_pixel},{y_min_pixel},{x_max_pixel},{y_max_pixel})px"
                        )
                    
                    return (x_min_pixel, y_min_pixel, x_max_pixel, y_max_pixel)
            except Exception as e:
                logger = self._node.get_logger() if self._node else None
                if logger:
                    logger.debug(f"Error sampling point cloud for 3D→2D projection: {e}")
            
            # FALLBACK: Estimate 2D bbox from 3D size and center position
            # This is approximate but better than heuristic
            # Use typical camera FOV and distance to estimate pixel size
            # At 2.5m distance (typical approach distance), with ~60° FOV, 
            # 1m in 3D ≈ 200-300 pixels (depends on camera resolution)
            
            distance_to_center = math.sqrt(center_x_3d**2 + center_y_3d**2 + center_z_3d**2)
            if distance_to_center > 0.1:  # Valid distance
                # Estimate pixel size based on distance (perspective projection)
                # FOV typically 60-90° for wide-angle cameras
                fov_degrees = 70.0  # Approximate - should use actual camera FOV
                fov_radians = math.radians(fov_degrees)
                pixel_per_meter = (image_width / 2.0) / (distance_to_center * math.tan(fov_radians / 2.0))
                
                # Estimate pixel bbox from 3D bbox size
                pixel_width = max(50, int(bbox_width_3d * pixel_per_meter))  # Minimum 50px width
                pixel_height = max(20, int(bbox_height_3d * pixel_per_meter))  # Minimum 20px height
                
                # Estimate center pixel: assume license plate is in upper center region
                # When rover is at license plate position, plate is typically centered horizontally,
                # in upper third vertically
                center_pixel_x = image_width // 2  # Centered horizontally
                center_pixel_y = image_height // 3  # Upper third (where license plate typically is)
                
                # Calculate pixel bbox around estimated center
                x_min_pixel = max(0, center_pixel_x - pixel_width // 2)
                y_min_pixel = max(0, center_pixel_y - pixel_height // 2)
                x_max_pixel = min(image_width, center_pixel_x + pixel_width // 2)
                y_max_pixel = min(image_height, center_pixel_y + pixel_height // 2)
                
                logger = self._node.get_logger() if self._node else None
                if logger:
                    logger.info(
                        f"📐 Estimated 2D license plate bbox from 3D size (fallback projection): "
                        f"3D center=({center_x_3d:.3f},{center_y_3d:.3f},{center_z_3d:.3f})m @ {distance_to_center:.2f}m, "
                        f"3D size=({bbox_width_3d:.3f}x{bbox_height_3d:.3f}m) → "
                        f"2D=({x_min_pixel},{y_min_pixel},{x_max_pixel},{y_max_pixel})px "
                        f"(estimated - proper implementation needs camera calibration)"
                    )
                
                return (x_min_pixel, y_min_pixel, x_max_pixel, y_max_pixel)
            else:
                logger = self._node.get_logger() if self._node else None
                if logger:
                    logger.warn("Invalid distance to 3D bbox center. Cannot estimate 2D bbox.")
                return None
            
        except Exception as e:
            logger = self._node.get_logger() if self._node else None
            if logger:
                logger.warn(f"Error projecting 3D bbox to 2D: {e}. Will use heuristic fallback.")
            return None
    
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
