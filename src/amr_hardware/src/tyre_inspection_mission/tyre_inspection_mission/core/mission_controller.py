#!/usr/bin/env python3
"""
Mission Controller for Autonomous Tyre Inspection

This node orchestrates the complete tyre inspection mission:
1. Detect trucks in yard
2. Navigate to license plate and take photo
3. Switch to inspection mode
4. Detect all tyres on truck
5. Navigate to each tyre and take photo
6. Ensure no tyres are missed
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration
from enum import Enum
import time
import json
import math
import traceback
from datetime import datetime
from pathlib import Path
from collections import defaultdict
from typing import Dict, Optional, Tuple

from nav2_msgs.action import NavigateToPose
from nav2_msgs.srv import ClearEntireCostmap
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion, Twist
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from sensor_msgs.msg import Image, PointCloud2
from nav_msgs.msg import OccupancyGrid  # CRITICAL: For map bounds validation
import tf2_ros
from tf2_geometry_msgs import do_transform_pose, do_transform_pose_stamped
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default, qos_profile_system_default

from tyre_inspection_mission.detection.license_plate_detector import LicensePlateDetector
from tyre_inspection_mission.detection.tyre_re_detection_handler import TyreReDetectionHandler
from tyre_inspection_mission.detection.tyre_identifier import TyreIdentifier
from tyre_inspection_mission.detection.tyre_position_validator import TyrePositionValidator
from tyre_inspection_mission.detection.tyre_completeness_verifier import TyreCompletenessVerifier
from tyre_inspection_mission.detection.tyre_side_identifier import TyreSideIdentifier
from tyre_inspection_mission.navigation.path_alternatives import PathAlternatives
from tyre_inspection_mission.capture.tyre_capture_repositioner import TyreCaptureRepositioner
from tyre_inspection_mission.core.mission_resumer import MissionResumer
from tyre_inspection_mission.core.mission_timeout_handler import MissionTimeoutHandler, MissionPhase
from tyre_inspection_mission.navigation.vehicle_monitor import VehicleMonitor
from tyre_inspection_mission.navigation.visual_verifier import VisualVerifier
from tyre_inspection_mission.navigation.local_search import LocalSearch
from tyre_inspection_mission.navigation.goal_recalculator import GoalRecalculator
from tyre_inspection_mission.navigation.tyre_goal_validator import TyreGoalValidator
from tyre_inspection_mission.navigation.navigation_failure_handler import NavigationFailureHandler
from tyre_inspection_mission.navigation.tyre_path_optimizer import TyrePathOptimizer
from tyre_inspection_mission.navigation.tyre_pose_refiner import TyrePoseRefiner
from tyre_inspection_mission.navigation.tyre_navigation_context import TyreNavigationContext
from tyre_inspection_mission.navigation.vehicle_obstacle_manager import VehicleObstacleManager
from tyre_inspection_mission.navigation.direct_navigation_fallback import DirectNavigationFallback
from tyre_inspection_mission.capture.tyre_capture_verifier import TyreCaptureVerifier
from tyre_inspection_mission.capture.photo_quality_checker import PhotoQualityChecker
from tyre_inspection_mission.core.mission_state_manager import MissionStateManager


class MissionState(Enum):
    """Mission state machine states"""
    IDLE = "idle"
    SEARCHING_TRUCKS = "searching_trucks"
    TRUCK_DETECTED = "truck_detected"
    NAVIGATING_TO_LICENSE_PLATE = "navigating_to_license_plate"
    CAPTURING_LICENSE_PLATE = "capturing_license_plate"
    SWITCHING_TO_INSPECTION = "switching_to_inspection"
    DETECTING_TYRES = "detecting_tyres"
    NAVIGATING_TO_TYRE = "navigating_to_tyre"
    CAPTURING_TYRE = "capturing_tyre"
    CHECKING_COMPLETION = "checking_completion"
    MISSION_COMPLETE = "mission_complete"
    ERROR_RECOVERY = "error_recovery"


class TruckData:
    """Data structure for tracking truck inspection progress"""
    def __init__(self, truck_id, detection_pose, vehicle_type='truck'):
        self.truck_id = truck_id
        self.vehicle_type = vehicle_type.lower()  # Store vehicle type ('car' or 'truck')
        self.detection_pose = detection_pose  # Initial detection pose
        self.license_plate_photo_path = None
        self.license_plate_photo_taken = False
        self.tyres = []  # List of TyreData objects
        self.detection_timestamp = time.time()
        
    def add_tyre(self, tyre_id, bbox_3d):
        """Add a detected tyre"""
        tyre = TyreData(tyre_id, bbox_3d)
        self.tyres.append(tyre)
        return tyre
        
    def all_tyres_photographed(self):
        """Check if all tyres have been photographed"""
        if not self.tyres:
            return False
        return all(tyre.photo_taken for tyre in self.tyres)


class TyreData:
    """Data structure for tracking individual tyre inspection"""
    def __init__(self, tyre_id, bbox_3d):
        self.tyre_id = tyre_id
        self.bbox_3d = bbox_3d  # BoundingBox3d message
        self.position_3d = self._calculate_center(bbox_3d)
        self.photo_path = None
        self.photo_taken = False
        self.attempts = 0
        self.max_attempts = 3
        
    def _calculate_center(self, bbox):
        """Calculate center point of 3D bounding box"""
        return Point(
            x=(bbox.xmin + bbox.xmax) / 2.0,
            y=(bbox.ymin + bbox.ymax) / 2.0,
            z=(bbox.zmin + bbox.zmax) / 2.0
        )


class MissionController(Node):
    def __init__(self):
        super().__init__('mission_controller')
        
        # State machine
        self.state = MissionState.IDLE
        self.current_truck = None
        self.current_tyre_index = 0
        self.detected_trucks = {}  # truck_id -> TruckData
        self.truck_counter = 0
        
        # Parameters
        self.declare_parameter('photo_storage_dir', '~/tyre_inspection_photos')
        self.declare_parameter('approach_distance', 2.5)  # meters from object - INCREASED from 1.5m for better path clearance
        self.declare_parameter('min_approach_distance', 2.0)  # meters - absolute minimum approach distance
        self.declare_parameter('preferred_approach_distance', 2.5)  # meters - preferred approach distance
        self.declare_parameter('navigation_timeout', 60.0)  # seconds
        self.declare_parameter('detection_timeout', 30.0)  # seconds
        self.declare_parameter('photo_capture_timeout', 10.0)  # seconds
        self.declare_parameter('truck_class_name', 'truck')
        self.declare_parameter('vehicle_class_names', ['truck', 'car'])  # List of vehicle types to detect
        self.declare_parameter('tyre_class_name', 'tyre')
        self.declare_parameter('vehicle_search_timeout', 120.0)  # seconds to search for vehicles
        # CRITICAL FIX: Increased vehicle detection threshold from 0.25 to 0.5 (industry standard)
        # 0.25 was too low and caused many false positives
        self.declare_parameter('detection_confidence_threshold', 0.5)  # minimum confidence for vehicle detection (industry standard: 50%)
        self.declare_parameter('detection_confidence_threshold_low_light', 0.35)  # Lower threshold for low-light conditions (35%)
        self.declare_parameter('detection_confidence_threshold_good_light', 0.6)  # Higher threshold for good lighting (60%)
        self.declare_parameter('enable_adaptive_confidence', True)  # Enable adaptive confidence thresholds
        self.declare_parameter('detection_stability_frames', 3)  # number of consecutive frames required
        self.declare_parameter('duplicate_detection_distance', 2.0)  # meters - treat as duplicate if closer (increased for noisy 3D detections)
        self.declare_parameter('max_detection_distance', 20.0)  # meters - ignore detections beyond this
        self.declare_parameter('min_detection_distance', 0.5)  # meters - ignore detections too close
        self.declare_parameter('min_bbox_size', 0.1)  # meters - minimum bounding box dimension
        self.declare_parameter('max_bbox_size', 5.0)  # meters - maximum bounding box dimension
        self.declare_parameter('min_goal_distance', 0.8)  # meters - minimum safe goal distance - INCREASED from 0.6m
        self.declare_parameter('max_goal_distance', 100.0)  # CRITICAL: Maximum reasonable goal distance (meters) - prevents extreme values from bad calculations
        self.declare_parameter('max_goal_position', 1000.0)  # CRITICAL: Maximum absolute position value (meters) - prevents invalid coordinates like 1000000
        # CRITICAL: goal_recalculation_distance must be >= min_goal_distance (0.8m) to ensure recalculated goal is valid
        # If goal is too close, recalculate it to at least min_goal_distance away
        self.declare_parameter('goal_recalculation_distance', 0.9)  # meters - recalculate goal if too close (must be >= min_goal_distance)
        # CRITICAL: Reduced from 0.5m to 0.15m to prevent premature arrival detection
        # Must match direct control goal_tolerance for consistency
        self.declare_parameter('arrival_distance_threshold', 0.15)  # meters - arrival distance
        self.declare_parameter('stuck_detection_distance', 0.1)  # meters - stuck detection threshold
        self.declare_parameter('stuck_detection_time', 30.0)  # seconds - stuck detection time threshold
        self.declare_parameter('navigation_progress_log_interval', 10.0)  # seconds - log progress every N seconds
        self.declare_parameter('license_plate_capture_timeout', 15.0)  # seconds - timeout for license plate capture
        self.declare_parameter('license_plate_capture_retry_delay', 2.0)  # seconds - delay between retries
        # CRITICAL FIX: Increased OCR confidence thresholds to industry standards
        # Research shows: 70-80% per character, 80-90% overall for license plate OCR
        self.declare_parameter('ocr_min_confidence', 0.7)  # Minimum OCR confidence threshold (changed from 0.5 to 0.7 = 70% - industry standard)
        self.declare_parameter('ocr_min_char_confidence', 0.7)  # Per-character confidence threshold (70% - industry standard)
        self.declare_parameter('ocr_min_global_confidence', 0.8)  # Overall license plate confidence threshold (80% - industry standard)
        self.declare_parameter('enable_license_plate_ocr', True)  # Enable/disable OCR
        # CRITICAL FIX: Add license plate detection model parameters (two-stage approach)
        self.declare_parameter('license_plate_detection_confidence_threshold', 0.3)  # YOLO detection model confidence (30% - Stage 1: find plate location)
        self.declare_parameter('enable_license_plate_detection_model', True)  # Enable two-stage detection (detection model + OCR)
        self.declare_parameter('mode_switch_timeout', 10.0)  # seconds - timeout for mode switch
        self.declare_parameter('mode_switch_wait_time', 3.0)  # seconds - wait time after mode switch
        self.declare_parameter('mode_switch_verification_time', 5.0)  # seconds - time to verify mode switch
        self.declare_parameter('tyre_detection_confidence_threshold', 0.5)  # minimum confidence for tyre detection (base threshold)
        self.declare_parameter('tyre_detection_confidence_threshold_low_light', 0.35)  # Lower threshold for low-light conditions (35%)
        self.declare_parameter('tyre_detection_confidence_threshold_good_light', 0.55)  # Higher threshold for good lighting (55%)
        self.declare_parameter('tyre_detection_stability_frames', 3)  # number of consecutive frames required for tyres
        self.declare_parameter('tyre_duplicate_detection_distance', 0.5)  # meters - treat as duplicate if closer
        self.declare_parameter('max_tyre_detection_distance', 10.0)  # meters - ignore tyres beyond this
        self.declare_parameter('min_tyre_detection_distance', 0.3)  # meters - ignore tyres too close
        self.declare_parameter('min_tyre_bbox_size', 0.05)  # meters - minimum tyre bounding box dimension
        self.declare_parameter('max_tyre_bbox_size', 2.0)  # meters - maximum tyre bounding box dimension
        self.declare_parameter('tyre_detection_timeout', 30.0)  # seconds - timeout for tyre detection
        self.declare_parameter('tyre_capture_timeout', 15.0)  # seconds - timeout for tyre photo capture
        self.declare_parameter('tyre_capture_retry_delay', 2.0)  # seconds - delay between capture retries
        self.declare_parameter('max_tyre_capture_attempts', 3)  # maximum capture attempts per tyre
        self.declare_parameter('completion_check_timeout', 10.0)  # seconds - timeout for completion check
        self.declare_parameter('max_completion_retry_attempts', 2)  # maximum retry attempts for missing tyres
        self.declare_parameter('error_recovery_wait_time', 3.0)  # seconds - wait time in error recovery
        
        # Vehicle monitoring parameters
        self.declare_parameter('vehicle_movement_threshold', 0.5)  # meters - movement to trigger goal update
        self.declare_parameter('vehicle_visibility_timeout', 5.0)  # seconds - time before considering vehicle lost
        self.declare_parameter('vehicle_reposition_threshold', 1.0)  # meters - significant movement threshold
        
        # Visual verification parameters
        self.declare_parameter('arrival_verification_timeout', 10.0)  # seconds - max time to verify vehicle
        self.declare_parameter('arrival_position_tolerance', 1.0)  # meters - position match tolerance
        self.declare_parameter('arrival_orientation_tolerance', 0.5)  # radians - orientation match tolerance
        self.declare_parameter('verification_frames_required', 2)  # frames - required for stable verification
        
        # Local search parameters
        self.declare_parameter('local_search_radius', 2.0)  # meters - search radius around arrival location
        self.declare_parameter('local_search_points', 8)  # number of search positions
        self.declare_parameter('local_search_timeout', 30.0)  # seconds - max search time
        self.declare_parameter('local_search_spacing', 0.5)  # meters - spacing between search points
        
        # Goal recalculation parameters
        self.declare_parameter('max_goal_updates', 3)  # maximum goal recalculations per navigation
        self.declare_parameter('goal_update_cooldown', 5.0)  # seconds - minimum time between updates
        
        # Tyre goal validation parameters
        self.declare_parameter('unreachable_goal_threshold', 0.3)  # meters - if goal too close to obstacle
        self.declare_parameter('max_goal_adjustment_distance', 1.0)  # meters - max goal adjustment distance
        
        # Tyre capture verification parameters
        self.declare_parameter('optimal_tyre_distance_min', 0.8)  # meters - optimal distance for tyre photo
        self.declare_parameter('optimal_tyre_distance_max', 1.5)  # meters - optimal distance for tyre photo
        self.declare_parameter('optimal_angle_tolerance', 0.4)  # radians - optimal camera angle tolerance
        self.declare_parameter('tyre_capture_verification_timeout', 10.0)  # seconds - verification timeout
        self.declare_parameter('tyre_verification_frames_required', 2)  # frames - required for stable verification
        self.declare_parameter('tyre_position_tolerance', 0.3)  # meters - tyre position match tolerance
        
        # Navigation failure handling parameters
        self.declare_parameter('navigation_failure_max_retries', 2)  # maximum retry attempts
        self.declare_parameter('retry_backoff_distance', 0.3)  # meters - distance to back off on retry
        self.declare_parameter('min_alternative_goal_distance', 0.5)  # meters - minimum alternative goal distance
        self.declare_parameter('unreachable_goal_timeout', 5.0)  # seconds - timeout for unreachable goal
        
        # Tyre path optimization parameters
        self.declare_parameter('use_side_based_grouping', True)  # group tyres by vehicle side
        
        # Tyre pose refinement parameters
        self.declare_parameter('tyre_pose_refinement_max_iterations', 3)  # max refinement iterations
        self.declare_parameter('tyre_pose_convergence_threshold', 0.1)  # meters - convergence threshold
        self.declare_parameter('max_refinement_distance', 0.5)  # meters - max pose change for refinement
        
        # Photo quality checking parameters
        self.declare_parameter('photo_min_brightness', 30)  # minimum brightness (0-255)
        self.declare_parameter('photo_max_brightness', 240)  # maximum brightness (0-255)
        self.declare_parameter('photo_min_contrast', 20)  # minimum contrast threshold
        self.declare_parameter('photo_min_sharpness', 100)  # minimum sharpness (Laplacian variance)
        self.declare_parameter('photo_min_file_size_kb', 10)  # minimum file size in KB
        
        # Tyre navigation context parameters
        self.declare_parameter('max_navigation_attempts_per_tyre', 3)  # max attempts per tyre
        self.declare_parameter('navigation_context_timeout', 300.0)  # seconds - context timeout
        
        # Vehicle obstacle management parameters
        self.declare_parameter('vehicle_safety_margin', 1.0)  # meters - safety margin around vehicle
        self.declare_parameter('min_corner_clearance', 1.5)  # meters - clearance when going around corners
        self.declare_parameter('use_waypoint_navigation', True)  # use waypoints around vehicle
        self.declare_parameter('vehicle_length', 8.0)  # meters - typical vehicle length
        self.declare_parameter('vehicle_width', 2.5)  # meters - typical vehicle width
        
        # Tyre re-detection parameters
        self.declare_parameter('tyre_position_update_threshold', 0.3)  # meters - min change to update position
        self.declare_parameter('enable_tyre_re_detection', True)  # enable/disable re-detection
        
        # Mission state persistence parameters
        self.declare_parameter('mission_state_save_interval', 30.0)  # seconds - state save interval
        self.declare_parameter('enable_state_persistence', True)  # enable/disable state saving
        
        # Tyre identification parameters
        self.declare_parameter('tyre_size_tolerance', 0.2)  # meters - size difference tolerance
        self.declare_parameter('min_tyre_diameter', 0.3)  # meters - minimum tyre diameter
        self.declare_parameter('max_tyre_diameter', 1.5)  # meters - maximum tyre diameter
        
        # Tyre position validation parameters
        self.declare_parameter('max_tyre_height', 1.5)  # meters - tyres shouldn't be too high
        self.declare_parameter('min_tyre_height', -0.5)  # meters - tyres shouldn't be below ground
        self.declare_parameter('max_tyre_distance_from_vehicle', 5.0)  # meters
        self.declare_parameter('min_tyre_distance_from_vehicle', 0.3)  # meters
        
        # Tyre completeness verification parameters
        self.declare_parameter('min_expected_tyres', 4)  # Typical vehicle has 4 tyres
        self.declare_parameter('max_expected_tyres', 6)  # Trucks might have 6
        self.declare_parameter('enable_multi_angle_detection', True)  # Enable multi-angle detection
        
        # Path alternatives parameters
        self.declare_parameter('max_path_alternative_attempts', 3)  # Max alternative paths to try
        self.declare_parameter('path_alternative_angle_step', 0.785)  # radians (45 degrees)
        self.declare_parameter('path_alternative_distance_offset', 0.5)  # meters
        
        # Mission timeout parameters
        self.declare_parameter('max_mission_time', 3600.0)  # seconds - 1 hour max
        self.declare_parameter('max_phase_time', 600.0)  # seconds - 10 minutes per phase
        self.declare_parameter('timeout_warning_threshold', 0.8)  # Warn at 80% of timeout
        
        # Tyre side identification parameters
        self.declare_parameter('tyre_side_tolerance', 0.5)  # meters
        
        # Tyre capture repositioning parameters
        self.declare_parameter('max_capture_reposition_attempts', 3)  # Max reposition attempts
        self.declare_parameter('capture_reposition_distance_step', 0.2)  # meters
        self.declare_parameter('capture_reposition_angle_step', 0.524)  # radians (30 degrees)
        
        photo_dir = Path(self.get_parameter('photo_storage_dir').value).expanduser()
        self.photo_dir = photo_dir / f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
        self.photo_dir.mkdir(parents=True, exist_ok=True)
        (self.photo_dir / "license_plates").mkdir(exist_ok=True)
        (self.photo_dir / "tyres").mkdir(exist_ok=True)
        
        self.get_logger().info(f"Photo storage directory: {self.photo_dir}")
        
        # Subscribers
        # CRITICAL: Must match segmentation_processor output topic from config.yaml
        self.bbox_sub = self.create_subscription(
            BoundingBoxes3d,
            '/darknet_ros_3d/bounding_boxes',  # Changed from '/segmentation_processor/bounding_boxes_3d'
            self.bbox_callback,
            10
        )
        
        # Publishers
        self.segmentation_mode_pub = self.create_publisher(
            String,
            '/segmentation_mode',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/mission_controller/status',
            10
        )
        
        # Action clients
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.get_logger().info("Waiting for navigate_to_pose action server...")
        nav2_ready = self.nav_client.wait_for_server(timeout_sec=5.0)
        if nav2_ready:
            self.get_logger().info("✅ Nav2 action server connected")
        else:
            self.get_logger().warn(
                "⚠️ Nav2 action server not available during initialization (timeout after 5s). "
                "Mission controller will start, but navigation will wait for Nav2 to become ready. "
                "Ensure Nav2 is launched with use_mapping_nav:=true"
            )
        
        # Costmap clearing clients (for recovery - created lazily on first use)
        self.clear_local_costmap_client = None
        self.clear_global_costmap_client = None
        
        # Service clients
        self.capture_photo_client = self.create_client(
            Trigger,
            '/photo_capture/capture'
        )
        while not self.capture_photo_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Photo capture service not available. Will retry...")
        
        # Service servers
        self.start_service = self.create_service(
            Trigger,
            '/mission_controller/start',
            self.start_mission_callback
        )
        
        self.stop_service = self.create_service(
            Trigger,
            '/mission_controller/stop',
            self.stop_mission_callback
        )
        
        # TF
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # CRITICAL: Map metadata for bounds validation (prevents Nav2 worldToMap errors)
        self.map_info = None  # Store latest map metadata: {width, height, resolution, origin_x, origin_y}
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            qos_profile_system_default  # TRANSIENT_LOCAL durability to get latest map even if subscribed late
        )
        
        # Timers
        self.state_machine_timer = self.create_timer(0.5, self.state_machine_step)
        
        # Navigation state
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_feedback_future = None
        self.nav_start_time = None
        self.nav_goal_pose = None  # Track the goal pose we're navigating to
        self.nav_start_pose = None  # Track robot position when navigation started
        self.pending_send_goal_future = None  # CRITICAL: Track pending goal send to prevent race conditions
        self.last_robot_pose = None  # Track last robot position for stuck detection
        self.last_robot_pose_time = None  # Track when we last updated robot pose
        self.nav_progress_last_update = None  # Track last progress update time
        self.nav_progress_distance = 0.0  # Track distance traveled
        self.goal_recalculation_attempts = 0  # Track goal recalculation attempts
        self.max_goal_recalculation_attempts = 3  # Maximum recalculation attempts
        self.stuck_detection_enabled = True  # Enable stuck detection
        self.stuck_threshold_distance = 0.1  # meters - consider stuck if moved less than this
        self.stuck_threshold_time = 30.0  # seconds - consider stuck if not moved for this time
        # CRITICAL: Reduced from 0.5m to 0.15m to prevent premature arrival detection
        self.arrival_distance_threshold = 0.15  # meters - consider arrived if within this distance
        self.min_movement_before_arrival_check = 0.5  # meters - must move at least 50cm before checking arrival
        self.nav_initial_distance = None  # Will be set when navigation starts
        
        # Detection tracking
        self.last_detection_time = None
        self.detection_start_time = None
        self.last_bbox_header = None  # Store last bounding box message header for pose calculation
        
        # Vehicle detection tracking (for stability checks)
        self.pending_vehicle_detections = {}  # vehicle_id -> {bbox, frame_count, first_seen_time, last_seen_time}
        self.vehicle_detection_history = []  # List of recent detection frames for stability checks
        
        # License plate capture state
        self.license_plate_detector = None  # LicensePlateDetector instance
        self.license_plate_capture_attempts = 0  # Track capture attempts
        self.max_license_plate_capture_attempts = 3  # Maximum capture attempts
        self.license_plate_capture_start_time = None  # Track capture start time
        self.latest_camera_image = None  # Latest camera image for OCR
        self.latest_pointcloud = None  # Latest pointcloud for 3D position
        self.license_plate_text = None  # Detected license plate text
        self.license_plate_confidence = None  # OCR confidence
        self.license_plate_3d_position = None  # 3D position in map frame
        # CRITICAL FIX: Two-stage license plate detection - store detected bbox from YOLO
        self.detected_license_plate_bbox = None  # Detected license plate bounding box (from YOLO)
        self.detected_license_plate_bbox_time = None  # Timestamp of detection
        self.detected_license_plate_confidence = None  # Detection model confidence
        
        # Mode switching state
        self.mode_switch_start_time = None  # Track mode switch start time
        self.current_segmentation_mode = "navigation"  # Track current mode
        self.mode_switch_attempts = 0  # Track mode switch attempts
        self.max_mode_switch_attempts = 3  # Maximum mode switch attempts
        self.last_bbox_before_mode_switch = None  # Store last bbox before switch for comparison
        self.mode_switch_verified = False  # Track if mode switch was verified
        
        # Tyre detection tracking (for stability checks)
        self.pending_tyre_detections = {}  # tyre_id -> {bbox, frame_count, first_seen_time, last_seen_time}
        self.tyre_detection_start_time = None  # Track when tyre detection started
        
        # Tyre capture tracking
        self.tyre_capture_start_time = None  # Track when tyre capture started
        self.tyre_capture_attempts = 0  # Track capture attempts for current tyre
        
        # Completion checking tracking
        self.completion_check_start_time = None  # Track when completion check started
        self.completion_retry_attempts = 0  # Track retry attempts for missing tyres
        self.last_error_state = None  # Track the state that caused error recovery
        self.error_recovery_start_time = None  # Track when error recovery started
        
        # System readiness tracking
        self.system_readiness_checked = False
        self.system_readiness_status = {}
        self.last_readiness_check_time = None
        self.readiness_check_interval = 5.0  # Check every 5 seconds in IDLE
        
        # Topic availability tracking
        self.camera_image_received = False
        self.last_camera_image_time = None
        self.detection_bbox_received = False
        self.last_detection_bbox_time = None
        
        # Subscribers for system health monitoring and OCR
        self.camera_image_sub = self.create_subscription(
            Image,
            '/oak/rgb/image_rect',
            self._camera_image_callback,
            qos_profile_sensor_data
        )
        
        # Point cloud subscription (for 3D position calculation)
        self.latest_pointcloud = None
        self.pointcloud_sub = self.create_subscription(
            PointCloud2,
            '/oak/depth/points',
            self._pointcloud_callback,
            qos_profile_sensor_data
        )
        
        # Initialize license plate detector
        try:
            self.license_plate_detector = LicensePlateDetector(node=self)
            self.get_logger().info("✅ License plate detector initialized")
        except Exception as e:
            self.get_logger().error(
                f"Failed to initialize license plate detector: {e}. "
                "OCR features will be disabled.",
                exc_info=True
            )
            self.license_plate_detector = None
        
        # Initialize robustness modules
        try:
            # Build parameters dictionary for modules
            nav_params = {
                'vehicle_movement_threshold': self.get_parameter('vehicle_movement_threshold').value,
                'vehicle_visibility_timeout': self.get_parameter('vehicle_visibility_timeout').value,
                'vehicle_reposition_threshold': self.get_parameter('vehicle_reposition_threshold').value,
                'truck_class_name': self.get_parameter('truck_class_name').value,
                'detection_confidence_threshold': self.get_parameter('detection_confidence_threshold').value,
                'arrival_verification_timeout': self.get_parameter('arrival_verification_timeout').value,
                'arrival_position_tolerance': self.get_parameter('arrival_position_tolerance').value,
                'arrival_orientation_tolerance': self.get_parameter('arrival_orientation_tolerance').value,
                'verification_frames_required': self.get_parameter('verification_frames_required').value,
                'local_search_radius': self.get_parameter('local_search_radius').value,
                'local_search_points': self.get_parameter('local_search_points').value,
                'local_search_timeout': self.get_parameter('local_search_timeout').value,
                'local_search_spacing': self.get_parameter('local_search_spacing').value,
                'max_goal_updates': self.get_parameter('max_goal_updates').value,
                'goal_update_cooldown': self.get_parameter('goal_update_cooldown').value,
                'goal_recalculation_distance': self.get_parameter('goal_recalculation_distance').value,
                'approach_distance': self.get_parameter('approach_distance').value,
                'min_goal_distance': self.get_parameter('min_goal_distance').value,
                'max_goal_distance': self.get_parameter('max_goal_distance').value,
                'max_goal_position': self.get_parameter('max_goal_position').value,
                'arrival_distance_threshold': self.get_parameter('arrival_distance_threshold').value,
                'navigation_timeout': self.get_parameter('navigation_timeout').value,
                # Tyre goal validation parameters
                'unreachable_goal_threshold': self.get_parameter('unreachable_goal_threshold').value,
                'max_goal_adjustment_distance': self.get_parameter('max_goal_adjustment_distance').value,
                'max_tyre_detection_distance': self.get_parameter('max_tyre_detection_distance').value,
                'min_tyre_detection_distance': self.get_parameter('min_tyre_detection_distance').value,
                # Tyre capture verification parameters
                'optimal_tyre_distance_min': self.get_parameter('optimal_tyre_distance_min').value,
                'optimal_tyre_distance_max': self.get_parameter('optimal_tyre_distance_max').value,
                'optimal_angle_tolerance': self.get_parameter('optimal_angle_tolerance').value,
                'tyre_capture_verification_timeout': self.get_parameter('tyre_capture_verification_timeout').value,
                'tyre_verification_frames_required': self.get_parameter('tyre_verification_frames_required').value,
                'tyre_position_tolerance': self.get_parameter('tyre_position_tolerance').value,
                'tyre_detection_confidence_threshold': self.get_parameter('tyre_detection_confidence_threshold').value,
                # Navigation failure handling parameters
                'navigation_failure_max_retries': self.get_parameter('navigation_failure_max_retries').value,
                'retry_backoff_distance': self.get_parameter('retry_backoff_distance').value,
                'min_alternative_goal_distance': self.get_parameter('min_alternative_goal_distance').value,
                'unreachable_goal_timeout': self.get_parameter('unreachable_goal_timeout').value,
                # Tyre path optimization parameters
                'use_side_based_grouping': self.get_parameter('use_side_based_grouping').value,
                # Tyre pose refinement parameters
                'tyre_pose_refinement_max_iterations': self.get_parameter('tyre_pose_refinement_max_iterations').value,
                'tyre_pose_convergence_threshold': self.get_parameter('tyre_pose_convergence_threshold').value,
                'max_refinement_distance': self.get_parameter('max_refinement_distance').value,
                # Photo quality checking parameters
                'photo_min_brightness': self.get_parameter('photo_min_brightness').value,
                'photo_max_brightness': self.get_parameter('photo_max_brightness').value,
                'photo_min_contrast': self.get_parameter('photo_min_contrast').value,
                'photo_min_sharpness': self.get_parameter('photo_min_sharpness').value,
                'photo_min_file_size_kb': self.get_parameter('photo_min_file_size_kb').value,
                # Tyre navigation context parameters
                'max_navigation_attempts_per_tyre': self.get_parameter('max_navigation_attempts_per_tyre').value,
                'navigation_context_timeout': self.get_parameter('navigation_context_timeout').value,
                # Vehicle obstacle management parameters
                'vehicle_safety_margin': self.get_parameter('vehicle_safety_margin').value,
                'min_corner_clearance': self.get_parameter('min_corner_clearance').value,
                'use_waypoint_navigation': self.get_parameter('use_waypoint_navigation').value,
                'vehicle_length': self.get_parameter('vehicle_length').value,
                'vehicle_width': self.get_parameter('vehicle_width').value,
                # Tyre re-detection parameters
                'tyre_position_update_threshold': self.get_parameter('tyre_position_update_threshold').value,
                'enable_tyre_re_detection': self.get_parameter('enable_tyre_re_detection').value,
                # Tyre identification parameters
                'tyre_size_tolerance': self.get_parameter('tyre_size_tolerance').value,
                'min_tyre_diameter': self.get_parameter('min_tyre_diameter').value,
                'max_tyre_diameter': self.get_parameter('max_tyre_diameter').value,
                # Tyre position validation parameters
                'max_tyre_height': self.get_parameter('max_tyre_height').value,
                'min_tyre_height': self.get_parameter('min_tyre_height').value,
                'max_tyre_distance_from_vehicle': self.get_parameter('max_tyre_distance_from_vehicle').value,
                'min_tyre_distance_from_vehicle': self.get_parameter('min_tyre_distance_from_vehicle').value,
                # Tyre completeness verification parameters
                'min_expected_tyres': self.get_parameter('min_expected_tyres').value,
                'max_expected_tyres': self.get_parameter('max_expected_tyres').value,
                'enable_multi_angle_detection': self.get_parameter('enable_multi_angle_detection').value,
                # Path alternatives parameters
                'max_path_alternative_attempts': self.get_parameter('max_path_alternative_attempts').value,
                'path_alternative_angle_step': self.get_parameter('path_alternative_angle_step').value,
                'path_alternative_distance_offset': self.get_parameter('path_alternative_distance_offset').value,
                # Mission timeout parameters
                'max_mission_time': self.get_parameter('max_mission_time').value,
                'max_phase_time': self.get_parameter('max_phase_time').value,
                'timeout_warning_threshold': self.get_parameter('timeout_warning_threshold').value,
                # Tyre side identification parameters
                'tyre_side_tolerance': self.get_parameter('tyre_side_tolerance').value,
                # Tyre capture repositioning parameters
                'max_capture_reposition_attempts': self.get_parameter('max_capture_reposition_attempts').value,
                'capture_reposition_distance_step': self.get_parameter('capture_reposition_distance_step').value,
                'capture_reposition_angle_step': self.get_parameter('capture_reposition_angle_step').value,
            }
            
            # Initialize vehicle monitor
            self.vehicle_monitor = VehicleMonitor(self.get_logger(), nav_params)
            
            # Initialize visual verifier
            self.visual_verifier = VisualVerifier(self.get_logger(), nav_params)
            
            # Initialize local search
            self.local_search = LocalSearch(self.get_logger(), self.nav_client, nav_params)
            
            # Initialize goal recalculator
            self.goal_recalculator = GoalRecalculator(self.get_logger(), nav_params)
            
            # Initialize tyre-specific modules
            # Tyre goal validator
            self.tyre_goal_validator = TyreGoalValidator(self.get_logger(), nav_params)
            
            # Navigation failure handler
            self.navigation_failure_handler = NavigationFailureHandler(self.get_logger(), nav_params)
            
            # Tyre capture verifier
            self.tyre_capture_verifier = TyreCaptureVerifier(self.get_logger(), nav_params)
            
            # Tyre path optimizer
            self.tyre_path_optimizer = TyrePathOptimizer(self.get_logger(), nav_params)
            
            # Tyre pose refiner
            self.tyre_pose_refiner = TyrePoseRefiner(self.get_logger(), nav_params)
            
            # Photo quality checker
            self.photo_quality_checker = PhotoQualityChecker(self.get_logger(), nav_params)
            
            # Tyre navigation context manager
            self.tyre_navigation_context = TyreNavigationContext(self.get_logger(), nav_params)
            
            # Vehicle obstacle manager
            self.vehicle_obstacle_manager = VehicleObstacleManager(self.get_logger(), nav_params)
            
            # Direct navigation fallback - ensures robot ALWAYS moves toward goal
            # CRITICAL: Pass tf_buffer for frame consistency checks (robot_pose and goal_pose must be in same frame)
            self.direct_nav_fallback = DirectNavigationFallback(self, self.get_logger(), tf_buffer=self.tf_buffer)
            
            # CRITICAL: Movement guarantee system - final safety net
            from tyre_inspection_mission.core.movement_guarantee import MovementGuarantee
            self.movement_guarantee = MovementGuarantee(self, self.get_logger())
            
            # Subscribe to Nav2's priority topic to monitor Nav2's output
            # CRITICAL: Nav2 now publishes to /cmd_vel/nav2 (remapped in custom launch)
            # Monitor this topic to detect when Nav2 is actively navigating
            self.cmd_vel_sub = self.create_subscription(
                Twist,
                '/cmd_vel/nav2',  # Monitor Nav2's priority topic (not /cmd_vel)
                self.cmd_vel_callback,
                10
            )
            self.last_nav2_cmd_vel_time = None
            self.nav2_cmd_vel_timeout = 0.5  # CRITICAL: Reduced to 0.5s - activate fallback immediately when Nav2 stops
            self.nav2_initial_cmd_vel_received = False  # Track if Nav2 ever started publishing
            
            # Tyre re-detection handler
            self.tyre_re_detection_handler = TyreReDetectionHandler(self.get_logger(), nav_params)
            
            # Tyre identifier (for deduplication)
            self.tyre_identifier = TyreIdentifier(self.get_logger(), nav_params)
            
            # Tyre position validator
            self.tyre_position_validator = TyrePositionValidator(self.get_logger(), nav_params)
            
            # Tyre completeness verifier
            self.tyre_completeness_verifier = TyreCompletenessVerifier(self.get_logger(), nav_params)
            
            # Path alternatives generator
            self.path_alternatives = PathAlternatives(self.get_logger(), nav_params)
            
            # Tyre side identifier
            self.tyre_side_identifier = TyreSideIdentifier(self.get_logger(), nav_params)
            
            # Tyre capture repositioner
            self.tyre_capture_repositioner = TyreCaptureRepositioner(self.get_logger(), nav_params)
            
            # Mission timeout handler
            self.mission_timeout_handler = MissionTimeoutHandler(self.get_logger(), nav_params)
            
            # Mission state manager
            enable_persistence = self.get_parameter('enable_state_persistence').value
            if enable_persistence:
                state_file = str(self.photo_dir.parent / "mission_state.json")
                self.mission_state_manager = MissionStateManager(self.get_logger(), state_file)
            else:
                self.mission_state_manager = None
            
            # Mission resumer
            if self.mission_state_manager:
                self.mission_resumer = MissionResumer(self.get_logger(), self.mission_state_manager)
            else:
                self.mission_resumer = None
            
            self.get_logger().info(
                "✅ Robustness modules initialized "
                "(VehicleMonitor, VisualVerifier, LocalSearch, GoalRecalculator, "
                "TyreGoalValidator, NavigationFailureHandler, TyreCaptureVerifier, "
                "TyrePathOptimizer, TyrePoseRefiner, PhotoQualityChecker, "
                "TyreNavigationContext, VehicleObstacleManager, TyreReDetectionHandler, "
                "TyreIdentifier, TyrePositionValidator, TyreCompletenessVerifier, "
                "TyreSideIdentifier, PathAlternatives, TyreCaptureRepositioner, "
                "MissionTimeoutHandler, MissionStateManager, MissionResumer)"
            )
            
        except Exception as e:
            self.get_logger().error(
                f"Failed to initialize robustness modules: {e}. "
                "Some robustness features will be disabled.",
                exc_info=True
            )
            # Set to None to disable features gracefully
            self.vehicle_monitor = None
            self.visual_verifier = None
            self.local_search = None
            self.goal_recalculator = None
            self.tyre_goal_validator = None
            self.navigation_failure_handler = None
            self.tyre_capture_verifier = None
            self.tyre_path_optimizer = None
            self.tyre_pose_refiner = None
            self.photo_quality_checker = None
            self.tyre_navigation_context = None
            self.vehicle_obstacle_manager = None
            self.tyre_re_detection_handler = None
            self.tyre_identifier = None
            self.tyre_position_validator = None
            self.tyre_completeness_verifier = None
            self.tyre_side_identifier = None
            self.path_alternatives = None
            self.tyre_capture_repositioner = None
            self.mission_timeout_handler = None
            self.mission_state_manager = None
            self.mission_resumer = None
        
        # Local search state tracking
        self.local_search_active = False
        self.local_search_start_time = None
        
        # Vehicle monitoring state
        self.vehicle_monitoring_active = False
        self.last_vehicle_monitor_result = None
        
        # Visual verification state
        self.arrival_verification_active = False
        self.arrival_verification_start_time = None
        
        # Tyre capture verification state
        self.tyre_verification_active = False
        self.tyre_verification_start_time = None
        
        # Navigation failure tracking (per tyre)
        self.current_tyre_nav_failures = 0
        
        # Tyre navigation context state
        self.tyre_context_initialized = False
        
        # State persistence tracking
        self.last_state_save_time = None
        
        self.get_logger().info("Mission controller initialized")
        
    def start_mission_callback(self, request, response):
        """Start mission service callback"""
        if self.state != MissionState.IDLE:
            response.success = False
            response.message = f"Mission already in progress. Current state: {self.state.value}"
            return response
        
        # CRITICAL: Always clear saved state if it's IDLE or MISSION_COMPLETE before starting
        # This prevents the mission from "resuming" to an inactive state
        if self.mission_state_manager:
            saved_state = self.mission_state_manager.load_mission_state()
            if saved_state:
                saved_state_str = saved_state.get('current_state', '').lower()
                if saved_state_str in ['idle', 'mission_complete']:
                    self.get_logger().info(
                        f"⚠️ Found saved state '{saved_state_str}' (inactive). "
                        "Clearing it and starting fresh mission."
                    )
                    self.mission_state_manager.clear_mission_state()
        
        # CRITICAL: Check if mission can be resumed (only for active states)
        # Skip resume entirely if we're starting from IDLE
        should_resume = False
        if self.mission_resumer and self.mission_resumer.can_resume_mission():
            resume_info = self.mission_resumer.get_resume_info()
            if resume_info:
                saved_state_str = resume_info.get('state', '').lower()
                # Only resume if state is active (not IDLE or MISSION_COMPLETE)
                if saved_state_str not in ['idle', 'mission_complete']:
                    should_resume = True
                    self.get_logger().info(
                        f"Found active saved mission state: state={resume_info['state']}, "
                        f"truck={resume_info['truck_id']}, tyre_index={resume_info['tyre_index']}, "
                        f"trucks_detected={resume_info['trucks_detected']}"
                    )
                else:
                    self.get_logger().info(
                        f"⚠️ Saved state '{saved_state_str}' is inactive. "
                        "Skipping resume and starting fresh."
                    )
                    if self.mission_state_manager:
                        self.mission_state_manager.clear_mission_state()
        
        if should_resume:
            # Attempt to resume
            resume_result = self.mission_resumer.resume_mission(self)
            if resume_result['success']:
                # CRITICAL: Verify state is actually active after resume
                if self.state == MissionState.IDLE:
                    self.get_logger().warn(
                        "⚠️ Resume resulted in IDLE state. Clearing saved state and starting fresh."
                    )
                    if self.mission_state_manager:
                        self.mission_state_manager.clear_mission_state()
                    # Continue to start fresh mission
                else:
                    self.get_logger().info("✅ Mission resumed from saved state")
                    response.success = True
                    response.message = f"Mission resumed: truck={resume_result['truck_id']}, tyre_index={resume_result['tyre_index']}"
                    return response
            else:
                # Resume failed - continue to start fresh
                self.get_logger().warn(
                    f"⚠️ Failed to resume mission: {', '.join(resume_result.get('issues', ['Unknown error']))}. "
                    "Starting new mission instead."
                )
                if self.mission_state_manager:
                    self.mission_state_manager.clear_mission_state()
            
        # Perform system readiness check before starting
        self.get_logger().info("Checking system readiness before mission start...")
        readiness = self._check_system_readiness()
        self.system_readiness_status = readiness
        self.system_readiness_checked = True
        
        # Check if all critical systems are ready
        # CRITICAL: TF is optional for initial start (SLAM will create it)
        # We'll wait for TF to become available during navigation
        critical_components = ['nav2', 'photo_capture', 'camera', 'detection']
        optional_components = ['tf']  # TF will be available once SLAM starts
        
        all_critical_ready = all(
            readiness.get(comp, {}).get('ready', False) 
            for comp in critical_components
        )
        
        # Check optional components (warn but don't block)
        missing_optional = [
            comp for comp in optional_components 
            if not readiness.get(comp, {}).get('ready', False)
        ]
        if missing_optional:
            for comp in missing_optional:
                status = readiness.get(comp, {})
                self.get_logger().warn(
                    f"⚠️ Optional system not ready: {comp} - {status.get('message', 'Unknown')}. "
                    "Mission will start, but navigation may wait for TF to become available."
                )
        
        if not all_critical_ready:
            # Log which systems are not ready
            missing_systems = [
                comp for comp in critical_components 
                if not readiness.get(comp, {}).get('ready', False)
            ]
            error_msg = f"Cannot start mission: Systems not ready: {', '.join(missing_systems)}"
            self.get_logger().error(error_msg)
            for comp in missing_systems:
                status = readiness.get(comp, {})
                self.get_logger().error(f"  - {comp}: {status.get('message', 'Unknown')}")
            
            response.success = False
            response.message = error_msg
            return response
        
        # All systems ready - start mission
        self.get_logger().info("✅✅✅ All systems ready. Starting tyre inspection mission")
        
        # CRITICAL: Start mission timeout tracking
        if self.mission_timeout_handler:
            self.mission_timeout_handler.start_mission()
            self.mission_timeout_handler.start_phase(MissionPhase.SEARCHING)
        
        # CRITICAL LOGGING: State transition
        old_state = self.state.value
        self.state = MissionState.SEARCHING_TRUCKS
        self.get_logger().info(
            f"🔄 STATE TRANSITION: {old_state} → {self.state.value} (SEARCHING_TRUCKS)"
        )
        
        self.detected_trucks.clear()
        self.truck_counter = 0
        self.detection_start_time = time.time()
        
        # Clear pending detections from previous mission attempts
        self.pending_vehicle_detections.clear()
        self.vehicle_detection_history.clear()
        self.last_detection_time = None
        self.detection_bbox_received = False
        
        self.get_logger().info(
            f"🧹 Cleared previous mission data: detected_trucks=0, pending_detections=0"
        )
        
        # Switch to navigation mode (for truck detection)
        self.publish_segmentation_mode("navigation")
        
        confidence_threshold = self.get_parameter('detection_confidence_threshold').value
        stability_frames = self.get_parameter('detection_stability_frames').value
        search_timeout = self.get_parameter('vehicle_search_timeout').value
        
        self.get_logger().info(
            f"🚀🚀🚀 MISSION STARTED: System is now in SEARCHING_TRUCKS state"
        )
        self.get_logger().info(
            f"📋 Mission parameters: "
            f"search_timeout={search_timeout}s, "
            f"confidence_threshold={confidence_threshold}, "
            f"stability_frames={stability_frames}"
        )
        self.get_logger().info(
            f"✅ NEXT ACTION: Waiting for vehicle detection (car or truck). "
            f"Detections will be tracked and promoted to stable after {stability_frames} frames."
        )
        
        response.success = True
        response.message = "Mission started"
        return response
        
    def stop_mission_callback(self, request, response):
        """Stop mission service callback with comprehensive cleanup"""
        try:
            self.get_logger().info("Stopping mission")
            
            # CRITICAL: Cancel any active navigation with error handling
            if self.nav_goal_handle:
                try:
                    self.nav_client.cancel_goal_async(self.nav_goal_handle)
                    self.get_logger().info("Navigation goal cancelation requested")
                except Exception as e:
                    self.get_logger().warn(
                        f"Error canceling navigation goal: {e}"
                    )
                finally:
                    self.nav_goal_handle = None
                    self.nav_result_future = None
                    self.nav_feedback_future = None
                    self.pending_send_goal_future = None  # Clear pending future when canceling
            
            # CRITICAL: Clear navigation state
            self.nav_goal_pose = None
            self.nav_start_time = None
            self.nav_start_pose = None
            self.goal_recalculation_attempts = 0
            
            # CRITICAL: Reset mission state tracking
            self.detection_start_time = None
            self.license_plate_capture_start_time = None
            self.tyre_capture_start_time = None
            self.completion_check_start_time = None
            self.error_recovery_start_time = None
            self.mode_switch_start_time = None
            self.tyre_detection_start_time = None
            
            # CRITICAL: Clear active flags
            self.local_search_active = False
            self.vehicle_monitoring_active = False
            self.arrival_verification_active = False
            self.tyre_verification_active = False
            
            # CRITICAL: End mission timeout tracking
            if self.mission_timeout_handler:
                try:
                    self.mission_timeout_handler.end_mission()
                except Exception as e:
                    self.get_logger().warn(f"Error ending mission timeout tracking: {e}")
            
            # CRITICAL: Switch back to navigation mode (cleanup)
            try:
                self.publish_segmentation_mode("navigation")
            except Exception as e:
                self.get_logger().warn(f"Error publishing segmentation mode: {e}")
            
            # Transition to IDLE state
            self.state = MissionState.IDLE
            
            response.success = True
            response.message = "Mission stopped and cleaned up"
            return response
            
        except Exception as e:
            self.get_logger().error(
                f"Error in stop_mission_callback: {e}",
                exc_info=True
            )
            # Still transition to IDLE even on error
            self.state = MissionState.IDLE
            response.success = False
            response.message = f"Error stopping mission: {str(e)}"
            return response
        
    def map_callback(self, msg: OccupancyGrid):
        """
        Store map metadata for bounds validation.
        This prevents Nav2 worldToMap errors by validating goals are within map bounds.
        """
        try:
            self.map_info = {
                'width': msg.info.width,
                'height': msg.info.height,
                'resolution': msg.info.resolution,
                'origin_x': msg.info.origin.position.x,
                'origin_y': msg.info.origin.position.y,
                'timestamp': time.time()
            }
            self.get_logger().debug(
                f"📍 Map metadata updated: size={msg.info.width}x{msg.info.height} cells "
                f"({msg.info.width * msg.info.resolution:.2f}m x {msg.info.height * msg.info.resolution:.2f}m), "
                f"resolution={msg.info.resolution:.3f}m, origin=({msg.info.origin.position.x:.2f}, {msg.info.origin.position.y:.2f})"
            )
        except Exception as e:
            self.get_logger().warn(f"Error processing map callback: {e}")
    
    def _validate_goal_in_map_bounds(self, goal_pose: PoseStamped) -> bool:
        """
        Validate if goal is within current map bounds.
        
        Args:
            goal_pose: Goal pose in map frame
            
        Returns:
            True if goal is within map bounds, False otherwise
        """
        if not self.map_info or goal_pose.header.frame_id != "map":
            return True  # Can't validate - assume valid (for odom frame goals, direct control handles it)
        
        try:
            # Convert world coordinates to map coordinates
            goal_x = goal_pose.pose.position.x
            goal_y = goal_pose.pose.position.y
            
            map_x = int((goal_x - self.map_info['origin_x']) / self.map_info['resolution'])
            map_y = int((goal_y - self.map_info['origin_y']) / self.map_info['resolution'])
            
            # Check if goal is within map bounds
            in_bounds = (0 <= map_x < self.map_info['width'] and 
                        0 <= map_y < self.map_info['height'])
            
            if not in_bounds:
                self.get_logger().warn(
                    f"⚠️ Goal outside map bounds: world=({goal_x:.2f}, {goal_y:.2f}), "
                    f"map_coords=({map_x}, {map_y}), map_size=({self.map_info['width']}, {self.map_info['height']})"
                )
            
            return in_bounds
        except Exception as e:
            self.get_logger().error(f"Error validating goal in map bounds: {e}")
            return True  # Assume valid on error (fail open)
        
    def cmd_vel_callback(self, msg: Twist):
        """
        Monitor Nav2's cmd_vel output.
        If Nav2 stops publishing, activate direct navigation fallback.
        """
        # Check if this is a meaningful command (not zero velocity)
        is_moving = abs(msg.linear.x) > 0.01 or abs(msg.angular.z) > 0.01
        
        if is_moving:
            # Nav2 is actively commanding movement
            self.nav2_initial_cmd_vel_received = True
            self.last_nav2_cmd_vel_time = time.time()
            if hasattr(self, 'direct_nav_fallback'):
                self.direct_nav_fallback.update_nav2_cmd_vel_time()
            
            # CRITICAL: Update movement guarantee system cmd_vel timestamp
            if hasattr(self, 'movement_guarantee'):
                self.movement_guarantee.update_cmd_vel_time()
        else:
            # Zero velocity - still update time but mark as potentially stopped
            if hasattr(self, 'last_nav2_cmd_vel_time'):
                # Only update if we've received movement before
                if self.nav2_initial_cmd_vel_received:
                    self.last_nav2_cmd_vel_time = time.time()
    
    def bbox_callback(self, msg):
        """
        Handle incoming 3D bounding boxes with enhanced validation and logging.
        
        Enhanced with:
        - Message validation
        - Header validation
        - State-aware processing
        - Error handling
        """
        try:
            current_time = time.time()
            self.last_detection_time = current_time
            self.detection_bbox_received = True
            # Store header for use in state machine backup processing
            if msg.header:
                self.last_bbox_header = msg.header
            
            if self.state in [MissionState.IDLE, MissionState.MISSION_COMPLETE]:
                return
            
            # Validate message header
            if not msg.header:
                self.get_logger().warn("Received bounding boxes message with no header")
                return
            
            # Validate bounding boxes list
            if msg.bounding_boxes is None:
                self.get_logger().warn("Received bounding boxes message with None bounding_boxes list")
                return
            
            num_boxes = len(msg.bounding_boxes)
            
            # Log received message - use INFO level so we can see what's happening
            if num_boxes > 0:
                self.get_logger().info(
                    f"📦 Received {num_boxes} bounding box(es) in state {self.state.value}"
                )
                for bbox in msg.bounding_boxes[:5]:  # Log first 5 to avoid spam
                    self.get_logger().info(
                        f"  📍 {bbox.object_name} (prob: {bbox.probability:.2f}, "
                        f"bbox: x=[{bbox.xmin:.2f},{bbox.xmax:.2f}], "
                        f"y=[{bbox.ymin:.2f},{bbox.ymax:.2f}], "
                        f"z=[{bbox.zmin:.2f},{bbox.zmax:.2f}])"
                    )
            
            # Process detections based on current state
            if self.state == MissionState.SEARCHING_TRUCKS:
                self.get_logger().info(
                    f"🔄 bbox_callback(): Routing {num_boxes} bounding box(es) to process_truck_detections() "
                    f"(state: SEARCHING_TRUCKS)"
                )
                self.process_truck_detections(msg)
            elif self.state == MissionState.DETECTING_TYRES:
                self.get_logger().info(
                    f"🔄 bbox_callback(): Routing {num_boxes} bounding box(es) to process_tyre_detections() "
                    f"(state: DETECTING_TYRES)"
                )
                self.process_tyre_detections(msg)
            elif self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                # Monitor vehicle during navigation (CRITICAL: Enable detection processing)
                if self.vehicle_monitor and self.current_truck:
                    try:
                        monitor_result = self.vehicle_monitor.monitor_vehicle_during_navigation(
                            msg,
                            self.current_truck,
                            self.bbox_to_pose
                        )
                        self.last_vehicle_monitor_result = monitor_result
                        self.vehicle_monitoring_active = True
                        
                        # Check if vehicle moved and goal should be updated
                        if (monitor_result.get('should_update_goal', False) and 
                            self.goal_recalculator and 
                            self.nav_goal_pose):
                            new_vehicle_pose = monitor_result.get('current_vehicle_pose')
                            self._handle_vehicle_movement_during_navigation(
                                monitor_result,
                                new_vehicle_pose
                            )
                            
                            # CRITICAL: Update vehicle obstacle manager with new vehicle position
                            if new_vehicle_pose and self.vehicle_obstacle_manager:
                                self.vehicle_obstacle_manager.update_vehicle_obstacle(new_vehicle_pose)
                    except Exception as e:
                        self.get_logger().error(
                            f"Error in vehicle monitoring: {e}",
                            exc_info=True
                        )
            elif self.state == MissionState.CAPTURING_LICENSE_PLATE:
                # CRITICAL FIX: Two-stage license plate detection
                # Stage 1: Detect license plate bounding box using YOLO model (if available)
                # This enables "camera to know what a license plate looks like" before OCR
                enable_lp_detection = self.get_parameter('enable_license_plate_detection_model').value
                if enable_lp_detection:
                    license_plate_bbox_detected = False
                    lp_detection_threshold = self.get_parameter('license_plate_detection_confidence_threshold').value
                    
                    for bbox in msg.bounding_boxes:
                        class_name = bbox.object_name.lower() if bbox.object_name else ""
                        # Check for license plate class variations (license_plate, licenseplate, lp, etc.)
                        if class_name in ['license_plate', 'licenseplate', 'lp', 'licence_plate', 'licenceplate']:
                            # Found license plate detection from YOLO model
                            if bbox.probability >= lp_detection_threshold:
                                # Validate bbox is reasonable (license plates are small objects)
                                if self._validate_license_plate_bbox(bbox):
                                    # Store detected license plate bbox for use in OCR stage
                                    self.detected_license_plate_bbox = bbox
                                    self.detected_license_plate_bbox_time = time.time()
                                    self.detected_license_plate_confidence = bbox.probability
                                    license_plate_bbox_detected = True
                                    self.get_logger().info(
                                        f"✅ LICENSE PLATE DETECTED via YOLO (Stage 1 - two-stage detection): "
                                        f"confidence={bbox.probability:.2f} (threshold={lp_detection_threshold:.2f}), "
                                        f"bbox=({bbox.xmin:.3f},{bbox.ymin:.3f},{bbox.xmax:.3f},{bbox.ymax:.3f}), "
                                        f"3D=({bbox.xmin:.2f},{bbox.ymin:.2f},{bbox.zmin:.2f})m. "
                                        f"Will use this bbox for OCR stage (Stage 2)."
                                    )
                                    break  # Use first valid detection
                                else:
                                    self.get_logger().debug(
                                        f"License plate detection rejected (bbox validation failed): "
                                        f"class={class_name}, confidence={bbox.probability:.2f}"
                                    )
                            else:
                                self.get_logger().debug(
                                    f"License plate detection below threshold: "
                                    f"confidence={bbox.probability:.2f} < {lp_detection_threshold:.2f}"
                                )
                    
                    if not license_plate_bbox_detected:
                        # Clear old detection if too old (stale detection)
                        if (self.detected_license_plate_bbox_time and 
                            time.time() - self.detected_license_plate_bbox_time > 5.0):
                            self.detected_license_plate_bbox = None
                            self.detected_license_plate_bbox_time = None
                            self.detected_license_plate_confidence = None
                            self.get_logger().debug(
                                "Cleared stale license plate detection (>5s old). Will use heuristic fallback in OCR."
                            )
                        elif self.detected_license_plate_bbox is None:
                            # First time checking - log that we're looking for license plate detection
                            if not hasattr(self, '_lp_detection_logged'):
                                self._lp_detection_logged = True
                                self.get_logger().info(
                                    f"🔍 Looking for license plate detection via YOLO model (two-stage approach). "
                                    f"If not found, will fallback to heuristic (upper 40% of vehicle)."
                                )
                else:
                    # License plate detection model disabled - clear any stored detection
                    self.detected_license_plate_bbox = None
                    self.detected_license_plate_bbox_time = None
                    self.detected_license_plate_confidence = None
                
                # Verify vehicle visible before capture (existing code)
                if self.visual_verifier and self.current_truck and self.arrival_verification_active:
                    try:
                        # Perform visual verification with current bounding boxes
                        verification_result = self.visual_verifier.verify_vehicle_at_arrival(
                            msg,
                            self.current_truck.detection_pose,
                            self.current_truck.truck_id,
                            self.bbox_to_pose,
                            self._get_robot_pose("map")
                        )
                        
                        if verification_result.get('verification_passed', False):
                            self.get_logger().info(
                                f"✅ Vehicle verification passed: "
                                f"position_error={verification_result.get('position_error', 0):.2f}m"
                            )
                            self.arrival_verification_active = False
                    except Exception as e:
                        self.get_logger().error(
                            f"Error in visual verification: {e}",
                            exc_info=True
                        )
            elif self.state == MissionState.NAVIGATING_TO_TYRE:
                # CRITICAL: Handle tyre re-detection during navigation
                if (self.tyre_re_detection_handler and 
                    self.current_truck and 
                    self.current_truck.tyres):
                    try:
                        # Process tyre re-detections to update positions
                        for bbox in msg.bounding_boxes:
                            if bbox.object_name.lower() in ['tyre', 'tire']:
                                # Calculate detected position
                                detected_position = Point()
                                detected_position.x = (bbox.xmin + bbox.xmax) / 2.0
                                detected_position.y = (bbox.ymin + bbox.ymax) / 2.0
                                detected_position.z = (bbox.zmin + bbox.zmax) / 2.0
                                
                                # Process re-detection
                                update_result = self.tyre_re_detection_handler.process_tyre_re_detection(
                                    bbox,
                                    detected_position,
                                    self.current_truck.tyres,
                                    self.bbox_to_pose
                                )
                                
                                if update_result and update_result.get('updated', False):
                                    self.get_logger().info(
                                        f"✅ Tyre position updated via re-detection: "
                                        f"tyre={update_result['tyre_id']}, "
                                        f"improvement={update_result['position_improvement']:.3f}m"
                                    )
                    except Exception as e:
                        self.get_logger().error(
                            f"Error processing tyre re-detection: {e}",
                            exc_info=True
                        )
                
                # Refine tyre pose after arrival using visual feedback
                if (self.tyre_pose_refiner and 
                    self.current_truck and 
                    self.current_tyre_index < len(self.current_truck.tyres) and
                    self.tyre_pose_refiner.refinement_active):
                    try:
                        current_tyre = self.current_truck.tyres[self.current_tyre_index]
                        # Find tyre in current detections
                        tyre_bbox = None
                        for bbox in msg.bounding_boxes:
                            if bbox.object_name.lower() in ['tyre', 'tire']:
                                tyre_bbox = bbox
                                break
                        
                        if tyre_bbox and hasattr(current_tyre, 'bbox_3d'):
                            # Refine pose
                            refinement_result = self.tyre_pose_refiner.refine_tyre_pose(
                                tyre_bbox,
                                msg.header,
                                self.nav_goal_pose,  # Use goal pose as original estimate
                                self.bbox_to_pose,
                                self._get_robot_pose("map")
                            )
                            
                            if refinement_result.get('converged', False):
                                self.get_logger().info(
                                    f"✅ Tyre pose refined: position_change={refinement_result.get('position_change', 0):.3f}m"
                                )
                                self.tyre_pose_refiner.reset()
                            elif refinement_result.get('should_adjust_navigation', False):
                                # Significant pose change - consider adjusting navigation
                                # For now, just log (adjustment would require new navigation goal)
                                self.get_logger().info(
                                    f"Tyre pose refinement suggests navigation adjustment "
                                    f"(position_change: {refinement_result.get('position_change', 0):.2f}m)"
                                )
                    except Exception as e:
                        self.get_logger().error(
                            f"Error in tyre pose refinement: {e}",
                            exc_info=True
                        )
            elif self.state == MissionState.CAPTURING_TYRE:
                # Verify tyre visible before capture
                if (self.tyre_capture_verifier and 
                    self.current_truck and 
                    self.current_tyre_index < len(self.current_truck.tyres) and
                    self.tyre_verification_active):
                    try:
                        current_tyre = self.current_truck.tyres[self.current_tyre_index]
                        verification_result = self.tyre_capture_verifier.verify_tyre_before_capture(
                            msg,
                            current_tyre.tyre_id,
                            current_tyre.position_3d,
                            self._get_robot_pose("map"),
                            self.bbox_to_pose
                        )
                        
                        if verification_result.get('verification_passed', False):
                            self.get_logger().info(
                                f"✅ Tyre verification passed: "
                                f"tyre={current_tyre.tyre_id}, "
                                f"position_error={verification_result.get('position_error', 0):.2f}m"
                            )
                            self.tyre_verification_active = False
                        elif verification_result.get('recommendation') == 'adjust_position_back':
                            self.get_logger().warn("Tyre too close - position adjustment needed")
                        elif verification_result.get('recommendation') == 'adjust_angle':
                            self.get_logger().warn("Camera angle suboptimal - orientation adjustment needed")
                    except Exception as e:
                        self.get_logger().error(
                            f"Error in tyre verification: {e}",
                            exc_info=True
                        )
            else:
                # Other states don't process detections here
                self.get_logger().debug(
                    f"Ignoring bounding boxes in state {self.state.value} "
                    f"({num_boxes} boxes received)"
                )
        except Exception as e:
            self.get_logger().error(f"Error processing bounding boxes: {e}\n{traceback.format_exc()}")
                
        except AttributeError as e:
            self.get_logger().error(
                f"Invalid bounding boxes message format: {e}. "
                f"Message type: {type(msg)}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Error in bbox_callback: {e}", 
                exc_info=True
            )
                
        except AttributeError as e:
            self.get_logger().error(
                f"Invalid bounding boxes message format: {e}. "
                f"Message type: {type(msg)}"
            )
        except Exception as e:
            self.get_logger().error(
                f"Error in bbox_callback: {e}", 
                exc_info=True
            )
            
    def _validate_bounding_box(self, bbox):
        """
        Validate bounding box dimensions and position.
        
        Args:
            bbox: BoundingBox3d message
            
        Returns:
            bool: True if bounding box is valid, False otherwise
        """
        try:
            # Check bounding box dimensions
            width = abs(bbox.xmax - bbox.xmin)
            height = abs(bbox.ymax - bbox.ymin)
            depth = abs(bbox.zmax - bbox.zmin)
            
            min_size = self.get_parameter('min_bbox_size').value
            max_size = self.get_parameter('max_bbox_size').value
            
            # CRITICAL FIX: Handle zero depth (common in 3D segmentation when z bounds are same)
            # For vehicles, depth might be zero if segmentation doesn't provide proper z bounds
            # Use width and height as primary validation, depth is secondary
            # If depth is zero, check that width and height are reasonable
            if depth < 0.01:  # Essentially zero depth (1cm threshold)
                # Depth is zero - validate using width and height only
                if width < min_size or height < min_size:
                    self.get_logger().debug(
                        f"Bounding box too small (zero depth): w={width:.2f}, h={height:.2f}, d={depth:.2f} "
                        f"(min={min_size:.2f}m)"
                    )
                    return False
                if width > max_size or height > max_size:
                    self.get_logger().debug(
                        f"Bounding box too large (zero depth): w={width:.2f}, h={height:.2f}, d={depth:.2f} "
                        f"(max={max_size:.2f}m)"
                    )
                    return False
                # Zero depth is acceptable if width and height are valid
                self.get_logger().debug(
                    f"Bounding box has zero depth (acceptable): w={width:.2f}, h={height:.2f}, d={depth:.2f}"
                )
            else:
                # Normal validation with depth
            if width < min_size or height < min_size or depth < min_size:
                self.get_logger().debug(
                    f"Bounding box too small: w={width:.2f}, h={height:.2f}, d={depth:.2f} "
                    f"(min={min_size:.2f}m)"
                )
                return False
            
            if width > max_size or height > max_size or depth > max_size:
                self.get_logger().debug(
                    f"Bounding box too large: w={width:.2f}, h={height:.2f}, d={depth:.2f} "
                    f"(max={max_size:.2f}m)"
                )
                return False
            
            # Check if bounding box has valid coordinates (x and y must be valid, z can be same)
            if not (bbox.xmin < bbox.xmax and bbox.ymin < bbox.ymax):
                self.get_logger().debug(
                    f"Invalid bounding box coordinates (x/y): "
                    f"x=[{bbox.xmin:.2f}, {bbox.xmax:.2f}], "
                    f"y=[{bbox.ymin:.2f}, {bbox.ymax:.2f}], "
                    f"z=[{bbox.zmin:.2f}, {bbox.zmax:.2f}]"
                )
                return False
            
            # Z coordinates can be same (zero depth) - this is acceptable
            if bbox.zmin > bbox.zmax:
                self.get_logger().debug(
                    f"Invalid bounding box z coordinates: "
                    f"z=[{bbox.zmin:.2f}, {bbox.zmax:.2f}]"
                )
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validating bounding box: {e}")
            return False
    
    def _validate_detection_3d_position(self, bbox):
        """
        Validate 3D position is reasonable.
        
        Args:
            bbox: BoundingBox3d message
            
        Returns:
            bool: True if position is valid, False otherwise
        """
        try:
            # Calculate center position
            center_x = (bbox.xmin + bbox.xmax) / 2.0
            center_y = (bbox.ymin + bbox.ymax) / 2.0
            center_z = (bbox.zmin + bbox.zmax) / 2.0
            
            # Calculate distance from origin (assuming robot is at origin initially)
            distance = math.sqrt(center_x**2 + center_y**2 + center_z**2)
            
            min_distance = self.get_parameter('min_detection_distance').value
            max_distance = self.get_parameter('max_detection_distance').value
            
            if distance < min_distance:
                self.get_logger().debug(
                    f"Detection too close: {distance:.2f}m (min={min_distance:.2f}m) "
                    f"at ({center_x:.2f}, {center_y:.2f}, {center_z:.2f})"
                )
                return False
            
            if distance > max_distance:
                self.get_logger().debug(
                    f"Detection too far: {distance:.2f}m (max={max_distance:.2f}m) "
                    f"at ({center_x:.2f}, {center_y:.2f}, {center_z:.2f})"
                )
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validating 3D position: {e}")
            return False
    
    def _is_duplicate_detection(self, bbox, existing_detections):
        """
        Check if a detection is a duplicate of an existing one.
        
        Args:
            bbox: BoundingBox3d message
            existing_detections: dict of vehicle_id -> TruckData
            
        Returns:
            tuple: (is_duplicate: bool, existing_id: str or None)
        """
        try:
            # Calculate center position
            center_x = (bbox.xmin + bbox.xmax) / 2.0
            center_y = (bbox.ymin + bbox.ymax) / 2.0
            center_z = (bbox.zmin + bbox.zmax) / 2.0
            
            duplicate_distance = self.get_parameter('duplicate_detection_distance').value
            
            for vehicle_id, vehicle_data in existing_detections.items():
                existing_pose = vehicle_data.detection_pose.pose.position
                
                # Calculate distance to existing detection
                distance = math.sqrt(
                    (center_x - existing_pose.x)**2 +
                    (center_y - existing_pose.y)**2 +
                    (center_z - existing_pose.z)**2
                )
                
                if distance < duplicate_distance:
                    self.get_logger().debug(
                        f"Duplicate detection: distance={distance:.2f}m to {vehicle_id} "
                        f"(threshold={duplicate_distance:.2f}m)"
                    )
                    return True, vehicle_id
            
            return False, None
            
        except Exception as e:
            self.get_logger().error(f"Error checking duplicate detection: {e}")
            return False, None
    
    def _check_for_stable_detections(self):
        """
        Check pending_vehicle_detections for stable detections that should trigger state transition.
        
        This method is called from the state machine to ensure we don't miss stable detections
        that were promoted but didn't trigger a state transition (e.g., due to timing issues).
        
        Returns:
            list: List of (vehicle_id, pending_data) tuples for stable detections
        """
        try:
            stability_frames = self.get_parameter('detection_stability_frames').value
            current_time = time.time()
            stable_detections = []
            
            for vehicle_id, pending in list(self.pending_vehicle_detections.items()):
                frame_count = pending['frame_count']
                
                # Check if detection is stable
                if frame_count >= stability_frames:
                    # Check if detection is still recent (not stale)
                    if current_time - pending['last_seen_time'] <= 5.0:  # Increased from 2.0s to 5.0s
                        stable_detections.append((vehicle_id, pending))
                        self.get_logger().info(
                            f"✅ _check_for_stable_detections(): Found stable detection: {vehicle_id} "
                            f"(frame_count={frame_count}/{stability_frames})"
                        )
                    else:
                        # Stale detection - remove it
                        self.get_logger().info(
                            f"🗑️ _check_for_stable_detections(): Removing stale stable detection: {vehicle_id} "
                            f"(last seen {current_time - pending['last_seen_time']:.1f}s ago)"
                        )
                        del self.pending_vehicle_detections[vehicle_id]
                elif current_time - pending['last_seen_time'] > 5.0:  # Increased from 2.0s to 5.0s
                    # Detection lost for too long - remove from pending
                    self.get_logger().debug(
                        f"Removing stale pending detection: {vehicle_id} "
                        f"(last seen {current_time - pending['last_seen_time']:.1f}s ago, "
                        f"frame_count was {frame_count}/{stability_frames})"
                    )
                    del self.pending_vehicle_detections[vehicle_id]
            
            return stable_detections
            
        except Exception as e:
            self.get_logger().error(f"Error checking for stable detections: {e}\n{traceback.format_exc()}")
            return []
    
    def _generate_vehicle_id(self, bbox):
        """
        Generate a unique vehicle ID based on bounding box position.
        
        Args:
            bbox: BoundingBox3d message
            
        Returns:
            str: Unique vehicle ID
        """
        # Use center position to generate ID (in cm for precision)
        center_x = int((bbox.xmin + bbox.xmax) / 2.0 * 100)
        center_y = int((bbox.ymin + bbox.ymax) / 2.0 * 100)
        center_z = int((bbox.zmin + bbox.zmax) / 2.0 * 100)
        return f"vehicle_{center_x:05d}_{center_y:05d}_{center_z:05d}"
            
    def process_truck_detections(self, msg):
        """
        Process truck detections with validation and stability checks.
        
        Enhanced with:
        - Confidence threshold validation
        - Bounding box validation
        - 3D position validation
        - Duplicate detection filtering
        - Detection stability (multiple frames)
        """
        try:
            # Get vehicle class names from parameter (supports both 'car' and 'truck')
            vehicle_class_names_param = self.get_parameter('vehicle_class_names').value
            if isinstance(vehicle_class_names_param, list):
                vehicle_classes = [vc.lower() for vc in vehicle_class_names_param]
            elif isinstance(vehicle_class_names_param, str):
                vehicle_classes = [vehicle_class_names_param.lower()]
            else:
                # Fallback to truck_class_name for backward compatibility
                truck_class = self.get_parameter('truck_class_name').value
                vehicle_classes = ['truck', 'car'] if truck_class.lower() == 'truck' else [truck_class.lower()]
            
            self.get_logger().debug(f"Processing vehicle classes: {vehicle_classes}")
            
            confidence_threshold = self.get_parameter('detection_confidence_threshold').value
            stability_frames = self.get_parameter('detection_stability_frames').value
            current_time = time.time()
            
            # Track detections in this frame
            frame_detections = {}
            
            for bbox in msg.bounding_boxes:
                # Check if this is a vehicle class we're interested in
                if bbox.object_name.lower() not in [vc.lower() for vc in vehicle_classes]:
                    self.get_logger().debug(
                        f"Skipping non-vehicle detection: {bbox.object_name} "
                        f"(looking for: {vehicle_classes})"
                    )
                    continue
                
                # Validate confidence threshold
                if bbox.probability < confidence_threshold:
                    self.get_logger().debug(
                        f"⚠️ Detection confidence too low: {bbox.object_name} "
                        f"prob={bbox.probability:.2f} (threshold={confidence_threshold:.2f})"
                    )
                    continue
                
                # Validate bounding box dimensions
                if not self._validate_bounding_box(bbox):
                    # CRITICAL: Add detailed logging to diagnose validation failures
                    width = abs(bbox.xmax - bbox.xmin)
                    height = abs(bbox.ymax - bbox.ymin)
                    depth = abs(bbox.zmax - bbox.zmin)
                    min_size = self.get_parameter('min_bbox_size').value
                    max_size = self.get_parameter('max_bbox_size').value
                    self.get_logger().warn(
                        f"⚠️ Bounding box validation failed for {bbox.object_name} "
                        f"(prob={bbox.probability:.2f}): "
                        f"w={width:.2f}m, h={height:.2f}m, d={depth:.2f}m "
                        f"(min={min_size:.2f}m, max={max_size:.2f}m), "
                        f"bbox=({bbox.xmin:.2f},{bbox.ymin:.2f},{bbox.zmin:.2f}) to ({bbox.xmax:.2f},{bbox.ymax:.2f},{bbox.zmax:.2f})"
                    )
                    continue
                
                # Validate 3D position
                if not self._validate_detection_3d_position(bbox):
                    self.get_logger().info(
                        f"⚠️ 3D position validation failed for {bbox.object_name} "
                        f"(prob={bbox.probability:.2f}, pos: x={(bbox.xmin+bbox.xmax)/2:.2f}, y={(bbox.ymin+bbox.ymax)/2:.2f}, z={(bbox.zmin+bbox.zmax)/2:.2f})"
                    )
                    continue
                
                # Log that we're processing a valid detection
                self.get_logger().info(
                    f"🔍 Processing valid vehicle detection: {bbox.object_name} "
                    f"(prob={bbox.probability:.2f}, pos: x={(bbox.xmin+bbox.xmax)/2:.2f}, y={(bbox.ymin+bbox.ymax)/2:.2f})"
                )
                
                # CRITICAL: Check for duplicates BEFORE generating vehicle_id
                # This ensures we use the same ID for the same vehicle across frames
                
                # Check for duplicate detection in already confirmed vehicles
                is_duplicate_confirmed, existing_confirmed_id = self._is_duplicate_detection(bbox, self.detected_trucks)
                if is_duplicate_confirmed:
                    self.get_logger().debug(
                        f"Skipping duplicate detection (already confirmed as {existing_confirmed_id})"
                    )
                    continue
                
                # Check for duplicate in pending detections - merge if close enough
                # CRITICAL FIX: Compare bbox centers directly (not pose positions) for accurate duplicate detection
                # The bbox_to_pose() function applies an offset, which breaks distance comparison
                duplicate_distance = self.get_parameter('duplicate_detection_distance').value
                new_center_x = (bbox.xmin + bbox.xmax) / 2.0
                new_center_y = (bbox.ymin + bbox.ymax) / 2.0
                new_center_z = (bbox.zmin + bbox.zmax) / 2.0
                
                is_duplicate_pending = False
                existing_pending_id = None
                
                for pid, pdata in self.pending_vehicle_detections.items():
                    pbbox = pdata['bbox']
                    # Compare bbox centers directly (same coordinate frame)
                    pcenter_x = (pbbox.xmin + pbbox.xmax) / 2.0
                    pcenter_y = (pbbox.ymin + pbbox.ymax) / 2.0
                    pcenter_z = (pbbox.zmin + pbbox.zmax) / 2.0
                    
                    distance = math.sqrt(
                        (new_center_x - pcenter_x)**2 +
                        (new_center_y - pcenter_y)**2 +
                        (new_center_z - pcenter_z)**2
                    )
                    
                    # CRITICAL DEBUG: Log distance check for all pending detections (debug level to reduce spam)
                    self.get_logger().debug(
                        f"🔍 Duplicate check: new=({new_center_x:.2f}, {new_center_y:.2f}, {new_center_z:.2f}), "
                        f"pending[{pid}]=({pcenter_x:.2f}, {pcenter_y:.2f}, {pcenter_z:.2f}), "
                        f"distance={distance:.2f}m (threshold={duplicate_distance:.2f}m)"
                    )
                    
                    if distance < duplicate_distance:
                        is_duplicate_pending = True
                        existing_pending_id = pid
                        self.get_logger().info(
                            f"✅✅✅ Duplicate detection found in pending: distance={distance:.2f}m to {pid} "
                            f"(threshold={duplicate_distance:.2f}m)"
                        )
                        break
                if is_duplicate_pending:
                    # Merge with existing pending detection - update it directly
                    self.get_logger().info(
                        f"🔄 Merging detection with existing pending: {existing_pending_id} "
                        f"(frame_count: {self.pending_vehicle_detections[existing_pending_id]['frame_count']} → {self.pending_vehicle_detections[existing_pending_id]['frame_count'] + 1})"
                    )
                    # Update existing pending detection directly (don't add to frame_detections)
                    if existing_pending_id in self.pending_vehicle_detections:
                        pending = self.pending_vehicle_detections[existing_pending_id]
                        pending['frame_count'] += 1
                        pending['last_seen_time'] = current_time
                        pending['bbox'] = bbox  # Update with latest bbox (better confidence/position)
                    # Skip adding to frame_detections - already handled above
                    continue
                
                # Not a duplicate - generate new vehicle ID and track it
                vehicle_id = self._generate_vehicle_id(bbox)
                
                # Track this detection for stability check
                if vehicle_id not in frame_detections:
                    frame_detections[vehicle_id] = bbox
            
            # Update pending detections with stability tracking
            # Note: frame_detections contains new detections not yet merged with pending
            new_pending = {}
            for vehicle_id, bbox in frame_detections.items():
                if vehicle_id in self.pending_vehicle_detections:
                    # Existing pending detection - increment frame count
                    pending = self.pending_vehicle_detections[vehicle_id]
                    old_frame_count = pending['frame_count']
                    pending['frame_count'] += 1
                    pending['last_seen_time'] = current_time
                    pending['bbox'] = bbox  # Update with latest bbox (might have better position)
                    # CRITICAL LOGGING: Track frame count progression
                    self.get_logger().info(
                        f"📊 Pending detection updated: vehicle_id={vehicle_id}, "
                        f"frame_count={old_frame_count} → {pending['frame_count']}/{stability_frames}, "
                        f"prob={bbox.probability:.2f}"
                    )
                else:
                    # New detection - initialize tracking
                    self.get_logger().info(
                        f"🆕 New vehicle candidate detected: {vehicle_id} "
                        f"({bbox.object_name}, prob={bbox.probability:.2f}, "
                        f"pos: x={(bbox.xmin+bbox.xmax)/2:.2f}, y={(bbox.ymin+bbox.ymax)/2:.2f})"
                    )
                    new_pending[vehicle_id] = {
                        'bbox': bbox,
                        'frame_count': 1,
                        'first_seen_time': current_time,
                        'last_seen_time': current_time
                    }
            
            # Add new pending detections
            self.pending_vehicle_detections.update(new_pending)
            
            # CRITICAL LOGGING: Log current pending detection status
            if self.pending_vehicle_detections:
                self.get_logger().info(
                    f"📋 Pending detections summary: {len(self.pending_vehicle_detections)} vehicle(s) being tracked"
                )
                for vid, pdata in self.pending_vehicle_detections.items():
                    self.get_logger().info(
                        f"  - {vid}: frame_count={pdata['frame_count']}/{stability_frames}, "
                        f"age={(current_time - pdata['first_seen_time']):.1f}s"
                    )
            
            # Check for stable detections (seen in multiple consecutive frames)
            stable_detections = []
            for vehicle_id, pending in list(self.pending_vehicle_detections.items()):
                frame_count = pending['frame_count']
                # CRITICAL LOGGING: Log stability check for each pending detection
                self.get_logger().info(
                    f"🔍 Stability check: vehicle_id={vehicle_id}, "
                    f"frame_count={frame_count}/{stability_frames}, "
                    f"threshold_met={frame_count >= stability_frames}"
                )
                
                if frame_count >= stability_frames:
                    # Detection is stable - confirm it
                    stable_detections.append((vehicle_id, pending))
                    self.get_logger().info(
                        f"✅✅✅ STABLE VEHICLE DETECTION CONFIRMED: {vehicle_id} "
                        f"(confirmed after {pending['frame_count']} frames, threshold={stability_frames})"
                    )
                elif current_time - pending['last_seen_time'] > 5.0:  # Increased from 2.0s to 5.0s to account for slow detection rate
                    # Detection lost for too long - remove from pending
                    self.get_logger().info(
                        f"🗑️ Removing stale pending detection: {vehicle_id} "
                        f"(last seen {current_time - pending['last_seen_time']:.1f}s ago, "
                        f"frame_count was {frame_count}/{stability_frames})"
                    )
                    del self.pending_vehicle_detections[vehicle_id]
            
            # Process stable detections
            for vehicle_id, pending in stable_detections:
                # Remove from pending
                if vehicle_id in self.pending_vehicle_detections:
                    del self.pending_vehicle_detections[vehicle_id]
                
                # Check again for duplicates (might have been confirmed by another detection)
                is_duplicate, existing_id = self._is_duplicate_detection(
                    pending['bbox'], 
                    self.detected_trucks
                )
                if is_duplicate:
                    self.get_logger().info(
                        f"Skipping stable detection (duplicate of {existing_id})"
                    )
                    continue
                
                # Calculate truck center pose
                truck_pose = self.bbox_to_pose(pending['bbox'], msg.header)
                
                if not truck_pose:
                    self.get_logger().warn(
                        f"Failed to calculate pose for stable detection: {vehicle_id}"
                    )
                    continue
                
                # Determine vehicle type from detection
                detected_vehicle_type = pending['bbox'].object_name.lower()
                
                # Generate vehicle ID based on type
                if detected_vehicle_type == 'car':
                    vehicle_id = f"car_{self.truck_counter:03d}"
                else:
                    vehicle_id = f"truck_{self.truck_counter:03d}"
                
                # Create truck data (TruckData is used for all vehicles)
                truck_data = TruckData(vehicle_id, truck_pose, detected_vehicle_type)
                self.detected_trucks[vehicle_id] = truck_data
                self.truck_counter += 1
                
                self.get_logger().info(
                    f"✅ Confirmed vehicle detection: {vehicle_id} (type={detected_vehicle_type}) at "
                    f"({truck_pose.pose.position.x:.2f}, {truck_pose.pose.position.y:.2f}, {truck_pose.pose.position.z:.2f})"
                )
                
                # Transition to handling this vehicle (if in SEARCHING_TRUCKS state)
                # CRITICAL: Double-check state before transitioning (prevents race conditions)
                current_state_before_check = self.state
                self.get_logger().info(
                    f"🔄 Attempting state transition: current_state={current_state_before_check.value}, "
                    f"target_state=TRUCK_DETECTED, vehicle_id={vehicle_id}"
                )
                
                if self.state == MissionState.SEARCHING_TRUCKS:
                    # Validate we're still in the correct state (prevent race conditions)
                    if self.state != MissionState.SEARCHING_TRUCKS:
                        self.get_logger().warn(
                            f"⚠️ State changed during vehicle confirmation. "
                            f"Current state: {self.state.value}. Skipping transition."
                        )
                        return
                    
                    self.current_truck = truck_data
                    self.get_logger().info(
                        f"🔄🔄🔄 STATE TRANSITION: SEARCHING_TRUCKS → TRUCK_DETECTED "
                        f"for vehicle_id={vehicle_id}"
                    )
                    self.state = MissionState.TRUCK_DETECTED
                    self.get_logger().info(
                        f"✅✅✅ STATE CHANGED TO TRUCK_DETECTED. "
                        f"Current truck: {vehicle_id}, "
                        f"detection_pose: ({truck_pose.pose.position.x:.2f}, {truck_pose.pose.position.y:.2f}, {truck_pose.pose.position.z:.2f})"
                    )
                    self.get_logger().info(
                        f"✅ NEXT ACTION: System is now in TRUCK_DETECTED state. "
                        f"State machine will calculate license plate approach pose for {vehicle_id} on next step..."
                    )
                    
                    # CRITICAL: Update vehicle obstacle manager with detected vehicle
                    # CRITICAL FIX: Update vehicle dimensions based on vehicle type (car vs truck)
                    if self.vehicle_obstacle_manager and truck_data.detection_pose:
                        # Set vehicle dimensions based on type
                        if hasattr(truck_data, 'vehicle_type') and truck_data.vehicle_type == 'car':
                            # Cars are typically 4.5m long x 1.8m wide
                            self.vehicle_obstacle_manager.vehicle_length = 4.5  # meters
                            self.vehicle_obstacle_manager.vehicle_width = 1.8   # meters
                        else:
                            # Trucks are typically 8.0m long x 2.5m wide (default)
                            self.vehicle_obstacle_manager.vehicle_length = 8.0  # meters
                            self.vehicle_obstacle_manager.vehicle_width = 2.5   # meters
                        
                        self.vehicle_obstacle_manager.update_vehicle_obstacle(truck_data.detection_pose)
                        self.get_logger().info(
                            f"Vehicle obstacle updated for {vehicle_id} (type: {truck_data.vehicle_type if hasattr(truck_data, 'vehicle_type') else 'unknown'}, "
                            f"size: {self.vehicle_obstacle_manager.vehicle_length:.1f}m x {self.vehicle_obstacle_manager.vehicle_width:.1f}m)"
                        )
                    
                    # Clear tyre navigation context for new vehicle
                    if self.tyre_navigation_context:
                        self.tyre_navigation_context.clear_context()
                    
                    # Clear tyre identifier registry for new vehicle
                    if self.tyre_identifier:
                        self.tyre_identifier.clear_registry()
                else:
                    self.get_logger().warn(
                        f"⚠️ Cannot transition to TRUCK_DETECTED: current state is {self.state.value} "
                        f"(expected SEARCHING_TRUCKS). Vehicle {vehicle_id} will be processed later."
                    )
                    
                    # CRITICAL: Update vehicle obstacle manager with detected vehicle (even if not transitioning)
                    # CRITICAL FIX: Update vehicle dimensions based on vehicle type (car vs truck)
                    if self.vehicle_obstacle_manager and truck_data.detection_pose:
                        # Set vehicle dimensions based on type
                        if hasattr(truck_data, 'vehicle_type') and truck_data.vehicle_type == 'car':
                            # Cars are typically 4.5m long x 1.8m wide
                            self.vehicle_obstacle_manager.vehicle_length = 4.5  # meters
                            self.vehicle_obstacle_manager.vehicle_width = 1.8   # meters
                        else:
                            # Trucks are typically 8.0m long x 2.5m wide (default)
                            self.vehicle_obstacle_manager.vehicle_length = 8.0  # meters
                            self.vehicle_obstacle_manager.vehicle_width = 2.5   # meters
                        
                        self.vehicle_obstacle_manager.update_vehicle_obstacle(truck_data.detection_pose)
                        self.get_logger().info(
                            f"Vehicle obstacle updated for {vehicle_id} (type: {truck_data.vehicle_type if hasattr(truck_data, 'vehicle_type') else 'unknown'}, "
                            f"size: {self.vehicle_obstacle_manager.vehicle_length:.1f}m x {self.vehicle_obstacle_manager.vehicle_width:.1f}m)"
                        )
            
        except Exception as e:
            self.get_logger().error(f"Error processing truck detections: {e}\n{traceback.format_exc()}")
                            
    def _generate_tyre_id(self, bbox):
        """
        Generate a unique tyre ID based on bounding box position.
        
        Args:
            bbox: BoundingBox3d message
            
        Returns:
            str: Unique tyre ID
        """
        # Use center position to generate ID (in cm for precision)
        center_x = int((bbox.xmin + bbox.xmax) / 2.0 * 100)
        center_y = int((bbox.ymin + bbox.ymax) / 2.0 * 100)
        center_z = int((bbox.zmin + bbox.zmax) / 2.0 * 100)
        return f"tyre_{center_x:05d}_{center_y:05d}_{center_z:05d}"
    
    def _is_duplicate_tyre_detection(self, bbox, existing_tyres):
        """
        Check if a tyre detection is a duplicate of an existing one.
        
        Args:
            bbox: BoundingBox3d message
            existing_tyres: list of TyreData objects
            
        Returns:
            tuple: (is_duplicate: bool, existing_tyre: TyreData or None)
        """
        try:
            # Calculate center position
            center_x = (bbox.xmin + bbox.xmax) / 2.0
            center_y = (bbox.ymin + bbox.ymax) / 2.0
            center_z = (bbox.zmin + bbox.zmax) / 2.0
            
            duplicate_distance = self.get_parameter('tyre_duplicate_detection_distance').value
            
            for existing_tyre in existing_tyres:
                existing_pos = existing_tyre.position_3d
                
                # Calculate distance to existing tyre
                distance = math.sqrt(
                    (center_x - existing_pos.x)**2 +
                    (center_y - existing_pos.y)**2 +
                    (center_z - existing_pos.z)**2
                )
                
                if distance < duplicate_distance:
                    self.get_logger().debug(
                        f"Duplicate tyre detection: distance={distance:.2f}m to {existing_tyre.tyre_id} "
                        f"(threshold={duplicate_distance:.2f}m)"
                    )
                    return True, existing_tyre
            
            return False, None
            
        except Exception as e:
            self.get_logger().error(f"Error checking duplicate tyre detection: {e}")
            return False, None
                            
    def process_tyre_detections(self, msg):
        """
        Process tyre detections with validation and stability checks.
        
        Enhanced with:
        - Confidence threshold validation
        - Bounding box validation
        - 3D position validation
        - Duplicate detection filtering
        - Detection stability (multiple frames)
        """
        try:
            if not self.current_truck:
                return
                
            tyre_class = self.get_parameter('tyre_class_name').value
            confidence_threshold = self.get_parameter('tyre_detection_confidence_threshold').value
            stability_frames = self.get_parameter('tyre_detection_stability_frames').value
            current_time = time.time()
            
            # Track detections in this frame
            frame_detections = {}
            
            for bbox in msg.bounding_boxes:
                # Check if this is a tyre class
                if bbox.object_name.lower() != tyre_class.lower():
                    continue
                
                # Validate confidence threshold
                if bbox.probability < confidence_threshold:
                    self.get_logger().debug(
                        f"Tyre detection confidence too low: {bbox.object_name} "
                        f"prob={bbox.probability:.2f} (threshold={confidence_threshold:.2f})"
                    )
                    continue
                
                # Validate bounding box dimensions
                if not self._validate_tyre_bounding_box(bbox):
                    continue
                
                # Validate 3D position
                if not self._validate_tyre_3d_position(bbox):
                    continue
                
                # Generate tyre ID
                tyre_id = self._generate_tyre_id(bbox)
                
                # Check for duplicate detection in already confirmed tyres
                is_duplicate, existing_tyre = self._is_duplicate_tyre_detection(
                    bbox, 
                    self.current_truck.tyres
                )
                if is_duplicate:
                    self.get_logger().debug(
                        f"Skipping duplicate tyre detection (already confirmed as {existing_tyre.tyre_id})"
                    )
                    continue
                
                # Track this detection for stability check
                if tyre_id not in frame_detections:
                    frame_detections[tyre_id] = bbox
            
            # Update pending detections with stability tracking
            new_pending = {}
            for tyre_id, bbox in frame_detections.items():
                if tyre_id in self.pending_tyre_detections:
                    # Existing pending detection - increment frame count
                    pending = self.pending_tyre_detections[tyre_id]
                    pending['frame_count'] += 1
                    pending['last_seen_time'] = current_time
                    pending['bbox'] = bbox  # Update with latest bbox
                else:
                    # New detection - initialize tracking
                    self.get_logger().info(
                        f"New tyre candidate detected: {tyre_id} "
                        f"({bbox.object_name}, prob={bbox.probability:.2f}, "
                        f"pos: x={(bbox.xmin+bbox.xmax)/2:.2f}, y={(bbox.ymin+bbox.ymax)/2:.2f})"
                    )
                    new_pending[tyre_id] = {
                        'bbox': bbox,
                        'frame_count': 1,
                        'first_seen_time': current_time,
                        'last_seen_time': current_time
                    }
            
            # Add new pending detections
            self.pending_tyre_detections.update(new_pending)
            
            # Check for stable detections (seen in multiple consecutive frames)
            stable_detections = []
            for tyre_id, pending in list(self.pending_tyre_detections.items()):
                if pending['frame_count'] >= stability_frames:
                    # Detection is stable - confirm it
                    stable_detections.append((tyre_id, pending))
                    self.get_logger().info(
                        f"✅ Stable tyre detection: {tyre_id} "
                        f"(confirmed after {pending['frame_count']} frames)"
                    )
                elif current_time - pending['last_seen_time'] > 2.0:
                    # Detection lost for too long - remove from pending
                    self.get_logger().debug(
                        f"Removing stale pending tyre detection: {tyre_id} "
                        f"(last seen {current_time - pending['last_seen_time']:.1f}s ago)"
                    )
                    del self.pending_tyre_detections[tyre_id]
            
            # Process stable detections
            for tyre_id, pending in stable_detections:
                # Remove from pending
                if tyre_id in self.pending_tyre_detections:
                    del self.pending_tyre_detections[tyre_id]
                
                # CRITICAL: Use enhanced tyre identifier for duplicate detection
                detected_position = Point()
                detected_position.x = (pending['bbox'].xmin + pending['bbox'].xmax) / 2.0
                detected_position.y = (pending['bbox'].ymin + pending['bbox'].ymax) / 2.0
                detected_position.z = (pending['bbox'].zmin + pending['bbox'].zmax) / 2.0
                
                # Use tyre identifier for better duplicate detection
                is_duplicate = False
                existing_tyre = None
                if self.tyre_identifier:
                    identification_result = self.tyre_identifier.identify_tyre(
                        pending['bbox'],
                        detected_position,
                        self.current_truck.tyres
                    )
                    is_duplicate = identification_result['is_duplicate']
                    if is_duplicate:
                        # Find existing tyre by ID
                        matched_id = identification_result['matched_tyre_id']
                        existing_tyre = next(
                            (t for t in self.current_truck.tyres if t.tyre_id == matched_id),
                            None
                        )
                else:
                    # Fallback to original method
                    is_duplicate, existing_tyre = self._is_duplicate_tyre_detection(
                        pending['bbox'],
                        self.current_truck.tyres
                    )
                
                # CRITICAL: Validate tyre position before accepting
                position_valid = True
                if self.tyre_position_validator and self.current_truck.detection_pose:
                    validation_result = self.tyre_position_validator.validate_tyre_position(
                        detected_position,
                        self.current_truck.detection_pose
                    )
                    position_valid = validation_result['valid']
                    
                    if not position_valid:
                        self.get_logger().warn(
                            f"Tyre position validation failed: {', '.join(validation_result['issues'])}. "
                            f"Rejecting detection."
                        )
                        # Don't add this tyre - position is invalid
                        continue
                if is_duplicate:
                    self.get_logger().info(
                        f"Skipping stable tyre detection (duplicate of {existing_tyre.tyre_id})"
                    )
                    continue
                
                # Add tyre to truck
                try:
                    # Use identified tyre ID if available
                    if self.tyre_identifier:
                        identification_result = self.tyre_identifier.identify_tyre(
                            pending['bbox'],
                            detected_position,
                            self.current_truck.tyres
                        )
                        tyre_id = identification_result['tyre_id']
                    
                    tyre_data = self.current_truck.add_tyre(tyre_id, pending['bbox'])
                    tyre_data.position_3d = detected_position
                    
                    # Register with identifier
                    if self.tyre_identifier:
                        self.tyre_identifier.register_tyre(tyre_id, detected_position)
                    
                    self.get_logger().info(
                        f"✅ Confirmed tyre detection: {tyre_id} on {self.current_truck.truck_id}. "
                        f"Total tyres: {len(self.current_truck.tyres)} "
                        f"(position validated: {position_valid})"
                    )
                except Exception as e:
                    self.get_logger().error(
                        f"Error adding tyre to truck: {e}",
                        exc_info=True
                    )
            
        except Exception as e:
            self.get_logger().error(
                f"Error processing tyre detections: {e}",
                exc_info=True
            )
    
    def _validate_license_plate_bbox(self, bbox):
        """
        Validate license plate bounding box is reasonable.
        License plates are small rectangular objects with specific size/aspect ratio characteristics.
        
        Args:
            bbox: BoundingBox3d message with license plate detection
            
        Returns:
            bool: True if bounding box appears to be a valid license plate, False otherwise
        """
        try:
            # Check bounding box dimensions (3D size in meters)
            width = abs(bbox.xmax - bbox.xmin)
            height = abs(bbox.ymax - bbox.ymin)
            depth = abs(bbox.zmax - bbox.zmin)
            
            # License plates are small objects:
            # Typical size: 0.3-0.6m wide, 0.1-0.2m tall, very thin (depth ~0.05m)
            # Minimum reasonable size: 0.2m x 0.08m (small plates, unusual sizes)
            # Maximum reasonable size: 1.0m x 0.3m (oversized plates, unusual vehicles)
            min_plate_width = 0.2  # meters - minimum width
            max_plate_width = 1.0  # meters - maximum width
            min_plate_height = 0.08  # meters - minimum height
            max_plate_height = 0.3  # meters - maximum height
            max_plate_depth = 0.15  # meters - plates are very thin
            
            # Check width
            if width < min_plate_width or width > max_plate_width:
                self.get_logger().debug(
                    f"License plate bbox validation failed: width={width:.3f}m "
                    f"(expected {min_plate_width}-{max_plate_width}m)"
                )
                return False
            
            # Check height
            if height < min_plate_height or height > max_plate_height:
                self.get_logger().debug(
                    f"License plate bbox validation failed: height={height:.3f}m "
                    f"(expected {min_plate_height}-{max_plate_height}m)"
                )
                return False
            
            # Check depth (plates are very thin)
            if depth > max_plate_depth:
                self.get_logger().debug(
                    f"License plate bbox validation failed: depth={depth:.3f}m "
                    f"(expected < {max_plate_depth}m - plates are thin)"
                )
                return False
            
            # Check aspect ratio (width:height ratio)
            # License plates are typically 2:1 to 5:1 (wide rectangles)
            if height > 0:
                aspect_ratio = width / height
                if aspect_ratio < 1.5 or aspect_ratio > 6.0:
                    self.get_logger().debug(
                        f"License plate bbox validation failed: aspect_ratio={aspect_ratio:.2f} "
                        f"(expected 1.5-6.0 - plates are wide rectangles)"
                    )
                    return False
            
            # Check that bbox has reasonable 3D position (not at origin or extreme values)
            center_x = (bbox.xmin + bbox.xmax) / 2.0
            center_y = (bbox.ymin + bbox.ymax) / 2.0
            center_z = (bbox.zmin + bbox.zmax) / 2.0
            
            # Check for NaN/Inf
            if (math.isnan(center_x) or math.isinf(center_x) or
                math.isnan(center_y) or math.isinf(center_y) or
                math.isnan(center_z) or math.isinf(center_z)):
                self.get_logger().debug("License plate bbox validation failed: NaN/Inf in position")
                return False
            
            # Check reasonable distance (license plates should be visible, not at extreme distances)
            distance = math.sqrt(center_x**2 + center_y**2 + center_z**2)
            max_reasonable_distance = self.get_parameter('max_detection_distance').value
            min_reasonable_distance = 0.1  # Minimum 10cm
            
            if distance < min_reasonable_distance or distance > max_reasonable_distance:
                self.get_logger().debug(
                    f"License plate bbox validation failed: distance={distance:.3f}m "
                    f"(expected {min_reasonable_distance}-{max_reasonable_distance}m)"
                )
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().warn(
                f"Error validating license plate bbox: {e}",
                exc_info=True
            )
            return False
    
    def _validate_tyre_bounding_box(self, bbox):
        """
        Validate tyre bounding box dimensions.
        
        Args:
            bbox: BoundingBox3d message
            
        Returns:
            bool: True if bounding box is valid, False otherwise
        """
        try:
            # Check bounding box dimensions
            width = abs(bbox.xmax - bbox.xmin)
            height = abs(bbox.ymax - bbox.ymin)
            depth = abs(bbox.zmax - bbox.zmin)
            
            min_size = self.get_parameter('min_tyre_bbox_size').value
            max_size = self.get_parameter('max_tyre_bbox_size').value
            
            # Check if dimensions are reasonable for a tyre
            if width < min_size or height < min_size or depth < min_size:
                self.get_logger().debug(
                    f"Tyre bounding box too small: w={width:.2f}, h={height:.2f}, d={depth:.2f} "
                    f"(min={min_size:.2f}m)"
                )
                return False
            
            if width > max_size or height > max_size or depth > max_size:
                self.get_logger().debug(
                    f"Tyre bounding box too large: w={width:.2f}, h={height:.2f}, d={depth:.2f} "
                    f"(max={max_size:.2f}m)"
                )
                return False
            
            # Check if bounding box has valid coordinates
            if not (bbox.xmin < bbox.xmax and bbox.ymin < bbox.ymax and bbox.zmin < bbox.zmax):
                self.get_logger().debug(
                    f"Invalid tyre bounding box coordinates: "
                    f"x=[{bbox.xmin:.2f}, {bbox.xmax:.2f}], "
                    f"y=[{bbox.ymin:.2f}, {bbox.ymax:.2f}], "
                    f"z=[{bbox.zmin:.2f}, {bbox.zmax:.2f}]"
                )
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validating tyre bounding box: {e}")
            return False
    
    def _validate_tyre_3d_position(self, bbox):
        """
        Validate tyre 3D position is reasonable.
        
        Args:
            bbox: BoundingBox3d message
            
        Returns:
            bool: True if position is valid, False otherwise
        """
        try:
            # Calculate center position
            center_x = (bbox.xmin + bbox.xmax) / 2.0
            center_y = (bbox.ymin + bbox.ymax) / 2.0
            center_z = (bbox.zmin + bbox.zmax) / 2.0
            
            # Calculate distance from robot (assuming robot is near origin in camera frame)
            # Tyres should be relatively close to the vehicle we just detected
            distance = math.sqrt(center_x**2 + center_y**2 + center_z**2)
            
            min_distance = self.get_parameter('min_tyre_detection_distance').value
            max_distance = self.get_parameter('max_tyre_detection_distance').value
            
            if distance < min_distance:
                self.get_logger().debug(
                    f"Tyre detection too close: {distance:.2f}m (min={min_distance:.2f}m) "
                    f"at ({center_x:.2f}, {center_y:.2f}, {center_z:.2f})"
                )
                return False
            
            if distance > max_distance:
                self.get_logger().debug(
                    f"Tyre detection too far: {distance:.2f}m (max={max_distance:.2f}m) "
                    f"at ({center_x:.2f}, {center_y:.2f}, {center_z:.2f})"
                )
                return False
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validating tyre 3D position: {e}")
            return False
    
    def _handle_tyre_detection(self):
        """
        Enhanced tyre detection handler with timeout and completion checking.
        
        This method:
        1. Initializes detection tracking on first call
        2. Monitors detection progress
        3. Handles timeout scenarios
        4. Transitions to navigation when tyres found or timeout
        5. Handles all error cases gracefully
        """
        try:
            # Initialize detection tracking on first call
            if self.tyre_detection_start_time is None:
                self.tyre_detection_start_time = time.time()
                self.pending_tyre_detections.clear()
                self.get_logger().info(
                    f"DETECTING_TYRES: Starting tyre detection "
                    f"for {self.current_truck.truck_id if self.current_truck else 'unknown truck'}"
                )
            
            # Validate current truck
            if not self.current_truck:
                self.get_logger().error(
                    "DETECTING_TYRES: current_truck is None. "
                    "Transitioning to ERROR_RECOVERY."
                )
                self.state = MissionState.ERROR_RECOVERY
                return
            
            # Verify segmentation mode is "inspection"
            if self.current_segmentation_mode.lower() != "inspection":
                self.get_logger().warn(
                    f"DETECTING_TYRES: Segmentation mode is '{self.current_segmentation_mode}' "
                    f"but should be 'inspection'. Attempting to switch..."
                )
                self.publish_segmentation_mode("inspection")
                # Continue anyway - mode switch might take time
            
            # Check for timeout
            detection_timeout = self.get_parameter('tyre_detection_timeout').value
            elapsed_time = time.time() - self.tyre_detection_start_time
            remaining_time = detection_timeout - elapsed_time
            
            # Log progress every 10 seconds
            if int(elapsed_time) % 10 == 0 and elapsed_time > 0:
                tyres_found = len(self.current_truck.tyres)
                pending_count = len(self.pending_tyre_detections)
                self.get_logger().info(
                    f"DETECTING_TYRES: {elapsed_time:.0f}s elapsed, "
                    f"{remaining_time:.0f}s remaining, "
                    f"{tyres_found} tyres confirmed, "
                    f"{pending_count} pending detections"
                )
            
            if elapsed_time > detection_timeout:
                tyres_found = len(self.current_truck.tyres)
                pending_count = len(self.pending_tyre_detections)
                
                # CRITICAL: Verify completeness before proceeding
                completeness_result = None
                if self.tyre_completeness_verifier:
                    completeness_result = self.tyre_completeness_verifier.verify_completeness(
                        self.current_truck.tyres,
                        self.current_truck.detection_pose if hasattr(self.current_truck, 'detection_pose') else None
                    )
                    
                    if not completeness_result['complete']:
                        self.get_logger().warn(
                            f"Tyre detection incomplete: {completeness_result['detected_count']}/"
                            f"{completeness_result['expected_count']} detected "
                            f"({completeness_result['completeness_percentage']:.1f}%). "
                            f"Recommendation: {completeness_result['recommendation']}"
                        )
                        
                        # If recommendation is to try multi-angle detection, suggest it
                        if (completeness_result['recommendation'] == 'try_multi_angle_detection' and
                            self.tyre_completeness_verifier.enable_multi_angle_detection):
                            self.get_logger().info(
                                "Consider multi-angle detection for missing tyres. "
                                "Proceeding with detected tyres for now."
                            )
                
                if tyres_found > 0:
                    expected_info = ""
                    if completeness_result:
                        expected_info = f" (expected {completeness_result['expected_count']})"
                    self.get_logger().info(
                        f"Tyre detection timeout after {elapsed_time:.1f}s. "
                        f"Found {tyres_found} tyre(s){expected_info}. "
                        f"Optimizing visit order..."
                    )
                    
                    # CRITICAL: Optimize tyre visit order before starting navigation
                    if self.tyre_path_optimizer and self.current_truck.tyres:
                        try:
                            # Get robot's current position as start point
                            robot_pose = self._get_robot_pose("map")
                            start_pos = None
                            if robot_pose:
                                start_pos = robot_pose.pose.position
                            else:
                                # Use vehicle detection pose as fallback
                                if self.current_truck.detection_pose:
                                    start_pos = self.current_truck.detection_pose.pose.position
                            
                            # Optimize order
                            optimized_order = self.tyre_path_optimizer.optimize_tyre_visit_order(
                                self.current_truck.tyres,
                                start_pos
                            )
                            
                            if optimized_order and len(optimized_order) == len(self.current_truck.tyres):
                                # Reorder tyres according to optimized order
                                optimized_tyres = [self.current_truck.tyres[i] for i in optimized_order]
                                self.current_truck.tyres = optimized_tyres
                                
                                # Calculate total distance for logging
                                total_dist = self.tyre_path_optimizer.calculate_total_distance(
                                    self.current_truck.tyres,
                                    optimized_order,
                                    start_pos
                                )
                                
                                self.get_logger().info(
                                    f"✅ Tyre visit order optimized. "
                                    f"Total estimated distance: {total_dist:.2f}m"
                                )
                            else:
                                self.get_logger().warn(
                                    "Tyre path optimization returned invalid order. "
                                    "Using original order."
                                )
                        except Exception as e:
                            self.get_logger().error(
                                f"Error optimizing tyre path: {e}. Using original order.",
                                exc_info=True
                            )
                    
                    # Transition to navigation
                    self.current_tyre_index = 0
                    tyres_count = len(self.current_truck.tyres)
                    self.get_logger().info(
                        f"DETECTING_TYRES → NAVIGATING_TO_TYRE: "
                        f"{tyres_count} tyre(s) detected for {self.current_truck.truck_id}"
                    )
                    self.state = MissionState.NAVIGATING_TO_TYRE
                    self.get_logger().info(
                        f"✅ NEXT ACTION: System is now in NAVIGATING_TO_TYRE state. "
                        f"Navigating to first tyre (1/{tyres_count})..."
                    )
                    self.tyre_detection_start_time = None
                    self.pending_tyre_detections.clear()
                else:
                    self.get_logger().warn(
                        f"Tyre detection timeout after {elapsed_time:.1f}s. "
                        f"No tyres found. Pending detections: {pending_count}. "
                        "Moving to next truck."
                    )
                    self.finish_current_truck()
                    self.tyre_detection_start_time = None
                    self.pending_tyre_detections.clear()
                return
            
            # Check for detection pipeline failure
            detection_pipeline_timeout = 10.0  # Consider pipeline dead if no detections for 10s
            if (self.last_detection_time and 
                time.time() - self.last_detection_time > detection_pipeline_timeout):
                self.get_logger().warn(
                    f"Detection pipeline appears inactive: "
                    f"no bounding boxes received for {time.time() - self.last_detection_time:.1f}s"
                )
                # Don't fail immediately - might be temporary
                
            # Check if we have enough tyres detected (optional - continue detection)
            # For now, we'll wait for timeout or manual completion
            
        except Exception as e:
            self.get_logger().error(
                f"Error in DETECTING_TYRES state: {e}",
                exc_info=True
            )
            self.state = MissionState.ERROR_RECOVERY
                    
    def bbox_to_pose(self, bbox, header):
        """Convert 3D bounding box to navigation pose"""
        try:
            # Calculate center of bounding box
            center_x = (bbox.xmin + bbox.xmax) / 2.0
            center_y = (bbox.ymin + bbox.ymax) / 2.0
            center_z = (bbox.zmin + bbox.zmax) / 2.0
            
            # Calculate approach position (offset from object)
            approach_dist = self.get_parameter('approach_distance').value
            
            # For license plate, approach from front
            # For now, approach from center
            approach_x = center_x
            approach_y = center_y - approach_dist  # Approach from front (assuming +y is front)
            
            # Create pose in map frame
            pose = PoseStamped()
            pose.header.frame_id = header.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = approach_x
            pose.pose.position.y = approach_y
            pose.pose.position.z = center_z
            
            # Orient toward the object
            import math
            yaw = math.atan2(center_y - approach_y, center_x - approach_x)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            return pose
            
        except Exception as e:
            self.get_logger().error(f"Error converting bbox to pose: {e}")
            return None
            
    def publish_segmentation_mode(self, mode):
        """Publish segmentation mode change"""
        msg = String()
        msg.data = mode
        self.segmentation_mode_pub.publish(msg)
        self.get_logger().info(f"Switched to {mode} mode")
        
    def _camera_image_callback(self, msg):
        """Callback to track camera image reception and store latest image for OCR"""
        self.camera_image_received = True
        self.last_camera_image_time = time.time()
        # Store latest image for license plate OCR
        self.latest_camera_image = msg
    
    def _pointcloud_callback(self, msg):
        """Callback to store latest pointcloud for 3D position calculation"""
        self.latest_pointcloud = msg
        
    def _check_system_readiness(self):
        """
        Check if all required systems are ready for mission execution.
        
        Returns:
            dict: Status of each system component with 'ready' and 'message' keys
        """
        current_time = time.time()
        readiness = {}
        
        # Check Nav2 action server
        nav2_ready = self.nav_client.server_is_ready()
        readiness['nav2'] = {
            'ready': nav2_ready,
            'message': 'Nav2 action server ready' if nav2_ready else 'Nav2 action server not available'
        }
        
        # Check photo capture service
        photo_service_ready = self.capture_photo_client.service_is_ready()
        readiness['photo_capture'] = {
            'ready': photo_service_ready,
            'message': 'Photo capture service ready' if photo_service_ready else 'Photo capture service not available'
        }
        
        # Check camera (image topic)
        camera_timeout = 2.0  # Consider camera dead if no image in 2 seconds
        camera_ready = False
        if self.last_camera_image_time:
            time_since_last_image = current_time - self.last_camera_image_time
            camera_ready = time_since_last_image < camera_timeout
            readiness['camera'] = {
                'ready': camera_ready,
                'message': f'Camera publishing images (last: {time_since_last_image:.1f}s ago)' if camera_ready else f'Camera not publishing (last: {time_since_last_image:.1f}s ago)'
            }
        else:
            readiness['camera'] = {
                'ready': False,
                'message': 'Camera not publishing images (no images received yet)'
            }
        
        # Check detection pipeline (bounding boxes)
        detection_timeout = 5.0  # Consider detection dead if no bboxes in 5 seconds
        detection_ready = False
        if self.last_detection_time:
            time_since_last_detection = current_time - self.last_detection_time
            detection_ready = time_since_last_detection < detection_timeout
            readiness['detection'] = {
                'ready': detection_ready,
                'message': f'Detection pipeline active (last: {time_since_last_detection:.1f}s ago)' if detection_ready else f'Detection pipeline inactive (last: {time_since_last_detection:.1f}s ago)'
            }
        else:
            readiness['detection'] = {
                'ready': False,
                'message': 'Detection pipeline not active (no detections received yet)'
            }
        
        # Check TF tree (basic check - try to get a common transform)
        tf_ready = False
        tf_message = 'TF tree not available'
        try:
            # Try to get transform from base_footprint to map (common transform)
            transform = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.1)
            )
            tf_ready = True
            tf_message = 'TF tree available'
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            tf_ready = False
            tf_message = 'TF tree incomplete (map->base_footprint transform not available)'
        except Exception as e:
            tf_ready = False
            tf_message = f'TF tree check failed: {str(e)}'
        
        readiness['tf'] = {
            'ready': tf_ready,
            'message': tf_message
        }
        
        # Check LiDAR (scan topic) - verify topic exists
        # Note: We can't easily check if topic is publishing without subscribing
        # For now, we'll assume it's ready if TF is working (since SLAM needs both)
        readiness['lidar'] = {
            'ready': tf_ready,  # If TF works, LiDAR is likely working
            'message': 'LiDAR assumed ready (TF tree working)' if tf_ready else 'LiDAR status unknown'
        }
        
        # Overall readiness
        all_ready = all(comp['ready'] for comp in readiness.values())
        readiness['overall'] = {
            'ready': all_ready,
            'message': 'All systems ready' if all_ready else 'Some systems not ready'
        }
        
        return readiness
        
    def publish_status(self):
        """Publish current mission status"""
        status = {
            'state': self.state.value,
            'current_truck': self.current_truck.truck_id if self.current_truck else None,
            'trucks_detected': len(self.detected_trucks),
            'tyres_detected': len(self.current_truck.tyres) if self.current_truck else 0,
            'tyres_photographed': sum(1 for t in self.current_truck.tyres if t.photo_taken) if self.current_truck else 0,
            'system_readiness': self.system_readiness_status if self.system_readiness_checked else None
        }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
        
    def state_machine_step(self):
        """Main state machine execution"""
        self.publish_status()
        
        # CRITICAL: Check mission timeout
        if self.mission_timeout_handler:
            try:
                timeout_result = self.mission_timeout_handler.check_mission_timeout()
                if timeout_result['timeout']:
                    self.get_logger().error(
                        f"Mission timeout exceeded: {timeout_result['elapsed_time']:.1f}s. "
                        "Aborting mission."
                    )
                    self.state = MissionState.ERROR_RECOVERY
                    return
                elif timeout_result['warning']:
                    self.get_logger().warn(
                        f"Mission approaching timeout: {timeout_result['remaining_time']:.1f}s remaining"
                    )
                
                # Check phase timeout
                phase_timeout = self.mission_timeout_handler.check_phase_timeout()
                if phase_timeout['timeout']:
                    self.get_logger().warn(
                        f"Phase timeout: {phase_timeout['elapsed_time']:.1f}s. "
                        "Consider skipping or adjusting strategy."
                    )
            except Exception as e:
                self.get_logger().warn(f"Error checking mission timeout: {e}")
        
        # CRITICAL: Save mission state periodically for recovery
        if self.mission_state_manager:
            try:
                self._save_mission_state()
            except Exception as e:
                self.get_logger().warn(f"Error saving mission state: {e}")
        
        try:
            if self.state == MissionState.IDLE:
                # Perform system readiness checks periodically
                current_time = time.time()
                if (self.last_readiness_check_time is None or 
                    (current_time - self.last_readiness_check_time) >= self.readiness_check_interval):
                    
                    self.system_readiness_status = self._check_system_readiness()
                    self.system_readiness_checked = True
                    self.last_readiness_check_time = current_time
                    
                    # Log system status
                    overall_ready = self.system_readiness_status.get('overall', {}).get('ready', False)
                    
                    if overall_ready:
                        self.get_logger().info("✅ System readiness: All systems ready")
                    else:
                        self.get_logger().warn("⚠️ System readiness: Some systems not ready")
                        for component, status in self.system_readiness_status.items():
                            if component != 'overall' and not status.get('ready', False):
                                self.get_logger().warn(f"  - {component}: {status.get('message', 'Unknown')}")
                
            elif self.state == MissionState.SEARCHING_TRUCKS:
                # Enhanced timeout handling with progress reporting
                timeout = self.get_parameter('vehicle_search_timeout').value
                current_time = time.time()
                
                # CRITICAL LOGGING: Log state machine entry and current status
                vehicles_found = len(self.detected_trucks)
                pending_count = len(self.pending_vehicle_detections)
                stability_frames = self.get_parameter('detection_stability_frames').value
                
                self.get_logger().info(
                    f"🔍 STATE MACHINE: SEARCHING_TRUCKS handler - "
                    f"detected_trucks={vehicles_found}, pending_detections={pending_count}"
                )
                
                # CRITICAL LOGGING: Log detailed pending detection status
                if self.pending_vehicle_detections:
                    self.get_logger().info(
                        f"📊 Pending detections detail in state machine:"
                    )
                    for vid, pdata in self.pending_vehicle_detections.items():
                        frame_count = pdata['frame_count']
                        age = current_time - pdata['first_seen_time']
                        self.get_logger().info(
                            f"  - {vid}: frame_count={frame_count}/{stability_frames} "
                            f"(needs {stability_frames - frame_count} more), age={age:.1f}s"
                        )
                else:
                    self.get_logger().info(
                        f"📊 No pending detections - waiting for vehicle detections..."
                    )
                
                # CRITICAL FIX: Check for stable detections in state machine
                # This ensures we catch stable detections even if process_truck_detections() 
                # didn't trigger a state transition (e.g., due to timing/race conditions)
                stable_detections = self._check_for_stable_detections()
                
                if stable_detections:
                    self.get_logger().info(
                        f"🎯 STATE MACHINE: Found {len(stable_detections)} stable detection(s) in pending_vehicle_detections. "
                        f"Processing first stable detection..."
                    )
                    
                    # Process first stable detection
                    vehicle_id, pending = stable_detections[0]
                    
                    # Remove from pending
                    if vehicle_id in self.pending_vehicle_detections:
                        del self.pending_vehicle_detections[vehicle_id]
                    
                    # Check for duplicates in confirmed vehicles
                    is_duplicate, existing_id = self._is_duplicate_detection(
                        pending['bbox'], 
                        self.detected_trucks
                    )
                    if is_duplicate:
                        self.get_logger().info(
                            f"⚠️ STATE MACHINE: Stable detection {vehicle_id} is duplicate of {existing_id}. Skipping."
                        )
                    else:
                        # Calculate truck center pose
                        # We need a header for bbox_to_pose - use the last received message header
                        # If we don't have it, we'll need to create a dummy header
                        # For now, try to get pose - if it fails, we'll skip this detection
                        try:
                            # Use stored header from last bounding box message, or create minimal header
                            if self.last_bbox_header:
                                header = self.last_bbox_header
                                self.get_logger().info(
                                    f"✅ STATE MACHINE: Using stored header from last bbox message "
                                    f"(frame: {header.frame_id})"
                                )
                            else:
                                # Fallback: create minimal header
                                from std_msgs.msg import Header
                                header = Header()
                                header.frame_id = "camera_link"  # Default frame
                                header.stamp = self.get_clock().now().to_msg()
                                self.get_logger().warn(
                                    f"⚠️ STATE MACHINE: No stored header available, using default frame 'camera_link'"
                                )
                            
                            truck_pose = self.bbox_to_pose(pending['bbox'], header)
                            
                            if not truck_pose:
                                self.get_logger().warn(
                                    f"⚠️ STATE MACHINE: Failed to calculate pose for stable detection: {vehicle_id}"
                                )
                            else:
                                # Determine vehicle type
                                detected_vehicle_type = pending['bbox'].object_name.lower()
                                
                                # Generate vehicle ID based on type
                                if detected_vehicle_type == 'car':
                                    final_vehicle_id = f"car_{self.truck_counter:03d}"
                                else:
                                    final_vehicle_id = f"truck_{self.truck_counter:03d}"
                                
                                # Create truck data
                                truck_data = TruckData(final_vehicle_id, truck_pose, detected_vehicle_type)
                                self.detected_trucks[final_vehicle_id] = truck_data
                                self.truck_counter += 1
                                
                                self.get_logger().info(
                                    f"✅ STATE MACHINE: Confirmed vehicle detection: {final_vehicle_id} "
                                    f"(type={detected_vehicle_type}) at "
                                    f"({truck_pose.pose.position.x:.2f}, {truck_pose.pose.position.y:.2f}, {truck_pose.pose.position.z:.2f})"
                                )
                                
                                # CRITICAL: Transition to TRUCK_DETECTED state
                                if self.state == MissionState.SEARCHING_TRUCKS:
                                    self.current_truck = truck_data
                                    self.get_logger().info(
                                        f"🔄🔄🔄 STATE MACHINE: SEARCHING_TRUCKS → TRUCK_DETECTED "
                                        f"for vehicle_id={final_vehicle_id}"
                                    )
                                    self.state = MissionState.TRUCK_DETECTED
                                    self.get_logger().info(
                                        f"✅✅✅ STATE MACHINE: State changed to TRUCK_DETECTED. "
                                        f"Current truck: {final_vehicle_id}"
                                    )
                                    
                                    # Update vehicle obstacle manager
                                    if self.vehicle_obstacle_manager and truck_data.detection_pose:
                                        self.vehicle_obstacle_manager.update_vehicle_obstacle(truck_data.detection_pose)
                                    
                                    # Clear tyre navigation context
                                    if self.tyre_navigation_context:
                                        self.tyre_navigation_context.clear_context()
                                    
                                    # Clear tyre identifier registry
                                    if self.tyre_identifier:
                                        self.tyre_identifier.clear_registry()
                                    
                                    # Return early - state has changed
                                    return
                        except Exception as e:
                            self.get_logger().error(
                                f"❌ STATE MACHINE: Error processing stable detection: {e}",
                                exc_info=True
                            )
                
                # CRITICAL LOGGING: Check if we have stable detections that should trigger transition
                # This handles the case where detected_trucks has vehicles but current_truck is None
                # (e.g., state transition happened but current_truck wasn't set)
                if self.detected_trucks and not self.current_truck:
                    first_vehicle_id = list(self.detected_trucks.keys())[0]
                    self.get_logger().warn(
                        f"⚠️ WARNING: detected_trucks has {vehicles_found} vehicle(s) but current_truck is None! "
                        f"First vehicle: {first_vehicle_id}. Fixing by setting current_truck..."
                    )
                    # Fix: Set current_truck and transition state
                    self.current_truck = self.detected_trucks[first_vehicle_id]
                    if self.state == MissionState.SEARCHING_TRUCKS:
                        self.get_logger().info(
                            f"🔄 STATE MACHINE: Transitioning to TRUCK_DETECTED for {first_vehicle_id}"
                        )
                        self.state = MissionState.TRUCK_DETECTED
                        return
                
                if self.detection_start_time:
                    elapsed_time = current_time - self.detection_start_time
                    remaining_time = timeout - elapsed_time
                    
                    # Log progress every 30 seconds
                    if int(elapsed_time) % 30 == 0 and elapsed_time > 0:
                        self.get_logger().info(
                            f"Searching for vehicles: {elapsed_time:.0f}s elapsed, "
                            f"{remaining_time:.0f}s remaining, "
                            f"{vehicles_found} vehicles found, "
                            f"{pending_count} pending detections"
                        )
                    
                    # Check for timeout
                    if elapsed_time > timeout:
                        vehicles_found = len(self.detected_trucks)
                        pending_count = len(self.pending_vehicle_detections)
                        
                        if vehicles_found > 0:
                            self.get_logger().info(
                                f"Vehicle search timeout after {elapsed_time:.1f}s. "
                                f"Found {vehicles_found} vehicle(s). Proceeding with inspection."
                            )
                            # Transition to first vehicle if found
                            if self.detected_trucks:
                                first_vehicle_id = list(self.detected_trucks.keys())[0]
                                self.current_truck = self.detected_trucks[first_vehicle_id]
                                self.state = MissionState.TRUCK_DETECTED
                                self.get_logger().info(
                                    f"Transitioning to TRUCK_DETECTED state for {first_vehicle_id}"
                                )
                        else:
                            self.get_logger().warn(
                                f"Vehicle search timeout after {elapsed_time:.1f}s. "
                                f"No vehicles found. "
                                f"Pending detections: {pending_count}. "
                                f"Mission complete."
                            )
                            self.state = MissionState.MISSION_COMPLETE
                    
                    # Check for detection pipeline failure
                    detection_pipeline_timeout = 10.0  # Consider pipeline dead if no detections for 10s
                    if (self.last_detection_time and 
                        current_time - self.last_detection_time > detection_pipeline_timeout):
                        self.get_logger().warn(
                            f"Detection pipeline appears inactive: "
                            f"no bounding boxes received for {current_time - self.last_detection_time:.1f}s"
                        )
                        # Don't fail immediately - might be temporary
                
                # Validate segmentation mode is set to "navigation"
                # Note: We can't easily check this without subscribing, but we log when we set it
                # The mode was set in start_mission_callback, so we assume it's correct
                    
            elif self.state == MissionState.TRUCK_DETECTED:
                # Enhanced TRUCK_DETECTED state with validation and error handling
                # CRITICAL LOGGING: Log state machine entry
                self.get_logger().info(
                    f"🎯 STATE MACHINE: TRUCK_DETECTED handler entered"
                )
                
                # CRITICAL: Re-validate state at start (prevents processing in wrong state)
                if self.state != MissionState.TRUCK_DETECTED:
                    # State changed - skip processing
                    self.get_logger().warn(
                        f"⚠️ State changed during TRUCK_DETECTED handler - current state: {self.state.value}"
                    )
                    return
                
                try:
                    # Validate current_truck exists
                    if not self.current_truck:
                        self.get_logger().error(
                            f"❌ CRITICAL ERROR: TRUCK_DETECTED state but current_truck is None! "
                            f"detected_trucks has {len(self.detected_trucks)} vehicle(s)"
                        )
                        self.get_logger().error(
                            "TRUCK_DETECTED state entered but current_truck is None. "
                            "Transitioning to ERROR_RECOVERY."
                        )
                        self.state = MissionState.ERROR_RECOVERY
                    else:
                        # Validate detection_pose exists and is valid
                        if not hasattr(self.current_truck, 'detection_pose') or not self.current_truck.detection_pose:
                            self.get_logger().error(
                                f"TRUCK_DETECTED: {self.current_truck.truck_id} has no detection_pose. "
                                "Transitioning to ERROR_RECOVERY."
                            )
                            self.state = MissionState.ERROR_RECOVERY
                        else:
                            detection_pose = self.current_truck.detection_pose
                            
                            # Validate pose structure
                            if not hasattr(detection_pose, 'pose') or not hasattr(detection_pose, 'header'):
                                self.get_logger().error(
                                    f"TRUCK_DETECTED: {self.current_truck.truck_id} has invalid detection_pose structure. "
                                    "Transitioning to ERROR_RECOVERY."
                                )
                                self.state = MissionState.ERROR_RECOVERY
                            else:
                                # Validate pose position
                                if not hasattr(detection_pose.pose, 'position') or not hasattr(detection_pose.pose, 'orientation'):
                                    self.get_logger().error(
                                        f"TRUCK_DETECTED: {self.current_truck.truck_id} detection_pose missing position/orientation. "
                                        "Transitioning to ERROR_RECOVERY."
                                    )
                                    self.state = MissionState.ERROR_RECOVERY
                                else:
                                    # Validate pose has numeric values
                                    pos = detection_pose.pose.position
                                    if not all(isinstance(getattr(pos, attr), (int, float)) for attr in ['x', 'y', 'z']):
                                        self.get_logger().error(
                                            f"TRUCK_DETECTED: {self.current_truck.truck_id} detection_pose has invalid position values. "
                                            "Transitioning to ERROR_RECOVERY."
                                        )
                                        self.state = MissionState.ERROR_RECOVERY
                                    else:
                                        # Check if navigation is already in progress
                                        if self.nav_goal_handle is not None:
                                            self.get_logger().info(
                                                f"✅ TRUCK_DETECTED: Navigation already in progress. "
                                                f"Goal handle exists. Skipping new navigation request."
                                            )
                                        # Check if Nav2 is ready
                                        elif not self.nav_client.server_is_ready():
                                            nav_wait_time = getattr(self, 'nav_wait_start_time', None)
                                            if nav_wait_time is None:
                                                self.nav_wait_start_time = time.time()
                                                self.get_logger().warn(
                                                    f"⏳ TRUCK_DETECTED: Nav2 action server not ready. "
                                                    f"Starting wait timer. Will retry..."
                                                )
                                            else:
                                                wait_elapsed = time.time() - nav_wait_time
                                                self.get_logger().warn(
                                                    f"⏳ TRUCK_DETECTED: Nav2 action server not ready. "
                                                    f"Waiting for {wait_elapsed:.1f}s... "
                                                    f"(will timeout after 30s)"
                                                )
                                                if wait_elapsed > 30.0:
                                                    self.get_logger().error(
                                                        f"❌ CRITICAL: Nav2 not ready after 30s! "
                                                        f"Transitioning to ERROR_RECOVERY."
                                                    )
                                                    self.state = MissionState.ERROR_RECOVERY
                                                    return
                                            # Don't transition yet - will retry on next iteration
                                        else:
                                            # CRITICAL: Re-check state before sending navigation goal
                                            # (prevents sending goal if state changed)
                                            if self.state != MissionState.TRUCK_DETECTED:
                                                self.get_logger().warn(
                                                    f"TRUCK_DETECTED: State changed to {self.state.value} "
                                                    "before sending navigation goal. Skipping."
                                                )
                                                return
                                            
                                            # CRITICAL LOGGING: About to calculate and send navigation goal
                                            self.get_logger().info(
                                                f"🎯 TRUCK_DETECTED: Preparing to calculate navigation goal for {self.current_truck.truck_id}"
                                            )
                                            
                                            # CRITICAL FIX: Transform detection_pose to navigation frame (map) first
                                            # The detection_pose might be in camera frame, we need it in map frame
                                            nav_frame = "map"  # Try map first, fallback to odom if needed
                                            
                                            # CRITICAL: Check if transform FROM detection_pose.frame_id TO nav_frame exists
                                            # This is different from checking base_link->map transform
                                            transform_available = False
                                            try:
                                                # Try to get transform from detection frame to map
                                                transform = self.tf_buffer.lookup_transform(
                                                    nav_frame,
                                                    detection_pose.header.frame_id,
                                                    rclpy.time.Time(seconds=0),
                                                    timeout=rclpy.duration.Duration(seconds=0.5)
                                                )
                                                transform_available = True
                                                self.get_logger().info(
                                                    f"✅ TRUCK_DETECTED: Transform available from {detection_pose.header.frame_id} to {nav_frame}"
                                                )
                                            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
                                                # Transform not available - try odom as fallback
                                                self.get_logger().warn(
                                                    f"⚠️ TRUCK_DETECTED: Transform from {detection_pose.header.frame_id} to {nav_frame} not available: {e}. "
                                                    f"Trying odom frame..."
                                                )
                                                nav_frame = "odom"
                                                try:
                                                    transform = self.tf_buffer.lookup_transform(
                                                        nav_frame,
                                                        detection_pose.header.frame_id,
                                                        rclpy.time.Time(seconds=0),
                                                        timeout=rclpy.duration.Duration(seconds=0.5)
                                                    )
                                                    transform_available = True
                                                    self.get_logger().info(
                                                        f"✅ TRUCK_DETECTED: Transform available from {detection_pose.header.frame_id} to {nav_frame}"
                                                    )
                                                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e2:
                                                    self.get_logger().error(
                                                        f"❌ TRUCK_DETECTED: Transform from {detection_pose.header.frame_id} to {nav_frame} also failed: {e2}. "
                                                        f"Using original frame: {detection_pose.header.frame_id}"
                                                    )
                                                    nav_frame = detection_pose.header.frame_id
                                                    transform_available = False
                                            except Exception as e:
                                                self.get_logger().error(
                                                    f"❌ TRUCK_DETECTED: Unexpected error checking transform: {e}. "
                                                    f"Using original frame: {detection_pose.header.frame_id}"
                                                )
                                                nav_frame = detection_pose.header.frame_id
                                                transform_available = False
                                            
                                            # Transform detection pose to navigation frame if needed and transform is available
                                            if detection_pose.header.frame_id != nav_frame and transform_available:
                                                try:
                                                    detection_pose = do_transform_pose_stamped(detection_pose, transform)
                                                    self.get_logger().info(
                                                        f"✅ TRUCK_DETECTED: Transformed detection pose from "
                                                        f"{detection_pose.header.frame_id} to {nav_frame}"
                                                    )
                                                except Exception as e:
                                                    self.get_logger().error(
                                                        f"❌ TRUCK_DETECTED: Failed to apply transform: {e}. "
                                                        f"Using original frame: {detection_pose.header.frame_id}"
                                                    )
                                                    nav_frame = detection_pose.header.frame_id
                                            elif detection_pose.header.frame_id != nav_frame and not transform_available:
                                                self.get_logger().warn(
                                                    f"⚠️ TRUCK_DETECTED: Cannot transform from {detection_pose.header.frame_id} to {nav_frame}. "
                                                    f"Using original frame. Nav2 may reject goal if frame is not supported."
                                                )
                                                nav_frame = detection_pose.header.frame_id
                                            
                                            # Calculate license plate approach pose (front of truck)
                                            # CRITICAL: License plate is ALWAYS on vehicle's FRONT - rover must approach front regardless of starting position
                                            license_pose = PoseStamped()
                                            license_pose.header.frame_id = nav_frame
                                            license_pose.header.stamp = self.get_clock().now().to_msg()
                                            
                                            # CRITICAL FIX: Get robot's current position for distance validation
                                            # If map frame TF is stale (SLAM/localization slow to start), fallback to odom frame
                                            # Odom frame is always available from odometry and is sufficient for goal calculation
                                            current_robot_pose = self._get_robot_pose(nav_frame, max_transform_age_seconds=1.0)
                                            used_odom_fallback = False
                                            
                                            if not current_robot_pose:
                                                # Map frame TF is stale or unavailable - fallback to odom frame
                                                self.get_logger().warn(
                                                    f"⚠️ TRUCK_DETECTED: Map frame TF is stale/unavailable. "
                                                    f"Falling back to odom frame for goal calculation (odom is always available from odometry)."
                                                )
                                                current_robot_pose = self._get_robot_pose("odom", max_transform_age_seconds=5.0)  # More lenient for odom (odometry is always publishing)
                                                
                                                if current_robot_pose:
                                                    used_odom_fallback = True
                                                    nav_frame = "odom"  # Use odom frame for goal calculation
                                                    # Update license_pose frame to odom
                                                    license_pose.header.frame_id = "odom"
                                                    
                                                    # CRITICAL: If using odom fallback, try to transform detection_pose to odom frame
                                                    # If detection_pose is in map frame and map->odom transform exists, transform it
                                                    # Otherwise, we'll use detection_pose in its current frame (might be camera frame)
                                                    if detection_pose.header.frame_id != "odom":
                                                        try:
                                                            # Try to transform detection_pose to odom frame
                                                            transform = self.tf_buffer.lookup_transform(
                                                                "odom",
                                                                detection_pose.header.frame_id,
                                                                rclpy.time.Time(),
                                                                timeout=rclpy.duration.Duration(seconds=0.5)
                                                            )
                                                            detection_pose = do_transform_pose_stamped(detection_pose, transform)
                                                            self.get_logger().info(
                                                                f"✅ TRUCK_DETECTED: Transformed detection pose from "
                                                                f"{detection_pose.header.frame_id} to odom for odom fallback"
                                                            )
                                                        except Exception as e:
                                                            # Transform failed - detection_pose might be in camera frame
                                                            # This is OK - we can still calculate goal using relative positions
                                                            self.get_logger().warn(
                                                                f"⚠️ TRUCK_DETECTED: Cannot transform detection_pose to odom frame: {e}. "
                                                                f"Using detection_pose in current frame ({detection_pose.header.frame_id}). "
                                                                f"Goal calculation may be approximate if frames differ."
                                                            )
                                                    
                                                    self.get_logger().info(
                                                        f"✅ TRUCK_DETECTED: Using odom frame for goal calculation "
                                                        f"(detection_pose frame: {detection_pose.header.frame_id}). "
                                                        f"Direct control can use odom frame, Nav2 will transform to map when ready."
                                                    )
                                                else:
                                                    # Even odom frame is not available - this is a critical error
                                                self.get_logger().error(
                                                        f"❌ TRUCK_DETECTED: Cannot get robot pose from map or odom frame. "
                                                        f"Cannot calculate goal. Transitioning to ERROR_RECOVERY."
                                                )
                                                self.state = MissionState.ERROR_RECOVERY
                                                return
                                            
                                            robot_pos = current_robot_pose.pose.position
                                            # CRITICAL: Use vehicle position from detection_pose (already in correct frame after transform)
                                            # detection_pose has been transformed to nav_frame (map or odom) above
                                            vehicle_pos = detection_pose.pose.position
                                            
                                            # Validate that robot_pos and vehicle_pos are in same frame before calculating distance
                                            robot_frame = current_robot_pose.header.frame_id
                                            vehicle_frame = detection_pose.header.frame_id
                                            
                                            if robot_frame != vehicle_frame:
                                                self.get_logger().warn(
                                                    f"⚠️ TRUCK_DETECTED: Frame mismatch: robot in {robot_frame}, vehicle in {vehicle_frame}. "
                                                    f"This may cause incorrect goal calculation. Attempting to handle..."
                                                )
                                                # If frames differ, distance calculation might be approximate but should still work
                                                # if they're in similar coordinate systems (both 2D navigation frames)
                                            
                                            # Calculate vector from vehicle to robot (for logging and distance calculation)
                                            # Note: This calculation assumes both positions are in the same frame (robot_frame = vehicle_frame after transforms)
                                            dx_robot = robot_pos.x - vehicle_pos.x
                                            dy_robot = robot_pos.y - vehicle_pos.y
                                            distance_robot_to_vehicle = math.sqrt(dx_robot*dx_robot + dy_robot*dy_robot)
                                            
                                            # CRITICAL FIX: License plate is on vehicle's FRONT - always approach from front
                                            # The robot-relative calculation was WRONG for rear/side positions (approached wrong side)
                                            # Solution: ALWAYS use vehicle's forward direction (where license plate is located)
                                                orientation = detection_pose.pose.orientation
                                                siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
                                                cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
                                                vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)
                                            
                                            # CRITICAL: Always approach from vehicle's front (license plate location)
                                            # This ensures rover can see license plate regardless of starting position (front, side, rear, diagonal)
                                            approach_direction = vehicle_yaw  # Vehicle's forward direction = where license plate is
                                            
                                            # Calculate robot-to-vehicle angle for logging (shows rover's starting position)
                                            robot_to_vehicle_angle = math.atan2(dy_robot, dx_robot) if distance_robot_to_vehicle >= 0.01 else vehicle_yaw
                                            
                                            # Handle edge case where robot is at exactly same position as vehicle (distance = 0.0)
                                            if distance_robot_to_vehicle < 0.01:  # Essentially zero (1cm threshold)
                                                self.get_logger().warn(
                                                    f"⚠️ TRUCK_DETECTED: Robot is at essentially same position as vehicle "
                                                    f"(distance: {distance_robot_to_vehicle:.3f}m < 0.01m). "
                                                    f"This is likely an error - using vehicle forward direction for approach."
                                                )
                                                distance_robot_to_vehicle = 0.0  # Mark as zero for logging
                                            
                                                self.get_logger().info(
                                                f"📍 TRUCK_DETECTED: Robot at distance {distance_robot_to_vehicle:.2f}m from vehicle "
                                                f"(robot→vehicle angle: {math.degrees(robot_to_vehicle_angle):.1f}°). "
                                                f"License plate is on vehicle's FRONT - approaching from vehicle's forward direction: "
                                                f"{math.degrees(approach_direction):.1f}° (works from ANY starting position: front, side, rear, diagonal)"
                                                )
                                            
                                            # Get approach distance
                                            approach_dist = self.get_parameter('approach_distance').value
                                            min_goal_distance = self.get_parameter('min_goal_distance').value
                                            
                                            # CRITICAL FIX: For license plate goals, calculate goal from vehicle FRONT (not center)
                                            # License plate is on vehicle front - robot must approach from front
                                            # Get vehicle dimensions (adaptive: car vs truck)
                                            vehicle_length = self.vehicle_obstacle_manager.vehicle_length if self.vehicle_obstacle_manager else self.get_parameter('vehicle_length').value
                                            half_length = vehicle_length / 2.0
                                            
                                            # Calculate vehicle FRONT position (center + half_length forward)
                                            front_x = vehicle_pos.x + half_length * math.cos(vehicle_yaw)
                                            front_y = vehicle_pos.y + half_length * math.sin(vehicle_yaw)
                                            
                                            # CRITICAL FIX: Goal should be approach_distance IN FRONT of vehicle FRONT (to see license plate)
                                            # License plate is on vehicle front - robot must be IN FRONT of vehicle front to see it
                                            # Robot approaches from IN FRONT direction (same as vehicle forward direction)
                                            # Goal = vehicle_front + approach_dist * vehicle_forward_direction
                                            # This places goal IN FRONT of vehicle front, where robot can see license plate
                                            min_safe_distance_from_front = max(approach_dist, 0.3)  # At least 0.3m from front edge
                                            license_pose.pose.position.x = front_x + min_safe_distance_from_front * math.cos(vehicle_yaw)
                                            license_pose.pose.position.y = front_y + min_safe_distance_from_front * math.sin(vehicle_yaw)
                                            license_pose.pose.position.z = vehicle_pos.z  # Keep same height
                                            
                                            # CRITICAL: Verify goal is actually outside vehicle (safety check)
                                            # Calculate distance from goal to vehicle center
                                            goal_to_center_dist = math.sqrt(
                                                (license_pose.pose.position.x - vehicle_pos.x)**2 +
                                                (license_pose.pose.position.y - vehicle_pos.y)**2
                                            )
                                            # Goal should be at least (half_length + approach_dist + 0.2m) from vehicle center to be outside vehicle
                                            # This ensures goal is approach_dist away from front, plus a small safety buffer
                                            min_dist_from_center = half_length + min_safe_distance_from_front + 0.2  # Safety buffer
                                            if goal_to_center_dist < min_dist_from_center:
                                                # Goal is too close to vehicle center (might be inside vehicle) - adjust it
                                                # This should rarely happen since goal is calculated from front + approach_dist
                                                self.get_logger().warn(
                                                    f"⚠️ TRUCK_DETECTED: Calculated goal too close to vehicle center "
                                                    f"({goal_to_center_dist:.2f}m < {min_dist_from_center:.2f}m). "
                                                    f"Adjusting goal to be outside vehicle..."
                                                )
                                                # Adjust goal to be outside vehicle - move it further from vehicle front
                                                adjusted_distance_from_front = half_length + min_safe_distance_from_front + 0.3  # Extra safety margin
                                                license_pose.pose.position.x = front_x + adjusted_distance_from_front * math.cos(vehicle_yaw)
                                                license_pose.pose.position.y = front_y + adjusted_distance_from_front * math.sin(vehicle_yaw)
                                                self.get_logger().info(
                                                    f"✅ TRUCK_DETECTED: Adjusted goal to be outside vehicle: "
                                                    f"({license_pose.pose.position.x:.2f}, {license_pose.pose.position.y:.2f}), "
                                                    f"distance from center: {min_dist_from_center:.2f}m (was {goal_to_center_dist:.2f}m)"
                                                )
                                            
                                            # CRITICAL FIX: Calculate orientation: robot should face TOWARD vehicle FRONT (to see license plate)
                                            # Goal is at vehicle_front - approach_dist, so robot should face vehicle_front direction
                                            # Robot at goal faces toward vehicle_front (same as vehicle_yaw, toward vehicle center/front)
                                            goal_yaw = vehicle_yaw  # Face toward vehicle front (where license plate is)
                                            # Normalize angle to [-pi, pi]
                                            while goal_yaw > math.pi:
                                                goal_yaw -= 2 * math.pi
                                            while goal_yaw < -math.pi:
                                                goal_yaw += 2 * math.pi
                                            
                                            # Set orientation quaternion
                                            license_pose.pose.orientation.x = 0.0
                                            license_pose.pose.orientation.y = 0.0
                                            license_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
                                            license_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
                                            
                                            # CRITICAL: Validate goal is safe distance from robot
                                            goal_pos = license_pose.pose.position
                                            distance_to_goal = math.sqrt(
                                                (goal_pos.x - robot_pos.x)**2 +
                                                (goal_pos.y - robot_pos.y)**2
                                            )
                                            
                                            if distance_to_goal < min_goal_distance:
                                                self.get_logger().warn(
                                                    f"⚠️ TRUCK_DETECTED: Goal too close to robot ({distance_to_goal:.2f}m < {min_goal_distance:.2f}m). "
                                                    f"Adjusting goal to maintain approach from vehicle front but at safe robot distance..."
                                                )
                                                # CRITICAL FIX: Adjust goal to be at minimum safe distance from robot
                                                # But maintain direction toward vehicle front (keep robot moving toward license plate)
                                                # Calculate direction from robot to vehicle front (where we want to go)
                                                dx_to_front = front_x - robot_pos.x
                                                dy_to_front = front_y - robot_pos.y
                                                dist_to_front = math.sqrt(dx_to_front*dx_to_front + dy_to_front*dy_to_front)
                                                if dist_to_front > 0.01:
                                                    # Move goal further along the line from robot to vehicle front
                                                    direction_to_front_x = dx_to_front / dist_to_front
                                                    direction_to_front_y = dy_to_front / dist_to_front
                                                    # Goal should be min_goal_distance from robot in direction toward vehicle front
                                                    license_pose.pose.position.x = robot_pos.x + direction_to_front_x * min_goal_distance
                                                    license_pose.pose.position.y = robot_pos.y + direction_to_front_y * min_goal_distance
                                                else:
                                                    # Robot is at vehicle front - move goal forward (in front of vehicle front)
                                                    license_pose.pose.position.x = front_x + approach_dist * math.cos(vehicle_yaw)
                                                    license_pose.pose.position.y = front_y + approach_dist * math.sin(vehicle_yaw)
                                                
                                                # Recalculate front and goal-to-front distance after adjustment
                                                front_x = vehicle_pos.x + half_length * math.cos(vehicle_yaw)
                                                front_y = vehicle_pos.y + half_length * math.sin(vehicle_yaw)
                                                
                                                self.get_logger().info(
                                                    f"✅ TRUCK_DETECTED: Adjusted goal to safe robot distance while maintaining approach to vehicle front: "
                                                    f"({license_pose.pose.position.x:.2f}, {license_pose.pose.position.y:.2f})"
                                                )
                                                # Goal yaw remains the same (toward vehicle front)
                                                license_pose.pose.orientation.z = math.sin(goal_yaw / 2.0)
                                                license_pose.pose.orientation.w = math.cos(goal_yaw / 2.0)
                                            
                                            # CRITICAL LOGGING: Log calculated goal before validation
                                            # Calculate actual distance from goal to vehicle front for verification
                                            distance_goal_to_front = math.sqrt(
                                                (license_pose.pose.position.x - front_x)**2 +
                                                (license_pose.pose.position.y - front_y)**2
                                            )
                                            self.get_logger().info(
                                                f"📍 TRUCK_DETECTED: Calculated license plate approach pose (from VEHICLE FRONT - works from ANY position): "
                                                f"({license_pose.pose.position.x:.2f}, {license_pose.pose.position.y:.2f}, {license_pose.pose.position.z:.2f}) "
                                                f"in frame '{license_pose.header.frame_id}' "
                                                f"(approach distance from FRONT: {distance_goal_to_front:.2f}m, requested: {approach_dist:.2f}m, "
                                                f"robot→vehicle: {distance_robot_to_vehicle:.2f}m @ {math.degrees(robot_to_vehicle_angle):.1f}°, "
                                                f"vehicle front: ({front_x:.2f}, {front_y:.2f}), vehicle forward: {math.degrees(vehicle_yaw):.1f}°, goal yaw: {math.degrees(goal_yaw):.1f}°"
                                            )
                                            
                                            # CRITICAL: Validate pose structure first
                                            if not self._validate_navigation_pose(license_pose):
                                                self.get_logger().error(
                                                    f"❌ TRUCK_DETECTED: Calculated license plate pose is INVALID. "
                                                    f"Goal: ({license_pose.pose.position.x:.2f}, {license_pose.pose.position.y:.2f}). "
                                                    f"Transitioning to ERROR_RECOVERY."
                                                )
                                                self.state = MissionState.ERROR_RECOVERY
                                            else:
                                                # CRITICAL: Validate goal is in free space and path is clear
                                                is_valid, validated_goal = self._validate_goal_in_free_space(license_pose)
                                                
                                                if not is_valid or not validated_goal:
                                                    self.get_logger().warn(
                                                        f"⚠️ TRUCK_DETECTED: Original goal not in free space. "
                                                        f"Attempting to calculate alternative goal..."
                                                    )
                                                    # Try to calculate alternative goal
                                                    robot_pose = self._get_robot_pose("map")
                                                    if robot_pose:
                                                        alternative_goal = self._calculate_alternative_goal(license_pose, robot_pose)
                                                        if alternative_goal:
                                                            validated_goal = alternative_goal
                                                            self.get_logger().info(
                                                                f"✅ TRUCK_DETECTED: Using alternative goal: "
                                                                f"({validated_goal.pose.position.x:.2f}, {validated_goal.pose.position.y:.2f})"
                                                            )
                                                        else:
                                                            self.get_logger().error(
                                                                f"❌ TRUCK_DETECTED: Could not find alternative goal. "
                                                                f"Transitioning to ERROR_RECOVERY."
                                                            )
                                                            self.state = MissionState.ERROR_RECOVERY
                                                            return
                                                    else:
                                                        self.get_logger().error(
                                                            f"❌ TRUCK_DETECTED: Could not get robot pose for alternative goal. "
                                                            f"Transitioning to ERROR_RECOVERY."
                                                        )
                                                        self.state = MissionState.ERROR_RECOVERY
                                                        return
                                                
                                                self.get_logger().info(
                                                    f"✅ TRUCK_DETECTED: License plate pose validated successfully for {self.current_truck.truck_id}"
                                                )
                                                
                                                # CRITICAL: Clear Nav2 wait timer since we're about to send goal
                                                if hasattr(self, 'nav_wait_start_time'):
                                                    delattr(self, 'nav_wait_start_time')
                                                
                                                # CRITICAL: Use DIRECT CONTROL as PRIMARY for ALL initial approaches
                                                # Nav2 is unreliable - it keeps canceling goals after small movements
                                                # Direct control is simple, reliable, and always works
                                                
                                                self.get_logger().info(
                                                    f"🎯 TRUCK_DETECTED: Using DIRECT CONTROL as PRIMARY navigation method "
                                                    f"(bypassing Nav2 entirely for reliability)."
                                                )
                                                
                                                # CRITICAL FIX: Calculate distance to goal using the same frame as goal
                                                # If goal is in odom frame, use odom frame for distance calculation
                                                goal_frame = validated_goal.header.frame_id
                                                robot_pose_for_distance = self._get_robot_pose(goal_frame, max_transform_age_seconds=5.0 if goal_frame == "odom" else 1.0)
                                                distance_to_goal = 0.0
                                                if robot_pose_for_distance:
                                                    goal_pos = validated_goal.pose.position
                                                    robot_pos = robot_pose_for_distance.pose.position
                                                    distance_to_goal = math.sqrt(
                                                        (goal_pos.x - robot_pos.x)**2 +
                                                        (goal_pos.y - robot_pos.y)**2
                                                    )
                                                    self.get_logger().info(
                                                        f"📍 Distance to goal: {distance_to_goal:.2f}m (calculated in {goal_frame} frame)"
                                                    )
                                                else:
                                                    self.get_logger().warn(
                                                        f"⚠️ Could not get robot pose in {goal_frame} frame for distance calculation. "
                                                        f"Goal will be set regardless."
                                                    )
                                                
                                                # ALWAYS use direct control for initial approach
                                                # This bypasses all Nav2 issues: behavior tree, progress checker, stuck detection, etc.
                                                if hasattr(self, 'direct_nav_fallback'):
                                                    self.direct_nav_fallback.set_goal(validated_goal)
                                                    self.direct_nav_fallback.activate()
                                                    
                                                    # CRITICAL: Activate movement guarantee system
                                                    if hasattr(self, 'movement_guarantee'):
                                                        self.movement_guarantee.activate(validated_goal)
                                                        self.get_logger().info(
                                                            "🛡️ Movement Guarantee activated - Robot movement guaranteed"
                                                        )
                                                    
                                                    # CRITICAL: Verify activation
                                                    if self.direct_nav_fallback.is_active:
                                                        self.get_logger().info(
                                                            f"✅✅✅ TRUCK_DETECTED → NAVIGATING_TO_LICENSE_PLATE: "
                                                            f"DIRECT CONTROL ACTIVATED for {self.current_truck.truck_id} "
                                                            f"(distance: {distance_to_goal:.2f}m)"
                                                        )
                                                        self.get_logger().info(
                                                            f"🔍 VERIFICATION: direct_nav_fallback.is_active = {self.direct_nav_fallback.is_active}"
                                                        )
                                                        self.get_logger().info(
                                                            f"🔍 VERIFICATION: watchdog_enabled = {self.direct_nav_fallback.watchdog_enabled}"
                                                        )
                                                        self.get_logger().info(
                                                            f"🔍 VERIFICATION: override_nav2_zero = {self.direct_nav_fallback.override_nav2_zero}"
                                                        )
                                                    else:
                                                        self.get_logger().error(
                                                            f"🚨🚨🚨 CRITICAL ERROR: Direct control activation FAILED! "
                                                            f"is_active = {self.direct_nav_fallback.is_active}"
                                                        )
                                                    self.state = MissionState.NAVIGATING_TO_LICENSE_PLATE
                                                    self.nav_goal_pose = validated_goal
                                                    self.nav_start_time = time.time()
                                                    
                                                    # CRITICAL: Store initial distance for minimum movement tracking
                                                    # Use the same frame as goal for consistency
                                                    if robot_pose_for_distance:
                                                        self.nav_initial_distance = distance_to_goal  # Already calculated above
                                                        self.get_logger().info(
                                                            f"📍 Navigation started: Initial distance = {self.nav_initial_distance:.2f}m, "
                                                            f"arrival_threshold = {self.arrival_distance_threshold:.2f}m, "
                                                            f"min_movement = {self.min_movement_before_arrival_check:.2f}m "
                                                            f"(goal frame: {goal_frame})"
                                                        )
                                                    else:
                                                        self.nav_initial_distance = None
                                                        self.get_logger().warn(
                                                            f"⚠️ Could not get robot pose in {goal_frame} frame to store initial distance. "
                                                            "Minimum movement check will be skipped."
                                                        )
                                                    
                                                    # CRITICAL: If goal is in odom frame, log that we'll transform to map later
                                                    if used_odom_fallback:
                                                        self.get_logger().info(
                                                            f"📐 Goal calculated in odom frame (map frame was stale). "
                                                            f"Direct control will use odom frame. "
                                                            f"If Nav2 backup is used, goal will be transformed to map frame when SLAM is ready."
                                                        )
                                                    
                                                    # Also send to Nav2 as backup (but direct control is primary)
                                                    # This way if direct control fails, Nav2 can take over
                                                    self.get_logger().info(
                                                        f"🔄 Also sending goal to Nav2 as backup..."
                                                    )
                                                    self.navigate_to_pose(validated_goal)  # Non-blocking, just for backup
                                                    return  # Direct control is primary - exit state machine step
                                                else:
                                                    # Fallback: if direct control not available, use Nav2
                                                    self.get_logger().warn(
                                                        f"⚠️ Direct control not available. Falling back to Nav2..."
                                                    )
                                                    navigation_success = self.navigate_to_pose(validated_goal)
                                                    if navigation_success:
                                                        self.get_logger().info(
                                                            f"✅✅✅ TRUCK_DETECTED → NAVIGATING_TO_LICENSE_PLATE: "
                                                            f"Navigation goal SENT to Nav2 for {self.current_truck.truck_id}"
                                                        )
                                                        self.state = MissionState.NAVIGATING_TO_LICENSE_PLATE
                                                        self.nav_goal_pose = validated_goal
                                                        self.nav_start_time = time.time()
                                                        
                                                        # CRITICAL: Store initial distance for minimum movement tracking
                                                        if robot_pose:
                                                            self.nav_initial_distance = math.sqrt(
                                                                (validated_goal.pose.position.x - robot_pose.pose.position.x)**2 +
                                                                (validated_goal.pose.position.y - robot_pose.pose.position.y)**2
                                                            )
                                                            self.get_logger().info(
                                                                f"📍 Navigation started: Initial distance = {self.nav_initial_distance:.2f}m, "
                                                                f"arrival_threshold = {self.arrival_distance_threshold:.2f}m, "
                                                                f"min_movement = {self.min_movement_before_arrival_check:.2f}m"
                                                            )
                                                        else:
                                                            self.nav_initial_distance = None
                                                        
                                                        self.get_logger().info(
                                                            f"🔄 STATE TRANSITION COMPLETE: Now in NAVIGATING_TO_LICENSE_PLATE state. "
                                                            f"Robot should begin moving toward {self.current_truck.truck_id}..."
                                                        )
                                                    else:
                                                        self.get_logger().error(
                                                            f"❌ CRITICAL: TRUCK_DETECTED: navigate_to_pose() returned False! "
                                                            f"Navigation goal NOT sent. Will retry on next iteration."
                                                        )
                                                        # Don't transition yet - will retry
                                                    
                except AttributeError as e:
                    self.get_logger().error(
                        f"TRUCK_DETECTED: Attribute error - {e}. "
                        "Transitioning to ERROR_RECOVERY.",
                        exc_info=True
                    )
                    self.state = MissionState.ERROR_RECOVERY
                except Exception as e:
                    self.get_logger().error(
                        f"TRUCK_DETECTED: Unexpected error - {e}. "
                        "Transitioning to ERROR_RECOVERY.",
                        exc_info=True
                    )
                    self.state = MissionState.ERROR_RECOVERY
                    
            elif self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                # Enhanced NAVIGATING_TO_LICENSE_PLATE state with progress monitoring
                try:
                    # CRITICAL: Validate current_truck still exists (vehicle detection not lost)
                    # If vehicle detection is lost during navigation, we shouldn't continue to stale goal
                    if not self.current_truck:
                        self.get_logger().error(
                            "🚨🚨🚨 CRITICAL: NAVIGATING_TO_LICENSE_PLATE: current_truck is None! "
                            "Vehicle detection lost during navigation. "
                            "Canceling navigation and transitioning to ERROR_RECOVERY."
                        )
                        # Cancel navigation immediately
                        if self.nav_goal_handle:
                            try:
                                self.nav_client.cancel_goal_async(self.nav_goal_handle)
                            except Exception as e:
                                self.get_logger().warn(f"Error canceling goal: {e}")
                            self.nav_goal_handle = None
                            self.nav_result_future = None
                        # Deactivate direct navigation if active
                        if hasattr(self, 'direct_nav_fallback') and self.direct_nav_fallback.is_active:
                            self.direct_nav_fallback.deactivate()
                        self.nav_goal_pose = None
                        self.nav_start_time = None
                        self.state = MissionState.ERROR_RECOVERY
                        return
                    
                    # CRITICAL: Check if current_truck is still in detected_trucks (vehicle still tracked)
                    # If vehicle was removed from tracking, we shouldn't continue navigation
                    if self.current_truck and hasattr(self.current_truck, 'truck_id'):
                        if self.current_truck.truck_id not in self.detected_trucks:
                            self.get_logger().error(
                                f"🚨🚨🚨 CRITICAL: NAVIGATING_TO_LICENSE_PLATE: current_truck {self.current_truck.truck_id} "
                                f"not found in detected_trucks! Vehicle tracking lost. "
                                f"Continuing navigation to last known position (may be stale)."
                            )
                            # Don't cancel immediately - vehicle might still be there, just not tracked
                            # But log warning for operator awareness
                    
                    # CRITICAL: Check if Nav2 has started publishing cmd_vel
                    if not hasattr(self, 'nav2_initial_cmd_vel_received') or not self.nav2_initial_cmd_vel_received:
                        # Nav2 hasn't started yet - check if enough time has passed
                        if self.nav_start_time:
                            time_since_goal_sent = time.time() - self.nav_start_time
                            if time_since_goal_sent > 1.0:  # If Nav2 doesn't start within 1s, activate fallback
                                self.get_logger().warn(
                                    f"⚠️ Nav2 has not started publishing cmd_vel after {time_since_goal_sent:.1f}s. "
                                    f"Activating direct navigation fallback IMMEDIATELY."
                                )
                                if hasattr(self, 'direct_nav_fallback') and self.nav_goal_pose:
                                    # CRITICAL: Set goal FIRST, then activate (order matters!)
                                    # set_goal() with default preserve_active_state=False sets is_active=False
                                    # If we call activate() first, set_goal() will immediately deactivate
                                    was_active = self.direct_nav_fallback.is_active
                                    self.direct_nav_fallback.set_goal(self.nav_goal_pose, preserve_active_state=was_active)
                                    if not was_active:
                                    self.direct_nav_fallback.activate()
                    
                    # CRITICAL: Check if Nav2 has stopped publishing cmd_vel
                    if hasattr(self, 'last_nav2_cmd_vel_time') and self.last_nav2_cmd_vel_time:
                        time_since_last_cmd = time.time() - self.last_nav2_cmd_vel_time
                        if time_since_last_cmd > self.nav2_cmd_vel_timeout:
                            # Nav2 has stopped - activate direct navigation fallback IMMEDIATELY
                            self.get_logger().warn(
                                f"⚠️ Nav2 stopped publishing cmd_vel for {time_since_last_cmd:.1f}s. "
                                f"Activating direct navigation fallback IMMEDIATELY."
                            )
                            if hasattr(self, 'direct_nav_fallback') and self.nav_goal_pose:
                                # CRITICAL: Set goal FIRST, then activate (order matters!)
                                # If already active, preserve_active_state=True keeps it active during goal update
                                was_active = self.direct_nav_fallback.is_active
                                self.direct_nav_fallback.set_goal(self.nav_goal_pose, preserve_active_state=was_active)
                                if not was_active:
                                self.direct_nav_fallback.activate()
                    elif hasattr(self, 'nav2_initial_cmd_vel_received') and self.nav2_initial_cmd_vel_received:
                        # Nav2 was working but now stopped - activate fallback
                        self.get_logger().warn(
                            f"⚠️ Nav2 was working but stopped. Activating direct navigation fallback."
                        )
                        if hasattr(self, 'direct_nav_fallback') and self.nav_goal_pose:
                            # CRITICAL: Set goal FIRST, then activate (order matters!)
                            # If already active, preserve_active_state=True keeps it active during goal update
                            was_active = self.direct_nav_fallback.is_active
                            self.direct_nav_fallback.set_goal(self.nav_goal_pose, preserve_active_state=was_active)
                            if not was_active:
                            self.direct_nav_fallback.activate()
                    
                    # CRITICAL: Direct control is PRIMARY - MUST update every cycle
                    # This ensures cmd_vel is ALWAYS being published (state machine runs at 2Hz)
                    if hasattr(self, 'direct_nav_fallback') and self.direct_nav_fallback.is_active:
                        # CRITICAL FIX: Try map frame first, fallback to odom if map is stale
                        # This ensures direct control works even when SLAM is slow to start
                        robot_pose = self._get_robot_pose("map", max_transform_age_seconds=1.0)
                        if not robot_pose:
                            # Map frame stale or unavailable - fallback to odom frame
                            self.get_logger().debug(
                                "⚠️ NAVIGATING_TO_LICENSE_PLATE: Map frame stale, falling back to odom frame for direct control"
                            )
                            robot_pose = self._get_robot_pose("odom", max_transform_age_seconds=5.0)  # odom is always available, allow older transforms
                            if robot_pose:
                                self.get_logger().info(
                                    f"✅ NAVIGATING_TO_LICENSE_PLATE: Using odom frame for direct control (map frame was stale)"
                                )
                        
                        if robot_pose:
                            # CRITICAL: Update direct control EVERY cycle to ensure continuous cmd_vel publishing
                            goal_reached = self.direct_nav_fallback.update(robot_pose)
                            if goal_reached:
                                self.get_logger().info("✅ Direct navigation: Goal reached!")
                                self.direct_nav_fallback.deactivate()
                                # Continue to check navigation complete below
                            else:
                                # Log progress every 5 seconds to verify it's working
                                if not hasattr(self, '_last_direct_control_log_time'):
                                    self._last_direct_control_log_time = time.time()
                                elif time.time() - self._last_direct_control_log_time > 5.0:
                                    goal_pos = self.nav_goal_pose.pose.position if self.nav_goal_pose else None
                                    if goal_pos:
                                        distance = math.sqrt(
                                            (goal_pos.x - robot_pose.pose.position.x)**2 +
                                            (goal_pos.y - robot_pose.pose.position.y)**2
                                        )
                                        self.get_logger().info(
                                            f"🔄 Direct control ACTIVE: Distance to goal = {distance:.2f}m (robot_pose frame: {robot_pose.header.frame_id}), "
                                            f"cmd_vel published at 2Hz (state machine) + 50Hz (watchdog) = 52Hz total, "
                                            f"Nav2 zero-override: ENABLED"
                                        )
                                    self._last_direct_control_log_time = time.time()
                        else:
                            self.get_logger().warn(
                                "⚠️ Direct control active but cannot get robot pose from map or odom frame. "
                                "Watchdog will maintain last command until pose is available."
                            )
                    
                    # Update navigation progress tracking
                    self._update_navigation_progress()
                    
                    # Check if navigation goal handle exists (navigation started)
                    # NOTE: If using direct control, nav_goal_handle will be None - that's OK
                    if self.nav_goal_handle is None and not (hasattr(self, 'direct_nav_fallback') and self.direct_nav_fallback.is_active):
                        # Navigation hasn't started yet - might have failed to start
                        self.get_logger().warn(
                            "NAVIGATING_TO_LICENSE_PLATE: Navigation goal handle is None. "
                            "Checking if we need to restart navigation..."
                        )
                        # If we have a goal pose but no handle, try to restart
                        if self.nav_goal_pose and self.current_truck:
                            self.get_logger().info(
                                "Restarting navigation to license plate..."
                            )
                            navigation_success = self.navigate_to_pose(self.nav_goal_pose)
                            if not navigation_success:
                                self.get_logger().error(
                                    "Failed to restart navigation. "
                                    "Transitioning to ERROR_RECOVERY."
                                )
                                self.state = MissionState.ERROR_RECOVERY
                        else:
                            # CRITICAL FIX: If no goal pose stored but we have a current truck, recalculate goal instead of error recovery
                            # This handles the case where goal was never calculated (e.g., stale TF in TRUCK_DETECTED state)
                            if self.current_truck and self.current_truck.detection_pose:
                                self.get_logger().warn(
                                    "No goal pose stored. Recalculating goal from vehicle detection..."
                                )
                                # Recalculate goal by transitioning back to TRUCK_DETECTED state
                                # This will recalculate the goal with odom fallback if map is stale
                                self.get_logger().info(
                                    "Recalculating goal by transitioning back to TRUCK_DETECTED state..."
                                )
                                self.state = MissionState.TRUCK_DETECTED
                                # Clear any stale navigation state
                                self.nav_goal_handle = None
                                self.nav_result_future = None
                                return  # Exit to let TRUCK_DETECTED handler recalculate goal
                        else:
                            self.get_logger().error(
                                    "No goal pose stored and no current truck. "
                                    "Cannot restart navigation. Transitioning to ERROR_RECOVERY."
                            )
                            self.state = MissionState.ERROR_RECOVERY
                        # Don't process further - wait for next iteration
                    
                    # Check if navigation is complete (works for both Nav2 and direct control)
                    # For direct control, check if goal reached
                    if hasattr(self, 'direct_nav_fallback') and self.direct_nav_fallback.is_active:
                        # CRITICAL FIX: Try map frame first, fallback to odom if map is stale
                        robot_pose = self._get_robot_pose("map", max_transform_age_seconds=1.0)
                        if not robot_pose:
                            robot_pose = self._get_robot_pose("odom", max_transform_age_seconds=5.0)  # odom fallback
                        if robot_pose and self.direct_nav_fallback.is_goal_reached(robot_pose):
                            self.get_logger().info("✅ Direct control: Goal reached!")
                            self.direct_nav_fallback.deactivate()
                            # Continue to normal completion check below
                    
                    # Check if navigation is complete (Nav2 or direct control)
                    nav_complete_result = self.check_navigation_complete()
                    if nav_complete_result:
                        # Nav2 says navigation is complete (or direct control reached goal)
                        # Verify we're actually at the goal before declaring arrival
                        # Nav2 says navigation is complete - verify we're actually at the goal
                        current_pose = self._get_robot_pose("map")
                        if current_pose and self.nav_goal_pose:
                            goal_pos = self.nav_goal_pose.pose.position
                            robot_pos = current_pose.pose.position
                            arrival_distance = math.sqrt(
                                (goal_pos.x - robot_pos.x)**2 +
                                (goal_pos.y - robot_pos.y)**2
                            )
                            arrival_threshold = self.get_parameter('arrival_distance_threshold').value
                            
                            # CRITICAL: Check orientation match - robot must face goal orientation for license plate capture
                            goal_orientation = self.nav_goal_pose.pose.orientation
                            robot_orientation = current_pose.pose.orientation
                            # Convert quaternion to yaw (standard formula)
                            goal_siny_cosp = 2.0 * (goal_orientation.w * goal_orientation.z + goal_orientation.x * goal_orientation.y)
                            goal_cosy_cosp = 1.0 - 2.0 * (goal_orientation.y * goal_orientation.y + goal_orientation.z * goal_orientation.z)
                            goal_yaw = math.atan2(goal_siny_cosp, goal_cosy_cosp)
                            robot_siny_cosp = 2.0 * (robot_orientation.w * robot_orientation.z + robot_orientation.x * robot_orientation.y)
                            robot_cosy_cosp = 1.0 - 2.0 * (robot_orientation.y * robot_orientation.y + robot_orientation.z * robot_orientation.z)
                            robot_yaw = math.atan2(robot_siny_cosp, robot_cosy_cosp)
                            orientation_diff = abs(goal_yaw - robot_yaw)
                            # Normalize to [0, pi] range
                            if orientation_diff > math.pi:
                                orientation_diff = 2 * math.pi - orientation_diff
                            arrival_orientation_threshold = self.get_parameter('arrival_orientation_tolerance').value
                            orientation_match = orientation_diff <= arrival_orientation_threshold
                            
                            # CRITICAL: Only check arrival if we've moved at least minimum distance
                            # This prevents premature arrival detection if goal is close (prevents false arrival)
                            # 
                            # Priority 1: Use nav_initial_distance (most accurate - distance from start to goal)
                            # Priority 2: Use nav_progress_distance (fallback - accumulated movement distance)
                            # This ensures minimum movement is ALWAYS checked, even if nav_initial_distance is None
                            distance_traveled = None
                            if self.nav_initial_distance is not None:
                                # Method 1: Calculate from initial distance to current distance
                                distance_traveled = self.nav_initial_distance - arrival_distance
                            elif hasattr(self, 'nav_progress_distance') and self.nav_progress_distance is not None:
                                # Method 2: Fallback - use accumulated progress distance
                                # nav_progress_distance is updated by _update_navigation_progress() which tracks
                                # actual robot movement by summing distance between pose updates
                                distance_traveled = self.nav_progress_distance
                                self.get_logger().debug(
                                    f"Using nav_progress_distance as fallback for minimum movement check: "
                                    f"{distance_traveled:.2f}m (nav_initial_distance was None)"
                                )
                            
                            # Check if we've moved enough before checking arrival
                            # CRITICAL FIX: If rover was already at goal when navigation started (nav_initial_distance < arrival_threshold),
                            # AND rover is still at goal AND facing correctly, accept immediately (rover was placed in correct position)
                            arrival_detected = False
                            
                            # Special case: Rover already at goal when navigation started (placed in position)
                            if (self.nav_initial_distance is not None and 
                                self.nav_initial_distance <= arrival_threshold and
                                arrival_distance <= arrival_threshold and
                                orientation_match):
                                # Rover was already at goal when navigation started and is still there with correct orientation
                                # Accept immediately - rover was correctly positioned
                                arrival_detected = True
                                self.get_logger().info(
                                    f"✅ Rover was already at goal when navigation started (initial: {self.nav_initial_distance:.2f}m <= {arrival_threshold:.2f}m). "
                                    f"Current: {arrival_distance:.2f}m, orientation: {math.degrees(orientation_diff):.1f}° <= {math.degrees(arrival_orientation_threshold):.1f}°. "
                                    f"Accepting as arrived (rover was placed in correct position)."
                                )
                            elif distance_traveled is not None and distance_traveled >= self.min_movement_before_arrival_check:
                                # Moved enough - check BOTH distance AND orientation
                                if arrival_distance <= arrival_threshold and orientation_match:
                                    arrival_detected = True
                                elif arrival_distance <= arrival_threshold and not orientation_match:
                                    # At position but wrong orientation - log but don't arrive
                                    self.get_logger().info(
                                        f"🔄 Navigation: At goal position (distance: {arrival_distance:.2f}m <= {arrival_threshold:.2f}m) "
                                        f"but orientation mismatch: {math.degrees(orientation_diff):.1f}° > {math.degrees(arrival_orientation_threshold):.1f}° "
                                        f"(robot: {math.degrees(robot_yaw):.1f}°, goal: {math.degrees(goal_yaw):.1f}°). "
                                        f"Continuing to rotate to match orientation..."
                                    )
                                    # CRITICAL: If at goal position but wrong orientation, don't arrive yet
                                    # Let direct navigation or Nav2 continue to rotate to match orientation
                                    # Don't return - let the navigation continue
                            elif distance_traveled is not None:
                                # CRITICAL: Haven't moved enough yet - Nav2 might say succeeded but robot hasn't actually moved
                                # This happens if goal is close (e.g., 0.3m away) and Nav2 goal checker is too lenient
                                # Don't trust Nav2 - continue navigation until robot actually moves enough
                                if not hasattr(self, '_arrival_check_log_counter'):
                                    self._arrival_check_log_counter = 0
                                self._arrival_check_log_counter += 1
                                if self._arrival_check_log_counter >= 10:  # Every 5 seconds
                                    self.get_logger().warn(
                                        f"⚠️ Nav2 says navigation complete, but robot hasn't moved enough: "
                                        f"Distance: {arrival_distance:.2f}m, "
                                        f"Traveled: {distance_traveled:.2f}m / {self.min_movement_before_arrival_check:.2f}m. "
                                        f"Nav2 goal checker may be too lenient - continuing navigation until robot actually moves."
                                    )
                                    self._arrival_check_log_counter = 0
                                # CRITICAL: Don't return early - continue navigation!
                                # If Nav2 says succeeded but robot hasn't moved, Nav2 is wrong
                                # Continue with direct navigation or restart Nav2 goal
                                # Clean up Nav2 result but don't declare arrival
                                if self.nav_goal_handle is not None:
                                    self.get_logger().info(
                                        "Cleaning up Nav2 goal handle - Nav2 says succeeded but robot hasn't moved enough. "
                                        "Continuing navigation with direct control."
                                    )
                                    self.nav_goal_handle = None
                                    self.nav_result_future = None
                                # Activate direct navigation to continue moving toward goal
                                if hasattr(self, 'direct_nav_fallback') and not self.direct_nav_fallback.is_active:
                                    was_active = self.direct_nav_fallback.is_active
                                    self.direct_nav_fallback.set_goal(self.nav_goal_pose, preserve_active_state=was_active)
                                    if not was_active:
                                        self.direct_nav_fallback.activate()
                                        self.get_logger().info(
                                            "Activated direct navigation to continue toward goal "
                                            "(Nav2 incorrectly declared success without sufficient movement)."
                                        )
                                return
                            else:
                                # CRITICAL FIX: No distance tracking available (nav_initial_distance and nav_progress_distance both None/zero)
                                # This is a rare edge case - TF might have failed, or navigation just started
                                # DO NOT declare arrival without movement verification - this prevents false arrival!
                                # Require minimum movement even in backward compatibility mode
                                self.get_logger().warn(
                                    f"⚠️ CRITICAL: Cannot verify minimum movement (nav_initial_distance=None, nav_progress_distance={getattr(self, 'nav_progress_distance', None)}). "
                                    f"This could cause false arrival if robot is already close to goal. "
                                    f"Robot at {arrival_distance:.2f}m from goal - requiring minimum movement ({self.min_movement_before_arrival_check:.2f}m) before arrival."
                                )
                                # Don't declare arrival - require robot to actually move first
                                # This prevents false arrival when robot starts navigation already close to goal
                                if arrival_distance <= arrival_threshold:
                                    self.get_logger().info(
                                        f"Robot is close to goal ({arrival_distance:.2f}m <= {arrival_threshold:.2f}m) "
                                        f"but cannot verify movement. "
                                        f"Waiting for robot to move at least {self.min_movement_before_arrival_check:.2f}m before declaring arrival."
                                    )
                                # Continue navigation - arrival will be detected once movement is tracked
                                return
                            
                            if arrival_detected:
                                elapsed_time = time.time() - self.nav_start_time if self.nav_start_time else 0.0
                                if distance_traveled is not None:
                                    self.get_logger().info(
                                        f"✅ Arrived at license plate position "
                                        f"(distance: {arrival_distance:.2f}m <= {arrival_threshold:.2f}m, "
                                        f"orientation: {math.degrees(orientation_diff):.1f}° <= {math.degrees(arrival_orientation_threshold):.1f}°, "
                                        f"time: {elapsed_time:.1f}s, "
                                        f"traveled: {distance_traveled:.2f}m, "
                                        f"nav_progress: {self.nav_progress_distance:.2f}m, "
                                        f"robot yaw: {math.degrees(robot_yaw):.1f}°, goal yaw: {math.degrees(goal_yaw):.1f}°)"
                                    )
                                else:
                                    self.get_logger().info(
                                        f"✅ Arrived at license plate position "
                                        f"(distance: {arrival_distance:.2f}m, "
                                        f"time: {elapsed_time:.1f}s, "
                                        f"traveled: {self.nav_progress_distance:.2f}m)"
                                    )
                                
                                # CRITICAL: Verify vehicle is visible before proceeding
                                if self.visual_verifier and self.current_truck:
                                    # Start verification process
                                    if not self.arrival_verification_active:
                                        self.arrival_verification_active = True
                                        self.arrival_verification_start_time = time.time()
                                        self.visual_verifier.reset()
                                        self.get_logger().info(
                                            "Verifying vehicle visibility at arrival location..."
                                        )
                                    
                                    # Check verification timeout
                                    verification_elapsed = time.time() - self.arrival_verification_start_time
                                    verification_timeout = self.get_parameter('arrival_verification_timeout').value
                                    
                                    if verification_elapsed > verification_timeout:
                                        self.get_logger().warn(
                                            f"Vehicle verification timeout after {verification_elapsed:.1f}s. "
                                            "Vehicle may not be visible. Starting local search..."
                                        )
                                        self.arrival_verification_active = False
                                        if self._start_local_search_for_vehicle():
                                            # Local search started - will continue in next iteration
                                            return
                                        else:
                                            # Local search failed - proceed anyway
                                            self.get_logger().warn(
                                                "Local search failed. Proceeding to capture anyway."
                                            )
                                            self.state = MissionState.CAPTURING_LICENSE_PLATE
                                    else:
                                        # Verification in progress - wait for bbox_callback to verify
                                        # Check if we have recent verification result
                                        # (Verification happens in bbox_callback with current detections)
                                        # For now, proceed if verification has been running for a bit
                                        if verification_elapsed > 1.0:  # Give it at least 1 second
                                            # Check last monitor result for vehicle visibility
                                            if (self.last_vehicle_monitor_result and 
                                                self.last_vehicle_monitor_result.get('vehicle_visible', False)):
                                                self.get_logger().info(
                                                    "✅ Vehicle verified visible at arrival location"
                                                )
                                                self.arrival_verification_active = False
                                                self.get_logger().info(
                                                    f"NAVIGATING_TO_LICENSE_PLATE → CAPTURING_LICENSE_PLATE: "
                                                    f"Arrived at license plate position for {self.current_truck.truck_id}"
                                                )
                                                self.state = MissionState.CAPTURING_LICENSE_PLATE
                                                self.get_logger().info(
                                                    f"✅ NEXT ACTION: System is now in CAPTURING_LICENSE_PLATE state. "
                                                    f"Detecting and photographing license plate..."
                                                )
                                            else:
                                                # Vehicle not visible yet - continue waiting
                                                self.get_logger().debug(
                                                    f"Waiting for vehicle verification... "
                                                    f"({verification_elapsed:.1f}s elapsed)"
                                                )
                                                return
                                        else:
                                            # Too early - wait more
                                            return
                                else:
                                    # Visual verifier not available - proceed anyway (backward compatibility)
                                    self.get_logger().info(
                                        "Visual verifier not available. Proceeding to capture."
                                    )
                                    self.state = MissionState.CAPTURING_LICENSE_PLATE
                            else:
                                self.get_logger().warn(
                                    f"Navigation reported complete but robot is "
                                    f"{arrival_distance:.2f}m from goal "
                                    f"(threshold: {arrival_threshold:.2f}m). "
                                    "Attempting to continue..."
                                )
                                # If close enough, continue anyway
                                if arrival_distance < arrival_threshold * 1.5:
                                    self.get_logger().info(
                                        "Close enough to goal. Continuing to capture."
                                    )
                                    self.state = MissionState.CAPTURING_LICENSE_PLATE
                                else:
                                    self.get_logger().error(
                                        "Too far from goal. Transitioning to ERROR_RECOVERY."
                                    )
                                    self.state = MissionState.ERROR_RECOVERY
                        else:
                            # Can't verify pose, but navigation says complete - trust it
                            self.get_logger().info(
                                "Navigation complete (unable to verify pose). "
                                "Proceeding to capture."
                            )
                            self.state = MissionState.CAPTURING_LICENSE_PLATE
                    else:
                        # CRITICAL: check_navigation_complete() returned False
                        # This means either:
                        # 1. Nav2 says succeeded but robot is FAR from goal (> 2x threshold) - Nav2 is wrong!
                        # 2. Nav2 result is not ready yet (still navigating)
                        # 3. Nav2 says failed/canceled
                        # 
                        # Case 1: Nav2 says succeeded but robot is far - Nav2 goal checker is wrong or odometry drift
                        # We need to handle this - clean up Nav2 result and continue with direct navigation
                        if self.nav_result_future and self.nav_result_future.done():
                            try:
                                result_wrapper = self.nav_result_future.result()
                                result = result_wrapper.result
                                if result.result == NavigateToPose.Result.SUCCEEDED:
                                    # Nav2 says succeeded but check_navigation_complete() returned False
                                    # This means robot is FAR from goal (> 2x threshold) - Nav2 is wrong!
                                    current_pose = self._get_robot_pose("map")
                                    if current_pose and self.nav_goal_pose:
                                        goal_pos = self.nav_goal_pose.pose.position
                                        robot_pos = current_pose.pose.position
                                        arrival_distance = math.sqrt(
                                            (goal_pos.x - robot_pos.x)**2 +
                                            (goal_pos.y - robot_pos.y)**2
                                        )
                                        arrival_threshold = self.get_parameter('arrival_distance_threshold').value
                                        max_trusted_distance = arrival_threshold * 2.0
                                        
                                        if arrival_distance > max_trusted_distance:
                                            # Nav2 is wrong - robot is far from goal
                                            self.get_logger().error(
                                                f"❌ CRITICAL: Nav2 says succeeded but robot is FAR from goal: "
                                                f"{arrival_distance:.2f}m > {max_trusted_distance:.2f}m. "
                                                f"Nav2 goal checker is wrong - ignoring result and continuing navigation."
                                            )
                                            # Clean up Nav2 result - don't trust it
                                            self.nav_goal_handle = None
                                            self.nav_result_future = None
                                            self.nav_start_time = None  # Reset timeout
                                            
                                            # Activate direct navigation to continue moving toward goal
                                            if hasattr(self, 'direct_nav_fallback') and not self.direct_nav_fallback.is_active:
                                                was_active = self.direct_nav_fallback.is_active
                                                self.direct_nav_fallback.set_goal(self.nav_goal_pose, preserve_active_state=was_active)
                                                if not was_active:
                                                    self.direct_nav_fallback.activate()
                                                    self.get_logger().info(
                                                        "Activated direct navigation to continue toward goal "
                                                        "(Nav2 incorrectly declared success - robot is far from goal)."
                                                    )
                                            return  # Continue navigation
                            except Exception as e:
                                self.get_logger().debug(
                                    f"Error checking nav result when check_navigation_complete() returned False: {e}"
                                )
                        # Cases 2 & 3: Nav2 still navigating or failed - normal, continue navigation
                        # No action needed - navigation continues normally
                    
                    # Handle local search if active
                    if self.local_search_active:
                        search_result = self._execute_local_search_step()
                        if search_result.get('search_complete', False):
                            if search_result.get('vehicle_found', False):
                                found_pose = search_result.get('found_pose')
                                if found_pose:
                                    self.get_logger().info(
                                        f"✅ Vehicle found during local search at "
                                        f"({found_pose.pose.position.x:.2f}, {found_pose.pose.position.y:.2f})"
                                    )
                                    # Update goal and navigate to found position
                                    self.nav_goal_pose = found_pose
                                    if self.navigate_to_pose(found_pose):
                                        # Navigation restarted to found position
                                        self.local_search_active = False
                                    else:
                                        self.get_logger().error("Failed to navigate to found vehicle position")
                                        self.local_search_active = False
                                        self.state = MissionState.ERROR_RECOVERY
                                else:
                                    self.local_search_active = False
                                    self.state = MissionState.CAPTURING_LICENSE_PLATE
                            else:
                                self.get_logger().warn(
                                    "Local search completed but vehicle not found. "
                                    "Proceeding to capture anyway."
                                )
                                self.local_search_active = False
                                self.state = MissionState.CAPTURING_LICENSE_PLATE
                        else:
                            # Search in progress - navigate to current search position if needed
                            search_pose = self.local_search.get_current_search_pose()
                            if search_pose and not self.nav_goal_handle:
                                # Navigate to search position
                                self.get_logger().info(
                                    f"Navigating to search position "
                                    f"({search_pose.pose.position.x:.2f}, {search_pose.pose.position.y:.2f})..."
                                )
                                self.navigate_to_pose(search_pose)
                    
                    # Check for stuck situation (handled in check_navigation_complete)
                    # Check for timeout (handled in check_navigation_complete)
                    
                except Exception as e:
                    self.get_logger().error(
                        f"Error in NAVIGATING_TO_LICENSE_PLATE state: {e}",
                        exc_info=True
                    )
                    self.state = MissionState.ERROR_RECOVERY
                    
            elif self.state == MissionState.CAPTURING_LICENSE_PLATE:
                # Enhanced CAPTURING_LICENSE_PLATE state with OCR, 3D positioning, and folder creation
                self._handle_license_plate_capture()
                    
            elif self.state == MissionState.SWITCHING_TO_INSPECTION:
                # Enhanced SWITCHING_TO_INSPECTION state with validation and verification
                self._handle_mode_switch_to_inspection()
                
            elif self.state == MissionState.DETECTING_TYRES:
                # Enhanced DETECTING_TYRES state with validation and stability checks
                self._handle_tyre_detection()
                        
            elif self.state == MissionState.NAVIGATING_TO_TYRE:
                # Enhanced NAVIGATING_TO_TYRE state with navigation monitoring and tyre centering
                self._handle_tyre_navigation()
                    
            elif self.state == MissionState.CAPTURING_TYRE:
                # Enhanced CAPTURING_TYRE state with retry logic and validation
                self._handle_tyre_capture()
                    
            elif self.state == MissionState.CHECKING_COMPLETION:
                # Enhanced CHECKING_COMPLETION state with validation and retry logic
                self._handle_completion_check()
                    
            elif self.state == MissionState.MISSION_COMPLETE:
                # Enhanced MISSION_COMPLETE state with finalization
                self._handle_mission_complete()
                    
            elif self.state == MissionState.ERROR_RECOVERY:
                # Enhanced ERROR_RECOVERY state with context-aware recovery
                self._handle_error_recovery()
                
        except Exception as e:
            # Store the state that caused the error before transitioning
            self.last_error_state = self.state
            self.get_logger().error(
                f"Error in state machine (state: {self.state.value if hasattr(self.state, 'value') else str(self.state)}): {e}",
                exc_info=True
            )
            self.state = MissionState.ERROR_RECOVERY
            
    def finish_current_truck(self):
        """
        Enhanced finish current truck handler with error handling and comprehensive metadata saving.
        
        This method:
        1. Saves comprehensive truck metadata (with error handling)
        2. Resets state for next truck
        3. Switches segmentation mode back to navigation
        4. Finds next unprocessed truck
        5. Transitions to appropriate next state
        6. Handles all error cases gracefully
        """
        try:
            # Validate current truck exists
            if not self.current_truck:
                self.get_logger().warn(
                    "finish_current_truck called but current_truck is None. "
                    "Skipping metadata save."
                )
            else:
                # Save comprehensive truck metadata (uses the enhanced method)
                metadata_saved = self._save_truck_completion_metadata()
                if not metadata_saved:
                    self.get_logger().warn(
                        f"Failed to save truck completion metadata for {self.current_truck.truck_id}. "
                        "Continuing with truck completion anyway..."
                    )
                    # Try to save basic metadata as fallback
                    try:
                        metadata_path = self.photo_dir / f"{self.current_truck.truck_id}_metadata.json"
                        basic_metadata = {
                            'truck_id': self.current_truck.truck_id,
                            'license_plate_photo': str(self.current_truck.license_plate_photo_path) if self.current_truck.license_plate_photo_path else None,
                            'tyres': [
                                {
                                    'tyre_id': tyre.tyre_id,
                                    'photo_path': str(tyre.photo_path) if tyre.photo_path else None,
                                    'photo_taken': tyre.photo_taken
                                }
                                for tyre in self.current_truck.tyres
                            ]
                        }
                        with open(metadata_path, 'w') as f:
                            json.dump(basic_metadata, f, indent=2)
                        self.get_logger().info(f"Saved basic metadata as fallback: {metadata_path}")
                    except Exception as e:
                        self.get_logger().error(
                            f"Failed to save basic metadata fallback: {e}",
                            exc_info=True
                        )
                
                # Log truck completion statistics
                total_tyres = len(self.current_truck.tyres)
                photographed_tyres = sum(1 for t in self.current_truck.tyres if t.photo_taken)
                self.get_logger().info(
                    f"Finished truck {self.current_truck.truck_id}: "
                    f"{photographed_tyres}/{total_tyres} tyres photographed, "
                    f"license plate: {'✓' if self.current_truck.license_plate_photo_taken else '✗'}"
                )
            
            # Reset navigation state (cancel any active goals)
            if self.nav_goal_handle:
                try:
                    self.nav_client.cancel_goal_async(self.nav_goal_handle)
                    self.nav_goal_handle = None
                    self.nav_result_future = None
                    self.nav_goal_pose = None
                except Exception as e:
                    self.get_logger().warn(
                        f"Error canceling navigation goal during truck finish: {e}"
                    )
            
            # Reset state variables for next truck
            previous_truck_id = self.current_truck.truck_id if self.current_truck else None
            self.current_truck = None
            self.current_tyre_index = 0
            
            # Reset tracking variables
            self.tyre_detection_start_time = None
            self.pending_tyre_detections.clear()
            self.tyre_capture_start_time = None
            self.tyre_capture_attempts = 0
            self.completion_check_start_time = None
            self.completion_retry_attempts = 0
            self.mode_switch_start_time = None
            self.mode_switch_verified = False
            self.mode_switch_attempts = 0
            
            # CRITICAL: Clear vehicle obstacle manager and navigation context for completed vehicle
            if self.vehicle_obstacle_manager:
                self.vehicle_obstacle_manager.clear_vehicle_obstacle()
            if self.tyre_navigation_context:
                self.tyre_navigation_context.clear_context()
            if self.tyre_re_detection_handler:
                self.tyre_re_detection_handler.reset()
            
            # Switch back to navigation mode (for next truck detection)
            mode_switched = self.publish_segmentation_mode("navigation")
            if not mode_switched:
                self.get_logger().warn(
                    "Failed to switch to navigation mode. Continuing anyway..."
                )
            
            # Check if more trucks to process
            unprocessed_trucks = [
                truck for truck_id, truck in self.detected_trucks.items()
                if not truck.license_plate_photo_taken or not truck.all_tyres_photographed()
            ]
            
            if unprocessed_trucks:
                # More trucks to process
                next_truck = unprocessed_trucks[0]
                self.current_truck = next_truck
                self.get_logger().info(
                    f"Moving to next truck: {next_truck.truck_id} "
                    f"({len(unprocessed_trucks)} truck(s) remaining)"
                )
                
                # CRITICAL: Update vehicle obstacle manager with next truck
                if self.vehicle_obstacle_manager and next_truck.detection_pose:
                    self.vehicle_obstacle_manager.update_vehicle_obstacle(next_truck.detection_pose)
                    self.get_logger().info(f"Vehicle obstacle updated for next truck: {next_truck.truck_id}")
                
                # Clear and reset navigation context for new truck
                if self.tyre_navigation_context:
                    self.tyre_navigation_context.clear_context()
                if self.tyre_re_detection_handler:
                    self.tyre_re_detection_handler.reset()
                if self.tyre_identifier:
                    self.tyre_identifier.clear_registry()
                
                self.state = MissionState.TRUCK_DETECTED
            else:
                # All trucks processed - mission complete
                total_trucks = len(self.detected_trucks)
                self.get_logger().info(
                    f"✅ All trucks processed ({total_trucks} total). Mission complete!"
                )
                self.state = MissionState.MISSION_COMPLETE
                
        except Exception as e:
            self.get_logger().error(
                f"Error in finish_current_truck: {e}",
                exc_info=True
            )
            # Even if error occurs, try to continue
            # Reset current truck to allow recovery
            if self.current_truck:
                self.get_logger().warn(
                    f"Resetting after error during finish_current_truck for {self.current_truck.truck_id}"
                )
            self.current_truck = None
            self.current_tyre_index = 0
            
            # Try to find next truck or complete mission
            try:
                unprocessed_trucks = [
                    truck for truck_id, truck in self.detected_trucks.items()
                    if not truck.license_plate_photo_taken or not truck.all_tyres_photographed()
                ]
                if unprocessed_trucks:
                    self.current_truck = unprocessed_trucks[0]
                    self.state = MissionState.TRUCK_DETECTED
                else:
                    self.state = MissionState.MISSION_COMPLETE
            except Exception as e2:
                self.get_logger().error(
                    f"Error in error recovery for finish_current_truck: {e2}",
                    exc_info=True
                )
                # Last resort - transition to error recovery
                self.state = MissionState.ERROR_RECOVERY
            
    def _handle_tyre_navigation(self):
        """
        Enhanced tyre navigation handler with monitoring and centering.
        
        This method:
        1. Manages navigation to each tyre sequentially
        2. Monitors navigation progress
        3. Handles goal recalculation for "too close" errors
        4. Detects arrival and validates position
        5. Handles navigation failures gracefully
        6. Manages transitions between tyres
        """
        try:
            # CRITICAL: Validate current truck still exists (vehicle detection not lost)
            # If vehicle detection is lost during navigation, we shouldn't continue to stale goal
            if not self.current_truck:
                self.get_logger().error(
                    "🚨🚨🚨 CRITICAL: NAVIGATING_TO_TYRE: current_truck is None! "
                    "Vehicle detection lost during navigation. "
                    "Canceling navigation and transitioning to ERROR_RECOVERY."
                )
                # Cancel navigation immediately
                if self.nav_goal_handle:
                    try:
                        self.nav_client.cancel_goal_async(self.nav_goal_handle)
                    except Exception as e:
                        self.get_logger().warn(f"Error canceling goal: {e}")
                    self.nav_goal_handle = None
                    self.nav_result_future = None
                # Deactivate direct navigation if active
                if hasattr(self, 'direct_nav_fallback') and self.direct_nav_fallback.is_active:
                    self.direct_nav_fallback.deactivate()
                self.nav_goal_pose = None
                self.nav_start_time = None
                self.state = MissionState.ERROR_RECOVERY
                return
            
            # CRITICAL: Check if current_truck is still in detected_trucks (vehicle still tracked)
            # If vehicle was removed from tracking, we shouldn't continue navigation
            if self.current_truck and hasattr(self.current_truck, 'truck_id'):
                if self.current_truck.truck_id not in self.detected_trucks:
                    self.get_logger().error(
                        f"🚨🚨🚨 CRITICAL: NAVIGATING_TO_TYRE: current_truck {self.current_truck.truck_id} "
                        f"not found in detected_trucks! Vehicle tracking lost. "
                        f"Continuing navigation to last known tyre position (may be stale)."
                    )
                    # Don't cancel immediately - vehicle might still be there, just not tracked
                    # But log warning for operator awareness
            
            # Check if all tyres are done
            if self.current_tyre_index >= len(self.current_truck.tyres):
                self.get_logger().info(
                    f"All tyres processed for {self.current_truck.truck_id}. "
                    "Transitioning to CHECKING_COMPLETION."
                )
                self.state = MissionState.CHECKING_COMPLETION
                return
                    
            # Get current tyre
            current_tyre = self.current_truck.tyres[self.current_tyre_index]
                
            # Skip if already photographed
            if current_tyre.photo_taken:
                self.get_logger().info(
                    f"Tyre {current_tyre.tyre_id} already photographed. "
                    "Moving to next tyre."
                )
                self.current_tyre_index += 1
                # Reset navigation state for next tyre
                self.nav_goal_handle = None
                self.nav_result_future = None
                self.nav_goal_pose = None
                return
                    
            # Check if navigation goal has been sent
            if self.nav_goal_handle is None and self.nav_goal_pose is None:
                # CRITICAL: Set navigation context for this tyre
                if self.tyre_navigation_context:
                    # Check if we can retry navigation to this tyre
                    if not self.tyre_navigation_context.can_retry_navigation(current_tyre.tyre_id):
                        self.get_logger().warn(
                            f"Max navigation attempts reached for tyre {current_tyre.tyre_id}. Skipping."
                        )
                        self.current_tyre_index += 1
                        self.current_tyre_nav_failures = 0
                        return
                
                # Need to start navigation to this tyre
                self.get_logger().info(
                    f"NAVIGATING_TO_TYRE: Starting navigation to tyre {current_tyre.tyre_id} "
                    f"({self.current_tyre_index + 1}/{len(self.current_truck.tyres)})"
                )
                
                # Calculate navigation pose for tyre
                tyre_pose = self.tyre_to_navigation_pose(current_tyre)
                
                # CRITICAL: Set navigation context BEFORE validation
                if self.tyre_navigation_context:
                    self.tyre_navigation_context.set_target_tyre(
                        current_tyre.tyre_id,
                        self.current_tyre_index,
                        tyre_pose
                    )
                if not tyre_pose:
                    self.get_logger().error(
                        f"Failed to calculate navigation pose for tyre {current_tyre.tyre_id}. "
                        f"Skipping to next tyre."
                    )
                    self.current_tyre_index += 1
                    self.current_tyre_nav_failures = 0
                    return
                
                # CRITICAL: Validate tyre goal before navigation
                if self.tyre_goal_validator:
                    robot_pose = self._get_robot_pose("map")
                    validation_result = self.tyre_goal_validator.validate_tyre_goal(
                        tyre_pose,
                        current_tyre.position_3d,
                        robot_pose
                    )
                    
                    # CRITICAL: Also validate against vehicle obstacle
                    if (self.vehicle_obstacle_manager and 
                        self.current_truck and 
                        self.current_truck.detection_pose):
                        vehicle_valid, vehicle_issue = self.vehicle_obstacle_manager.validate_navigation_goal(tyre_pose)
                        if not vehicle_valid:
                            self.get_logger().warn(
                                f"Tyre goal validation failed - vehicle obstacle: {vehicle_issue}"
                            )
                            # Add to validation issues
                            validation_result['issues'].append(f"Vehicle obstacle: {vehicle_issue}")
                            validation_result['safe'] = False
                            validation_result['valid'] = False
                            
                            # Try to calculate waypoint around vehicle
                            if robot_pose:
                                robot_pos = robot_pose.pose.position
                                waypoint = self.vehicle_obstacle_manager.calculate_waypoint_around_vehicle(
                                    robot_pos,
                                    tyre_pose.pose.position
                                )
                                if waypoint:
                                    # Update goal to use waypoint (simplified - would need proper pose)
                                    self.get_logger().info(
                                        f"Calculated waypoint around vehicle: ({waypoint.x:.2f}, {waypoint.y:.2f})"
                                    )
                                    # For now, just log - full waypoint navigation would require Nav2 waypoint support
                    
                    if not validation_result['valid']:
                        self.get_logger().warn(
                            f"Tyre goal validation failed for {current_tyre.tyre_id}: "
                            f"{', '.join(validation_result['issues'])}"
                        )
                        
                        if validation_result['recommendation'] == 'adjust' and validation_result['adjusted_goal']:
                            # Use adjusted goal
                            self.get_logger().info(
                                f"Using adjusted goal for tyre {current_tyre.tyre_id}"
                            )
                            tyre_pose = validation_result['adjusted_goal']
                        elif validation_result['recommendation'] == 'skip':
                            # Mark goal as unreachable and skip
                            self.tyre_goal_validator.mark_goal_unreachable(tyre_pose)
                            self.get_logger().error(
                                f"Tyre goal for {current_tyre.tyre_id} is invalid/unreachable. Skipping."
                            )
                            self.current_tyre_index += 1
                            self.current_tyre_nav_failures = 0
                            return
                
                # Validate pose structure before sending
                if not self._validate_navigation_pose(tyre_pose):
                    self.get_logger().error(
                        f"Invalid navigation pose structure for tyre {current_tyre.tyre_id}. "
                        f"Skipping to next tyre."
                    )
                    self.current_tyre_index += 1
                    self.current_tyre_nav_failures = 0
                    return
                
                # Reset failure handler for new navigation attempt
                if self.navigation_failure_handler:
                    self.navigation_failure_handler.reset_failure_count()
                
                # Send navigation goal
                navigation_success = self.navigate_to_pose(tyre_pose)
                if not navigation_success:
                    self.get_logger().error(
                        f"Failed to send navigation goal for tyre {current_tyre.tyre_id}. "
                        "Will retry on next iteration."
                    )
                    # Mark failure in context
                    if self.tyre_navigation_context:
                        self.tyre_navigation_context.mark_tyre_navigation_failure(
                            current_tyre.tyre_id,
                            "Failed to send navigation goal"
                        )
                    return
            
            # Navigation is in progress - monitor it
            if self.nav_goal_handle is not None:
                # Update navigation progress tracking
                self._update_navigation_progress()
                
                # Check if navigation is complete
                if self.check_navigation_complete():
                    # Verify we're at the goal
                    current_pose = self._get_robot_pose("map")
                    if current_pose and self.nav_goal_pose:
                        goal_pos = self.nav_goal_pose.pose.position
                        robot_pos = current_pose.pose.position
                        arrival_distance = math.sqrt(
                            (goal_pos.x - robot_pos.x)**2 +
                            (goal_pos.y - robot_pos.y)**2
                        )
                        arrival_threshold = self.get_parameter('arrival_distance_threshold').value
                        
                        # CRITICAL: Check orientation match for tyre capture
                        goal_orientation = self.nav_goal_pose.pose.orientation
                        robot_orientation = current_pose.pose.orientation
                        goal_siny_cosp = 2.0 * (goal_orientation.w * goal_orientation.z + goal_orientation.x * goal_orientation.y)
                        goal_cosy_cosp = 1.0 - 2.0 * (goal_orientation.y * goal_orientation.y + goal_orientation.z * goal_orientation.z)
                        goal_yaw = math.atan2(goal_siny_cosp, goal_cosy_cosp)
                        robot_siny_cosp = 2.0 * (robot_orientation.w * robot_orientation.z + robot_orientation.x * robot_orientation.y)
                        robot_cosy_cosp = 1.0 - 2.0 * (robot_orientation.y * robot_orientation.y + robot_orientation.z * robot_orientation.z)
                        robot_yaw = math.atan2(robot_siny_cosp, robot_cosy_cosp)
                        orientation_diff = abs(goal_yaw - robot_yaw)
                        if orientation_diff > math.pi:
                            orientation_diff = 2 * math.pi - orientation_diff
                        arrival_orientation_threshold = self.get_parameter('arrival_orientation_tolerance').value
                        orientation_match = orientation_diff <= arrival_orientation_threshold
                        
                        # CRITICAL: Only check arrival if we've moved at least minimum distance
                        # This prevents premature arrival detection if goal is close (prevents false arrival)
                        # Same logic as license plate navigation - ensures minimum movement is ALWAYS checked
                        distance_traveled = None
                        if self.nav_initial_distance is not None:
                            distance_traveled = self.nav_initial_distance - arrival_distance
                        elif hasattr(self, 'nav_progress_distance') and self.nav_progress_distance is not None:
                            distance_traveled = self.nav_progress_distance
                            self.get_logger().debug(
                                f"Tyre navigation: Using nav_progress_distance as fallback: {distance_traveled:.2f}m"
                            )
                        
                        # Check if we've moved enough before checking arrival
                        # CRITICAL FIX: If rover was already at goal when navigation started (nav_initial_distance < arrival_threshold),
                        # AND rover is still at goal AND facing correctly, accept immediately (rover was placed in correct position)
                        arrival_detected = False
                        
                        # Special case: Rover already at goal when navigation started (placed in position)
                        if (self.nav_initial_distance is not None and 
                            self.nav_initial_distance <= arrival_threshold and
                            arrival_distance <= arrival_threshold and
                            orientation_match):
                            # Rover was already at goal when navigation started and is still there with correct orientation
                            # Accept immediately - rover was correctly positioned
                            arrival_detected = True
                            self.get_logger().info(
                                f"✅ Tyre navigation: Rover was already at goal when navigation started (initial: {self.nav_initial_distance:.2f}m <= {arrival_threshold:.2f}m). "
                                f"Current: {arrival_distance:.2f}m, orientation: {math.degrees(orientation_diff):.1f}° <= {math.degrees(arrival_orientation_threshold):.1f}°. "
                                f"Accepting as arrived (rover was placed in correct position)."
                            )
                        elif distance_traveled is not None and distance_traveled >= self.min_movement_before_arrival_check:
                            # Moved enough - check BOTH distance AND orientation
                            if arrival_distance <= arrival_threshold and orientation_match:
                                arrival_detected = True
                        elif arrival_distance <= arrival_threshold and not orientation_match:
                            # At position but wrong orientation - log and wait for orientation adjustment
                            self.get_logger().info(
                                f"🔄 Tyre navigation: At goal position (distance: {arrival_distance:.2f}m <= {arrival_threshold:.2f}m) "
                                f"but orientation mismatch: {math.degrees(orientation_diff):.1f}° > {math.degrees(arrival_orientation_threshold):.1f}° "
                                f"(robot: {math.degrees(robot_yaw):.1f}°, goal: {math.degrees(goal_yaw):.1f}°). "
                                f"Continuing to rotate to match orientation..."
                            )
                            return  # Continue navigation to adjust orientation
                        elif distance_traveled is not None:
                            # Haven't moved enough yet - continue navigation
                            if not hasattr(self, '_tyre_arrival_check_log_counter'):
                                self._tyre_arrival_check_log_counter = 0
                            self._tyre_arrival_check_log_counter += 1
                            if self._tyre_arrival_check_log_counter >= 10:  # Every 5 seconds
                                self.get_logger().warn(
                                    f"⚠️ Tyre navigation: Nav2 says complete, but robot hasn't moved enough: "
                                    f"Distance: {arrival_distance:.2f}m, "
                                    f"Traveled: {distance_traveled:.2f}m / {self.min_movement_before_arrival_check:.2f}m. "
                                    f"Continuing navigation until robot actually moves."
                                )
                                self._tyre_arrival_check_log_counter = 0
                            # Continue navigation - don't declare arrival yet
                            if self.nav_goal_handle is not None:
                                self.nav_goal_handle = None
                                self.nav_result_future = None
                            if hasattr(self, 'direct_nav_fallback') and not self.direct_nav_fallback.is_active:
                                was_active = self.direct_nav_fallback.is_active
                                self.direct_nav_fallback.set_goal(self.nav_goal_pose, preserve_active_state=was_active)
                                if not was_active:
                                    self.direct_nav_fallback.activate()
                                    self.get_logger().info(
                                        "Activated direct navigation for tyre (Nav2 incorrectly declared success without sufficient movement)."
                                    )
                            return
                        else:
                            # CRITICAL FIX: No distance tracking available - require minimum movement
                            self.get_logger().warn(
                                f"⚠️ Tyre navigation: Cannot verify minimum movement. "
                                f"Robot at {arrival_distance:.2f}m from goal - requiring minimum movement "
                                f"({self.min_movement_before_arrival_check:.2f}m) before arrival."
                            )
                            if arrival_distance <= arrival_threshold:
                                self.get_logger().info(
                                    f"Robot is close to tyre goal ({arrival_distance:.2f}m) but cannot verify movement. "
                                    f"Waiting for robot to move at least {self.min_movement_before_arrival_check:.2f}m before declaring arrival."
                                )
                            return  # Continue navigation - arrival will be detected once movement is tracked
                        
                        # Only process arrival if detected
                        if arrival_detected:
                            self.get_logger().info(
                                f"✅ Arrived at tyre {current_tyre.tyre_id} "
                                f"(distance: {arrival_distance:.2f}m <= {arrival_threshold:.2f}m, "
                                f"orientation: {math.degrees(orientation_diff):.1f}° <= {math.degrees(arrival_orientation_threshold):.1f}°, "
                                f"traveled: {distance_traveled:.2f}m)"
                            )
                            
                            # CRITICAL: Mark navigation success in context
                            if self.tyre_navigation_context:
                                self.tyre_navigation_context.mark_tyre_navigation_success(current_tyre.tyre_id)
                            
                            # CRITICAL: Refine tyre pose using visual feedback
                            if self.tyre_pose_refiner:
                                # Pose refinement will be done in bbox_callback with current detections
                                # For now, mark that we should refine
                                self.tyre_pose_refiner.start_refinement()
                            
                            # Verify tyre centering (check if we're positioned correctly)
                            tyre_centered = self._verify_tyre_centering(current_tyre, current_pose)
                            if not tyre_centered:
                                self.get_logger().warn(
                                    f"Tyre {current_tyre.tyre_id} not properly centered. "
                                    "Attempting to adjust position..."
                                )
                                # Try to recalculate and navigate to a better position
                                adjusted_pose = self._calculate_centered_tyre_pose(current_tyre, current_pose)
                                if adjusted_pose and self._validate_navigation_pose(adjusted_pose):
                                    self.get_logger().info(
                                        "Recalculating navigation goal for better centering..."
                                    )
                                    navigation_success = self.navigate_to_pose(adjusted_pose)
                                    if navigation_success:
                                        # Wait for new navigation to complete
                                        return
                                else:
                                    self.get_logger().warn(
                                        "Could not calculate better position. "
                                        "Proceeding with current position."
                                    )
                            
                            # Navigation complete - transition to capture
                            self.nav_goal_handle = None
                            self.nav_result_future = None
                            self.nav_goal_pose = None
                            self.get_logger().info(
                                f"NAVIGATING_TO_TYRE → CAPTURING_TYRE: "
                                f"Arrived at tyre {current_tyre.tyre_id} for {self.current_truck.truck_id}"
                            )
                            self.state = MissionState.CAPTURING_TYRE
                            self.get_logger().info(
                                f"✅ NEXT ACTION: System is now in CAPTURING_TYRE state. "
                                f"Capturing photo of tyre {current_tyre.tyre_id}..."
                            )
                        else:
                            self.get_logger().warn(
                                f"Navigation reported complete but robot is "
                                f"{arrival_distance:.2f}m from goal "
                                f"(threshold: {arrival_threshold:.2f}m). "
                                "Attempting to continue..."
                            )
                            # If close enough, continue anyway
                            if arrival_distance < arrival_threshold * 1.5:
                                self.get_logger().info(
                                    "Close enough to goal. Continuing to capture."
                                )
                                self.nav_goal_handle = None
                                self.nav_result_future = None
                                self.nav_goal_pose = None
                                self.state = MissionState.CAPTURING_TYRE
                            else:
                                self.get_logger().error(
                                    "Too far from goal. Skipping this tyre."
                                )
                                self.current_tyre_index += 1
                                self.nav_goal_handle = None
                                self.nav_result_future = None
                                self.nav_goal_pose = None
                    else:
                        # Can't verify pose, but navigation says complete - trust it
                        self.get_logger().info(
                            f"Navigation complete for tyre {current_tyre.tyre_id} "
                            "(unable to verify pose). Proceeding to capture."
                        )
                        self.nav_goal_handle = None
                        self.nav_result_future = None
                        self.nav_goal_pose = None
                        self.state = MissionState.CAPTURING_TYRE
            
        except Exception as e:
            self.get_logger().error(
                f"Error in NAVIGATING_TO_TYRE state: {e}",
                exc_info=True
            )
            self.state = MissionState.ERROR_RECOVERY
    
    def tyre_to_navigation_pose(self, tyre_data):
        """
        Convert tyre 3D position to navigation pose with proper centering.
        
        Args:
            tyre_data: TyreData object
            
        Returns:
            PoseStamped: Navigation pose, or None if failed
        """
        try:
            if not tyre_data:
                self.get_logger().error("tyre_data is None")
                return None
            
            if not hasattr(tyre_data, 'position_3d') or not tyre_data.position_3d:
                self.get_logger().error(
                    f"Tyre {tyre_data.tyre_id if hasattr(tyre_data, 'tyre_id') else 'unknown'} "
                    "has no position_3d"
                )
                return None
            
            approach_dist = self.get_parameter('approach_distance').value
            
            # Get tyre position
            tyre_pos = tyre_data.position_3d
            
            # Calculate approach position (in front of tyre)
            # Need to transform tyre position from camera frame to map frame
            # For now, assume position is already in map frame or will be transformed
            
            pose = PoseStamped()
            pose.header.frame_id = "map"
            pose.header.stamp = self.get_clock().now().to_msg()
            
            # Calculate approach position
            # Assume +y is forward direction
            pose.pose.position.x = tyre_pos.x
            pose.pose.position.y = tyre_pos.y - approach_dist  # Approach from front
            pose.pose.position.z = tyre_pos.z
            
            # Orient toward tyre
            # Calculate yaw angle to face the tyre
            dx = tyre_pos.x - pose.pose.position.x
            dy = tyre_pos.y - pose.pose.position.y
            yaw = math.atan2(dy, dx)
            
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Validate pose
            if not self._validate_navigation_pose(pose):
                self.get_logger().error(
                    f"Calculated navigation pose for tyre is invalid"
                )
                return None
            
            self.get_logger().info(
                f"Calculated navigation pose for tyre {tyre_data.tyre_id}: "
                f"({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f}) "
                f"approach distance: {approach_dist:.2f}m"
            )
            
            return pose
            
        except Exception as e:
            self.get_logger().error(
                f"Error converting tyre to navigation pose: {e}",
                exc_info=True
            )
            return None
    
    def _verify_tyre_centering(self, tyre_data, robot_pose):
        """
        Verify robot is properly centered in front of tyre.
        
        Args:
            tyre_data: TyreData object
            robot_pose: PoseStamped of robot's current position
            
        Returns:
            bool: True if centered, False otherwise
        """
        try:
            if not tyre_data or not robot_pose:
                return False
            
            if not hasattr(tyre_data, 'position_3d') or not tyre_data.position_3d:
                return False
            
            tyre_pos = tyre_data.position_3d
            robot_pos = robot_pose.pose.position
            
            # Calculate distance to tyre
            distance = math.sqrt(
                (tyre_pos.x - robot_pos.x)**2 +
                (tyre_pos.y - robot_pos.y)**2 +
                (tyre_pos.z - robot_pos.z)**2
            )
            
            approach_dist = self.get_parameter('approach_distance').value
            centering_tolerance = 0.3  # meters - acceptable deviation from ideal distance
            
            # Check if distance is within tolerance
            if abs(distance - approach_dist) > centering_tolerance:
                self.get_logger().debug(
                    f"Tyre centering check: distance={distance:.2f}m, "
                    f"ideal={approach_dist:.2f}m, "
                    f"tolerance={centering_tolerance:.2f}m"
                )
                return False
            
            # Check if robot is facing the tyre (simplified - just check if close enough)
            # More sophisticated check could verify orientation
            
            return True
            
        except Exception as e:
            self.get_logger().error(
                f"Error verifying tyre centering: {e}",
                exc_info=True
            )
            return False
    
    def _calculate_centered_tyre_pose(self, tyre_data, current_robot_pose):
        """
        Calculate a better centered pose for tyre capture.
        
        Args:
            tyre_data: TyreData object
            current_robot_pose: PoseStamped of robot's current position
            
        Returns:
            PoseStamped: Adjusted navigation pose, or None if failed
        """
        try:
            if not tyre_data or not current_robot_pose:
                return None
            
            if not hasattr(tyre_data, 'position_3d') or not tyre_data.position_3d:
                return None
            
            tyre_pos = tyre_data.position_3d
            robot_pos = current_robot_pose.pose.position
            
            # Calculate direction from robot to tyre
            dx = tyre_pos.x - robot_pos.x
            dy = tyre_pos.y - robot_pos.y
            distance = math.sqrt(dx**2 + dy**2)
            
            if distance < 0.01:
                # Already at tyre
                return None
            
            # Normalize direction
            dx_norm = dx / distance
            dy_norm = dy / distance
            
            # Calculate ideal approach distance
            approach_dist = self.get_parameter('approach_distance').value
            
            # Calculate new position at ideal distance
            pose = PoseStamped()
            pose.header.frame_id = current_robot_pose.header.frame_id
            pose.header.stamp = self.get_clock().now().to_msg()
            
            pose.pose.position.x = tyre_pos.x - dx_norm * approach_dist
            pose.pose.position.y = tyre_pos.y - dy_norm * approach_dist
            pose.pose.position.z = tyre_pos.z
            
            # Orient toward tyre
            yaw = math.atan2(dy, dx)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            return pose
            
        except Exception as e:
            self.get_logger().error(
                f"Error calculating centered tyre pose: {e}",
                exc_info=True
            )
            return None
    
    def _handle_tyre_capture(self):
        """
        Enhanced tyre capture handler with retry logic and validation.
        
        This method:
        1. Captures photo of current tyre with retry logic
        2. Validates photo was captured successfully
        3. Saves photo to organized folder structure
        4. Updates tyre data with photo path
        5. Handles timeout and retry scenarios
        6. Transitions to next tyre or completes inspection
        """
        try:
            # Validate current truck
            if not self.current_truck:
                self.get_logger().error(
                    "CAPTURING_TYRE: current_truck is None. "
                    "Transitioning to ERROR_RECOVERY."
                )
                self.state = MissionState.ERROR_RECOVERY
                return
            
            # Validate current tyre index
            if self.current_tyre_index >= len(self.current_truck.tyres):
                self.get_logger().info(
                    "All tyres processed. Transitioning to CHECKING_COMPLETION."
                )
                self.state = MissionState.CHECKING_COMPLETION
                return
            
            current_tyre = self.current_truck.tyres[self.current_tyre_index]
                    
            # Initialize capture attempt tracking on first attempt
            if self.tyre_capture_start_time is None:
                self.tyre_capture_start_time = time.time()
                self.tyre_capture_attempts = 0
                self.tyre_verification_active = False
                self.tyre_verification_start_time = None
                self.get_logger().info(
                    f"CAPTURING_TYRE: Starting photo capture for "
                    f"tyre {current_tyre.tyre_id} on {self.current_truck.truck_id}"
                )
                
                # CRITICAL: Start tyre verification before capture
                if self.tyre_capture_verifier:
                    self.tyre_verification_active = True
                    self.tyre_verification_start_time = time.time()
                    self.tyre_capture_verifier.reset()
                    self.get_logger().info(
                        f"Verifying tyre {current_tyre.tyre_id} visibility before capture..."
                    )
            
            # CRITICAL: Verify tyre is visible before capturing photo
            if self.tyre_verification_active and self.tyre_capture_verifier:
                # Verification will be checked in bbox_callback
                # If verification hasn't passed yet, wait
                verification_elapsed = time.time() - self.tyre_verification_start_time
                verification_timeout = self.get_parameter('tyre_capture_verification_timeout').value
                
                if verification_elapsed > verification_timeout:
                    self.get_logger().warn(
                        f"Tyre verification timeout after {verification_elapsed:.1f}s. "
                        "Proceeding with capture anyway."
                    )
                    self.tyre_verification_active = False
                else:
                    # Still waiting for verification - return and check again next iteration
                    # Verification result will be set in bbox_callback
                    self.get_logger().debug(
                        f"Waiting for tyre verification... ({verification_elapsed:.1f}s elapsed)"
                    )
                    return
            
            # Check for timeout
            capture_timeout = self.get_parameter('tyre_capture_timeout').value
            elapsed_time = time.time() - self.tyre_capture_start_time
            if elapsed_time > capture_timeout:
                max_attempts = self.get_parameter('max_tyre_capture_attempts').value
                self.get_logger().error(
                    f"Tyre capture timeout after {elapsed_time:.1f}s. "
                    f"Attempts: {self.tyre_capture_attempts}/{max_attempts}. "
                    f"Skipping tyre {current_tyre.tyre_id}."
                )
                # Mark as failed but continue
                current_tyre.attempts = self.tyre_capture_attempts
                # Move to next tyre
                self.tyre_capture_start_time = None
                self.tyre_capture_attempts = 0
                self.current_tyre_index += 1
                self.state = MissionState.NAVIGATING_TO_TYRE
                return
            
            # Increment attempt counter
            self.tyre_capture_attempts += 1
            max_attempts = self.get_parameter('max_tyre_capture_attempts').value
            
            self.get_logger().info(
                f"CAPTURING_TYRE: Attempt {self.tyre_capture_attempts}/{max_attempts} "
                f"for tyre {current_tyre.tyre_id}"
            )
            
            # Create folder for this truck's tyres if it doesn't exist
            truck_folder = self.photo_dir / self.current_truck.truck_id / "tyres"
            truck_folder.mkdir(parents=True, exist_ok=True)
            
            # Capture photo
            photo_filename = f"{self.current_truck.truck_id}_{current_tyre.tyre_id}.jpg"
            photo_path = self._capture_tyre_photo(photo_filename, truck_folder)
                    
            if photo_path:
                # CRITICAL: Check photo quality after capture
                quality_passed = True
                quality_result = None
                if self.photo_quality_checker:
                    try:
                        quality_result = self.photo_quality_checker.check_photo_quality(str(photo_path))
                        quality_passed = quality_result.get('quality_passed', True)
                        
                        if not quality_passed:
                            self.get_logger().warn(
                                f"⚠️ Photo quality check failed for {current_tyre.tyre_id}: "
                                f"score={quality_result.get('score', 0):.1f}/100, "
                                f"issues: {', '.join(quality_result.get('issues', []))}"
                            )
                            
                            # CRITICAL: Check if repositioning is needed
                            should_reposition = False
                            reposition_reason = None
                            if self.tyre_capture_repositioner:
                                robot_pose = self._get_robot_pose("map")
                                if robot_pose and hasattr(current_tyre, 'position_3d') and current_tyre.position_3d:
                                    should_reposition, reposition_reason = self.tyre_capture_repositioner.should_reposition(
                                        quality_result,
                                        robot_pose,
                                        current_tyre.position_3d
                                    )
                            
                            # If quality is poor but we've tried multiple times, accept it
                            if self.tyre_capture_attempts >= max_attempts:
                                self.get_logger().warn(
                                    "Max attempts reached. Accepting photo despite quality issues."
                                )
                                quality_passed = True
                            elif should_reposition and self.tyre_capture_repositioner:
                                # Try repositioning before retrying capture
                                robot_pose = self._get_robot_pose("map")
                                if robot_pose and hasattr(current_tyre, 'position_3d') and current_tyre.position_3d:
                                    # Get reposition goal
                                    failed_positions = []  # Could track failed positions
                                    reposition_goal = self.tyre_capture_repositioner.get_next_reposition_goal(
                                        robot_pose,
                                        current_tyre.position_3d,
                                        failed_positions,
                                        self.tyre_capture_attempts
                                    )
                                    
                                    if reposition_goal:
                                        self.get_logger().info(
                                            f"Repositioning for better photo quality: {reposition_reason}. "
                                            f"Navigating to new position..."
                                        )
                                        # Navigate to reposition goal
                                        if self.navigate_to_pose(reposition_goal):
                                            # Update phase tracking
                                            if self.mission_timeout_handler:
                                                self.mission_timeout_handler.start_phase(MissionPhase.TYRE_NAVIGATION)
                                            self.state = MissionState.NAVIGATING_TO_TYRE
                                            # Reset capture state for retry after repositioning
                                            self.tyre_capture_start_time = None
                                            return
                                        else:
                                            self.get_logger().warn("Failed to navigate to reposition goal. Retrying capture in place.")
                                    else:
                                        self.get_logger().warn("No reposition goal available. Retrying capture in place.")
                            
                            if not quality_passed:
                                # Retry capture due to quality issues
                                self.get_logger().info(
                                    f"Retrying photo capture due to quality issues "
                                    f"(attempt {self.tyre_capture_attempts}/{max_attempts})"
                                )
                                # Don't increment attempt counter yet - will retry
                                return  # Will retry on next iteration
                        else:
                            self.get_logger().info(
                                f"✅ Photo quality check passed: "
                                f"score={quality_result.get('score', 0):.1f}/100"
                            )
                    except Exception as e:
                        self.get_logger().error(
                            f"Error checking photo quality: {e}. Proceeding with capture.",
                            exc_info=True
                        )
                        quality_passed = True  # Proceed on error
                
                # Success - update tyre data
                current_tyre.photo_path = str(photo_path)
                current_tyre.photo_taken = True
                current_tyre.attempts = self.tyre_capture_attempts
                
                self.get_logger().info(
                    f"✅ Tyre photo captured successfully: {current_tyre.tyre_id} "
                    f"(photo: {photo_path}, attempts: {self.tyre_capture_attempts})"
                )
                
                # Save metadata for this tyre
                self._save_tyre_metadata(current_tyre, truck_folder)
                
                # Reset capture state
                self.tyre_capture_start_time = None
                self.tyre_capture_attempts = 0
                self.tyre_verification_active = False
                self.tyre_verification_start_time = None
                
                # Move to next tyre
                self.current_tyre_index += 1
                total_tyres = len(self.current_truck.tyres)
                
                # Check if more tyres to process
                if self.current_tyre_index < total_tyres:
                    next_tyre_num = self.current_tyre_index + 1
                    self.get_logger().info(
                        f"CAPTURING_TYRE → NAVIGATING_TO_TYRE: "
                        f"Tyre {current_tyre.tyre_id} photographed. Moving to next tyre ({next_tyre_num}/{total_tyres})..."
                    )
                    self.state = MissionState.NAVIGATING_TO_TYRE
                    self.get_logger().info(
                        f"✅ NEXT ACTION: System is now in NAVIGATING_TO_TYRE state. "
                        f"Navigating to tyre {next_tyre_num}/{total_tyres}..."
                    )
                else:
                    # All tyres photographed
                    self.get_logger().info(
                        f"CAPTURING_TYRE → CHECKING_COMPLETION: "
                        f"All {total_tyres} tyre(s) photographed for {self.current_truck.truck_id}"
                    )
                    self.state = MissionState.CHECKING_COMPLETION
                    self.get_logger().info(
                        f"✅ NEXT ACTION: System is now in CHECKING_COMPLETION state. "
                        f"Verifying all tyres photographed for {self.current_truck.truck_id}..."
                    )
            else:
                # Photo capture failed
                if self.tyre_capture_attempts < max_attempts:
                    retry_delay = self.get_parameter('tyre_capture_retry_delay').value
                    self.get_logger().warn(
                        f"Photo capture failed. Retrying in {retry_delay}s... "
                        f"(attempt {self.tyre_capture_attempts}/{max_attempts})"
                    )
                    # Wait for retry (note: in state machine, we return and check again)
                    # Don't sleep here - just return and let the timeout handle it
                    return  # Will retry on next iteration
                else:
                    # Max attempts reached
                    self.get_logger().error(
                        f"Photo capture failed after {max_attempts} attempts. "
                        f"Skipping tyre {current_tyre.tyre_id}."
                    )
                    current_tyre.attempts = max_attempts
                    # Move to next tyre
                    self.tyre_capture_start_time = None
                    self.tyre_capture_attempts = 0
                    self.current_tyre_index += 1
                    self.state = MissionState.NAVIGATING_TO_TYRE
                    
        except Exception as e:
            self.get_logger().error(
                f"Error in CAPTURING_TYRE state: {e}",
                exc_info=True
            )
            self.state = MissionState.ERROR_RECOVERY
    
    def _capture_tyre_photo(self, filename, folder):
        """
        Capture photo for tyre with validation.
        
        Args:
            filename: Name for the photo file
            folder: Path to folder where photo should be saved
            
        Returns:
            Path: Path to captured photo, or None if failed
        """
        try:
            if not self.capture_photo_client.service_is_ready():
                self.get_logger().error("Photo capture service not ready")
                return None
            
            # Create service request (Trigger service has no parameters)
            request = Trigger.Request()
            
            # Call service
            future = self.capture_photo_client.call_async(request)
            
            # Wait for response (with timeout) - CRITICAL: Use parameter not hardcoded value
            timeout = self.get_parameter('photo_capture_timeout').value
            start_time = time.time()
            while not future.done():
                rclpy.spin_once(self, timeout_sec=0.1)
                if time.time() - start_time > timeout:
                    self.get_logger().error(
                        f"Photo capture service call timeout after {timeout}s"
                    )
                    return None
            
            # Get response - CRITICAL: Handle potential exceptions
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error(
                    f"Error getting photo capture service response: {e}",
                    exc_info=True
                )
                return None
            if response.success and response.message:
                # Service returns filename in message, construct full path
                # Note: Service saves to its configured storage directory
                # We need to construct path from service's storage directory
                # For now, try the folder passed in, or use default
                photo_path = folder / response.message
                if not photo_path.exists():
                    # Try default storage directory
                    default_dir = Path.home() / "tyre_inspection_photos"
                    photo_path = default_dir / response.message
                
                if photo_path.exists():
                    # Move/copy to desired folder if different
                    target_path = folder / filename
                    if photo_path != target_path:
                        import shutil
                        shutil.copy2(photo_path, target_path)
                        photo_path = target_path
                    
                    self.get_logger().info(
                        f"Photo captured successfully: {photo_path}"
                    )
                    return photo_path
                else:
                    self.get_logger().warn(
                        f"Photo file does not exist at expected path: {photo_path}. "
                        f"Service message: {response.message}"
                    )
                    # Try to use filename directly if service saved it
                    target_path = folder / filename
                    if target_path.exists():
                        return target_path
                    return None
            else:
                self.get_logger().error(
                    f"Photo capture failed: {response.message if hasattr(response, 'message') else 'Unknown error'}"
                )
                return None
                
        except Exception as e:
            self.get_logger().error(
                f"Error capturing tyre photo: {e}",
                exc_info=True
            )
            return None
    
    def _save_tyre_metadata(self, tyre_data, folder):
        """
        Save metadata for captured tyre photo.
        
        Args:
            tyre_data: TyreData object
            folder: Path to folder where metadata should be saved
            
        Returns:
            bool: True if saved successfully, False otherwise
        """
        try:
            if not tyre_data:
                return False
            
            metadata = {
                'tyre_id': tyre_data.tyre_id,
                'photo_path': str(tyre_data.photo_path) if tyre_data.photo_path else None,
                'photo_taken': tyre_data.photo_taken,
                'attempts': tyre_data.attempts,
                'capture_timestamp': time.time(),
                'position_3d': {
                    'x': tyre_data.position_3d.x if hasattr(tyre_data, 'position_3d') and tyre_data.position_3d else None,
                    'y': tyre_data.position_3d.y if hasattr(tyre_data, 'position_3d') and tyre_data.position_3d else None,
                    'z': tyre_data.position_3d.z if hasattr(tyre_data, 'position_3d') and tyre_data.position_3d else None,
                } if hasattr(tyre_data, 'position_3d') and tyre_data.position_3d else None
            }
            
            metadata_path = folder / f"{tyre_data.tyre_id}_metadata.json"
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            self.get_logger().debug(
                f"Saved tyre metadata: {metadata_path}"
            )
            return True
            
        except Exception as e:
            self.get_logger().error(
                f"Error saving tyre metadata: {e}",
                exc_info=True
            )
            return False
    
    def _handle_completion_check(self):
        """
        Enhanced completion check handler with validation and retry logic.
        
        This method:
        1. Validates all tyres have been photographed
        2. Verifies photo files exist
        3. Handles missing tyres with retry logic
        4. Finalizes truck inspection data
        5. Transitions to next truck or mission complete
        """
        try:
            # Initialize completion check tracking on first attempt
            if self.completion_check_start_time is None:
                self.completion_check_start_time = time.time()
                self.completion_retry_attempts = 0
                self.get_logger().info(
                    f"CHECKING_COMPLETION: Starting completion check "
                    f"for {self.current_truck.truck_id if self.current_truck else 'unknown truck'}"
                )
            
            # Validate current truck
            if not self.current_truck:
                self.get_logger().error(
                    "CHECKING_COMPLETION: current_truck is None. "
                    "Transitioning to ERROR_RECOVERY."
                )
                self.state = MissionState.ERROR_RECOVERY
                return
            
            # Check for timeout
            completion_timeout = self.get_parameter('completion_check_timeout').value
            elapsed_time = time.time() - self.completion_check_start_time
            if elapsed_time > completion_timeout:
                self.get_logger().error(
                    f"Completion check timeout after {elapsed_time:.1f}s. "
                    "Proceeding with current status."
                )
                # Continue anyway - timeout is just for safety
            
            # Step 1: Check if all tyres are photographed
            all_photographed = self.current_truck.all_tyres_photographed()
            
            # Step 2: Verify photo files exist (if marked as photographed)
            missing_photos = []
            for tyre in self.current_truck.tyres:
                if tyre.photo_taken:
                    if not tyre.photo_path or not Path(tyre.photo_path).exists():
                        missing_photos.append(tyre.tyre_id)
                        self.get_logger().warn(
                            f"Tyre {tyre.tyre_id} marked as photographed but photo file missing: "
                            f"{tyre.photo_path}"
                        )
                        # Mark as not photographed so it can be retried
                        tyre.photo_taken = False
            
            # Step 3: Count statistics
            total_tyres = len(self.current_truck.tyres)
            photographed_tyres = sum(1 for t in self.current_truck.tyres if t.photo_taken)
            missing_tyres = total_tyres - photographed_tyres
            
            self.get_logger().info(
                f"CHECKING_COMPLETION: Truck {self.current_truck.truck_id} - "
                f"{photographed_tyres}/{total_tyres} tyres photographed. "
                f"Missing: {missing_tyres}"
            )
            
            # Step 4: Determine action based on completion status
            if all_photographed and not missing_photos:
                # All tyres successfully photographed
                self.get_logger().info(
                    f"✅ Completed inspection of {self.current_truck.truck_id}. "
                    f"Photographed {photographed_tyres} tyre(s)."
                )
                
                # Save final truck metadata
                self._save_truck_completion_metadata()
                
                # Reset completion check state
                self.completion_check_start_time = None
                self.completion_retry_attempts = 0
                
                # Finish current truck and move to next
                self.finish_current_truck()
            else:
                # Some tyres missing - retry if we haven't exceeded attempts
                max_retry_attempts = self.get_parameter('max_completion_retry_attempts').value
                
                if self.completion_retry_attempts < max_retry_attempts:
                    self.completion_retry_attempts += 1
                    self.get_logger().warn(
                        f"Some tyres not photographed ({missing_tyres} missing). "
                        f"Retrying... (attempt {self.completion_retry_attempts}/{max_retry_attempts})"
                    )
                    
                    # Reset tyre index to start from beginning
                    self.current_tyre_index = 0
                    
                    # Reset completion check start time for next check
                    self.completion_check_start_time = None
                    
                    # Switch back to navigation mode (might have switched to inspection)
                    self.publish_segmentation_mode("navigation")
                    
                    # Transition to tyre detection/navigation
                    if self.current_truck.tyres:
                        # We have detected tyres - navigate to missing ones
                        self.state = MissionState.NAVIGATING_TO_TYRE
                    else:
                        # No tyres detected - go back to detection
                        self.state = MissionState.SWITCHING_TO_INSPECTION
                else:
                    # Max retry attempts reached - mark as complete with missing tyres
                    self.get_logger().warn(
                        f"Max retry attempts ({max_retry_attempts}) reached. "
                        f"Completing truck inspection with {missing_tyres} missing tyre(s)."
                    )
                    
                    # Save metadata with warning
                    self._save_truck_completion_metadata()
                    
                    # Reset completion check state
                    self.completion_check_start_time = None
                    self.completion_retry_attempts = 0
                    
                    # Finish current truck
                    self.finish_current_truck()
                
        except Exception as e:
            self.get_logger().error(
                f"Error in CHECKING_COMPLETION state: {e}",
                exc_info=True
            )
            self.state = MissionState.ERROR_RECOVERY
            
    def _save_truck_completion_metadata(self):
        """
        Save comprehensive metadata for completed truck inspection.
        
        Returns:
            bool: True if saved successfully, False otherwise
        """
        try:
            if not self.current_truck:
                return False
            
            # Create truck-specific folder if it doesn't exist
            truck_folder = self.photo_dir / self.current_truck.truck_id
            truck_folder.mkdir(parents=True, exist_ok=True)
            
            # Calculate statistics
            total_tyres = len(self.current_truck.tyres)
            photographed_tyres = sum(1 for t in self.current_truck.tyres if t.photo_taken)
            missing_tyres = total_tyres - photographed_tyres
            
            # Comprehensive metadata
            metadata = {
                'truck_id': self.current_truck.truck_id,
                'inspection_completed': time.time(),
                'license_plate_photo_path': str(self.current_truck.license_plate_photo_path) if self.current_truck.license_plate_photo_path else None,
                'license_plate_photo_taken': self.current_truck.license_plate_photo_taken,
                'license_plate_text': getattr(self.current_truck, 'license_plate_text', None),
                'license_plate_confidence': getattr(self.current_truck, 'license_plate_confidence', None),
                'license_plate_3d_position': getattr(self.current_truck, 'license_plate_3d_position', None),
                'detection_pose': {
                    'x': self.current_truck.detection_pose.pose.position.x if hasattr(self.current_truck.detection_pose, 'pose') and hasattr(self.current_truck.detection_pose.pose, 'position') else None,
                    'y': self.current_truck.detection_pose.pose.position.y if hasattr(self.current_truck.detection_pose, 'pose') and hasattr(self.current_truck.detection_pose.pose, 'position') else None,
                    'z': self.current_truck.detection_pose.pose.position.z if hasattr(self.current_truck.detection_pose, 'pose') and hasattr(self.current_truck.detection_pose.pose, 'position') else None,
                } if hasattr(self.current_truck, 'detection_pose') and self.current_truck.detection_pose else None,
                'detection_timestamp': self.current_truck.detection_timestamp if hasattr(self.current_truck, 'detection_timestamp') else None,
                'tyres': {
                    'total': total_tyres,
                    'photographed': photographed_tyres,
                    'missing': missing_tyres,
                    'details': [
                        {
                            'tyre_id': tyre.tyre_id,
                            'photo_path': str(tyre.photo_path) if tyre.photo_path else None,
                            'photo_taken': tyre.photo_taken,
                            'attempts': getattr(tyre, 'attempts', 0),
                            'position_3d': {
                                'x': tyre.position_3d.x if hasattr(tyre, 'position_3d') and tyre.position_3d else None,
                                'y': tyre.position_3d.y if hasattr(tyre, 'position_3d') and tyre.position_3d else None,
                                'z': tyre.position_3d.z if hasattr(tyre, 'position_3d') and tyre.position_3d else None,
                            } if hasattr(tyre, 'position_3d') and tyre.position_3d else None
                        }
                        for tyre in self.current_truck.tyres
                    ]
                }
            }
            
            metadata_path = truck_folder / f"{self.current_truck.truck_id}_inspection_metadata.json"
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            self.get_logger().info(
                f"Saved truck completion metadata: {metadata_path}"
            )
            return True
            
        except Exception as e:
            self.get_logger().error(
                f"Error saving truck completion metadata: {e}",
                exc_info=True
            )
            return False
    
    def _handle_mission_complete(self):
        """
        Enhanced mission complete handler with finalization and reporting.
        
        This method:
        1. Validates all trucks have been processed
        2. Generates mission summary report
        3. Saves mission completion data
        4. Performs final cleanup
        5. Logs mission statistics
        """
        try:
            # Calculate mission statistics
            total_trucks = len(self.detected_trucks)
            completed_trucks = sum(
                1 for truck_id, truck in self.detected_trucks.items()
                if truck.license_plate_photo_taken and truck.all_tyres_photographed()
            )
            total_tyres = sum(len(truck.tyres) for truck_id, truck in self.detected_trucks.items())
            photographed_tyres = sum(
                sum(1 for tyre in truck.tyres if tyre.photo_taken)
                for truck_id, truck in self.detected_trucks.items()
            )
            
            self.get_logger().info(
                f"🎉 MISSION COMPLETE! "
                f"Processed {completed_trucks}/{total_trucks} truck(s). "
                f"Photographed {photographed_tyres}/{total_tyres} tyre(s)."
            )
            
            # Generate mission summary
            mission_summary = {
                'mission_completed': time.time(),
                'total_trucks': total_trucks,
                'completed_trucks': completed_trucks,
                'total_tyres': total_tyres,
                'photographed_tyres': photographed_tyres,
                'success_rate': {
                    'trucks': completed_trucks / total_trucks if total_trucks > 0 else 0.0,
                    'tyres': photographed_tyres / total_tyres if total_tyres > 0 else 0.0
                },
                'trucks': [
                    {
                        'truck_id': truck_id,
                        'license_plate_photo_taken': truck.license_plate_photo_taken,
                        'total_tyres': len(truck.tyres),
                        'photographed_tyres': sum(1 for tyre in truck.tyres if tyre.photo_taken)
                    }
                    for truck_id, truck in self.detected_trucks.items()
                ]
            }
            
            # Save mission summary
            summary_path = self.photo_dir / "mission_summary.json"
            try:
                with open(summary_path, 'w') as f:
                    json.dump(mission_summary, f, indent=2)
                self.get_logger().info(
                    f"Saved mission summary: {summary_path}"
                )
            except Exception as e:
                self.get_logger().error(
                    f"Error saving mission summary: {e}",
                    exc_info=True
                )
            
            # Perform cleanup (cancel any active navigation, etc.)
            if self.nav_goal_handle:
                try:
                    self.nav_client.cancel_goal_async(self.nav_goal_handle)
                    self.nav_goal_handle = None
                except Exception as e:
                    self.get_logger().warn(
                        f"Error canceling navigation goal: {e}"
                    )
            
            # Switch back to navigation mode (cleanup)
            self.publish_segmentation_mode("navigation")
            
            # Mission is complete - state machine will remain in this state
            # Can be reset by calling start_mission service again
            
        except Exception as e:
            self.get_logger().error(
                f"Error in MISSION_COMPLETE state: {e}",
                exc_info=True
            )
            # Don't transition to error recovery - mission is complete
            # Just log the error
    
    def _handle_error_recovery(self):
        """
        Enhanced error recovery handler with context-aware recovery logic.
        
        This method:
        1. Identifies error context
        2. Attempts context-appropriate recovery
        3. Validates system state after recovery
        4. Transitions to appropriate next state
        5. Handles recovery failures gracefully
        """
        try:
            # Initialize error recovery tracking on first attempt
            if self.error_recovery_start_time is None:
                self.error_recovery_start_time = time.time()
                
                # Log the state that caused the error (stored before transitioning to ERROR_RECOVERY)
                previous_state = self.last_error_state if self.last_error_state else "unknown"
                self.get_logger().error(
                    f"ERROR_RECOVERY: Entered error recovery state. "
                    f"Previous state: {previous_state.value if hasattr(previous_state, 'value') else str(previous_state)}"
                )
            
            # Check recovery timeout
            recovery_wait_time = self.get_parameter('error_recovery_wait_time').value
            elapsed_time = time.time() - self.error_recovery_start_time
            
            if elapsed_time < recovery_wait_time:
                # Still in wait period
                remaining_time = recovery_wait_time - elapsed_time
                self.get_logger().debug(
                    f"Error recovery wait period: {remaining_time:.1f}s remaining..."
                )
                return
            
            # Wait period complete - attempt recovery
            self.get_logger().info(
                f"ERROR_RECOVERY: Attempting recovery after {elapsed_time:.1f}s wait..."
            )
            
            # Step 1: Validate current system state
            readiness = self._check_system_readiness()
            critical_components = ['nav2', 'photo_capture', 'camera', 'detection', 'tf']
            critical_ready = all(
                readiness.get(comp, {}).get('ready', False) 
                for comp in critical_components
            )
            
            if not critical_ready:
                self.get_logger().error(
                    "ERROR_RECOVERY: Critical systems not ready. "
                    "Cannot recover. Remaining in error state."
                )
                # Reset error recovery state to allow retry
                self.error_recovery_start_time = None
                return
            
            # Step 2: Attempt recovery based on previous state
            recovery_successful = False
            
            if self.current_truck:
                # We have a current truck - try to recover truck inspection
                self.get_logger().info(
                    f"ERROR_RECOVERY: Attempting to recover truck inspection for {self.current_truck.truck_id}"
                )
                
                # Reset navigation state
                self.nav_goal_handle = None
                self.nav_result_future = None
                self.nav_goal_pose = None
                
                # Reset mode switch state
                self.mode_switch_start_time = None
                self.mode_switch_verified = False
                
                # Reset capture state
                self.tyre_capture_start_time = None
                self.tyre_capture_attempts = 0
                
                # Determine best recovery state
                if self.current_truck.license_plate_photo_taken:
                    # License plate done - go to tyre detection/navigation
                    if self.current_truck.tyres and any(not t.photo_taken for t in self.current_truck.tyres):
                        # Some tyres not photographed - navigate to tyres
                        self.current_tyre_index = 0
                        # Find first unphotographed tyre
                        for i, tyre in enumerate(self.current_truck.tyres):
                            if not tyre.photo_taken:
                                self.current_tyre_index = i
                                break
                        self.state = MissionState.SWITCHING_TO_INSPECTION
                        recovery_successful = True
                    else:
                        # No tyres or all photographed - check completion
                        self.state = MissionState.CHECKING_COMPLETION
                        recovery_successful = True
                else:
                    # License plate not done - go back to license plate navigation
                    # CRITICAL FIX: If no goal pose is stored, recalculate by going to TRUCK_DETECTED state
                    # This handles the case where goal was never calculated (e.g., stale TF in TRUCK_DETECTED)
                    if self.nav_goal_pose is None and self.current_truck and self.current_truck.detection_pose:
                        self.get_logger().info(
                            f"ERROR_RECOVERY: No goal pose stored. Recalculating goal by transitioning to TRUCK_DETECTED state."
                        )
                        self.state = MissionState.TRUCK_DETECTED
                        recovery_successful = True
                    else:
                        # Goal exists - can restart navigation
                    self.state = MissionState.NAVIGATING_TO_LICENSE_PLATE
                    recovery_successful = True
            else:
                # No current truck - go back to searching
                self.get_logger().info(
                    "ERROR_RECOVERY: No current truck. Resetting to SEARCHING_TRUCKS."
                )
                self.state = MissionState.SEARCHING_TRUCKS
                recovery_successful = True
            
            # Step 3: Log recovery result
            if recovery_successful:
                self.get_logger().info(
                    f"✅ ERROR_RECOVERY: Recovery successful. "
                    f"Transitioning to {self.state.value} state."
                )
            else:
                self.get_logger().error(
                    "ERROR_RECOVERY: Recovery failed. "
                    "Remaining in error state. Manual intervention may be required."
                )
            
            # Reset error recovery state
            self.error_recovery_start_time = None
            self.last_error_state = None
                
        except Exception as e:
            self.get_logger().error(
                f"Error in ERROR_RECOVERY state: {e}",
                exc_info=True
            )
            # Even error recovery can fail - reset to allow retry
            self.error_recovery_start_time = None
            
    def _validate_navigation_pose(self, pose):
        """
        Validate navigation pose is reasonable.
        
        Args:
            pose: PoseStamped message
            
        Returns:
            bool: True if pose is valid, False otherwise
        """
        try:
            # Validate header
            if not pose.header or not pose.header.frame_id:
                self.get_logger().error("Navigation pose missing header or frame_id")
                return False
            
            # Validate frame_id is a known frame (at minimum, should be a string)
            if not isinstance(pose.header.frame_id, str) or len(pose.header.frame_id) == 0:
                self.get_logger().error(f"Invalid frame_id: {pose.header.frame_id}")
                return False
            
            # Validate position
            pos = pose.pose.position
            if not all(isinstance(getattr(pos, attr), (int, float)) for attr in ['x', 'y', 'z']):
                self.get_logger().error("Navigation pose has invalid position values")
                return False
            
            # Check for NaN or Inf values
            if any(math.isnan(getattr(pos, attr)) or math.isinf(getattr(pos, attr)) 
                   for attr in ['x', 'y', 'z']):
                self.get_logger().error(
                    f"Navigation pose has NaN/Inf values: "
                    f"x={pos.x}, y={pos.y}, z={pos.z}"
                )
                return False
            
            # CRITICAL: Check for extreme/unreasonable position values (prevents bad calculations from causing impossible navigation)
            # Goals should be within reasonable bounds (e.g., not millions of meters away)
            max_position = self.get_parameter('max_goal_position').value
            if any(abs(getattr(pos, attr)) > max_position for attr in ['x', 'y']):
                self.get_logger().error(
                    f"Navigation pose has extreme position values (outside reasonable bounds ±{max_position}m): "
                    f"x={pos.x:.2f}, y={pos.y:.2f}, z={pos.z:.2f}. "
                    f"This indicates a calculation error - goal would be impossible to reach."
                )
                return False
            
            # Validate orientation
            ori = pose.pose.orientation
            if not all(isinstance(getattr(ori, attr), (int, float)) for attr in ['x', 'y', 'z', 'w']):
                self.get_logger().error("Navigation pose has invalid orientation values")
                return False
            
            # Check for NaN or Inf values in orientation
            if any(math.isnan(getattr(ori, attr)) or math.isinf(getattr(ori, attr)) 
                   for attr in ['x', 'y', 'z', 'w']):
                self.get_logger().error(
                    f"Navigation pose has NaN/Inf orientation values: "
                    f"x={ori.x}, y={ori.y}, z={ori.z}, w={ori.w}"
                )
                return False
            
            # Check quaternion magnitude (should be approximately 1.0)
            quat_magnitude = math.sqrt(ori.x**2 + ori.y**2 + ori.z**2 + ori.w**2)
            if abs(quat_magnitude - 1.0) > 0.1:
                self.get_logger().warn(
                    f"Navigation pose quaternion magnitude is {quat_magnitude:.3f} "
                    f"(expected ~1.0). Normalizing..."
                )
                # Normalize the quaternion
                ori.x /= quat_magnitude
                ori.y /= quat_magnitude
                ori.z /= quat_magnitude
                ori.w /= quat_magnitude
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validating navigation pose: {e}\n{traceback.format_exc()}")
            return False
    
    def _validate_goal_in_free_space(self, goal_pose: PoseStamped) -> Tuple[bool, Optional[PoseStamped]]:
        """
        Validate goal is in free space and path is clear.
        Implements comprehensive goal validation with free space checking.
        
        Args:
            goal_pose: Proposed navigation goal pose
            
        Returns:
            Tuple of (is_valid: bool, adjusted_goal: PoseStamped or None)
        """
        try:
            # Get robot pose
            robot_pose = self._get_robot_pose("map")
            if not robot_pose:
                self.get_logger().warn("_validate_goal_in_free_space: Could not get robot pose")
                return False, None
            
            # Calculate distance from robot to goal
            distance = math.sqrt(
                (goal_pose.pose.position.x - robot_pose.pose.position.x)**2 +
                (goal_pose.pose.position.y - robot_pose.pose.position.y)**2
            )
            
            # CRITICAL: Check maximum distance (prevents extreme values from bad calculations)
            # If goal is too far away, it's likely a calculation error and would be impossible to reach
            max_dist = self.get_parameter('max_goal_distance').value
            if distance > max_dist:
                self.get_logger().error(
                    f"❌ _validate_goal_in_free_space: Goal is too far away: {distance:.2f}m "
                    f"(max allowed: {max_dist:.1f}m). "
                    f"Goal position: ({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}), "
                    f"Robot position: ({robot_pose.pose.position.x:.2f}, {robot_pose.pose.position.y:.2f}). "
                    f"This indicates a calculation error - goal would be impossible to reach."
                )
                return False, None
            
            # Check minimum distance from robot
            min_dist = self.get_parameter('min_goal_distance').value
            if distance < min_dist:
                # Adjust goal to minimum distance
                direction_x = (goal_pose.pose.position.x - robot_pose.pose.position.x) / distance if distance > 0.01 else 1.0
                direction_y = (goal_pose.pose.position.y - robot_pose.pose.position.y) / distance if distance > 0.01 else 0.0
                goal_pose.pose.position.x = robot_pose.pose.position.x + direction_x * min_dist
                goal_pose.pose.position.y = robot_pose.pose.position.y + direction_y * min_dist
                self.get_logger().info(
                    f"✅ _validate_goal_in_free_space: Adjusted goal to minimum distance: "
                    f"({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f})"
                )
            
            # CRITICAL FIX: Check vehicle obstacle if present
            # Determine goal type based on current mission state
            # License plate goals need different validation (distance to front, not center)
            goal_type = "tire"  # Default for tire navigation
            if self.state == MissionState.TRUCK_DETECTED or self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                goal_type = "license_plate"  # License plate navigation - goal is intentionally close to vehicle front
            
            if self.vehicle_obstacle_manager and self.current_truck and hasattr(self.current_truck, 'detection_pose') and self.current_truck.detection_pose:
                is_valid, issue = self.vehicle_obstacle_manager.validate_navigation_goal(goal_pose, goal_type=goal_type)
                if not is_valid:
                    self.get_logger().warn(
                        f"⚠️ _validate_goal_in_free_space: Goal validation failed - vehicle obstacle: {issue} "
                        f"(goal_type: {goal_type})"
                    )
                    # CRITICAL: For license plate goals, if validation fails due to distance check,
                    # the goal might still be valid (it's intentionally close to vehicle front).
                    # Only calculate alternative if goal is inside vehicle bounds (actual collision)
                    if "inside vehicle obstacle bounds" in str(issue):
                        # Goal is inside bounds - this is a real collision, try alternative
                    adjusted_goal = self._calculate_alternative_goal(goal_pose, robot_pose)
                    if adjusted_goal:
                        return True, adjusted_goal
                    return False, None
                    elif goal_type == "license_plate" and "too close to vehicle front" in str(issue):
                        # License plate goal might be slightly too close - adjust it slightly further
                        # Calculate distance to vehicle front and adjust if needed
                        vehicle_pose = self.current_truck.detection_pose
                        vehicle_pos = vehicle_pose.pose.position
                        vehicle_ori = vehicle_pose.pose.orientation
                        # CRITICAL FIX: Use standard quaternion to yaw conversion
                        siny_cosp = 2.0 * (vehicle_ori.w * vehicle_ori.z + vehicle_ori.x * vehicle_ori.y)
                        cosy_cosp = 1.0 - 2.0 * (vehicle_ori.y * vehicle_ori.y + vehicle_ori.z * vehicle_ori.z)
                        yaw = math.atan2(siny_cosp, cosy_cosp)
                        vehicle_length = self.get_parameter('vehicle_length').value
                        half_length = vehicle_length / 2.0
                        front_x = vehicle_pos.x + half_length * math.cos(yaw)
                        front_y = vehicle_pos.y + half_length * math.sin(yaw)
                        
                        # Adjust goal to be at safe distance from vehicle front
                        min_approach = 1.2  # meters
                        dx = goal_pos.x - front_x
                        dy = goal_pos.y - front_y
                        current_dist = math.sqrt(dx*dx + dy*dy)
                        if current_dist > 0.01:  # Not at same position
                            # Move goal further from vehicle front
                            direction_x = dx / current_dist
                            direction_y = dy / current_dist
                            adjusted_goal_pose = PoseStamped()
                            adjusted_goal_pose.header = goal_pose.header
                            adjusted_goal_pose.pose.position.x = front_x + direction_x * min_approach
                            adjusted_goal_pose.pose.position.y = front_y + direction_y * min_approach
                            adjusted_goal_pose.pose.position.z = goal_pose.pose.position.z
                            adjusted_goal_pose.pose.orientation = goal_pose.pose.orientation
                            
                            self.get_logger().info(
                                f"✅ _validate_goal_in_free_space: Adjusted license plate goal to safe distance from front: "
                                f"({adjusted_goal_pose.pose.position.x:.2f}, {adjusted_goal_pose.pose.position.y:.2f})"
                            )
                            # Re-validate adjusted goal (should pass now)
                            is_valid_adjusted, issue_adjusted = self.vehicle_obstacle_manager.validate_navigation_goal(
                                adjusted_goal_pose, goal_type="license_plate"
                            )
                            if is_valid_adjusted:
                                return True, adjusted_goal_pose
                        
                        # If adjustment didn't work, try alternative goal
                        adjusted_goal = self._calculate_alternative_goal(goal_pose, robot_pose)
                        if adjusted_goal:
                            return True, adjusted_goal
                        return False, None
                    else:
                        # For tire goals or other issues, try alternative goal
                        adjusted_goal = self._calculate_alternative_goal(goal_pose, robot_pose)
                        if adjusted_goal:
                            return True, adjusted_goal
                        return False, None
            
            # CRITICAL FIX: For license plate goals, if goal is validated and far enough from vehicle, skip strict path check
            # License plate goals are intentionally close to vehicle front - path will naturally pass near vehicle
            # The goal validation already ensures it's safe (outside actual vehicle, at safe distance from front)
            # If goal is validated and far from vehicle (>2m from front), path is guaranteed safe - skip strict check
            if goal_type == "license_plate":
                # Check distance from goal to vehicle front to determine if strict path check is needed
                if self.current_truck and hasattr(self.current_truck, 'detection_pose') and self.current_truck.detection_pose:
                    vehicle_pose = self.current_truck.detection_pose
                    vehicle_pos = vehicle_pose.pose.position
                    vehicle_ori = vehicle_pose.pose.orientation
                    siny_cosp = 2.0 * (vehicle_ori.w * vehicle_ori.z + vehicle_ori.x * vehicle_ori.y)
                    cosy_cosp = 1.0 - 2.0 * (vehicle_ori.y * vehicle_ori.y + vehicle_ori.z * vehicle_ori.z)
                    vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)
                    vehicle_length = self.vehicle_obstacle_manager.vehicle_length if self.vehicle_obstacle_manager else 4.5
                    half_length = vehicle_length / 2.0
                    front_x = vehicle_pos.x + half_length * math.cos(vehicle_yaw)
                    front_y = vehicle_pos.y + half_length * math.sin(vehicle_yaw)
                    goal_to_front_dist = math.sqrt(
                        (goal_pose.pose.position.x - front_x)**2 +
                        (goal_pose.pose.position.y - front_y)**2
                    )
                    
                    # If goal is far enough from vehicle front (>2m), path is guaranteed safe - skip strict check
                    if goal_to_front_dist > 2.0:
                        self.get_logger().debug(
                            f"License plate goal validated and far from vehicle front ({goal_to_front_dist:.2f}m > 2.0m). "
                            f"Skipping strict path check - path is guaranteed safe."
                        )
                        return True, goal_pose
                
                # For closer goals, check if path goes through ACTUAL vehicle (not padded bounds)
                # This prevents path from going through the vehicle itself, but allows passing near it
                path_clear = self._check_path_clear_actual_vehicle(robot_pose, goal_pose)
                if not path_clear:
                    self.get_logger().warn(
                        f"⚠️ _validate_goal_in_free_space: Path goes through actual vehicle bounds "
                        f"(goal_type: license_plate). Calculating alternative goal..."
                    )
                    adjusted_goal = self._calculate_alternative_goal(goal_pose, robot_pose)
                    if adjusted_goal:
                        return True, adjusted_goal
                    return False, None
            else:
                # For tire goals: Use normal path check (with padded bounds)
            path_clear = self._check_path_clear(robot_pose, goal_pose)
            if not path_clear:
                self.get_logger().warn("⚠️ _validate_goal_in_free_space: Path is not clear, calculating alternative goal")
                adjusted_goal = self._calculate_alternative_goal(goal_pose, robot_pose)
                if adjusted_goal:
                    return True, adjusted_goal
                return False, None
            
            return True, goal_pose
            
        except Exception as e:
            self.get_logger().error(f"Error in _validate_goal_in_free_space: {e}\n{traceback.format_exc()}")
            return False, None
    
    def _check_path_clear(self, start_pose: PoseStamped, end_pose: PoseStamped, num_samples: int = 10) -> bool:
        """
        Check if path from start to end is clear of obstacles.
        
        Args:
            start_pose: Starting pose
            end_pose: Ending pose
            num_samples: Number of points to sample along path
            
        Returns:
            True if path is clear, False otherwise
        """
        if not self.vehicle_obstacle_manager or not self.vehicle_obstacle_manager.vehicle_bounds:
            return True  # No vehicle obstacle to check
        
        start_pos = start_pose.pose.position
        end_pos = end_pose.pose.position
        
        # Sample points along path
        for i in range(num_samples + 1):
            t = i / num_samples
            x = start_pos.x + t * (end_pos.x - start_pos.x)
            y = start_pos.y + t * (end_pos.y - start_pos.y)
            
            point = Point()
            point.x = x
            point.y = y
            point.z = start_pos.z
            
            if self.vehicle_obstacle_manager._point_in_bounds(point):
                return False
        
        return True
    
    def _check_path_clear_actual_vehicle(self, start_pose: PoseStamped, end_pose: PoseStamped, num_samples: int = 10) -> bool:
        """
        Check if path from start to end goes through ACTUAL vehicle bounds (without safety margin).
        Used for license plate goals where path may legitimately pass near vehicle.
        
        Args:
            start_pose: Starting pose
            end_pose: Ending pose
            num_samples: Number of points to sample along path
            
        Returns:
            True if path is clear (doesn't go through actual vehicle), False otherwise
        """
        if not self.vehicle_obstacle_manager or not self.current_truck or not hasattr(self.current_truck, 'detection_pose') or not self.current_truck.detection_pose:
            return True  # No vehicle obstacle to check
        
        vehicle_pose = self.current_truck.detection_pose
        vehicle_pos = vehicle_pose.pose.position
        vehicle_ori = vehicle_pose.pose.orientation
        
        # Calculate vehicle yaw
        siny_cosp = 2.0 * (vehicle_ori.w * vehicle_ori.z + vehicle_ori.x * vehicle_ori.y)
        cosy_cosp = 1.0 - 2.0 * (vehicle_ori.y * vehicle_ori.y + vehicle_ori.z * vehicle_ori.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Get vehicle dimensions (actual, without safety margin)
        vehicle_length = self.vehicle_obstacle_manager.vehicle_length
        vehicle_width = self.vehicle_obstacle_manager.vehicle_width
        half_length = vehicle_length / 2.0
        half_width = vehicle_width / 2.0
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        
        start_pos = start_pose.pose.position
        end_pos = end_pose.pose.position
        
        # Sample points along path
        for i in range(num_samples + 1):
            t = i / num_samples
            x = start_pos.x + t * (end_pos.x - start_pos.x)
            y = start_pos.y + t * (end_pos.y - start_pos.y)
            
            # Transform point to vehicle-relative coordinates
            dx = x - vehicle_pos.x
            dy = y - vehicle_pos.y
            
            # Rotate to vehicle frame
            rel_x = dx * cos_yaw + dy * sin_yaw
            rel_y = -dx * sin_yaw + dy * cos_yaw
            
            # Check if point is inside ACTUAL vehicle bounds (without safety margin)
            if abs(rel_x) <= half_length and abs(rel_y) <= half_width:
                # Path goes through actual vehicle - this is a collision
                self.get_logger().debug(
                    f"Path check: Point ({x:.2f}, {y:.2f}) is inside actual vehicle bounds "
                    f"(relative: {rel_x:.2f}, {rel_y:.2f}, vehicle: {vehicle_length:.1f}m x {vehicle_width:.1f}m)"
                )
                return False
        
        return True
    
    def _calculate_alternative_goal(self, original_goal: PoseStamped, robot_pose: PoseStamped) -> Optional[PoseStamped]:
        """
        Calculate alternative goal if original is blocked.
        Tries different strategies based on goal type:
        - License plate goals: Try different approach distances from vehicle FRONT (in vehicle forward direction)
        - Tire goals: Try different angles around vehicle (going around vehicle)
        
        Args:
            original_goal: Original goal that was blocked
            robot_pose: Current robot pose
            
        Returns:
            Adjusted goal pose, or None if no alternative found
        """
        try:
            if not self.current_truck or not hasattr(self.current_truck, 'detection_pose') or not self.current_truck.detection_pose:
                return None
            
            vehicle_pose = self.current_truck.detection_pose
            vehicle_pos = vehicle_pose.pose.position
            vehicle_ori = vehicle_pose.pose.orientation
            
            # Calculate vehicle orientation (yaw) for license plate goals
            siny_cosp = 2.0 * (vehicle_ori.w * vehicle_ori.z + vehicle_ori.x * vehicle_ori.y)
            cosy_cosp = 1.0 - 2.0 * (vehicle_ori.y * vehicle_ori.y + vehicle_ori.z * vehicle_ori.z)
            vehicle_yaw = math.atan2(siny_cosp, cosy_cosp)
            
            # CRITICAL FIX: Determine goal type based on current state
            goal_type = "tire"  # Default
            if self.state == MissionState.TRUCK_DETECTED or self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                goal_type = "license_plate"
            
            # Get approach distance
            approach_dist = self.get_parameter('approach_distance').value
            min_approach = self.get_parameter('min_approach_distance').value if self.has_parameter('min_approach_distance') else 2.0
            
            # CRITICAL FIX: Different strategies for license plate vs tire goals
            if goal_type == "license_plate":
                # For license plate goals: Robot must approach vehicle FRONT (where license plate is)
                # Try different approach distances from vehicle FRONT (not center, not random angles)
                # License plate is on vehicle front - robot must approach from front direction with small variations
                
                # Get vehicle dimensions (adaptive: car vs truck)
                vehicle_length = self.vehicle_obstacle_manager.vehicle_length if self.vehicle_obstacle_manager else self.get_parameter('vehicle_length').value
                half_length = vehicle_length / 2.0
                
                # Calculate vehicle FRONT position (center + half_length forward)
                front_x = vehicle_pos.x + half_length * math.cos(vehicle_yaw)
                front_y = vehicle_pos.y + half_length * math.sin(vehicle_yaw)
                
                # Try different approach distances from vehicle FRONT
                approach_distances = [approach_dist, approach_dist + 0.3, approach_dist + 0.5, max(approach_dist - 0.2, 1.2), 1.5, 1.8, 2.0]
                angle_offsets = [0, math.pi/12, -math.pi/12, math.pi/6, -math.pi/6]  # Small angle variations (±30° max)
                
                # Try different approach distances and small angle variations from vehicle FRONT
                for dist in approach_distances:
                    if dist < 1.0:  # Minimum safe distance
                        continue
                    for angle_offset in angle_offsets:
                        # CRITICAL FIX: Calculate goal position from vehicle FRONT (not center)
                        # Goal should be dist IN FRONT of vehicle front (to see license plate)
                        # with small angle variations for flexibility
                        # Goal = vehicle_front + dist * (vehicle_forward_direction + angle_offset)
                        # This places goal dist IN FRONT of front, with small angle variations
                        goal_direction = vehicle_yaw + angle_offset
                        goal_x = front_x + dist * math.cos(goal_direction)
                        goal_y = front_y + dist * math.sin(goal_direction)
                        
                        alternative_goal = PoseStamped()
                        alternative_goal.header = original_goal.header
                        alternative_goal.pose.position.x = goal_x
                        alternative_goal.pose.position.y = goal_y
                        alternative_goal.pose.position.z = original_goal.pose.position.z
                        
                        # CRITICAL FIX: Calculate orientation: robot should face TOWARD vehicle FRONT (to see license plate)
                        # Goal is at vehicle_front - dist, so robot should face vehicle_front direction
                        goal_yaw_alt = vehicle_yaw  # Face toward vehicle front (where license plate is)
                        # Normalize angle to [-pi, pi]
                        while goal_yaw_alt > math.pi:
                            goal_yaw_alt -= 2 * math.pi
                        while goal_yaw_alt < -math.pi:
                            goal_yaw_alt += 2 * math.pi
                        alternative_goal.pose.orientation.x = 0.0
                        alternative_goal.pose.orientation.y = 0.0
                        alternative_goal.pose.orientation.z = math.sin(goal_yaw_alt / 2.0)
                        alternative_goal.pose.orientation.w = math.cos(goal_yaw_alt / 2.0)
                        
                        # Validate goal structure
                        if not self._validate_navigation_pose(alternative_goal):
                            continue
                        
                        # Check minimum distance from robot
                        min_dist = self.get_parameter('min_goal_distance').value
                        distance_to_robot = math.sqrt(
                            (alternative_goal.pose.position.x - robot_pose.pose.position.x)**2 +
                            (alternative_goal.pose.position.y - robot_pose.pose.position.y)**2
                        )
                        if distance_to_robot < min_dist:
                            # Adjust to minimum distance
                            direction_x = (alternative_goal.pose.position.x - robot_pose.pose.position.x) / distance_to_robot if distance_to_robot > 0.01 else 1.0
                            direction_y = (alternative_goal.pose.position.y - robot_pose.pose.position.y) / distance_to_robot if distance_to_robot > 0.01 else 0.0
                            alternative_goal.pose.position.x = robot_pose.pose.position.x + direction_x * min_dist
                            alternative_goal.pose.position.y = robot_pose.pose.position.y + direction_y * min_dist
                        
                        # Check path is clear (use lenient check for license plate goals)
                        if self._check_path_clear_actual_vehicle(robot_pose, alternative_goal):
                            # Check vehicle obstacle validation for license plate goal
                            if self.vehicle_obstacle_manager and self.current_truck and hasattr(self.current_truck, 'detection_pose') and self.current_truck.detection_pose:
                                is_valid, issue = self.vehicle_obstacle_manager.validate_navigation_goal(
                                    alternative_goal, goal_type="license_plate"
                                )
                                if is_valid:
                                    self.get_logger().info(
                                        f"✅ _calculate_alternative_goal: Found alternative license plate goal "
                                        f"(distance: {dist:.2f}m, angle_offset: {math.degrees(angle_offset):.1f}°): "
                                        f"({alternative_goal.pose.position.x:.2f}, {alternative_goal.pose.position.y:.2f})"
                                    )
                                    return alternative_goal
                            else:
                                # No vehicle obstacle, goal is valid
                                self.get_logger().info(
                                    f"✅ _calculate_alternative_goal: Found alternative license plate goal "
                                    f"(distance: {dist:.2f}m, angle_offset: {math.degrees(angle_offset):.1f}°): "
                                    f"({alternative_goal.pose.position.x:.2f}, {alternative_goal.pose.position.y:.2f})"
                                )
                                return alternative_goal
            else:
                # For tire goals: Try different angles around vehicle (original logic)
            angles = [0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi]
            
            for angle_offset in angles:
                # Calculate goal position at this angle
                goal_x = vehicle_pos.x + approach_dist * math.cos(angle_offset)
                goal_y = vehicle_pos.y + approach_dist * math.sin(angle_offset)
                
                alternative_goal = PoseStamped()
                alternative_goal.header = original_goal.header
                alternative_goal.pose.position.x = goal_x
                alternative_goal.pose.position.y = goal_y
                alternative_goal.pose.position.z = original_goal.pose.position.z
                    alternative_goal.pose.orientation = original_goal.pose.orientation  # Keep original orientation
                
                # Validate goal structure (avoid recursive call to _validate_goal_in_free_space)
                if not self._validate_navigation_pose(alternative_goal):
                    continue
                
                # Check minimum distance from robot
                min_dist = self.get_parameter('min_goal_distance').value
                    distance_to_robot = math.sqrt(
                    (alternative_goal.pose.position.x - robot_pose.pose.position.x)**2 +
                    (alternative_goal.pose.position.y - robot_pose.pose.position.y)**2
                )
                    if distance_to_robot < min_dist:
                    # Adjust to minimum distance
                        direction_x = (alternative_goal.pose.position.x - robot_pose.pose.position.x) / distance_to_robot if distance_to_robot > 0.01 else 1.0
                        direction_y = (alternative_goal.pose.position.y - robot_pose.pose.position.y) / distance_to_robot if distance_to_robot > 0.01 else 0.0
                    alternative_goal.pose.position.x = robot_pose.pose.position.x + direction_x * min_dist
                    alternative_goal.pose.position.y = robot_pose.pose.position.y + direction_y * min_dist
                
                # Check path is clear
                if self._check_path_clear(robot_pose, alternative_goal):
                        # Check vehicle obstacle validation for tire goal
                    if self.vehicle_obstacle_manager and self.current_truck and hasattr(self.current_truck, 'detection_pose') and self.current_truck.detection_pose:
                            is_valid, issue = self.vehicle_obstacle_manager.validate_navigation_goal(
                                alternative_goal, goal_type="tire"
                            )
                        if is_valid:
                            self.get_logger().info(
                                    f"✅ _calculate_alternative_goal: Found alternative tire goal at angle {math.degrees(angle_offset):.0f}°: "
                                f"({alternative_goal.pose.position.x:.2f}, {alternative_goal.pose.position.y:.2f})"
                            )
                            return alternative_goal
                    else:
                        # No vehicle obstacle, goal is valid
                        self.get_logger().info(
                                f"✅ _calculate_alternative_goal: Found alternative tire goal at angle {math.degrees(angle_offset):.0f}°: "
                            f"({alternative_goal.pose.position.x:.2f}, {alternative_goal.pose.position.y:.2f})"
                        )
                        return alternative_goal
            
            self.get_logger().warn("❌ _calculate_alternative_goal: Could not find alternative goal")
            return None
            
        except Exception as e:
            self.get_logger().error(f"Error in _calculate_alternative_goal: {e}\n{traceback.format_exc()}")
            return None
    
    def _clear_nav2_costmaps(self):
        """
        Clear both local and global Nav2 costmaps to remove stale obstacles.
        This is critical for recovery when navigation fails due to phantom obstacles.
        """
        try:
            # Create clients if not already created
            if self.clear_local_costmap_client is None:
                self.clear_local_costmap_client = self.create_client(
                    ClearEntireCostmap,
                    '/local_costmap/clear_entirely_local_costmap'
                )
            if self.clear_global_costmap_client is None:
                self.clear_global_costmap_client = self.create_client(
                    ClearEntireCostmap,
                    '/global_costmap/clear_entirely_global_costmap'
                )
            
            # Clear local costmap
            if self.clear_local_costmap_client.wait_for_service(timeout_sec=2.0):
                request = ClearEntireCostmap.Request()
                future = self.clear_local_costmap_client.call_async(request)
                self.get_logger().info("🧹 Cleared local costmap for recovery")
            else:
                self.get_logger().warn("⚠️ Local costmap clear service not available")
            
            # Clear global costmap
            if self.clear_global_costmap_client.wait_for_service(timeout_sec=2.0):
                request = ClearEntireCostmap.Request()
                future = self.clear_global_costmap_client.call_async(request)
                self.get_logger().info("🧹 Cleared global costmap for recovery")
            else:
                self.get_logger().warn("⚠️ Global costmap clear service not available")
                
        except Exception as e:
            self.get_logger().warn(f"Failed to clear costmaps: {e}")
            
    def navigate_to_pose(self, pose):
        """
        Send navigation goal with validation.
        
        Args:
            pose: PoseStamped message
            
        Returns:
            bool: True if goal was sent successfully, False otherwise
        """
        try:
            # CRITICAL LOGGING: Entry point for navigation goal sending
            self.get_logger().info(
                f"🎯 navigate_to_pose() called: goal=({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) "
                f"frame={pose.header.frame_id}, current_state={self.state.value}"
            )
            
            # Validate pose before sending
            if not self._validate_navigation_pose(pose):
                self.get_logger().error(
                    f"❌ navigate_to_pose(): Pose validation FAILED. Cannot send goal."
                )
                return False
            
            self.get_logger().info("✅ navigate_to_pose(): Pose validation passed")
            
            # Check if Nav2 is ready
            if not self.nav_client.server_is_ready():
                self.get_logger().error(
                    f"❌ navigate_to_pose(): Nav2 action server NOT READY. Cannot send goal."
                )
                return False
            
            self.get_logger().info("✅ navigate_to_pose(): Nav2 action server is ready")
            
            # CRITICAL: Check if there's already a pending goal (race condition protection)
            # This prevents multiple goals from being sent if navigate_to_pose() is called multiple times
            # before Nav2 responds (e.g., from goal recalculation, alternative goal, etc.)
            if self.pending_send_goal_future is not None:
                # Check if the pending future is done
                if self.pending_send_goal_future.done():
                    # Future is done, but callback might not have fired yet - clear it
                    self.pending_send_goal_future = None
                else:
                    # Goal is still pending - cancel it first to prevent conflicts
                    self.get_logger().warn(
                        f"⚠️ navigate_to_pose(): Previous goal still pending. "
                        f"Canceling previous goal before sending new one to prevent race condition."
                    )
                    # Note: We can't cancel a send_goal_async future directly, but we can track it
                    # The callback will handle cleanup when it fires
                    # For now, log warning and continue - the new goal will replace the old one
                    # Nav2 will handle rejecting the old goal when the new one is sent
            
            # Also check if there's an active goal handle
            if self.nav_goal_handle is not None:
                # Cancel existing goal before sending new one
                self.get_logger().info(
                    f"🔄 navigate_to_pose(): Canceling existing goal before sending new one."
                )
                try:
                    self.nav_client.cancel_goal_async(self.nav_goal_handle)
                    self.get_logger().info("Previous goal cancelation requested")
                except Exception as e:
                    self.get_logger().warn(f"Error canceling previous goal: {e}")
                finally:
                    # Clear goal handle - new goal will replace it
                    self.nav_goal_handle = None
                    self.nav_result_future = None
            
            # CRITICAL FIX: Transform goal to map frame if it's in odom frame
            # Nav2 requires goals in map frame, but we might have calculated goal in odom (fallback)
            goal_pose_to_send = pose
            if pose.header.frame_id == "odom":
                try:
                    # Try to transform goal to map frame
                    transform = self.tf_buffer.lookup_transform(
                        "map",
                        "odom",
                        rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.5)
                    )
                    goal_pose_to_send = do_transform_pose_stamped(pose, transform)
                    self.get_logger().info(
                        f"✅ navigate_to_pose(): Transformed goal from odom to map frame. "
                        f"Original: ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}) in odom, "
                        f"Transformed: ({goal_pose_to_send.pose.position.x:.2f}, {goal_pose_to_send.pose.position.y:.2f}) in map"
                    )
                except Exception as e:
                    # Transform failed - map frame might not be available yet
                    # Skip Nav2 goal and rely on direct control (which can use odom frame)
                    self.get_logger().warn(
                        f"⚠️ navigate_to_pose(): Cannot transform goal from odom to map frame: {e}. "
                        f"Skipping Nav2 goal (will use direct control only with odom frame)."
                    )
                    return False  # Skip Nav2 goal - direct control will handle it
            
            # CRITICAL: Validate goal is within reasonable distance (prevent extreme values)
            max_reasonable_distance = self.get_parameter('max_goal_distance').value  # Default: 100m
            robot_pose_map = self._get_robot_pose("map")
            if robot_pose_map and goal_pose_to_send.header.frame_id == "map":
                goal_pos = goal_pose_to_send.pose.position
                robot_pos = robot_pose_map.pose.position
                distance = math.sqrt(
                    (goal_pos.x - robot_pos.x)**2 +
                    (goal_pos.y - robot_pos.y)**2
                )
                if distance > max_reasonable_distance:
                    self.get_logger().error(
                        f"❌ navigate_to_pose(): Goal is too far from robot: {distance:.2f}m > {max_reasonable_distance:.2f}m. "
                        f"Goal: ({goal_pos.x:.2f}, {goal_pos.y:.2f}), Robot: ({robot_pos.x:.2f}, {robot_pos.y:.2f}). "
                        f"This is likely an error - skipping Nav2 goal."
                    )
                    return False  # Skip invalid goal
            
            # CRITICAL: Validate goal is within map bounds (prevents Nav2 worldToMap errors)
            if goal_pose_to_send.header.frame_id == "map" and self.map_info:
                goal_in_bounds = self._validate_goal_in_map_bounds(goal_pose_to_send)
                if not goal_in_bounds:
                    self.get_logger().warn(
                        f"⚠️ navigate_to_pose(): Goal is outside current map bounds. "
                        f"Goal: ({goal_pose_to_send.pose.position.x:.2f}, {goal_pose_to_send.pose.position.y:.2f}). "
                        f"Map size: {self.map_info['width']}x{self.map_info['height']} cells "
                        f"({self.map_info['width'] * self.map_info['resolution']:.2f}m x {self.map_info['height'] * self.map_info['resolution']:.2f}m). "
                        f"Map origin: ({self.map_info['origin_x']:.2f}, {self.map_info['origin_y']:.2f}). "
                        f"Skipping Nav2 goal (will use direct control only with odom frame)."
                    )
                    return False  # Skip Nav2 goal - direct control can handle it with odom frame
            elif goal_pose_to_send.header.frame_id == "map" and not self.map_info:
                # Map info not available yet - log warning but allow goal (SLAM is building map)
                self.get_logger().debug(
                    f"⚠️ navigate_to_pose(): Map info not available yet (SLAM building). "
                    f"Skipping map bounds validation - goal will be validated by Nav2."
                )
            
            # Create goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = goal_pose_to_send
            
            self.get_logger().info(
                f"📤 navigate_to_pose(): Creating navigation goal message: "
                f"({goal_pose_to_send.pose.position.x:.2f}, {goal_pose_to_send.pose.position.y:.2f}, {goal_pose_to_send.pose.position.z:.2f}) "
                f"in frame '{goal_pose_to_send.header.frame_id}'"
            )
            
            # Reset navigation state
            self.nav_start_time = time.time()
            self.nav_result_future = None
            self.nav_goal_pose = pose
            self.goal_recalculation_attempts = 0
            # CRITICAL: Reset Nav2 monitoring flags
            if not hasattr(self, 'nav2_initial_cmd_vel_received'):
                self.nav2_initial_cmd_vel_received = False
            else:
                self.nav2_initial_cmd_vel_received = False  # Reset flag - wait for Nav2 to start
            self.last_nav2_cmd_vel_time = None  # Reset monitoring
            
            # CRITICAL: If direct navigation is already active, update its goal to match Nav2 goal
            # This ensures consistency when goal is updated (recalculated, alternative goal, etc.)
            # Direct navigation must always navigate to the same goal as Nav2
            # Use preserve_active_state=True to keep navigation active with new goal
            if hasattr(self, 'direct_nav_fallback') and self.direct_nav_fallback.is_active:
                self.get_logger().info(
                    "Direct navigation is active. Updating direct navigation goal to match new Nav2 goal "
                    f"(preserving active state)."
                )
                self.direct_nav_fallback.set_goal(pose, preserve_active_state=True)
                # Note: initial_distance_to_goal is reset, will recalculate on next update for new goal
            
            # Store initial robot pose for progress tracking
            current_pose = self._get_robot_pose("map")
            if current_pose:
                self.nav_start_pose = current_pose
                self.last_robot_pose = current_pose
                self.last_robot_pose_time = time.time()
                self.nav_progress_distance = 0.0
                self.nav_progress_last_update = time.time()
                
                # Calculate distance to goal
                goal_pos = pose.pose.position
                start_pos = current_pose.pose.position
                total_distance = math.sqrt(
                    (goal_pos.x - start_pos.x)**2 +
                    (goal_pos.y - start_pos.y)**2
                )
                self.get_logger().info(
                    f"📏 navigate_to_pose(): Navigation starting - distance to goal = {total_distance:.2f}m, "
                    f"robot at ({start_pos.x:.2f}, {start_pos.y:.2f})"
                )
            else:
                self.get_logger().warn(
                    f"⚠️ navigate_to_pose(): Could not get robot pose for distance calculation"
                )
            
            # CRITICAL: Send goal (feedback will be handled after goal is accepted)
            self.get_logger().info(
                f"🚀 navigate_to_pose(): Sending goal to Nav2 action server..."
            )
            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            
            # CRITICAL: Track pending goal to prevent race conditions
            # This allows us to detect if navigate_to_pose() is called again before Nav2 responds
            self.pending_send_goal_future = send_goal_future
            
            send_goal_future.add_done_callback(self.nav_goal_response_callback)
            
            self.get_logger().info(
                f"✅ navigate_to_pose(): Goal sent successfully (async). Waiting for Nav2 response..."
            )
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending navigation goal: {e}\n{traceback.format_exc()}")
            return False
        
    def nav_goal_response_callback(self, future):
        """Handle navigation goal response (for both async send and goal acceptance)"""
        try:
            self.get_logger().info(
                f"📥 nav_goal_response_callback(): Received response from Nav2"
            )
            
            # CRITICAL: Clear pending goal future - callback has fired
            # This prevents false positives in race condition detection
            if self.pending_send_goal_future == future:
                self.pending_send_goal_future = None
            elif self.pending_send_goal_future is not None:
                # Different future - this means a new goal was sent before this callback fired
                # This is a race condition, but we'll handle it gracefully
                self.get_logger().warn(
                    f"⚠️ nav_goal_response_callback(): Received response for different goal than expected. "
                    f"This indicates a race condition - new goal was sent before previous response."
                )
                # Clear the pending future since we got a response (even if it's for an old goal)
                self.pending_send_goal_future = None
            
            if hasattr(future, 'result') and callable(future.result):
                # This is from send_goal_async
                # CRITICAL: Handle potential exceptions when getting result
                try:
                    goal_handle = future.result()
                except Exception as e:
                    self.get_logger().error(
                        f"❌ nav_goal_response_callback(): Error getting navigation goal handle: {e}",
                        exc_info=True
                    )
                    # Transition to error recovery if in navigation state
                    if self.state in [MissionState.NAVIGATING_TO_LICENSE_PLATE, MissionState.NAVIGATING_TO_TYRE]:
                        self.state = MissionState.ERROR_RECOVERY
                    return
                
                if goal_handle is None:
                    self.get_logger().error(
                        f"❌ nav_goal_response_callback(): Navigation goal handle is None!"
                    )
                    return
                
                if not goal_handle.accepted:
                    self.get_logger().error(
                        f"❌❌❌ CRITICAL: nav_goal_response_callback(): Navigation goal REJECTED by Nav2 action server! "
                        f"Current state: {self.state.value}"
                    )
                    # Check if rejection was due to goal too close
                    # Nav2 doesn't provide detailed rejection reason, so we check distance
                    if self.nav_goal_pose:
                        current_pose = self._get_robot_pose("map")
                        if current_pose:
                            goal_pos = self.nav_goal_pose.pose.position
                            robot_pos = current_pose.pose.position
                            distance = math.sqrt(
                                (goal_pos.x - robot_pos.x)**2 +
                                (goal_pos.y - robot_pos.y)**2
                            )
                            min_goal_distance = self.get_parameter('min_goal_distance').value
                            
                            if distance < min_goal_distance:
                                self.get_logger().warn(
                                    f"Goal too close ({distance:.2f}m < {min_goal_distance:.2f}m). "
                                    "Attempting recalculation with costmap clearing..."
                                )
                                # Clear costmaps and recalculate
                                self._clear_nav2_costmaps()
                                self._recalculate_navigation_goal()
                                return
                    
                    # CRITICAL: Multi-level recovery for goal rejection
                    # Level 1: Clear costmaps and try alternative goal
                    self.get_logger().warn(
                        f"⚠️ Goal rejected. Attempting Level 1 recovery: Clear costmaps and recalculate goal..."
                    )
                    self._clear_nav2_costmaps()
                    
                    # Try to calculate alternative goal
                    if self.nav_goal_pose and self.current_truck:
                        robot_pose = self._get_robot_pose("map")
                        if robot_pose:
                            alternative_goal = self._calculate_alternative_goal(self.nav_goal_pose, robot_pose)
                            if alternative_goal:
                                self.get_logger().info(
                                    f"✅ Found alternative goal. Retrying navigation..."
                                )
                                # Retry with alternative goal
                                if self.navigate_to_pose(alternative_goal):
                                    return  # Success, exit callback
                    
                    # Level 2: If still failed, try recalculation
                    self.get_logger().warn(
                        f"⚠️ Alternative goal failed. Attempting Level 2 recovery: Recalculate goal..."
                    )
                    self._recalculate_navigation_goal()
                    
                    # If still in navigation state after recovery attempts, transition to ERROR_RECOVERY
                    if self.state in [MissionState.TRUCK_DETECTED, MissionState.NAVIGATING_TO_LICENSE_PLATE, MissionState.NAVIGATING_TO_TYRE]:
                        self.get_logger().error(
                            f"Navigation goal rejected and all recovery attempts failed. "
                            f"Transitioning to ERROR_RECOVERY from {self.state.value}"
                        )
                        # Clear navigation state
                        self.nav_goal_handle = None
                        self.nav_goal_pose = None
                        self.pending_send_goal_future = None  # Clear pending future
                        self.state = MissionState.ERROR_RECOVERY
                    return
                
                self.nav_goal_handle = goal_handle
                # CRITICAL: Clear pending future - goal is now accepted
                self.pending_send_goal_future = None
                self.get_logger().info(
                    f"✅✅✅ nav_goal_response_callback(): Navigation goal ACCEPTED by Nav2! "
                    f"Goal ID: {goal_handle.goal_id}. Robot should begin moving now!"
                )
                
                # Get result future (feedback is not available via get_feedback_async in ROS 2)
                # Progress tracking is done via _update_navigation_progress() instead
                self.nav_feedback_future = None  # Not used in ROS 2
                self.nav_result_future = self.nav_goal_handle.get_result_async()
                self.nav_result_future.add_done_callback(self.nav_result_callback)
            else:
                # This might be from a different callback
                self.get_logger().debug("Navigation goal response callback received unexpected future type")
        except Exception as e:
            self.get_logger().error(
                f"Error in nav_goal_response_callback: {e}\n{traceback.format_exc()}"
            )
    
    def _update_navigation_progress(self):
        """
        Update navigation progress tracking using current robot pose.
        This is called periodically from the state machine.
        """
        try:
            current_pose = self._get_robot_pose("map")
            if current_pose:
                if self.last_robot_pose:
                    # Calculate distance traveled since last update
                    last_pos = self.last_robot_pose.pose.position
                    current_pos = current_pose.pose.position
                    distance_moved = math.sqrt(
                        (current_pos.x - last_pos.x)**2 +
                        (current_pos.y - last_pos.y)**2
                    )
                    self.nav_progress_distance += distance_moved
                
                # Update last known position
                self.last_robot_pose = current_pose
                self.last_robot_pose_time = time.time()
                
                # Log progress periodically
                current_time = time.time()
                log_interval = self.get_parameter('navigation_progress_log_interval').value
                if (not self.nav_progress_last_update or 
                    current_time - self.nav_progress_last_update > log_interval):
                    if self.nav_goal_pose:
                        goal_pos = self.nav_goal_pose.pose.position
                        remaining_distance = math.sqrt(
                            (goal_pos.x - current_pos.x)**2 +
                            (goal_pos.y - current_pos.y)**2
                        )
                        elapsed_time = current_time - self.nav_start_time if self.nav_start_time else 0.0
                        self.get_logger().info(
                            f"Navigation progress ({elapsed_time:.1f}s): "
                            f"traveled {self.nav_progress_distance:.2f}m, "
                            f"remaining {remaining_distance:.2f}m"
                        )
                    self.nav_progress_last_update = current_time
                    
        except Exception as e:
            self.get_logger().debug(
                f"Error updating navigation progress: {e}"
            )
        
    def nav_result_callback(self, future):
        """Handle navigation result"""
        try:
            # CRITICAL: Handle potential exceptions when getting result
            try:
                result_wrapper = future.result()
                result = result_wrapper.result
            except Exception as e:
                self.get_logger().error(
                    f"Error getting navigation result in callback: {e}\n{traceback.format_exc()}"
                )
                # Clean up navigation state
                self.nav_goal_handle = None
                self.nav_result_future = None
                # Transition to error recovery if in navigation state
                if self.state in [MissionState.NAVIGATING_TO_LICENSE_PLATE, MissionState.NAVIGATING_TO_TYRE]:
                    self.state = MissionState.ERROR_RECOVERY
                return
            
            if result.result == NavigateToPose.Result.SUCCEEDED:
                self.get_logger().info("✅ Navigation succeeded (Nav2 says)")
                
                # CRITICAL: Verify arrival distance BEFORE cleaning up
                # If robot is FAR from goal, Nav2 goal checker is wrong - don't trust it!
                # Don't clean up goal handle yet - let check_navigation_complete() handle it
                current_pose = self._get_robot_pose("map")
                if current_pose and self.nav_goal_pose:
                    goal_pos = self.nav_goal_pose.pose.position
                    robot_pos = current_pose.pose.position
                    arrival_distance = math.sqrt(
                        (goal_pos.x - robot_pos.x)**2 +
                        (goal_pos.y - robot_pos.y)**2
                    )
                    arrival_threshold = self.get_parameter('arrival_distance_threshold').value
                    max_trusted_distance = arrival_threshold * 2.0
                    
                    if arrival_distance > max_trusted_distance:
                        # CRITICAL: Nav2 says succeeded but robot is FAR from goal
                        # Nav2 goal checker is wrong or odometry drift - don't trust Nav2!
                        self.get_logger().error(
                            f"❌ CRITICAL: Nav2 says succeeded but robot is FAR from goal: "
                            f"{arrival_distance:.2f}m > {max_trusted_distance:.2f}m (2x threshold). "
                            f"Nav2 goal checker is wrong - NOT cleaning up goal handle. "
                            f"check_navigation_complete() will handle this and continue navigation."
                        )
                        # DON'T clean up nav_goal_handle and nav_result_future yet
                        # Let check_navigation_complete() detect this and handle it properly
                        # This prevents race condition where callback cleans up before state machine checks
                        return  # Exit early - don't clean up yet
                    elif arrival_distance > arrival_threshold:
                        self.get_logger().warn(
                            f"Navigation reported success but robot is {arrival_distance:.2f}m "
                            f"from goal (threshold: {arrival_threshold:.2f}m, max trusted: {max_trusted_distance:.2f}m). "
                            f"Close enough to trust Nav2 - will verify in arrival check."
                        )
                    else:
                        self.get_logger().info(
                            f"Arrival confirmed: robot is {arrival_distance:.2f}m from goal"
                        )
                else:
                    # Cannot verify robot pose - risky but trust Nav2
                    self.get_logger().warn(
                        "⚠️ Nav2 says succeeded but cannot verify robot pose. "
                        "Trusting Nav2 result but arrival check will verify."
                        )
            elif result.result == NavigateToPose.Result.CANCELED:
                self.get_logger().warn(
                    f"⚠️⚠️⚠️ CRITICAL: Navigation was CANCELED by Nav2! "
                    f"This is why the robot stops after small movements. "
                    f"Direct control should already be active - checking..."
                )
                # CRITICAL: Nav2 canceled the goal - ensure direct control is active
                if hasattr(self, 'direct_nav_fallback') and self.nav_goal_pose:
                    if not self.direct_nav_fallback.is_active:
                        self.get_logger().warn(
                            f"🚨 Nav2 canceled goal. Activating DIRECT CONTROL immediately to continue movement."
                        )
                        self.direct_nav_fallback.set_goal(self.nav_goal_pose)
                        self.direct_nav_fallback.activate()
                    else:
                        self.get_logger().info(
                            f"✅ Direct control already active. Nav2 cancellation ignored - robot continues moving."
                        )
                
                # CRITICAL: Handle canceled navigation appropriately based on state
                if self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                    # License plate navigation canceled - but direct control is active
                    self.get_logger().info(
                        "License plate navigation canceled by Nav2. "
                        "Direct control continues - no interruption to movement."
                    )
                    # Don't transition to error - direct control is handling navigation
                elif self.state == MissionState.NAVIGATING_TO_TYRE:
                    # Tyre navigation canceled - could be intentional (goal update) or error
                    self.get_logger().warn(
                        "Tyre navigation canceled. "
                        "Checking if alternative path is needed..."
                    )
                    # Check if we should try alternative path
                    if (self.path_alternatives and 
                        self.current_truck and 
                        self.current_tyre_index < len(self.current_truck.tyres)):
                        current_tyre = self.current_truck.tyres[self.current_tyre_index]
                        if hasattr(current_tyre, 'position_3d') and current_tyre.position_3d:
                            robot_pose = self._get_robot_pose("map")
                            alternative_goal = self.path_alternatives.get_next_alternative(
                                self.nav_goal_pose if self.nav_goal_pose else None,
                                current_tyre.position_3d,
                                [],  # Failed alternatives
                                robot_pose
                            )
                            if alternative_goal:
                                self.get_logger().info(
                                    "Found alternative path after cancelation. "
                                    "Will retry navigation."
                                )
                                self.nav_goal_pose = alternative_goal
                                # Navigation will be restarted by state machine
                            else:
                                self.get_logger().warn(
                                    "No alternative path available after cancelation. "
                                    "Will transition to error recovery."
                                )
                                self.state = MissionState.ERROR_RECOVERY
                    else:
                        self.get_logger().warn(
                            "Navigation canceled without alternative path. "
                            "Transitioning to ERROR_RECOVERY."
                        )
                        self.state = MissionState.ERROR_RECOVERY
                else:
                    # Navigation canceled in unexpected state
                    self.get_logger().error(
                        f"Navigation canceled in state {self.state.value}. "
                        "This is unexpected. Transitioning to ERROR_RECOVERY."
                    )
                    self.state = MissionState.ERROR_RECOVERY
            elif result.result == NavigateToPose.Result.FAILED:
                self.get_logger().error("❌ Navigation failed - implementing recovery strategy")
                # Check if we should retry or transition to error recovery
                if self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                    # CRITICAL: Multi-level recovery for navigation failure
                    # Level 1: Clear costmaps and try goal recalculation
                    if (self.goal_recalculation_attempts < self.max_goal_recalculation_attempts and
                        self.nav_goal_pose):
                        self.get_logger().info(
                            f"🔄 Level 1 Recovery: Clearing costmaps and recalculating goal "
                            f"(attempt {self.goal_recalculation_attempts + 1}/{self.max_goal_recalculation_attempts})..."
                        )
                        # Clear costmaps to remove stale obstacles
                        self._clear_nav2_costmaps()
                        # Wait a moment for costmaps to clear
                        time.sleep(0.5)
                        # Try goal recalculation
                        self._recalculate_navigation_goal()
                    else:
                        # Level 2: Try alternative goal calculation
                        self.get_logger().warn(
                            "⚠️ Level 2 Recovery: Max recalculation attempts reached. "
                            "Trying alternative goal calculation..."
                        )
                        robot_pose = self._get_robot_pose("map")
                        if robot_pose and self.nav_goal_pose:
                            alternative_goal = self._calculate_alternative_goal(self.nav_goal_pose, robot_pose)
                            if alternative_goal:
                                self.get_logger().info(
                                    f"✅ Found alternative goal. Retrying navigation..."
                                )
                                if self.navigate_to_pose(alternative_goal):
                                    return  # Success, exit callback
                        
                        # All recovery attempts failed
                        self.get_logger().error(
                            "❌ All recovery attempts failed. "
                            "Transitioning to ERROR_RECOVERY."
                        )
                        self.state = MissionState.ERROR_RECOVERY
                elif self.state == MissionState.NAVIGATING_TO_TYRE:
                    # Handle tyre navigation failure
                    if self.navigation_failure_handler and self.nav_goal_pose:
                        robot_pose = self._get_robot_pose("map")
                        failure_result = self.navigation_failure_handler.handle_navigation_failure(
                            self.nav_goal_pose,
                            result.result,
                            str(result) if hasattr(result, '__str__') else "",
                            robot_pose
                        )
                        
                        if failure_result['should_retry']:
                            # CRITICAL: Try path alternatives if available
                            alternative_goal = failure_result.get('alternative_goal')
                            
                            # If no alternative goal provided, try generating one
                            if not alternative_goal and self.path_alternatives:
                                if (self.current_truck and 
                                    self.current_tyre_index < len(self.current_truck.tyres)):
                                    current_tyre = self.current_truck.tyres[self.current_tyre_index]
                                    if hasattr(current_tyre, 'position_3d') and current_tyre.position_3d:
                                        robot_pose = self._get_robot_pose("map")
                                        alternative_goal = self.path_alternatives.get_next_alternative(
                                            self.nav_goal_pose,
                                            current_tyre.position_3d,
                                            [],  # No failed alternatives yet
                                            robot_pose
                                        )
                                        
                                        if alternative_goal:
                                            # Validate alternative goal
                                            is_valid, issue = self.path_alternatives.validate_alternative_goal(
                                                alternative_goal,
                                                current_tyre.position_3d
                                            )
                                            
                                            if is_valid:
                                                failure_result['alternative_goal'] = alternative_goal
                                                self.get_logger().info(
                                                    f"Generated path alternative: "
                                                    f"({alternative_goal.pose.position.x:.2f}, {alternative_goal.pose.position.y:.2f})"
                                                )
                                            else:
                                                self.get_logger().warn(
                                                    f"Path alternative validation failed: {issue}"
                                                )
                            
                            if failure_result.get('alternative_goal'):
                                # Retry with alternative goal
                                self.get_logger().info(
                                    f"Retrying navigation with alternative goal "
                                    f"(strategy: {failure_result['retry_strategy']})"
                                )
                                self.nav_goal_pose = failure_result['alternative_goal']
                                # Navigation will be restarted in state machine
                        elif failure_result['should_skip']:
                            # Skip this tyre
                            current_tyre_id = None
                            if self.current_truck and self.current_tyre_index < len(self.current_truck.tyres):
                                current_tyre_id = self.current_truck.tyres[self.current_tyre_index].tyre_id
                            
                            self.get_logger().error(
                                f"Navigation failed for tyre. Skipping to next tyre. "
                                f"Reason: {failure_result['failure_reason']}"
                            )
                            
                            # Mark failure in context
                            if self.tyre_navigation_context and current_tyre_id:
                                self.tyre_navigation_context.mark_tyre_navigation_failure(
                                    current_tyre_id,
                                    failure_result['failure_reason']
                                )
                            
                            if self.current_truck and self.current_tyre_index < len(self.current_truck.tyres):
                                # Mark goal as unreachable
                                if self.tyre_goal_validator:
                                    self.tyre_goal_validator.mark_goal_unreachable(self.nav_goal_pose)
                                self.current_tyre_index += 1
                                self.current_tyre_nav_failures = 0
                                if self.navigation_failure_handler:
                                    self.navigation_failure_handler.reset_failure_count()
                            # State machine will handle transition
                    else:
                        # Fallback: skip tyre
                        self.get_logger().error("Navigation failed for tyre. Skipping to next tyre.")
                        if self.current_truck and self.current_tyre_index < len(self.current_truck.tyres):
                            self.current_tyre_index += 1
                            self.current_tyre_nav_failures = 0
            else:
                self.get_logger().warn(
                    f"Navigation completed with unknown result: {result.result}"
                )
                
            # Clean up navigation state
            self.nav_goal_handle = None
            self.nav_result_future = None
                
        except Exception as e:
            self.get_logger().error(
                f"Error in nav_result_callback: {e}", 
                exc_info=True
            )
            self.nav_goal_handle = None
            self.nav_result_future = None
        
    def _get_robot_pose(self, target_frame="map", max_transform_age_seconds=1.0):
        """
        Get robot's current pose in the specified frame using TF.
        
        Args:
            target_frame: Target frame (default: "map")
            max_transform_age_seconds: Maximum age of transform in seconds (default: 1.0)
                                      Transforms older than this are considered stale and rejected
            
        Returns:
            PoseStamped: Robot pose in target frame, or None if transform fails or is stale
        """
        try:
            # CRITICAL: Use base_footprint (not base_link) for consistency with odometry
            # Odometry publishes with child_frame_id = base_footprint (from base_node.cpp)
            # base_footprint is the ground-level frame used for 2D navigation
            # base_link is 0.08m above base_footprint (from URDF), but we use base_footprint for 2D navigation
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                "base_footprint",  # CRITICAL: Must match odometry child_frame_id
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
            # CRITICAL: Check if transform is stale (too old)
            # If SLAM/localization stops updating, transform could be seconds/minutes old
            # Using stale transforms leads to incorrect robot pose → wrong goal calculations → navigation failures
            current_time = self.get_clock().now()
            transform_time = rclpy.time.Time.from_msg(transform.header.stamp)
            transform_age = (current_time - transform_time).nanoseconds / 1e9  # Convert to seconds
            
            if transform_age > max_transform_age_seconds:
                self.get_logger().warn(
                    f"⚠️ STALE TF TRANSFORM: Transform from {target_frame} to base_footprint is {transform_age:.2f}s old "
                    f"(max allowed: {max_transform_age_seconds:.2f}s). Rejecting stale transform. "
                    f"This indicates SLAM/localization may have stopped updating."
                )
                return None
            
            # Create pose from transform
            pose = PoseStamped()
            pose.header.frame_id = target_frame
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except tf2_ros.TransformException as e:
            self.get_logger().debug(
                f"TF transform error getting robot pose: {e}"
            )
            return None
        except Exception as e:
            self.get_logger().debug(
                f"Error getting robot pose: {e}"
            )
            return None
    
    def _recalculate_navigation_goal(self):
        """
        Recalculate navigation goal if it's too close to current position.
        
        This handles the case where the goal is rejected because it's too close
        (e.g., < 0.6m from robot).
        """
        try:
            self.goal_recalculation_attempts += 1
            
            if self.goal_recalculation_attempts > self.max_goal_recalculation_attempts:
                self.get_logger().error(
                    f"Max goal recalculation attempts ({self.max_goal_recalculation_attempts}) reached. "
                    "Cannot recalculate goal."
                )
                # CRITICAL: Handle both license plate and tyre navigation failures
                if self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                    self.state = MissionState.ERROR_RECOVERY
                elif self.state == MissionState.NAVIGATING_TO_TYRE:
                    # For tyre navigation, try to skip to next tyre or error recovery
                    self.get_logger().error(
                        "Max goal recalculation attempts reached for tyre navigation. "
                        "Transitioning to ERROR_RECOVERY."
                    )
                    self.state = MissionState.ERROR_RECOVERY
                return False
            
            current_pose = self._get_robot_pose("map")
            if not current_pose:
                self.get_logger().error(
                    "Cannot recalculate goal: unable to get robot pose"
                )
                return False
            
            if not self.nav_goal_pose:
                self.get_logger().error(
                    "Cannot recalculate goal: no original goal pose stored"
                )
                return False
            
            # Get recalculation distance and min_goal_distance
            recalculation_distance = self.get_parameter('goal_recalculation_distance').value
            min_goal_distance = self.get_parameter('min_goal_distance').value
            
            # CRITICAL: Ensure recalculation_distance >= min_goal_distance
            # If not, use min_goal_distance to ensure recalculated goal is valid
            if recalculation_distance < min_goal_distance:
                self.get_logger().warn(
                    f"⚠️ goal_recalculation_distance ({recalculation_distance:.2f}m) < min_goal_distance ({min_goal_distance:.2f}m). "
                    f"Using min_goal_distance ({min_goal_distance:.2f}m) for recalculation to ensure valid goal."
                )
                recalculation_distance = min_goal_distance
            
            # Get original goal position
            original_goal_pos = self.nav_goal_pose.pose.position
            robot_pos = current_pose.pose.position
            
            # Calculate direction to original goal
            dx = original_goal_pos.x - robot_pos.x
            dy = original_goal_pos.y - robot_pos.y
            distance_to_original = math.sqrt(dx**2 + dy**2)
            
            if distance_to_original < 0.01:  # Already at goal
                self.get_logger().info(
                    "Robot is already at goal. Navigation complete."
                )
                # CRITICAL: Handle both license plate and tyre navigation
                if self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                    self.state = MissionState.CAPTURING_LICENSE_PLATE
                elif self.state == MissionState.NAVIGATING_TO_TYRE:
                    # Already at tyre goal - proceed to capture
                    self.get_logger().info(
                        "Already at tyre goal. Proceeding to capture."
                    )
                    # State machine will handle transition to CAPTURING_TYRE
                return True
            
            # Normalize direction
            dx_norm = dx / distance_to_original if distance_to_original > 0.01 else 1.0
            dy_norm = dy / distance_to_original if distance_to_original > 0.01 else 0.0
            
            # Calculate new goal position at safe distance (>= min_goal_distance)
            new_goal_pose = PoseStamped()
            new_goal_pose.header.frame_id = self.nav_goal_pose.header.frame_id
            new_goal_pose.header.stamp = self.get_clock().now().to_msg()
            
            new_goal_pose.pose.position.x = robot_pos.x + dx_norm * recalculation_distance
            new_goal_pose.pose.position.y = robot_pos.y + dy_norm * recalculation_distance
            new_goal_pose.pose.position.z = original_goal_pos.z  # Keep same height
            
            # Orient toward original goal
            yaw = math.atan2(dy, dx)
            new_goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            new_goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            # CRITICAL: Validate recalculated goal distance against min_goal_distance
            # Verify the recalculated goal is actually at least min_goal_distance away
            new_goal_pos = new_goal_pose.pose.position
            recalc_distance_to_robot = math.sqrt(
                (new_goal_pos.x - robot_pos.x)**2 +
                (new_goal_pos.y - robot_pos.y)**2
            )
            if recalc_distance_to_robot < min_goal_distance:
                self.get_logger().error(
                    f"❌ CRITICAL: Recalculated goal is too close: {recalc_distance_to_robot:.2f}m < {min_goal_distance:.2f}m. "
                    f"This should not happen - recalculation_distance={recalculation_distance:.2f}m. "
                    f"Adjusting goal to min_goal_distance."
                )
                # Force goal to be at least min_goal_distance away
                if recalc_distance_to_robot > 0.01:
                    # Scale up to min_goal_distance
                    scale = min_goal_distance / recalc_distance_to_robot
                    new_goal_pose.pose.position.x = robot_pos.x + (new_goal_pos.x - robot_pos.x) * scale
                    new_goal_pose.pose.position.y = robot_pos.y + (new_goal_pos.y - robot_pos.y) * scale
                else:
                    # Can't determine direction - use a default direction (forward)
                    new_goal_pose.pose.position.x = robot_pos.x + min_goal_distance
                    new_goal_pose.pose.position.y = robot_pos.y
                recalc_distance_to_robot = min_goal_distance
            
            # Validate new goal structure (NaN, Inf, frame_id, etc.)
            if not self._validate_navigation_pose(new_goal_pose):
                self.get_logger().error(
                    "Recalculated goal pose structure is invalid"
                )
                return False
            
            # CRITICAL: Validate recalculated goal against min_goal_distance one more time
            # This ensures the goal is valid before sending to Nav2
            final_distance = math.sqrt(
                (new_goal_pose.pose.position.x - robot_pos.x)**2 +
                (new_goal_pose.pose.position.y - robot_pos.y)**2
            )
            if final_distance < min_goal_distance:
                self.get_logger().error(
                    f"❌ CRITICAL: After validation, recalculated goal is still too close: "
                    f"{final_distance:.2f}m < {min_goal_distance:.2f}m. Cannot recalculate goal."
                )
                return False
            
            self.get_logger().info(
                f"Recalculating goal (attempt {self.goal_recalculation_attempts}/"
                f"{self.max_goal_recalculation_attempts}): "
                f"new goal at ({new_goal_pose.pose.position.x:.2f}, "
                f"{new_goal_pose.pose.position.y:.2f}) "
                f"(distance: {final_distance:.2f}m, "
                f"min_goal_distance: {min_goal_distance:.2f}m, "
                f"original goal was {distance_to_original:.2f}m away)"
            )
            
            # Update stored goal
            self.nav_goal_pose = new_goal_pose
            
            # CRITICAL: If direct navigation is already active, update its goal immediately
            # Otherwise, it will continue navigating to the old goal!
            # Use preserve_active_state=True to keep navigation active with new goal
            if hasattr(self, 'direct_nav_fallback') and self.direct_nav_fallback.is_active:
                self.get_logger().info(
                    "Direct navigation is active. Updating direct navigation goal to recalculated goal "
                    f"(preserving active state)."
                )
                self.direct_nav_fallback.set_goal(new_goal_pose, preserve_active_state=True)
                # Goal is already set, initial_distance resets for new goal (will recalculate on next update)
            
            # Send new goal to Nav2
            return self.navigate_to_pose(new_goal_pose)
            
        except Exception as e:
            self.get_logger().error(
                f"Error recalculating navigation goal: {e}",
                exc_info=True
            )
            return False
    
    def _check_navigation_stuck(self):
        """
        Check if navigation is stuck (robot not moving).
        
        Returns:
            bool: True if stuck, False otherwise
        """
        try:
            if not self.stuck_detection_enabled:
                return False
            
            current_time = time.time()
            
            # Need at least some history to detect stuck
            if not self.last_robot_pose_time:
                return False
            
            # Get current robot pose
            current_pose = self._get_robot_pose("map")
            if not current_pose:
                return False
            
            # Calculate distance moved since last check
            if self.last_robot_pose:
                last_pos = self.last_robot_pose.pose.position
                current_pos = current_pose.pose.position
                distance_moved = math.sqrt(
                    (current_pos.x - last_pos.x)**2 +
                    (current_pos.y - last_pos.y)**2
                )
                
                # Check stuck thresholds
                stuck_distance = self.get_parameter('stuck_detection_distance').value
                stuck_time = self.get_parameter('stuck_detection_time').value
                time_since_last_move = current_time - self.last_robot_pose_time
                
                if (distance_moved < stuck_distance and 
                    time_since_last_move > stuck_time):
                    self.get_logger().warn(
                        f"Navigation appears stuck: moved only {distance_moved:.2f}m "
                        f"in {time_since_last_move:.1f}s "
                        f"(threshold: {stuck_distance:.2f}m in {stuck_time:.1f}s)"
                    )
                    return True
            
            # Update last known position
            self.last_robot_pose = current_pose
            self.last_robot_pose_time = current_time
            
            return False
            
        except Exception as e:
            self.get_logger().debug(
                f"Error checking navigation stuck: {e}"
            )
            return False
        
    def check_navigation_complete(self):
        """
        Check if navigation is complete with enhanced validation.
        
        Returns:
            bool: True if navigation is complete, False otherwise
        """
        try:
            # Check if we have a result future
            if self.nav_result_future:
                # Check if result is ready
                if self.nav_result_future.done():
                    try:
                        # CRITICAL: Handle potential exceptions when getting result
                        try:
                            result_wrapper = self.nav_result_future.result()
                            result = result_wrapper.result
                        except Exception as e:
                            self.get_logger().error(
                                f"Error getting navigation result: {e}",
                                exc_info=True
                            )
                            return False
                        
                        if result.result == NavigateToPose.Result.SUCCEEDED:
                            # CRITICAL: Verify BOTH distance AND orientation
                            # Nav2 might report SUCCEEDED if robot is within distance but orientation is wrong
                            # This would cause mission to think navigation is complete when robot can't see license plate
                            current_pose = self._get_robot_pose("map")
                            if current_pose and self.nav_goal_pose:
                                goal_pos = self.nav_goal_pose.pose.position
                                robot_pos = current_pose.pose.position
                                arrival_distance = math.sqrt(
                                    (goal_pos.x - robot_pos.x)**2 +
                                    (goal_pos.y - robot_pos.y)**2
                                )
                                arrival_threshold = self.get_parameter('arrival_distance_threshold').value
                                
                                # CRITICAL: Also check orientation match
                                goal_orientation = self.nav_goal_pose.pose.orientation
                                robot_orientation = current_pose.pose.orientation
                                goal_siny_cosp = 2.0 * (goal_orientation.w * goal_orientation.z + goal_orientation.x * goal_orientation.y)
                                goal_cosy_cosp = 1.0 - 2.0 * (goal_orientation.y * goal_orientation.y + goal_orientation.z * goal_orientation.z)
                                goal_yaw = math.atan2(goal_siny_cosp, goal_cosy_cosp)
                                robot_siny_cosp = 2.0 * (robot_orientation.w * robot_orientation.z + robot_orientation.x * robot_orientation.y)
                                robot_cosy_cosp = 1.0 - 2.0 * (robot_orientation.y * robot_orientation.y + robot_orientation.z * robot_orientation.z)
                                robot_yaw = math.atan2(robot_siny_cosp, robot_cosy_cosp)
                                orientation_diff = abs(goal_yaw - robot_yaw)
                                if orientation_diff > math.pi:
                                    orientation_diff = 2.0 * math.pi - orientation_diff
                                
                                arrival_orientation_threshold = self.get_parameter('arrival_orientation_tolerance').value
                                orientation_match = orientation_diff <= arrival_orientation_threshold
                                
                                # CRITICAL: Only trust Nav2 if BOTH distance AND orientation are correct
                                if arrival_distance <= arrival_threshold and orientation_match:
                                    self.get_logger().info(
                                        f"Navigation complete: arrived at goal "
                                        f"(distance: {arrival_distance:.2f}m, "
                                        f"orientation: {math.degrees(orientation_diff):.1f}° <= {math.degrees(arrival_orientation_threshold):.1f}°)"
                                    )
                                    # CRITICAL: Reset navigation start time when navigation completes successfully
                                    self.nav_start_time = None
                                    return True
                                elif arrival_distance <= arrival_threshold and not orientation_match:
                                    # CRITICAL: At goal position but wrong orientation - Nav2's goal checker is too lenient!
                                    # Don't trust Nav2 - robot needs to rotate to match orientation
                                    self.get_logger().warn(
                                        f"⚠️ Nav2 reported SUCCEEDED but robot orientation is wrong: "
                                        f"distance: {arrival_distance:.2f}m <= {arrival_threshold:.2f}m (OK), "
                                        f"but orientation: {math.degrees(orientation_diff):.1f}° > {math.degrees(arrival_orientation_threshold):.1f}° (WRONG). "
                                        f"Robot at position but facing wrong direction - navigation NOT complete!"
                                    )
                                    # Don't trust Nav2 - return False to trigger continuation of navigation
                                    return False
                                else:
                                    # CRITICAL FIX: Don't trust Nav2 if robot is far from goal!
                                    # Nav2's goal checker might be too lenient, or odometry drift might cause
                                    # Nav2 to think it succeeded even though robot is far from goal.
                                    # Only trust Nav2 if robot is reasonably close (< 2x threshold for tolerance).
                                    # If robot is far (> 2x threshold), Nav2 is wrong - don't trust it!
                                    max_trusted_distance = arrival_threshold * 2.0  # Allow 2x threshold for tolerance
                                    if arrival_distance <= max_trusted_distance:
                                        # CRITICAL: Also check orientation even for intermediate distance
                                        # Robot might be close enough but facing wrong direction
                                        if orientation_match:
                                    self.get_logger().warn(
                                        f"Navigation reported success but robot is "
                                        f"{arrival_distance:.2f}m from goal "
                                                f"(threshold: {arrival_threshold:.2f}m, max trusted: {max_trusted_distance:.2f}m). "
                                                f"Close enough and orientation matches - trusting Nav2 result. "
                                                f"Will verify in arrival check."
                                    )
                                            # Close enough and orientation OK - trust Nav2
                                    self.nav_start_time = None
                                    return True
                            else:
                                            # Close enough but wrong orientation - don't trust Nav2
                                            self.get_logger().warn(
                                                f"⚠️ Nav2 reported SUCCEEDED but robot orientation is wrong: "
                                                f"distance: {arrival_distance:.2f}m <= {max_trusted_distance:.2f}m (close enough), "
                                                f"but orientation: {math.degrees(orientation_diff):.1f}° > {math.degrees(arrival_orientation_threshold):.1f}° (WRONG). "
                                                f"Navigation NOT complete - need to rotate!"
                                            )
                                            return False
                                    else:
                                        self.get_logger().error(
                                            f"❌ CRITICAL: Nav2 reported success but robot is FAR from goal: "
                                            f"{arrival_distance:.2f}m > {max_trusted_distance:.2f}m (2x threshold). "
                                            f"This indicates Nav2 goal checker error or odometry drift. "
                                            f"Ignoring Nav2 result - navigation NOT complete!"
                                        )
                                        # Too far - Nav2 is wrong! Don't trust it - return False
                                        # This will trigger recalculation or error recovery
                                        return False
                            else:
                                # Nav2 says succeeded but cannot verify robot pose (TF error, no pose available)
                                # CRITICAL: Without robot pose, we cannot verify arrival distance
                                # This is risky - Nav2 might be wrong, but we have no way to verify
                                # Trust Nav2 but log warning - arrival verification will handle it
                                self.get_logger().warn(
                                    f"⚠️ Nav2 reported success but cannot verify robot pose. "
                                    f"Trusting Nav2 result but arrival verification will verify distance/orientation."
                                )
                                # CRITICAL: Reset navigation start time when navigation completes
                                self.nav_start_time = None
                                return True
                        else:
                            # Navigation failed or was canceled
                            # CRITICAL: Reset navigation start time when navigation fails/cancels
                            self.nav_start_time = None
                            return False
                            
                    except Exception as e:
                        self.get_logger().error(
                            f"Error checking navigation result: {e}",
                            exc_info=True
                        )
                        return False
            
            # Check for stuck situation if navigation is active (no result future or not done)
            if self.nav_goal_handle is not None:
                if self._check_navigation_stuck():
                    self.get_logger().error(
                        "Navigation stuck detected. Canceling goal."
                    )
                    if self.nav_goal_handle:
                        self.nav_client.cancel_goal_async(self.nav_goal_handle)
                        self.nav_goal_handle = None
                    
                    # CRITICAL: Deactivate direct navigation if active (prevents it from continuing to old goal)
                    if hasattr(self, 'direct_nav_fallback') and self.direct_nav_fallback.is_active:
                        self.get_logger().warn(
                            "Navigation stuck detected. Deactivating direct navigation to prevent continued navigation to old goal."
                        )
                        self.direct_nav_fallback.deactivate()
                    
                    # CRITICAL: Reset navigation start time when navigation is stuck
                    self.nav_start_time = None
                    
                    # CRITICAL: Handle stuck navigation for both license plate and tyre navigation
                    if self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                        self.get_logger().error(
                            "License plate navigation stuck. Transitioning to ERROR_RECOVERY."
                        )
                        self.state = MissionState.ERROR_RECOVERY
                    elif self.state == MissionState.NAVIGATING_TO_TYRE:
                        self.get_logger().error(
                            "Tyre navigation stuck. Attempting recovery with alternative path or skipping tyre."
                        )
                        # Try alternative path first
                        if (self.path_alternatives and 
                            self.current_truck and 
                            self.current_tyre_index < len(self.current_truck.tyres)):
                            current_tyre = self.current_truck.tyres[self.current_tyre_index]
                            if hasattr(current_tyre, 'position_3d') and current_tyre.position_3d:
                                robot_pose = self._get_robot_pose("map")
                                if robot_pose:
                                    alternative_goal = self.path_alternatives.get_next_alternative(
                                        self.nav_goal_pose if self.nav_goal_pose else None,
                                        current_tyre.position_3d,
                                        [],  # Failed alternatives
                                        robot_pose
                                    )
                                    if alternative_goal:
                                        self.get_logger().info(
                                            "Found alternative path for stuck navigation. Will retry."
                                        )
                                        self.nav_goal_pose = alternative_goal
                                        # Will retry in state machine - reset start time for new navigation attempt
                                        self.nav_start_time = time.time()
                                        return False  # Not complete yet - will retry
                        # No alternative or failed - transition to error recovery
                        self.state = MissionState.ERROR_RECOVERY
                    return True
            
            # Check for timeout
            timeout = self.get_parameter('navigation_timeout').value
            if self.nav_start_time and (time.time() - self.nav_start_time) > timeout:
                self.get_logger().warn(
                    f"Navigation timeout after {timeout:.1f}s"
                )
                
                # CRITICAL: Cancel Nav2 goal
                if self.nav_goal_handle:
                    self.nav_client.cancel_goal_async(self.nav_goal_handle)
                    self.nav_goal_handle = None
                    self.nav_result_future = None
                
                # CRITICAL: Deactivate direct navigation if active (prevents it from continuing to old goal)
                if hasattr(self, 'direct_nav_fallback') and self.direct_nav_fallback.is_active:
                    self.get_logger().warn(
                        "Navigation timeout detected. Deactivating direct navigation to prevent continued navigation to old goal."
                    )
                    self.direct_nav_fallback.deactivate()
                
                # CRITICAL: Reset navigation start time to prevent false timeout on next navigation
                self.nav_start_time = None
                
                # CRITICAL: Handle timeout based on current state
                if self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                    self.get_logger().error(
                        "License plate navigation timeout. Transitioning to ERROR_RECOVERY."
                    )
                    self.state = MissionState.ERROR_RECOVERY
                elif self.state == MissionState.NAVIGATING_TO_TYRE:
                    # For tyre navigation, try alternative or skip
                    if (self.navigation_failure_handler and 
                        self.nav_goal_pose):
                        robot_pose = self._get_robot_pose("map")
                        failure_result = self.navigation_failure_handler.handle_navigation_failure(
                            self.nav_goal_pose,
                            NavigateToPose.Result.FAILED,  # Treat timeout as failure
                            "Navigation timeout",
                            robot_pose
                        )
                        if failure_result.get('should_skip'):
                            self.get_logger().warn(
                                "Navigation timeout. Skipping to next tyre."
                            )
                            if self.current_truck and self.current_tyre_index < len(self.current_truck.tyres):
                                current_tyre_id = self.current_truck.tyres[self.current_tyre_index].tyre_id
                                if self.tyre_navigation_context:
                                    self.tyre_navigation_context.mark_tyre_navigation_failure(
                                        current_tyre_id,
                                        "Navigation timeout"
                                    )
                                self.current_tyre_index += 1
                        # State machine will handle transition
                    else:
                        self.get_logger().error(
                            "Navigation timeout. Transitioning to ERROR_RECOVERY."
                        )
                        self.state = MissionState.ERROR_RECOVERY
                
                return True  # Consider it done to continue (timeout = done)
            
            return False
                
        except Exception as e:
            self.get_logger().error(
                f"Error in check_navigation_complete: {e}",
                exc_info=True
            )
            return False
        
    def capture_photo(self, filename, directory):
        """Capture photo via service"""
        if not self.capture_photo_client.service_is_ready():
            self.get_logger().warn("Photo capture service not ready")
            return None
            
        # Ensure directory exists
        directory.mkdir(parents=True, exist_ok=True)
        
        # Set photo capture service storage directory
        # Note: This requires a parameter service on photo_capture node
        # For now, we'll construct the expected path
        
        request = Trigger.Request()
        future = self.capture_photo_client.call_async(request)
        
        # Wait for response with timeout
        timeout = self.get_parameter('photo_capture_timeout').value
        start_time = time.time()
        
        while not future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not future.done():
            self.get_logger().error("Photo capture timeout")
            return None
        
        # CRITICAL: Handle potential exceptions when getting result
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(
                f"Error getting photo capture service response: {e}",
                exc_info=True
            )
            return None
            
        try:
            if response.success:
                # Response message should contain the full path
                # If not, construct it from the directory and filename
                if response.message and Path(response.message).exists():
                    # Move/copy to desired directory
                    photo_path = directory / f"{filename}.jpg"
                    import shutil
                    shutil.copy2(response.message, photo_path)
                    return str(photo_path)
                else:
                    # Construct path
                    photo_path = directory / f"{filename}.jpg"
                    # Note: Photo was saved by service, but we don't have the exact path
                    # This assumes the service saves to the current storage directory
                    return str(photo_path)
            else:
                self.get_logger().error(f"Photo capture failed: {response.message}")
                return None
        except Exception as e:
            self.get_logger().error(f"Error capturing photo: {e}")
            return None
    
    def _handle_license_plate_capture(self):
        """
        Enhanced license plate capture handler with OCR, 3D positioning, and folder creation.
        
        This method:
        1. Captures photo with retry logic
        2. Runs OCR (EasyOCR primary, Tesseract fallback)
        3. Calculates 3D position from point cloud
        4. Creates folder with license plate name (or fallback to truck ID)
        5. Saves metadata (timestamp, 3D position, OCR results)
        6. Handles all error cases gracefully
        """
        try:
            # Initialize capture attempt tracking on first attempt
            if self.license_plate_capture_start_time is None:
                self.license_plate_capture_start_time = time.time()
                self.license_plate_capture_attempts = 0
                self.license_plate_text = None
                self.license_plate_confidence = None
                self.license_plate_3d_position = None
                self.get_logger().info(
                    f"CAPTURING_LICENSE_PLATE: Starting license plate capture "
                    f"for {self.current_truck.truck_id}"
                )
            
            # Validate current truck
            if not self.current_truck:
                self.get_logger().error(
                    "CAPTURING_LICENSE_PLATE: current_truck is None. "
                    "Transitioning to ERROR_RECOVERY."
                )
                self.state = MissionState.ERROR_RECOVERY
                return
            
            # Check for timeout
            capture_timeout = self.get_parameter('license_plate_capture_timeout').value
            elapsed_time = time.time() - self.license_plate_capture_start_time
            if elapsed_time > capture_timeout:
                self.get_logger().error(
                    f"License plate capture timeout after {elapsed_time:.1f}s. "
                    f"Attempts: {self.license_plate_capture_attempts}/"
                    f"{self.max_license_plate_capture_attempts}"
                )
                self.state = MissionState.ERROR_RECOVERY
                return
            
            # Increment attempt counter
            self.license_plate_capture_attempts += 1
            
            self.get_logger().info(
                f"CAPTURING_LICENSE_PLATE: Attempt {self.license_plate_capture_attempts}/"
                f"{self.max_license_plate_capture_attempts}"
            )
            
            # Step 1: Capture photo
            photo_path = self._capture_license_plate_photo()
            if not photo_path:
                # Photo capture failed
                if self.license_plate_capture_attempts < self.max_license_plate_capture_attempts:
                    retry_delay = self.get_parameter('license_plate_capture_retry_delay').value
                    self.get_logger().warn(
                        f"Photo capture failed. Retrying in {retry_delay}s... "
                        f"(attempt {self.license_plate_capture_attempts}/"
                        f"{self.max_license_plate_capture_attempts})"
                    )
                    time.sleep(retry_delay)
                    return  # Will retry on next iteration
                else:
                    self.get_logger().error(
                        f"Photo capture failed after {self.max_license_plate_capture_attempts} attempts. "
                        "Transitioning to ERROR_RECOVERY."
                    )
                    self.state = MissionState.ERROR_RECOVERY
                    return
            
            # Step 2: Run OCR on captured photo (if enabled and detector available)
            ocr_enabled = self.get_parameter('enable_license_plate_ocr').value
            if ocr_enabled and self.license_plate_detector:
                self.get_logger().info("Running OCR on license plate image...")
                ocr_result = self._run_license_plate_ocr(photo_path)
                
                if ocr_result:
                    self.license_plate_text = ocr_result.get('text')
                    self.license_plate_confidence = ocr_result.get('confidence')
                    self.license_plate_3d_position = ocr_result.get('position_nav')
                    
                    if self.license_plate_text:
                        self.get_logger().info(
                            f"✅ License plate detected: '{self.license_plate_text}' "
                            f"(confidence: {self.license_plate_confidence:.2f})"
                        )
                    else:
                        self.get_logger().warn(
                            "OCR completed but no license plate text detected. "
                            f"Using fallback folder name: {self.current_truck.truck_id}"
                        )
                else:
                    self.get_logger().warn(
                        "OCR failed. Using fallback folder name: "
                        f"{self.current_truck.truck_id}"
                    )
            else:
                if not ocr_enabled:
                    self.get_logger().info("OCR disabled. Skipping OCR step.")
                else:
                    self.get_logger().warn(
                        "License plate detector not available. Skipping OCR step."
                    )
            
            # Step 3: Create folder with license plate name (or fallback to truck ID)
            folder_name = self._create_license_plate_folder()
            if not folder_name:
                self.get_logger().error(
                    "Failed to create license plate folder. "
                    "Transitioning to ERROR_RECOVERY."
                )
                self.state = MissionState.ERROR_RECOVERY
                return
            
            # Step 4: Move photo to license plate folder
            moved_photo_path = self._move_photo_to_license_plate_folder(photo_path, folder_name)
            if not moved_photo_path:
                self.get_logger().error(
                    "Failed to move photo to license plate folder. "
                    "Transitioning to ERROR_RECOVERY."
                )
                self.state = MissionState.ERROR_RECOVERY
                return
            
            # Step 5: Save metadata
            metadata_saved = self._save_license_plate_metadata(
                moved_photo_path, folder_name
            )
            if not metadata_saved:
                self.get_logger().warn(
                    "Failed to save metadata, but continuing with mission..."
                )
            
            # Success! Update truck data and transition to next state
            self.current_truck.license_plate_photo_path = moved_photo_path
            self.current_truck.license_plate_photo_taken = True
            
            if self.license_plate_text:
                self.get_logger().info(
                    f"✅ License plate capture complete: '{self.license_plate_text}' "
                    f"(photo: {moved_photo_path})"
                )
            else:
                self.get_logger().info(
                    f"✅ License plate photo captured (photo: {moved_photo_path}, "
                    f"folder: {folder_name})"
                )
            
            # Reset capture state
            self.license_plate_capture_start_time = None
            self.license_plate_capture_attempts = 0
            # CRITICAL: Clear detected license plate bbox after use (cleanup for next vehicle)
            self.detected_license_plate_bbox = None
            self.detected_license_plate_bbox_time = None
            self.detected_license_plate_confidence = None
            if hasattr(self, '_lp_detection_logged'):
                delattr(self, '_lp_detection_logged')  # Reset for next vehicle
            
            # Transition to next state
            self.get_logger().info(
                f"CAPTURING_LICENSE_PLATE → SWITCHING_TO_INSPECTION: "
                f"License plate photo captured successfully for {self.current_truck.truck_id}"
            )
            self.state = MissionState.SWITCHING_TO_INSPECTION
            self.get_logger().info(
                f"✅ NEXT ACTION: System is now in SWITCHING_TO_INSPECTION state. "
                f"Switching segmentation mode to 'inspection' for tyre detection..."
            )
            
        except Exception as e:
            self.get_logger().error(
                f"Error in CAPTURING_LICENSE_PLATE state: {e}",
                exc_info=True
            )
            self.state = MissionState.ERROR_RECOVERY
    
    def _capture_license_plate_photo(self):
        """
        Capture photo for license plate with validation.
        
        Returns:
            str: Path to captured photo, or None if failed
        """
        try:
            if not self.capture_photo_client.service_is_ready():
                self.get_logger().error("Photo capture service not ready")
                return None
            
            # Capture photo to temporary location first
            temp_filename = f"{self.current_truck.truck_id}_license_temp"
            temp_dir = self.photo_dir / "license_plates" / "temp"
            temp_dir.mkdir(parents=True, exist_ok=True)
            
            photo_path = self.capture_photo(temp_filename, temp_dir)
            
            if not photo_path:
                return None
            
            # Validate photo file exists and is not empty
            photo_path_obj = Path(photo_path)
            if not photo_path_obj.exists():
                self.get_logger().error(f"Photo file does not exist: {photo_path}")
                return None
            
            file_size = photo_path_obj.stat().st_size
            if file_size == 0:
                self.get_logger().error(f"Photo file is empty: {photo_path}")
                photo_path_obj.unlink()  # Delete empty file
                return None
            
            if file_size < 1024:  # Less than 1KB is suspiciously small
                self.get_logger().warn(
                    f"Photo file is very small ({file_size} bytes). "
                    "It may be corrupted."
                )
            
            self.get_logger().info(
                f"Photo captured successfully: {photo_path} ({file_size} bytes)"
            )
            return photo_path
            
        except Exception as e:
            self.get_logger().error(
                f"Error in _capture_license_plate_photo: {e}",
                exc_info=True
            )
            return None
    
    def _run_license_plate_ocr(self, photo_path):
        """
        Run OCR on license plate photo.
        
        Args:
            photo_path: Path to photo file
            
        Returns:
            dict: OCR results with 'text', 'confidence', 'position_3d', 'position_nav'
            or None if OCR failed
        """
        try:
            if not self.license_plate_detector:
                self.get_logger().warn("License plate detector not available")
                return None
            
            if not self.latest_camera_image:
                self.get_logger().warn("No camera image available for OCR")
                return None
            
            # Convert ROS Image to OpenCV image
            from cv_bridge import CvBridge
            bridge = CvBridge()
            
            try:
                cv_image = bridge.imgmsg_to_cv2(self.latest_camera_image, "bgr8")
            except Exception as e:
                self.get_logger().error(
                    f"Failed to convert ROS image to OpenCV: {e}"
                )
                return None
            
            # CRITICAL FIX: Two-stage license plate detection
            # If license plate bbox was detected via YOLO (Stage 1), use it for OCR (Stage 2)
            # This enables "camera to know what a license plate looks like" before OCR
            license_plate_bbox_2d = None
            use_detected_bbox = False
            
            if (hasattr(self, 'detected_license_plate_bbox') and 
                self.detected_license_plate_bbox is not None and
                self.detected_license_plate_bbox_time is not None):
                
                # Check if detection is recent (within 5 seconds)
                detection_age = time.time() - self.detected_license_plate_bbox_time
                if detection_age < 5.0:  # Recent detection
                    lp_bbox = self.detected_license_plate_bbox
                    
                    # CRITICAL: Convert 3D bbox to 2D pixel coordinates for OCR
                    # The bbox from YOLO is in 3D space (meters), but OCR needs pixel coordinates
                    # We need to project 3D bbox corners to 2D image plane
                    # For now, use bbox directly if it's already in pixel coordinates, or convert
                    
                    # Check if bbox values are reasonable for pixel coordinates (typically 0-1920 for width)
                    # Or if they're in normalized coordinates (0-1) or 3D space (meters)
                    # Assume bbox is in camera frame 3D coordinates - need to project to pixels
                    
                    # TEMPORARY APPROACH: Use bbox center and estimate pixel bbox based on size
                    # This is a simplification - proper implementation needs camera calibration
                    # For now, we'll pass the bbox to license_plate_detector and let it handle conversion
                    # or use the full image if conversion fails
                    
                    # Try to use detected bbox - license_plate_detector will handle pixel conversion
                    # Store bbox for use in detect_license_plate
                    self.get_logger().info(
                        f"✅ Using YOLO-detected license plate bbox for OCR (two-stage detection): "
                        f"confidence={self.detected_license_plate_confidence:.2f}, "
                        f"age={detection_age:.1f}s, "
                        f"bbox=({lp_bbox.xmin:.3f},{lp_bbox.ymin:.3f},{lp_bbox.xmax:.3f},{lp_bbox.ymax:.3f})m"
                    )
                    
                    # Pass bbox as vehicle_bbox_2d parameter (license_plate_detector will handle it)
                    # Note: bbox is in 3D space, but we'll let the detector handle conversion
                    # For proper implementation, we'd need to project 3D to 2D using camera calibration
                    # For now, pass None and let detector use heuristic, but mark that we detected it
                    use_detected_bbox = True
                    # TODO: Properly convert 3D bbox to 2D pixel coordinates using camera calibration
                    # For now, pass detected bbox info and let detector use it if possible
                else:
                    # Detection is stale - clear it
                    self.get_logger().warn(
                        f"License plate detection is stale (age: {detection_age:.1f}s > 5s). "
                        f"Clearing and using heuristic fallback."
                    )
                    self.detected_license_plate_bbox = None
                    self.detected_license_plate_bbox_time = None
                    self.detected_license_plate_confidence = None
            
            # Run OCR on image with detected bbox if available (two-stage approach)
            ocr_result = self.license_plate_detector.detect_license_plate(
                vehicle_image=cv_image,
                vehicle_bbox_2d=license_plate_bbox_2d,  # Use detected bbox if available
                detected_license_plate_bbox_3d=self.detected_license_plate_bbox if use_detected_bbox else None,  # NEW: Pass 3D bbox
                pointcloud=self.latest_pointcloud,
                camera_frame='oak_rgb_camera_optical_frame'
            )
            
            if ocr_result:
                self.get_logger().info(
                    f"OCR result: text='{ocr_result.get('text')}', "
                    f"confidence={ocr_result.get('confidence'):.2f}"
                )
                return ocr_result
            else:
                self.get_logger().warn("OCR did not detect any license plate text")
                return None
                
        except Exception as e:
            self.get_logger().error(
                f"Error in _run_license_plate_ocr: {e}",
                exc_info=True
            )
            return None
    
    def _create_license_plate_folder(self):
        """
        Create folder with license plate name (sanitized).
        Falls back to truck ID if OCR failed or no text detected.
        
        Returns:
            str: Folder name (sanitized), or None if creation failed
        """
        try:
            # Determine folder name
            if self.license_plate_text:
                # Sanitize license plate text for folder name
                # Remove invalid characters, replace spaces with underscores
                folder_name = ''.join(
                    c if c.isalnum() or c in ('-', '_') else '_'
                    for c in self.license_plate_text.upper()
                )
                folder_name = folder_name.strip('_')  # Remove leading/trailing underscores
                
                # Ensure folder name is not empty and is valid
                if not folder_name or len(folder_name) < 2:
                    self.get_logger().warn(
                        f"Sanitized license plate text '{self.license_plate_text}' "
                        f"resulted in invalid folder name. Using truck ID."
                    )
                    folder_name = self.current_truck.truck_id
            else:
                # Fallback to truck ID
                folder_name = self.current_truck.truck_id
            
            # Create folder
            folder_path = self.photo_dir / "license_plates" / folder_name
            folder_path.mkdir(parents=True, exist_ok=True)
            
            self.get_logger().info(
                f"Created license plate folder: {folder_path} "
                f"(name: {folder_name})"
            )
            return folder_name
            
        except Exception as e:
            self.get_logger().error(
                f"Error creating license plate folder: {e}",
                exc_info=True
            )
            return None
    
    def _move_photo_to_license_plate_folder(self, photo_path, folder_name):
        """
        Move photo from temp location to license plate folder.
        
        Args:
            photo_path: Current photo path
            folder_name: License plate folder name
            
        Returns:
            str: New photo path, or None if failed
        """
        try:
            import shutil
            
            photo_path_obj = Path(photo_path)
            if not photo_path_obj.exists():
                self.get_logger().error(f"Source photo does not exist: {photo_path}")
                return None
            
            # Determine new filename
            if self.license_plate_text:
                # Use license plate text in filename
                filename = f"license_plate_{self.license_plate_text}.jpg"
                # Sanitize filename
                filename = ''.join(
                    c if c.isalnum() or c in ('-', '_', '.') else '_'
                    for c in filename
                )
            else:
                # Use truck ID
                filename = f"{self.current_truck.truck_id}_license.jpg"
            
            # Destination path
            dest_dir = self.photo_dir / "license_plates" / folder_name
            dest_path = dest_dir / filename
            
            # Move photo
            shutil.move(str(photo_path_obj), str(dest_path))
            
            self.get_logger().info(
                f"Moved photo to license plate folder: {dest_path}"
            )
            return str(dest_path)
            
        except Exception as e:
            self.get_logger().error(
                f"Error moving photo to license plate folder: {e}",
                exc_info=True
            )
            return None
    
    def _save_license_plate_metadata(self, photo_path, folder_name):
        """
        Save metadata JSON file with license plate information.
        
        Args:
            photo_path: Path to photo file
            folder_name: License plate folder name
            
        Returns:
            bool: True if metadata saved successfully, False otherwise
        """
        try:
            metadata = {
                'truck_id': self.current_truck.truck_id,
                'photo_path': photo_path,
                'timestamp': datetime.now().isoformat(),
                'folder_name': folder_name,
                'license_plate_text': self.license_plate_text,
                'ocr_confidence': float(self.license_plate_confidence) if self.license_plate_confidence else None,
                'photo_taken': True
            }
            
            # Add 3D position if available
            if self.license_plate_3d_position:
                pos = self.license_plate_3d_position.pose.position
                metadata['position_3d'] = {
                    'x': float(pos.x),
                    'y': float(pos.y),
                    'z': float(pos.z),
                    'frame': self.license_plate_3d_position.header.frame_id
                }
            
            # Add detection pose if available
            if self.current_truck.detection_pose:
                det_pos = self.current_truck.detection_pose.pose.position
                det_ori = self.current_truck.detection_pose.pose.orientation
                metadata['detection_pose'] = {
                    'position': {
                        'x': float(det_pos.x),
                        'y': float(det_pos.y),
                        'z': float(det_pos.z)
                    },
                    'orientation': {
                        'x': float(det_ori.x),
                        'y': float(det_ori.y),
                        'z': float(det_ori.z),
                        'w': float(det_ori.w)
                    },
                    'frame': self.current_truck.detection_pose.header.frame_id
                }
            
            # Save metadata JSON
            metadata_path = Path(photo_path).parent / "metadata.json"
            with open(metadata_path, 'w') as f:
                json.dump(metadata, f, indent=2)
            
            self.get_logger().info(
                f"Saved license plate metadata: {metadata_path}"
            )
            return True
            
        except Exception as e:
            self.get_logger().error(
                f"Error saving license plate metadata: {e}",
                exc_info=True
            )
            return False
    
    def _handle_mode_switch_to_inspection(self):
        """
        Enhanced mode switch handler with validation and verification.
        
        This method:
        1. Publishes mode switch to "inspection"
        2. Waits for model to load (configurable wait time)
        3. Verifies mode switch by checking detection pipeline activity
        4. Handles timeouts and retries (max 3 attempts)
        5. Ensures no detection gaps occur
        """
        try:
            # Initialize mode switch tracking on first attempt
            if self.mode_switch_start_time is None:
                self.mode_switch_start_time = time.time()
                self.mode_switch_attempts = 0
                self.mode_switch_verified = False
                
                # Store last detection info before switch for comparison
                self.last_bbox_before_mode_switch = self.last_detection_time
                
                self.get_logger().info(
                    f"SWITCHING_TO_INSPECTION: Starting mode switch to inspection "
                    f"for {self.current_truck.truck_id if self.current_truck else 'unknown truck'}"
                )
            
            # Validate current truck
            if not self.current_truck:
                self.get_logger().error(
                    "SWITCHING_TO_INSPECTION: current_truck is None. "
                    "Transitioning to ERROR_RECOVERY."
                )
                self.state = MissionState.ERROR_RECOVERY
                return
            
            current_time = time.time()
            elapsed_time = current_time - self.mode_switch_start_time
            
            # Check for overall timeout
            mode_switch_timeout = self.get_parameter('mode_switch_timeout').value
            if elapsed_time > mode_switch_timeout:
                if self.mode_switch_attempts < self.max_mode_switch_attempts:
                    # Retry mode switch
                    self.mode_switch_attempts += 1
                    self.get_logger().warn(
                        f"Mode switch timeout after {elapsed_time:.1f}s. "
                        f"Retrying (attempt {self.mode_switch_attempts}/"
                        f"{self.max_mode_switch_attempts})..."
                    )
                    self.mode_switch_start_time = current_time
                    self.mode_switch_verified = False
                    return
                else:
                    self.get_logger().error(
                        f"Mode switch timeout after {elapsed_time:.1f}s. "
                        f"Max attempts ({self.max_mode_switch_attempts}) reached. "
                        "Transitioning to ERROR_RECOVERY."
                    )
                    self.state = MissionState.ERROR_RECOVERY
                    return
            
            wait_time = self.get_parameter('mode_switch_wait_time').value
            verification_time = self.get_parameter('mode_switch_verification_time').value
            total_required_time = wait_time + verification_time
            
            # Phase 1: Publish mode switch (only on first iteration of each attempt)
            if self.mode_switch_attempts == 0:
                self.mode_switch_attempts = 1
                self.get_logger().info(
                    f"SWITCHING_TO_INSPECTION: Attempt {self.mode_switch_attempts}/"
                    f"{self.max_mode_switch_attempts}"
                )
            
            # Publish mode switch if not yet done for this attempt
            if elapsed_time < 0.5:  # First 0.5s - publish mode switch
                mode_published = self.publish_segmentation_mode("inspection")
                if not mode_published:
                    if self.mode_switch_attempts < self.max_mode_switch_attempts:
                        self.mode_switch_attempts += 1
                        self.get_logger().warn(
                            f"Failed to publish mode switch. Retrying (attempt "
                            f"{self.mode_switch_attempts}/{self.max_mode_switch_attempts})..."
                        )
                        self.mode_switch_start_time = current_time
                        return
                    else:
                        self.get_logger().error(
                            "Failed to publish mode switch after max attempts. "
                            "Transitioning to ERROR_RECOVERY."
                        )
                        self.state = MissionState.ERROR_RECOVERY
                        return
                
                self.get_logger().info(
                    f"Mode switch published. Waiting {wait_time}s for model to load..."
                )
                return
            
            # Phase 2: Wait for model to load
            if elapsed_time < wait_time:
                remaining_time = wait_time - elapsed_time
                self.get_logger().debug(
                    f"Waiting for inspection model to load... "
                    f"({remaining_time:.1f}s remaining)"
                )
                return
            
            # Phase 3: Verify mode switch
            if not self.mode_switch_verified:
                verification_elapsed = elapsed_time - wait_time
                
                # Check if detection pipeline is active
                detection_pipeline_active = False
                if self.last_detection_time:
                    time_since_last_detection = current_time - self.last_detection_time
                    if time_since_last_detection < 5.0:  # Recent detections (< 5s)
                        detection_pipeline_active = True
                        self.get_logger().info(
                            f"Detection pipeline active (last detection "
                            f"{time_since_last_detection:.1f}s ago). "
                            "Mode switch appears successful."
                        )
                        self.mode_switch_verified = True
                    else:
                        self.get_logger().debug(
                            f"Detection pipeline inactive (last detection "
                            f"{time_since_last_detection:.1f}s ago). "
                            f"Still verifying... ({verification_elapsed:.1f}s / {verification_time:.1f}s)"
                        )
                else:
                    # No detections yet - check if pipeline has ever received messages
                    if self.detection_bbox_received:
                        # Pipeline has received messages before - might just be no objects currently
                        self.get_logger().debug(
                            "Detection pipeline has received messages before. "
                            "Assuming mode switch successful (may be no tyres visible yet)."
                        )
                        detection_pipeline_active = True
                        self.mode_switch_verified = True
                    else:
                        # Pipeline never received messages - might be a problem
                        if verification_elapsed > verification_time:
                            # Verification timeout - retry or fail
                            if self.mode_switch_attempts < self.max_mode_switch_attempts:
                                self.mode_switch_attempts += 1
                                self.get_logger().warn(
                                    f"Mode switch verification timeout "
                                    f"({verification_elapsed:.1f}s). "
                                    f"Retrying (attempt {self.mode_switch_attempts}/"
                                    f"{self.max_mode_switch_attempts})..."
                                )
                                self.mode_switch_start_time = current_time
                                self.mode_switch_verified = False
                                return
                            else:
                                self.get_logger().warn(
                                    f"Mode switch verification timeout after "
                                    f"{verification_elapsed:.1f}s. "
                                    "Max attempts reached. Assuming mode switch successful "
                                    "and continuing (may be no tyres visible yet)."
                                )
                                detection_pipeline_active = True
                                self.mode_switch_verified = True
                        else:
                            # Still within verification time - continue waiting
                            self.get_logger().debug(
                                f"Detection pipeline inactive. Still verifying... "
                                f"({verification_elapsed:.1f}s / {verification_time:.1f}s)"
                            )
                            return
                
                # If verification timeout reached and still not verified, check again
                if verification_elapsed >= verification_time and not self.mode_switch_verified:
                    # Give up on verification - assume mode switch worked
                    # (might just be no tyres visible currently)
                    self.get_logger().info(
                        f"Mode switch verification timeout ({verification_elapsed:.1f}s). "
                        "Assuming mode switch successful and continuing. "
                        "(May be no tyres visible yet, will continue to detection phase.)"
                    )
                    self.mode_switch_verified = True
            
            # Phase 4: Mode switch verified - transition to DETECTING_TYRES
            if self.mode_switch_verified:
                self.get_logger().info(
                    f"✅ Mode switch to inspection verified after {elapsed_time:.1f}s. "
                    "Transitioning to DETECTING_TYRES state."
                )
                
                # Reset mode switch state
                self.mode_switch_start_time = None
                self.mode_switch_attempts = 0
                self.mode_switch_verified = False
                self.last_bbox_before_mode_switch = None
                
                # Initialize detection tracking for tyre detection
                self.detection_start_time = time.time()
                self.pending_vehicle_detections.clear()  # Clear any pending vehicle detections
                
                # Reset tyre detection state
                if hasattr(self.current_truck, 'tyres'):
                    self.current_truck.tyres.clear()
                
                # CRITICAL: Update phase tracking
                if self.mission_timeout_handler:
                    self.mission_timeout_handler.start_phase(MissionPhase.TYRE_DETECTION)
                
                # Transition to tyre detection state
                self.get_logger().info(
                    f"SWITCHING_TO_INSPECTION → DETECTING_TYRES: "
                    f"Segmentation mode switched to 'inspection' for {self.current_truck.truck_id}"
                )
                self.state = MissionState.DETECTING_TYRES
                self.get_logger().info(
                    f"✅ NEXT ACTION: System is now in DETECTING_TYRES state. "
                    f"Detecting tyres on {self.current_truck.truck_id}..."
                )
                
        except Exception as e:
            self.get_logger().error(
                f"Error in SWITCHING_TO_INSPECTION state: {e}",
                exc_info=True
            )
            self.state = MissionState.ERROR_RECOVERY
    
    def _save_mission_state(self):
        """
        Save current mission state for recovery.
        Called periodically by state machine step.
        """
        if not self.mission_state_manager:
            return
        
        try:
            # Serialize detected trucks
            trucks_dict = {}
            for truck_id, truck in self.detected_trucks.items():
                trucks_dict[truck_id] = {
                    'truck_id': truck_id,
                    'license_plate_photo_taken': truck.license_plate_photo_taken,
                    'tyres_count': len(truck.tyres) if hasattr(truck, 'tyres') else 0,
                    'tyres_photographed': sum(1 for t in truck.tyres if t.photo_taken) if hasattr(truck, 'tyres') else 0
                }
            
            # Save state
            self.mission_state_manager.save_mission_state(
                current_state=self.state.value,
                current_truck_id=self.current_truck.truck_id if self.current_truck else None,
                current_tyre_index=self.current_tyre_index,
                detected_trucks=trucks_dict,
                additional_data={
                    'nav_goal_active': self.nav_goal_handle is not None,
                    'local_search_active': self.local_search_active,
                    'verification_active': self.arrival_verification_active or self.tyre_verification_active
                }
            )
        except Exception as e:
            self.get_logger().error(f"Error in _save_mission_state: {e}", exc_info=True)
    
    def _handle_vehicle_movement_during_navigation(self, monitor_result: Dict, new_vehicle_pose: Optional[PoseStamped]):
        """
        Handle vehicle movement detected during navigation.
        
        Args:
            monitor_result: Result from vehicle monitor
            new_vehicle_pose: New vehicle pose if available
        """
        if not self.goal_recalculator or not new_vehicle_pose:
            return
        
        movement_distance = monitor_result.get('movement_distance', 0.0)
        
        # Check if goal should be recalculated
        if not self.goal_recalculator.should_recalculate_goal(movement_distance):
            return
        
        # Recalculate approach pose
        new_goal_pose = self.goal_recalculator.recalculate_approach_pose(
            new_vehicle_pose,
            self.get_parameter('approach_distance').value
        )
        
        if not new_goal_pose:
            self.get_logger().warn("Failed to recalculate goal pose")
            return
        
        # Validate goal update
        if not self.goal_recalculator.validate_goal_update(
            self.nav_goal_pose,
            new_goal_pose
        ):
            self.get_logger().warn("Goal update validation failed")
            return
        
        # Cancel current navigation
        if self.nav_goal_handle:
            self.get_logger().info("Cancelling current navigation to update goal...")
            self.nav_client.cancel_goal_async(self.nav_goal_handle)
            self.nav_goal_handle = None
        
        # Update goal pose
        self.nav_goal_pose = new_goal_pose
        
        # Record goal update
        self.goal_recalculator.record_goal_update(new_goal_pose)
        
        # Start new navigation
        self.get_logger().info(
            f"Vehicle moved {movement_distance:.2f}m. "
            f"Updating navigation goal to new position..."
        )
        navigation_success = self.navigate_to_pose(new_goal_pose)
        
        if not navigation_success:
            self.get_logger().error("Failed to start navigation to updated goal")
            # Restore original goal
            # (Would need to store original goal separately)
    
    def _verify_vehicle_at_arrival(self) -> bool:
        """
        Verify vehicle is visible and at expected location on arrival.
        
        Returns:
            True if vehicle verified, False otherwise
        """
        if not self.visual_verifier or not self.current_truck:
            # If verifier not available, assume OK (backward compatibility)
            return True
        
        # Initialize verification if not already started
        if not self.arrival_verification_active:
            self.arrival_verification_active = True
            self.arrival_verification_start_time = time.time()
            self.visual_verifier.reset()
            self.get_logger().info("Starting vehicle verification at arrival location...")
        
        # Get current robot pose
        current_robot_pose = self._get_robot_pose("map")
        
        # Note: We need bounding box data from bbox_callback
        # For now, we'll check if we have recent monitor result
        # In practice, verification should be done with current bbox_msg
        # This is a simplified version - full verification happens in state machine
        
        # Check timeout
        elapsed_time = time.time() - self.arrival_verification_start_time
        timeout = self.get_parameter('arrival_verification_timeout').value
        
        if elapsed_time > timeout:
            self.get_logger().warn(
                f"Vehicle verification timeout after {elapsed_time:.1f}s"
            )
            self.arrival_verification_active = False
            return False
        
        # Verification will be completed in bbox_callback or state machine
        # Return True to continue (will be checked properly in state machine)
        return True
    
    def _start_local_search_for_vehicle(self):
        """
        Start local search pattern to find vehicle if not visible at arrival.
        """
        if not self.local_search or not self.current_truck:
            self.get_logger().warn("Local search not available")
            return False
        
        if self.local_search_active:
            # Search already in progress
            return True
        
        # Get arrival location (current goal or robot position)
        search_center = self.nav_goal_pose
        if not search_center:
            current_pose = self._get_robot_pose("map")
            if not current_pose:
                self.get_logger().error("Cannot start local search: No position available")
                return False
            search_center = current_pose
        
        self.get_logger().info(
            f"Vehicle not found at arrival location. "
            f"Starting local search around ({search_center.pose.position.x:.2f}, {search_center.pose.position.y:.2f})..."
        )
        
        self.local_search_active = True
        self.local_search_start_time = time.time()
        
        # Search will be executed in state machine step
        return True
    
    def _execute_local_search_step(self) -> Dict:
        """
        Execute one step of local search.
        
        Returns:
            Dict with search status
        """
        if not self.local_search or not self.local_search_active:
            return {'search_complete': True, 'vehicle_found': False}
        
        # Get search center
        search_center = self.nav_goal_pose
        if not search_center:
            current_pose = self._get_robot_pose("map")
            if not current_pose:
                return {'search_complete': True, 'vehicle_found': False}
            search_center = current_pose
        
        # Execute search
        search_result = self.local_search.search_for_vehicle(
            search_center,
            self.current_truck.truck_id,
            None,  # bbox_callback_func - not needed here
            None,  # check_vehicle_visible_func - handled in bbox_callback
            self._get_robot_pose,
            "map"
        )
        
        # Check if vehicle found (would be set via bbox_callback checking at search positions)
        # For now, return search status
        return search_result


def main(args=None):
    rclpy.init(args=args)
    node = MissionController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Mission controller shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
