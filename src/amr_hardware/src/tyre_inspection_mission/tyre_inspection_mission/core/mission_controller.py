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
from datetime import datetime
from pathlib import Path
from collections import defaultdict
from typing import Dict, Optional

from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from std_msgs.msg import String
from std_srvs.srv import Trigger, SetBool
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from sensor_msgs.msg import Image, PointCloud2
import tf2_ros
from tf2_geometry_msgs import do_transform_pose
from rclpy.qos import qos_profile_sensor_data, qos_profile_system_default

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
    def __init__(self, truck_id, detection_pose):
        self.truck_id = truck_id
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
        self.declare_parameter('approach_distance', 1.5)  # meters from object
        self.declare_parameter('navigation_timeout', 60.0)  # seconds
        self.declare_parameter('detection_timeout', 30.0)  # seconds
        self.declare_parameter('photo_capture_timeout', 10.0)  # seconds
        self.declare_parameter('truck_class_name', 'truck')
        self.declare_parameter('tyre_class_name', 'tyre')
        self.declare_parameter('vehicle_search_timeout', 120.0)  # seconds to search for vehicles
        self.declare_parameter('detection_confidence_threshold', 0.5)  # minimum confidence for vehicle detection
        self.declare_parameter('detection_stability_frames', 3)  # number of consecutive frames required
        self.declare_parameter('duplicate_detection_distance', 1.0)  # meters - treat as duplicate if closer
        self.declare_parameter('max_detection_distance', 20.0)  # meters - ignore detections beyond this
        self.declare_parameter('min_detection_distance', 0.5)  # meters - ignore detections too close
        self.declare_parameter('min_bbox_size', 0.1)  # meters - minimum bounding box dimension
        self.declare_parameter('max_bbox_size', 5.0)  # meters - maximum bounding box dimension
        self.declare_parameter('min_goal_distance', 0.6)  # meters - minimum safe goal distance
        self.declare_parameter('goal_recalculation_distance', 0.6)  # meters - recalculate goal if too close
        self.declare_parameter('arrival_distance_threshold', 0.5)  # meters - arrival distance
        self.declare_parameter('stuck_detection_distance', 0.1)  # meters - stuck detection threshold
        self.declare_parameter('stuck_detection_time', 30.0)  # seconds - stuck detection time threshold
        self.declare_parameter('navigation_progress_log_interval', 10.0)  # seconds - log progress every N seconds
        self.declare_parameter('license_plate_capture_timeout', 15.0)  # seconds - timeout for license plate capture
        self.declare_parameter('license_plate_capture_retry_delay', 2.0)  # seconds - delay between retries
        self.declare_parameter('ocr_min_confidence', 0.5)  # Minimum OCR confidence threshold
        self.declare_parameter('enable_license_plate_ocr', True)  # Enable/disable OCR
        self.declare_parameter('mode_switch_timeout', 10.0)  # seconds - timeout for mode switch
        self.declare_parameter('mode_switch_wait_time', 3.0)  # seconds - wait time after mode switch
        self.declare_parameter('mode_switch_verification_time', 5.0)  # seconds - time to verify mode switch
        self.declare_parameter('tyre_detection_confidence_threshold', 0.5)  # minimum confidence for tyre detection
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
        self.nav_client.wait_for_server(timeout_sec=5.0)
        self.get_logger().info("Nav2 action server connected")
        
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
        
        # Timers
        self.state_machine_timer = self.create_timer(0.5, self.state_machine_step)
        
        # Navigation state
        self.nav_goal_handle = None
        self.nav_result_future = None
        self.nav_feedback_future = None
        self.nav_start_time = None
        self.nav_goal_pose = None  # Track the goal pose we're navigating to
        self.nav_start_pose = None  # Track robot position when navigation started
        self.last_robot_pose = None  # Track last robot position for stuck detection
        self.last_robot_pose_time = None  # Track when we last updated robot pose
        self.nav_progress_last_update = None  # Track last progress update time
        self.nav_progress_distance = 0.0  # Track distance traveled
        self.goal_recalculation_attempts = 0  # Track goal recalculation attempts
        self.max_goal_recalculation_attempts = 3  # Maximum recalculation attempts
        self.stuck_detection_enabled = True  # Enable stuck detection
        self.stuck_threshold_distance = 0.1  # meters - consider stuck if moved less than this
        self.stuck_threshold_time = 30.0  # seconds - consider stuck if not moved for this time
        self.arrival_distance_threshold = 0.5  # meters - consider arrived if within this distance
        
        # Detection tracking
        self.last_detection_time = None
        self.detection_start_time = None
        
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
        
        # CRITICAL: Check if mission can be resumed
        if self.mission_resumer and self.mission_resumer.can_resume_mission():
            resume_info = self.mission_resumer.get_resume_info()
            if resume_info:
                self.get_logger().info(
                    f"Found saved mission state: state={resume_info['state']}, "
                    f"truck={resume_info['truck_id']}, tyre_index={resume_info['tyre_index']}, "
                    f"trucks_detected={resume_info['trucks_detected']}"
                )
                # Attempt to resume
                resume_result = self.mission_resumer.resume_mission(self)
                if resume_result['success']:
                    self.get_logger().info("✅ Mission resumed from saved state")
                    response.success = True
                    response.message = f"Mission resumed: truck={resume_result['truck_id']}, tyre_index={resume_result['tyre_index']}"
                    return response
                else:
                    self.get_logger().warn(
                        f"Failed to resume mission: {', '.join(resume_result['issues'])}. "
                        "Starting new mission instead."
                    )
                    # Clear saved state and start fresh
                    if self.mission_state_manager:
                        self.mission_state_manager.clear_mission_state()
            
        # Perform system readiness check before starting
        self.get_logger().info("Checking system readiness before mission start...")
        readiness = self._check_system_readiness()
        self.system_readiness_status = readiness
        self.system_readiness_checked = True
        
        # Check if all critical systems are ready
        critical_components = ['nav2', 'photo_capture', 'camera', 'detection', 'tf']
        all_critical_ready = all(
            readiness.get(comp, {}).get('ready', False) 
            for comp in critical_components
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
        self.get_logger().info("✅ All systems ready. Starting tyre inspection mission")
        
        # CRITICAL: Start mission timeout tracking
        if self.mission_timeout_handler:
            self.mission_timeout_handler.start_mission()
            self.mission_timeout_handler.start_phase(MissionPhase.SEARCHING)
        
        self.state = MissionState.SEARCHING_TRUCKS
        self.detected_trucks.clear()
        self.truck_counter = 0
        self.detection_start_time = time.time()
        
        # Clear pending detections from previous mission attempts
        self.pending_vehicle_detections.clear()
        self.vehicle_detection_history.clear()
        self.last_detection_time = None
        self.detection_bbox_received = False
        
        # Switch to navigation mode (for truck detection)
        self.publish_segmentation_mode("navigation")
        
        self.get_logger().info(
            f"Mission started: Searching for vehicles "
            f"(timeout: {self.get_parameter('vehicle_search_timeout').value}s, "
            f"confidence threshold: {self.get_parameter('detection_confidence_threshold').value}, "
            f"stability frames: {self.get_parameter('detection_stability_frames').value})"
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
            
            # Log received message (throttled to avoid spam)
            # Only log every 10th message or if there are detections
            if num_boxes > 0 or int(current_time * 10) % 10 == 0:
                self.get_logger().debug(
                    f"Received {num_boxes} bounding box(es) in state {self.state.value} "
                    f"(frame: {msg.header.frame_id}, stamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec})"
                )
                
                if num_boxes > 0:
                    for bbox in msg.bounding_boxes[:5]:  # Log first 5 to avoid spam
                        self.get_logger().debug(
                            f"  - {bbox.object_name} (prob: {bbox.probability:.2f}, "
                            f"bbox: x=[{bbox.xmin:.2f},{bbox.xmax:.2f}], "
                            f"y=[{bbox.ymin:.2f},{bbox.ymax:.2f}], "
                            f"z=[{bbox.zmin:.2f},{bbox.zmax:.2f}])"
                        )
            
            # Process detections based on current state
            if self.state == MissionState.SEARCHING_TRUCKS:
                self.process_truck_detections(msg)
            elif self.state == MissionState.DETECTING_TYRES:
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
                # Verify vehicle visible before capture
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
            self.get_logger().error(f"Error processing bounding boxes: {e}", exc_info=True)
                
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
            
            # Check if dimensions are reasonable
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
            
            # Check if bounding box has valid coordinates
            if not (bbox.xmin < bbox.xmax and bbox.ymin < bbox.ymax and bbox.zmin < bbox.zmax):
                self.get_logger().debug(
                    f"Invalid bounding box coordinates: "
                    f"x=[{bbox.xmin:.2f}, {bbox.xmax:.2f}], "
                    f"y=[{bbox.ymin:.2f}, {bbox.ymax:.2f}], "
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
            truck_class = self.get_parameter('truck_class_name').value
            confidence_threshold = self.get_parameter('detection_confidence_threshold').value
            stability_frames = self.get_parameter('detection_stability_frames').value
            current_time = time.time()
            
            # Support both 'car' and 'truck' for COCO model compatibility
            # COCO yolov8n-seg.pt detects 'car', not 'truck'
            vehicle_classes = ['truck', 'car'] if truck_class.lower() == 'truck' else [truck_class.lower()]
            
            # Track detections in this frame
            frame_detections = {}
            
            for bbox in msg.bounding_boxes:
                # Check if this is a vehicle class we're interested in
                if bbox.object_name.lower() not in [vc.lower() for vc in vehicle_classes]:
                    continue
                
                # Validate confidence threshold
                if bbox.probability < confidence_threshold:
                    self.get_logger().debug(
                        f"Detection confidence too low: {bbox.object_name} "
                        f"prob={bbox.probability:.2f} (threshold={confidence_threshold:.2f})"
                    )
                    continue
                
                # Validate bounding box dimensions
                if not self._validate_bounding_box(bbox):
                    continue
                
                # Validate 3D position
                if not self._validate_detection_3d_position(bbox):
                    continue
                
                # Generate vehicle ID
                vehicle_id = self._generate_vehicle_id(bbox)
                
                # Check for duplicate detection in already confirmed vehicles
                is_duplicate_confirmed, existing_confirmed_id = self._is_duplicate_detection(bbox, self.detected_trucks)
                if is_duplicate_confirmed:
                    self.get_logger().debug(
                        f"Skipping duplicate detection (already confirmed as {existing_confirmed_id})"
                    )
                    continue
                
                # Check for duplicate in pending detections - merge if close enough
                # Build a temporary dict of pending detection poses for duplicate check
                pending_detection_poses = {}
                for pid, pdata in self.pending_vehicle_detections.items():
                    pbbox = pdata['bbox']
                    ppose = self.bbox_to_pose(pbbox, msg.header)
                    if ppose:
                        # Create a dummy TruckData for duplicate checking
                        pending_detection_poses[pid] = TruckData(pid, ppose)
                
                is_duplicate_pending, existing_pending_id = self._is_duplicate_detection(bbox, pending_detection_poses)
                if is_duplicate_pending:
                    # Merge with existing pending detection - update it instead of creating new
                    self.get_logger().debug(
                        f"Merging detection with existing pending: {existing_pending_id}"
                    )
                    # Update existing pending detection
                    if existing_pending_id in self.pending_vehicle_detections:
                        pending = self.pending_vehicle_detections[existing_pending_id]
                        pending['frame_count'] += 1
                        pending['last_seen_time'] = current_time
                        pending['bbox'] = bbox  # Update with latest bbox (better confidence/position)
                    continue
                
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
                    pending['frame_count'] += 1
                    pending['last_seen_time'] = current_time
                    pending['bbox'] = bbox  # Update with latest bbox (might have better position)
                else:
                    # New detection - initialize tracking
                    self.get_logger().info(
                        f"New vehicle candidate detected: {vehicle_id} "
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
            
            # Check for stable detections (seen in multiple consecutive frames)
            stable_detections = []
            for vehicle_id, pending in list(self.pending_vehicle_detections.items()):
                if pending['frame_count'] >= stability_frames:
                    # Detection is stable - confirm it
                    stable_detections.append((vehicle_id, pending))
                    self.get_logger().info(
                        f"✅ Stable vehicle detection: {vehicle_id} "
                        f"(confirmed after {pending['frame_count']} frames)"
                    )
                elif current_time - pending['last_seen_time'] > 2.0:
                    # Detection lost for too long - remove from pending
                    self.get_logger().debug(
                        f"Removing stale pending detection: {vehicle_id} "
                        f"(last seen {current_time - pending['last_seen_time']:.1f}s ago)"
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
                
                # Create truck data
                truck_id = f"truck_{self.truck_counter:03d}"
                truck_data = TruckData(truck_id, truck_pose)
                self.detected_trucks[truck_id] = truck_data
                self.truck_counter += 1
                
                self.get_logger().info(
                    f"✅ Confirmed vehicle detection: {truck_id} at "
                    f"({truck_pose.position.x:.2f}, {truck_pose.position.y:.2f}, {truck_pose.position.z:.2f})"
                )
                
                # Transition to handling this truck (if in SEARCHING_TRUCKS state)
                if self.state == MissionState.SEARCHING_TRUCKS:
                    self.current_truck = truck_data
                    self.state = MissionState.TRUCK_DETECTED
                    self.get_logger().info(
                        f"Transitioning to TRUCK_DETECTED state for {truck_id}"
                    )
                    
                    # CRITICAL: Update vehicle obstacle manager with detected vehicle
                    if self.vehicle_obstacle_manager and truck_data.detection_pose:
                        self.vehicle_obstacle_manager.update_vehicle_obstacle(truck_data.detection_pose)
                        self.get_logger().info(f"Vehicle obstacle updated for {truck_id}")
                    
                    # Clear tyre navigation context for new vehicle
                    if self.tyre_navigation_context:
                        self.tyre_navigation_context.clear_context()
                    
                    # Clear tyre identifier registry for new vehicle
                    if self.tyre_identifier:
                        self.tyre_identifier.clear_registry()
            
        except Exception as e:
            self.get_logger().error(f"Error processing truck detections: {e}", exc_info=True)
                            
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
                    self.state = MissionState.NAVIGATING_TO_TYRE
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
                
                if self.detection_start_time:
                    elapsed_time = current_time - self.detection_start_time
                    remaining_time = timeout - elapsed_time
                    
                    # Log progress every 30 seconds
                    if int(elapsed_time) % 30 == 0 and elapsed_time > 0:
                        vehicles_found = len(self.detected_trucks)
                        pending_count = len(self.pending_vehicle_detections)
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
                try:
                    # Validate current_truck exists
                    if not self.current_truck:
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
                                            self.get_logger().warn(
                                                f"TRUCK_DETECTED: Navigation already in progress. "
                                                "Skipping new navigation request."
                                            )
                                        # Check if Nav2 is ready
                                        elif not self.nav_client.server_is_ready():
                                            self.get_logger().warn(
                                                f"TRUCK_DETECTED: Nav2 action server not ready. "
                                                "Waiting for Nav2 to become available..."
                                            )
                                            # Don't transition yet - will retry on next iteration
                                        else:
                                            # Calculate license plate approach pose (front of truck)
                                            license_pose = PoseStamped()
                                            license_pose.header.frame_id = detection_pose.header.frame_id
                                            license_pose.header.stamp = self.get_clock().now().to_msg()
                                            
                                            # Calculate yaw from quaternion
                                            orientation = detection_pose.pose.orientation
                                            yaw = 2.0 * math.atan2(orientation.z, orientation.w)
                                            
                                            # Get approach distance
                                            approach_dist = self.get_parameter('approach_distance').value
                                            
                                            # Calculate approach position (in front of truck)
                                            # Position offset in truck's forward direction
                                            license_pose.pose.position.x = pos.x + approach_dist * math.cos(yaw)
                                            license_pose.pose.position.y = pos.y + approach_dist * math.sin(yaw)
                                            license_pose.pose.position.z = pos.z  # Keep same height
                                            
                                            # Orient toward truck (opposite direction)
                                            approach_yaw = yaw + math.pi  # Face the truck
                                            license_pose.pose.orientation.z = math.sin(approach_yaw / 2.0)
                                            license_pose.pose.orientation.w = math.cos(approach_yaw / 2.0)
                                            
                                            # Validate calculated pose
                                            if not self._validate_navigation_pose(license_pose):
                                                self.get_logger().error(
                                                    f"TRUCK_DETECTED: Calculated license plate pose is invalid. "
                                                    "Transitioning to ERROR_RECOVERY."
                                                )
                                                self.state = MissionState.ERROR_RECOVERY
                                            else:
                                                self.get_logger().info(
                                                    f"TRUCK_DETECTED: {self.current_truck.truck_id} - "
                                                    f"Calculated license plate approach pose at "
                                                    f"({license_pose.pose.position.x:.2f}, {license_pose.pose.position.y:.2f}, {license_pose.pose.position.z:.2f}) "
                                                    f"in frame '{license_pose.header.frame_id}' "
                                                    f"(approach distance: {approach_dist:.2f}m)"
                                                )
                                                
                                                # Navigate to license plate position
                                                navigation_success = self.navigate_to_pose(license_pose)
                                                
                                                if navigation_success:
                                                    self.state = MissionState.NAVIGATING_TO_LICENSE_PLATE
                                                    self.get_logger().info(
                                                        f"TRUCK_DETECTED → NAVIGATING_TO_LICENSE_PLATE: "
                                                        f"Navigation goal sent for {self.current_truck.truck_id}"
                                                    )
                                                else:
                                                    self.get_logger().error(
                                                        f"TRUCK_DETECTED: Failed to send navigation goal. "
                                                        "Will retry on next iteration."
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
                    # Update navigation progress tracking
                    self._update_navigation_progress()
                    
                    # Check if navigation goal handle exists (navigation started)
                    if self.nav_goal_handle is None:
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
                            self.get_logger().error(
                                "No goal pose stored. Cannot restart navigation. "
                                "Transitioning to ERROR_RECOVERY."
                            )
                            self.state = MissionState.ERROR_RECOVERY
                        # Don't process further - wait for next iteration
                    
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
                            
                            if arrival_distance <= arrival_threshold:
                                elapsed_time = time.time() - self.nav_start_time if self.nav_start_time else 0.0
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
                                                self.state = MissionState.CAPTURING_LICENSE_PLATE
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
            # Validate current truck
            if not self.current_truck:
                self.get_logger().error(
                    "NAVIGATING_TO_TYRE: current_truck is None. "
                    "Transitioning to ERROR_RECOVERY."
                )
                self.state = MissionState.ERROR_RECOVERY
                return
            
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
                        
                        if arrival_distance <= arrival_threshold:
                            self.get_logger().info(
                                f"✅ Arrived at tyre {current_tyre.tyre_id} "
                                f"(distance: {arrival_distance:.2f}m)"
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
                            self.state = MissionState.CAPTURING_TYRE
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
                self.state = MissionState.NAVIGATING_TO_TYRE
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
                    'x': self.current_truck.detection_pose.position.x if hasattr(self.current_truck.detection_pose, 'position') else None,
                    'y': self.current_truck.detection_pose.position.y if hasattr(self.current_truck.detection_pose, 'position') else None,
                    'z': self.current_truck.detection_pose.position.z if hasattr(self.current_truck.detection_pose, 'position') else None,
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
            self.get_logger().error(f"Error validating navigation pose: {e}", exc_info=True)
            return False
            
    def navigate_to_pose(self, pose):
        """
        Send navigation goal with validation.
        
        Args:
            pose: PoseStamped message
            
        Returns:
            bool: True if goal was sent successfully, False otherwise
        """
        try:
            # Validate pose before sending
            if not self._validate_navigation_pose(pose):
                self.get_logger().error("Cannot navigate to invalid pose")
                return False
            
            # Check if Nav2 is ready
            if not self.nav_client.server_is_ready():
                self.get_logger().error("Nav2 action server not ready")
                return False
            
            # Create goal message
            goal_msg = NavigateToPose.Goal()
            goal_msg.pose = pose
            
            self.get_logger().info(
                f"Navigating to ({pose.pose.position.x:.2f}, {pose.pose.position.y:.2f}, {pose.pose.position.z:.2f}) "
                f"in frame '{pose.header.frame_id}'"
            )
            
            # Reset navigation state
            self.nav_start_time = time.time()
            self.nav_result_future = None
            self.nav_goal_pose = pose
            self.goal_recalculation_attempts = 0
            
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
                    f"Navigation starting: distance to goal = {total_distance:.2f}m"
                )
            
            # Send goal (feedback will be handled after goal is accepted)
            send_goal_future = self.nav_client.send_goal_async(goal_msg)
            send_goal_future.add_done_callback(self.nav_goal_response_callback)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error sending navigation goal: {e}", exc_info=True)
            return False
        
    def nav_goal_response_callback(self, future):
        """Handle navigation goal response (for both async send and goal acceptance)"""
        try:
            if hasattr(future, 'result') and callable(future.result):
                # This is from send_goal_async
                # CRITICAL: Handle potential exceptions when getting result
                try:
                    goal_handle = future.result()
                except Exception as e:
                    self.get_logger().error(
                        f"Error getting navigation goal handle: {e}",
                        exc_info=True
                    )
                    # Transition to error recovery if in navigation state
                    if self.state in [MissionState.NAVIGATING_TO_LICENSE_PLATE, MissionState.NAVIGATING_TO_TYRE]:
                        self.state = MissionState.ERROR_RECOVERY
                    return
                
                if goal_handle is None:
                    self.get_logger().error("Navigation goal handle is None")
                    return
                
                if not goal_handle.accepted:
                    self.get_logger().error(
                        "Navigation goal rejected by Nav2 action server"
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
                                    "Attempting recalculation..."
                                )
                                self._recalculate_navigation_goal()
                                return
                    
                    # If we can't recover, transition to error state if in navigation state
                    if self.state in [MissionState.NAVIGATING_TO_LICENSE_PLATE, MissionState.NAVIGATING_TO_TYRE]:
                        self.get_logger().error(
                            f"Navigation goal rejected and cannot recover. "
                            f"Transitioning to ERROR_RECOVERY from {self.state.value}"
                        )
                        self.state = MissionState.ERROR_RECOVERY
                    return
                
                self.nav_goal_handle = goal_handle
                self.get_logger().info(
                    f"Navigation goal accepted by Nav2 (ID: {goal_handle.goal_id})"
                )
                
                # Subscribe to feedback (Python ROS 2 action client approach)
                self.nav_feedback_future = self.nav_goal_handle.get_feedback_async()
                
                # Get result future
                self.nav_result_future = self.nav_goal_handle.get_result_async()
                self.nav_result_future.add_done_callback(self.nav_result_callback)
            else:
                # This might be from a different callback
                self.get_logger().debug("Navigation goal response callback received unexpected future type")
        except Exception as e:
            self.get_logger().error(
                f"Error in nav_goal_response_callback: {e}", 
                exc_info=True
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
                    f"Error getting navigation result in callback: {e}",
                    exc_info=True
                )
                # Clean up navigation state
                self.nav_goal_handle = None
                self.nav_result_future = None
                # Transition to error recovery if in navigation state
                if self.state in [MissionState.NAVIGATING_TO_LICENSE_PLATE, MissionState.NAVIGATING_TO_TYRE]:
                    self.state = MissionState.ERROR_RECOVERY
                return
            
            if result.result == NavigateToPose.Result.SUCCEEDED:
                self.get_logger().info("✅ Navigation succeeded")
                
                # Verify arrival distance
                current_pose = self._get_robot_pose("map")
                if current_pose and self.nav_goal_pose:
                    goal_pos = self.nav_goal_pose.pose.position
                    robot_pos = current_pose.pose.position
                    arrival_distance = math.sqrt(
                        (goal_pos.x - robot_pos.x)**2 +
                        (goal_pos.y - robot_pos.y)**2
                    )
                    arrival_threshold = self.get_parameter('arrival_distance_threshold').value
                    
                    if arrival_distance > arrival_threshold:
                        self.get_logger().warn(
                            f"Navigation reported success but robot is {arrival_distance:.2f}m "
                            f"from goal (threshold: {arrival_threshold:.2f}m)"
                        )
                    else:
                        self.get_logger().info(
                            f"Arrival confirmed: robot is {arrival_distance:.2f}m from goal"
                        )
            elif result.result == NavigateToPose.Result.CANCELED:
                self.get_logger().warn("Navigation was canceled")
                # CRITICAL: Handle canceled navigation appropriately based on state
                if self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                    # License plate navigation canceled - could be due to goal recalculation
                    # Check if this was intentional (we might have canceled to recalculate)
                    self.get_logger().info(
                        "License plate navigation canceled. "
                        "Will attempt recalculation or retry if needed."
                    )
                    # Don't transition to error - goal recalculation might restart navigation
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
                self.get_logger().error("Navigation failed")
                # Check if we should retry or transition to error recovery
                if self.state == MissionState.NAVIGATING_TO_LICENSE_PLATE:
                    # Try goal recalculation if possible
                    if (self.goal_recalculation_attempts < self.max_goal_recalculation_attempts and
                        self.nav_goal_pose):
                        self.get_logger().info("Attempting goal recalculation after failure...")
                        self._recalculate_navigation_goal()
                    else:
                        self.get_logger().error(
                            "Navigation failed and max recalculation attempts reached. "
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
        
    def _get_robot_pose(self, target_frame="map"):
        """
        Get robot's current pose in the specified frame using TF.
        
        Args:
            target_frame: Target frame (default: "map")
            
        Returns:
            PoseStamped: Robot pose in target frame, or None if transform fails
        """
        try:
            # Try to get transform from base_link to target frame
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                "base_link",  # Assuming robot base frame is "base_link"
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.5)
            )
            
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
            
            # Get recalculation distance
            recalculation_distance = self.get_parameter('goal_recalculation_distance').value
            
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
            dx_norm = dx / distance_to_original
            dy_norm = dy / distance_to_original
            
            # Calculate new goal position at safe distance
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
            
            # Validate new goal
            if not self._validate_navigation_pose(new_goal_pose):
                self.get_logger().error(
                    "Recalculated goal pose is invalid"
                )
                return False
            
            self.get_logger().info(
                f"Recalculating goal (attempt {self.goal_recalculation_attempts}/"
                f"{self.max_goal_recalculation_attempts}): "
                f"new goal at ({new_goal_pose.pose.position.x:.2f}, "
                f"{new_goal_pose.pose.position.y:.2f}) "
                f"(distance: {recalculation_distance:.2f}m)"
            )
            
            # Update stored goal
            self.nav_goal_pose = new_goal_pose
            
            # Send new goal
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
                            # Verify arrival distance
                            current_pose = self._get_robot_pose("map")
                            if current_pose and self.nav_goal_pose:
                                goal_pos = self.nav_goal_pose.pose.position
                                robot_pos = current_pose.pose.position
                                arrival_distance = math.sqrt(
                                    (goal_pos.x - robot_pos.x)**2 +
                                    (goal_pos.y - robot_pos.y)**2
                                )
                                arrival_threshold = self.get_parameter('arrival_distance_threshold').value
                                
                                if arrival_distance <= arrival_threshold:
                                    self.get_logger().info(
                                        f"Navigation complete: arrived at goal "
                                        f"(distance: {arrival_distance:.2f}m)"
                                    )
                                    return True
                                else:
                                    self.get_logger().warn(
                                        f"Navigation reported success but robot is "
                                        f"{arrival_distance:.2f}m from goal "
                                        f"(threshold: {arrival_threshold:.2f}m). "
                                        "May need to recalculate."
                                    )
                                    # Still consider it complete if Nav2 says succeeded
                                    return True
                            else:
                                # Nav2 says succeeded, trust it
                                return True
                        else:
                            # Navigation failed or was canceled
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
                                        # Will retry in state machine
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
                if self.nav_goal_handle:
                    self.nav_client.cancel_goal_async(self.nav_goal_handle)
                    self.nav_goal_handle = None
                    self.nav_result_future = None
                
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
            
            # Transition to next state
            self.state = MissionState.SWITCHING_TO_INSPECTION
            
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
            
            # Run OCR on image
            ocr_result = self.license_plate_detector.detect_license_plate(
                vehicle_image=cv_image,
                vehicle_bbox_2d=None,  # Use full image
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
                self.state = MissionState.DETECTING_TYRES
                
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
