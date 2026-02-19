import math
import os
import time
from typing import List, Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Header, String
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d
from ament_index_python.packages import get_package_share_directory
import yaml
import tf2_ros
from tf2_ros import TransformException


def quaternion_from_yaw(yaw: float) -> Quaternion:
    """Create a geometry_msgs/Quaternion from a yaw angle (Z-axis rotation)."""
    half_yaw = yaw * 0.5
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)
    # roll = pitch = 0, so x = y = 0
    return Quaternion(x=0.0, y=0.0, z=sy, w=cy)


def yaw_from_quaternion(q: Quaternion) -> float:
    """Extract yaw angle from a geometry_msgs/Quaternion."""
    # Roll (x-axis rotation)
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2 * (q.w * q.y - q.z * q.x)
    if abs(sinp) >= 1:
        pitch = math.copysign(math.pi / 2, sinp)  # use 90 degrees if out of range
    else:
        pitch = math.asin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return yaw


class MissionState:
    IDLE = "IDLE"
    SEARCH_VEHICLE = "SEARCH_VEHICLE"  # Search for vehicles (car or truck)
    TURN_IN_PLACE_SEARCH = "TURN_IN_PLACE_SEARCH"  # Rotate to look for vehicles when none detected
    WAIT_VEHICLE_BOX = "WAIT_VEHICLE_BOX"  # Wait for vehicle detection
    TURN_IN_PLACE_VEHICLE = "TURN_IN_PLACE_VEHICLE"  # Rotate to find vehicle
    APPROACH_VEHICLE = "APPROACH_VEHICLE"  # Navigate to detected vehicle
    WAIT_TIRE_BOX = "WAIT_TIRE_BOX"  # Wait for tire detection
    TURN_IN_PLACE_TIRE = "TURN_IN_PLACE_TIRE"  # Rotate to find tires
    INSPECT_TIRE = "INSPECT_TIRE"  # Navigate to tire and capture photo
    NEXT_VEHICLE = "NEXT_VEHICLE"  # Move to next vehicle
    DONE = "DONE"


class VehicleInspectionManager(Node):
    """Mission manager for autonomous vehicle tire inspection using Aurora SLAM and detection."""

    def __init__(self):
        super().__init__("inspection_manager")

        # Vehicle detection parameters - support both car and truck
        self.declare_parameter("vehicle_labels", "car,truck")  # Comma-separated string of vehicle class names (e.g., "car,truck")
        self.declare_parameter("tire_label", "tire")  # Tire/wheel class name
        self.declare_parameter("standoff_distance", 2.0)  # Distance to stop before vehicle
        self.declare_parameter("approach_offset", 0.5)  # Offset when approaching vehicle
        self.declare_parameter("tire_offset", 0.4)  # Offset when approaching tire
        self.declare_parameter("detection_topic", "/darknet_ros_3d/bounding_boxes")  # Aurora detection topic
        self.declare_parameter("world_frame", "map")  # Aurora map frame
        self.declare_parameter("base_frame", "base_link")  # Robot base frame
        self.declare_parameter("detection_timeout", 5.0)  # seconds to wait before recovery
        self.declare_parameter("rotation_angle", 0.785)  # 45 degrees in radians
        self.declare_parameter("max_rotation_attempts", 8)  # 8 * 45 = 360 degrees
        self.declare_parameter("rotation_position_offset", 0.1)  # Small forward offset to force Nav2 execution
        self.declare_parameter("segmentation_mode_topic", "/segmentation_mode")  # Topic to control segmentation model
        self.declare_parameter("tire_position_tolerance", 0.5)  # meters - tires closer than this are considered the same
        self.declare_parameter("max_tire_distance_from_vehicle", 5.0)  # meters - tires beyond this are from other vehicles
        self.declare_parameter("use_dynamic_detection", True)  # Use detection instead of YAML file
        self.declare_parameter("trucks_file", "")  # Optional YAML file (for backward compatibility, but not used if use_dynamic_detection=True)
        self.declare_parameter("min_vehicle_probability", 0.5)  # Minimum probability for vehicle detection
        self.declare_parameter("min_tire_probability", 0.35)  # Minimum probability for tire detection
        self.declare_parameter("photo_capture_topic", "/inspection_manager/capture_photo")  # Topic to trigger photo capture

        # Dynamic vehicle detection - vehicles found during mission
        self.detected_vehicles: List[dict] = []  # List of {"box": BoundingBox3d, "position": (x,y,z), "inspected": bool}
        self.current_vehicle_idx = 0
        self.current_state = MissionState.IDLE
        self.pending_goal_handle = None
        self.current_vehicle_box: Optional[BoundingBox3d] = None
        self.current_tire_idx = 0
        
        # Tire tracking: store positions of inspected tires to avoid duplicates
        self.inspected_tire_positions: List[tuple] = []  # List of (x, y, z) tuples
        
        # Recovery state tracking
        self.wait_start_time: Optional[float] = None
        self.rotation_attempts = 0
        self.initial_wait_yaw: Optional[float] = None  # Store yaw when starting to wait

        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        detection_topic = self.get_parameter("detection_topic").value
        self.detection_sub = self.create_subscription(
            BoundingBoxes3d, detection_topic, self._detection_cb, qos_profile=qos
        )

        # Publish current FSM state for debugging.
        self.state_pub = self.create_publisher(String, "inspection_state", 10)
        
        # Publish segmentation mode to control which model the segmentation node uses
        segmentation_mode_topic = self.get_parameter("segmentation_mode_topic").value
        self.segmentation_mode_pub = self.create_publisher(String, segmentation_mode_topic, 10)
        self.get_logger().info(f"Publishing segmentation mode to: {segmentation_mode_topic}")

        self.nav_client = ActionClient(self, NavigateToPose, "navigate_to_pose")
        
        # TF buffer and listener for getting robot pose
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Photo capture publisher (for future integration)
        from std_msgs.msg import Bool
        self.photo_capture_pub = self.create_publisher(Bool, self.get_parameter("photo_capture_topic").value, 10)

        self.timer = self.create_timer(1.0, self._tick)
        
        use_dynamic = self.get_parameter("use_dynamic_detection").value
        if use_dynamic:
            self.get_logger().info("Using dynamic vehicle detection from bounding boxes (Aurora-based)")
        else:
            trucks = self._load_trucks()
            self.get_logger().info(f"Loaded {len(trucks)} vehicles from YAML file (legacy mode)")

        # Publish initial state.
        self._set_state(MissionState.IDLE)

    # ----------------------- State Helpers ----------------------- #
    def _set_state(self, new_state: str):
        """Update internal state, log transition, and publish for debugging."""
        if new_state == self.current_state:
            return
        prev_state = self.current_state
        self.get_logger().info(f"State transition: {self.current_state} -> {new_state}")
        self.current_state = new_state
        msg = String()
        msg.data = self.current_state
        self.state_pub.publish(msg)
        
        # Publish segmentation mode based on state
        # Use navigation model (Model 1) when waiting for vehicle detection
        if new_state == MissionState.WAIT_VEHICLE_BOX:
            mode_msg = String()
            mode_msg.data = "navigation"
            self.segmentation_mode_pub.publish(mode_msg)
            self.get_logger().info("Published segmentation mode: navigation (Model 1) - detecting vehicles")
        # Use inspection model (Model 2) when waiting for tire detection
        elif new_state == MissionState.WAIT_TIRE_BOX:
            mode_msg = String()
            mode_msg.data = "inspection"
            self.segmentation_mode_pub.publish(mode_msg)
            self.get_logger().info("Published segmentation mode: inspection (Model 2) - detecting tires")
        
        # Reset wait timer and rotation tracking when entering wait states
        if new_state in [MissionState.WAIT_VEHICLE_BOX, MissionState.WAIT_TIRE_BOX]:
            self.wait_start_time = time.time()
            self.rotation_attempts = 0
        elif new_state == MissionState.SEARCH_VEHICLE:
            self.wait_start_time = time.time()
            if prev_state == MissionState.IDLE:
                self.rotation_attempts = 0  # Reset only when starting fresh from IDLE
            current_yaw = self._get_current_yaw()
            self.initial_wait_yaw = current_yaw
            if current_yaw is not None:
                self.get_logger().info(f"Starting wait state with initial yaw: {math.degrees(current_yaw):.2f}°")
        elif new_state not in [MissionState.TURN_IN_PLACE_VEHICLE, MissionState.TURN_IN_PLACE_TIRE]:
            self.wait_start_time = None
            # Don't reset rotation_attempts here - keep it for the next wait cycle

    def _load_trucks(self) -> List[dict]:
        """Load vehicles from YAML file (legacy mode, only used if use_dynamic_detection=False)."""
        trucks_file = self.get_parameter("trucks_file").value
        if not trucks_file:
            # Default to package share config path (works in install and from source)
            try:
                share_dir = get_package_share_directory("inspection_manager")
                trucks_file = os.path.join(share_dir, "config", "trucks.yaml")
            except Exception as exc:  # pragma: no cover - defensive
                self.get_logger().warn(
                    f"Failed to resolve package share directory: {exc}"
                )
                return []

        if not os.path.exists(trucks_file):
            self.get_logger().warn(f"No vehicles file at {trucks_file}, starting empty.")
            return []

        with open(trucks_file, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        trucks = data.get("trucks", [])
        return trucks
    
    def _save_vehicle_position(self, box: BoundingBox3d):
        """Save detected vehicle position to detected_vehicles list."""
        center_x = (box.xmin + box.xmax) / 2.0
        center_y = (box.ymin + box.ymax) / 2.0
        center_z = (box.zmin + box.zmax) / 2.0
        
        vehicle_data = {
            "box": box,
            "position": (center_x, center_y, center_z),
            "inspected": False,
            "class": box.object_name,
            "probability": box.probability
        }
        
        # Check if this vehicle was already detected (avoid duplicates)
        tolerance = 2.0  # meters - vehicles closer than this are considered the same
        for existing in self.detected_vehicles:
            existing_pos = existing["position"]
            dist = math.sqrt(
                (center_x - existing_pos[0])**2 +
                (center_y - existing_pos[1])**2 +
                (center_z - existing_pos[2])**2
            )
            if dist < tolerance:
                self.get_logger().debug(f"Vehicle at ({center_x:.2f}, {center_y:.2f}) already detected, skipping duplicate")
                return
        
        self.detected_vehicles.append(vehicle_data)
        self.get_logger().info(
            f"Saved vehicle position: {box.object_name} at ({center_x:.2f}, {center_y:.2f}, {center_z:.2f}) "
            f"(probability: {box.probability:.3f}). Total vehicles: {len(self.detected_vehicles)}"
        )

    # ----------------------- State Machine ----------------------- #
    def _tick(self):
        use_dynamic = self.get_parameter("use_dynamic_detection").value
        
        if self.current_state == MissionState.IDLE:
            if use_dynamic:
                self.get_logger().info("Starting mission with dynamic vehicle detection (Aurora-based).")
                self.get_logger().info("Robot will search for vehicles (car/truck) and save their positions.")
                self._set_state(MissionState.SEARCH_VEHICLE)
            else:
                trucks = self._load_trucks()
                if not trucks:
                    self.get_logger().warn("No vehicles configured; mission done.")
                    self._set_state(MissionState.DONE)
                    return
                self.get_logger().info("Starting mission with YAML vehicle list (legacy mode).")
                self._dispatch_standoff_goal()
            return

        if self.current_state == MissionState.SEARCH_VEHICLE:
            # In dynamic mode, wait for vehicle detection to populate detected_vehicles list
            # Once we have vehicles, move to first vehicle
            if len(self.detected_vehicles) > 0:
                # Find first un-inspected vehicle
                un_inspected = [v for v in self.detected_vehicles if not v["inspected"]]
                if un_inspected:
                    self.current_vehicle_idx = self.detected_vehicles.index(un_inspected[0])
                    self.current_vehicle_box = un_inspected[0]["box"]
                    self.get_logger().info(
                        f"Found {len(self.detected_vehicles)} vehicle(s). "
                        f"Starting inspection of vehicle {self.current_vehicle_idx + 1} ({un_inspected[0]['class']})."
                    )
                    self._set_state(MissionState.WAIT_VEHICLE_BOX)
                else:
                    self.get_logger().info("All detected vehicles have been inspected. Mission complete.")
                    self._set_state(MissionState.DONE)
                return
            # No vehicles detected: rotate in place to search (active search)
            if self.wait_start_time is not None:
                elapsed = time.time() - self.wait_start_time
                timeout = self.get_parameter("detection_timeout").value
                max_rot = self.get_parameter("max_rotation_attempts").value
                if elapsed > timeout:
                    if self.rotation_attempts < max_rot:
                        self.get_logger().info(
                            f"No vehicle detected after {elapsed:.1f}s. "
                            f"Rotating to search (attempt {self.rotation_attempts + 1}/{max_rot})."
                        )
                        self._dispatch_rotation_goal(is_vehicle=False, is_search=True)
                    else:
                        self.get_logger().warn(
                            "Max search rotations reached with no vehicles detected. Mission complete."
                        )
                        self._set_state(MissionState.DONE)
            return

        if self.current_state == MissionState.TURN_IN_PLACE_SEARCH:
            return  # waiting on rotation to complete

        if self.current_state == MissionState.WAIT_VEHICLE_BOX:
            # Check for timeout and trigger recovery if needed
            if self.wait_start_time is not None:
                elapsed = time.time() - self.wait_start_time
                timeout = self.get_parameter("detection_timeout").value
                if elapsed > timeout:
                    if self.rotation_attempts < self.get_parameter("max_rotation_attempts").value:
                        self.get_logger().warn(
                            f"No vehicle detection after {elapsed:.1f}s. Attempting recovery rotation {self.rotation_attempts + 1}."
                        )
                        self._dispatch_rotation_goal(is_vehicle=True)
                    else:
                        self.get_logger().warn(
                            f"Max rotation attempts reached. Moving to next vehicle or completing mission."
                        )
                        self._set_state(MissionState.NEXT_VEHICLE)
            return  # waiting on detections

        if self.current_state == MissionState.TURN_IN_PLACE_VEHICLE:
            return  # waiting on rotation to complete

        if self.current_state == MissionState.APPROACH_VEHICLE:
            return  # waiting on navigation to complete

        if self.current_state == MissionState.WAIT_TIRE_BOX:
            # Check for timeout and trigger recovery if needed
            if self.wait_start_time is not None:
                elapsed = time.time() - self.wait_start_time
                timeout = self.get_parameter("detection_timeout").value
                if elapsed > timeout:
                    if self.rotation_attempts < self.get_parameter("max_rotation_attempts").value:
                        self.get_logger().warn(
                            f"No un-inspected tire detection after {elapsed:.1f}s. "
                            f"Attempting recovery rotation {self.rotation_attempts + 1}. "
                            f"Already inspected {len(self.inspected_tire_positions)} tires."
                        )
                        self._dispatch_rotation_goal(is_vehicle=False)
                    else:
                        # Check if we have enough tires inspected (typically 4 per vehicle)
                        inspected_count = len(self.inspected_tire_positions)
                        self.get_logger().warn(
                            f"Max rotation attempts reached. Inspected {inspected_count} tires. "
                            f"Moving to next vehicle."
                        )
                        self._set_state(MissionState.NEXT_VEHICLE)
            return

        if self.current_state == MissionState.TURN_IN_PLACE_TIRE:
            return  # waiting on rotation to complete

        if self.current_state == MissionState.INSPECT_TIRE:
            return  # waiting on navigation and photo capture

        if self.current_state == MissionState.NEXT_VEHICLE:
            # Mark current vehicle as inspected
            if self.current_vehicle_idx < len(self.detected_vehicles):
                self.detected_vehicles[self.current_vehicle_idx]["inspected"] = True
            
            # Find next un-inspected vehicle
            un_inspected = [v for v in self.detected_vehicles if not v["inspected"]]
            if un_inspected:
                self.current_vehicle_idx = self.detected_vehicles.index(un_inspected[0])
                self.current_vehicle_box = un_inspected[0]["box"]
                self.current_tire_idx = 0
                self.inspected_tire_positions = []  # Reset for next vehicle
                self.rotation_attempts = 0  # Reset rotation attempts
                self.get_logger().info(
                    f"Moving to vehicle {self.current_vehicle_idx + 1}/{len(self.detected_vehicles)} "
                    f"({un_inspected[0]['class']})."
                )
                self._set_state(MissionState.WAIT_VEHICLE_BOX)
            else:
                self.get_logger().info("All vehicles inspected. Mission complete.")
                self._set_state(MissionState.DONE)
            return

    # ----------------------- Detections ----------------------- #
    def _detection_cb(self, msg: BoundingBoxes3d):
        use_dynamic = self.get_parameter("use_dynamic_detection").value
        vehicle_labels_str = self.get_parameter("vehicle_labels").value
        # Parse comma-separated vehicle labels
        if isinstance(vehicle_labels_str, str):
            vehicle_labels = [label.strip() for label in vehicle_labels_str.split(",")]
        else:
            vehicle_labels = vehicle_labels_str if isinstance(vehicle_labels_str, list) else ["car", "truck"]
        tire_label = self.get_parameter("tire_label").value
        min_vehicle_prob = self.get_parameter("min_vehicle_probability").value
        min_tire_prob = self.get_parameter("min_tire_probability").value
        
        # Always save vehicle positions when detected (for dynamic mode)
        if use_dynamic:
            for box in msg.bounding_boxes:
                if box.object_name.lower() in [label.lower() for label in vehicle_labels]:
                    if box.probability >= min_vehicle_prob:
                        self._save_vehicle_position(box)
        
        # Handle vehicle detection state
        if self.current_state == MissionState.SEARCH_VEHICLE or self.current_state == MissionState.WAIT_VEHICLE_BOX:
            vehicle_box = self._find_vehicle_box(msg.bounding_boxes, vehicle_labels, min_vehicle_prob)
            if vehicle_box:
                # Log bounding box details
                self._log_bounding_box(vehicle_box, f"VEHICLE_{vehicle_box.object_name.upper()}")
                self.current_vehicle_box = vehicle_box
                
                # Save vehicle position if in dynamic mode
                if use_dynamic:
                    self._save_vehicle_position(vehicle_box)
                
                self.get_logger().info(
                    f"Vehicle detected ({vehicle_box.object_name}, prob: {vehicle_box.probability:.3f}); approaching."
                )
                self._dispatch_box_goal(vehicle_box, offset=self.get_parameter("approach_offset").value)
                self._set_state(MissionState.APPROACH_VEHICLE)
            return

        # Handle tire detection state
        if self.current_state == MissionState.WAIT_TIRE_BOX:
            tire_box = self._find_tire_for_inspection(msg.bounding_boxes, tire_label, min_tire_prob)
            if tire_box:
                # Log bounding box details
                tire_num = len(self.inspected_tire_positions) + 1
                self._log_bounding_box(tire_box, f"TIRE_{tire_num}")
                tire_center = (
                    (tire_box.xmin + tire_box.xmax) / 2.0,
                    (tire_box.ymin + tire_box.ymax) / 2.0,
                    (tire_box.zmin + tire_box.zmax) / 2.0
                )
                self.get_logger().info(
                    f"Tire {tire_num} detected at ({tire_center[0]:.2f}, {tire_center[1]:.2f}, {tire_center[2]:.2f}); "
                    f"moving to inspect. Already inspected: {len(self.inspected_tire_positions)}"
                )
                # Store tire position BEFORE navigating to avoid detecting it again while navigating
                self.inspected_tire_positions.append(tire_center)
                self._dispatch_box_goal(tire_box, offset=self.get_parameter("tire_offset").value)
                self._set_state(MissionState.INSPECT_TIRE)
            else:
                # Log why no tire was selected
                all_tires = [b for b in msg.bounding_boxes 
                            if b.object_name.lower() == tire_label.lower() 
                            and b.probability >= min_tire_prob]
                if all_tires:
                    self.get_logger().debug(
                        f"Found {len(all_tires)} tire(s) but none selected. "
                        f"Already inspected: {len(self.inspected_tire_positions)}"
                    )
            return

    def _find_box(self, boxes: List[BoundingBox3d], label: str, index: Optional[int] = None) -> Optional[BoundingBox3d]:
        """Find a box matching the label (legacy method, kept for compatibility)."""
        filtered = [b for b in boxes if b.object_name.lower() == label.lower()]
        if not filtered:
            return None
        if index is not None and index < len(filtered):
            return filtered[index]
        # otherwise pick the highest probability
        return sorted(filtered, key=lambda b: b.probability, reverse=True)[0]
    
    def _find_vehicle_box(self, boxes: List[BoundingBox3d], vehicle_labels: List[str], min_prob: float) -> Optional[BoundingBox3d]:
        """Find a vehicle box (car or truck) matching the labels with minimum probability."""
        filtered = []
        for box in boxes:
            if box.object_name.lower() in [label.lower() for label in vehicle_labels]:
                if box.probability >= min_prob:
                    filtered.append(box)
        
        if not filtered:
            return None
        
        # Return highest probability vehicle
        return sorted(filtered, key=lambda b: b.probability, reverse=True)[0]

    def _find_tire_for_inspection(self, boxes: List[BoundingBox3d], tire_label: str, min_prob: float) -> Optional[BoundingBox3d]:
        """Find a tire that:
        1. Matches the tire label and meets minimum probability
        2. Has not been inspected yet (not in inspected_tire_positions)
        3. Belongs to the current vehicle (within reasonable distance)
        4. Returns the closest un-inspected tire to the ROBOT's current position
        """
        filtered = [b for b in boxes 
                   if b.object_name.lower() == tire_label.lower() 
                   and b.probability >= min_prob]
        
        if not filtered:
            return None
        
        # Get robot's current position (prefer this for selecting nearest tire)
        robot_pose = self._get_current_pose()
        if robot_pose is None:
            self.get_logger().warn("Cannot get robot pose for tire selection")
            # Fallback: just return first un-inspected tire
            robot_pos = None
        else:
            robot_pos = (
                robot_pose.pose.position.x,
                robot_pose.pose.position.y,
                robot_pose.pose.position.z
            )
        
        # Get current vehicle position for filtering (only tires near this vehicle)
        vehicle_pos = None
        max_tire_distance_from_vehicle = self.get_parameter("max_tire_distance_from_vehicle").value
        
        if self.current_vehicle_box:
            # Use detected vehicle box center
            vehicle_pos = (
                (self.current_vehicle_box.xmin + self.current_vehicle_box.xmax) / 2.0,
                (self.current_vehicle_box.ymin + self.current_vehicle_box.ymax) / 2.0,
                (self.current_vehicle_box.zmin + self.current_vehicle_box.zmax) / 2.0
            )
        elif self.current_vehicle_idx < len(self.detected_vehicles):
            # Fallback: use saved vehicle position
            vehicle_pos = self.detected_vehicles[self.current_vehicle_idx]["position"]
        
        # Filter out already inspected tires and tires from other vehicles
        un_inspected = []
        tolerance = self.get_parameter("tire_position_tolerance").value
        
        for box in filtered:
            tire_center = (
                (box.xmin + box.xmax) / 2.0,
                (box.ymin + box.ymax) / 2.0,
                (box.zmin + box.zmax) / 2.0
            )
            
            # Check if this tire position was already inspected
            already_inspected = False
            for inspected_pos in self.inspected_tire_positions:
                dist = math.sqrt(
                    (tire_center[0] - inspected_pos[0])**2 +
                    (tire_center[1] - inspected_pos[1])**2 +
                    (tire_center[2] - inspected_pos[2])**2
                )
                if dist < tolerance:
                    already_inspected = True
                    break
            
            if already_inspected:
                continue
            
            # Filter by vehicle proximity if we have vehicle position
            if vehicle_pos:
                dist_to_vehicle = math.sqrt(
                    (tire_center[0] - vehicle_pos[0])**2 +
                    (tire_center[1] - vehicle_pos[1])**2 +
                    (tire_center[2] - vehicle_pos[2])**2
                )
                if dist_to_vehicle > max_tire_distance_from_vehicle:
                    self.get_logger().debug(
                        f"Tire at ({tire_center[0]:.2f}, {tire_center[1]:.2f}) is "
                        f"{dist_to_vehicle:.2f}m from vehicle (max: {max_tire_distance_from_vehicle}m) - skipping"
                    )
                    continue
            
            un_inspected.append(box)
        
        if not un_inspected:
            self.get_logger().debug(
                f"No un-inspected tires found. Total tires: {len(filtered)}, "
                f"Already inspected: {len(self.inspected_tire_positions)}"
            )
            return None
        
        # Find tire closest to ROBOT's current position (not vehicle position)
        # This ensures robot goes to nearest un-inspected tire
        if robot_pos:
            def distance_to_robot(box: BoundingBox3d) -> float:
                tire_center = (
                    (box.xmin + box.xmax) / 2.0,
                    (box.ymin + box.ymax) / 2.0,
                    (box.zmin + box.zmax) / 2.0
                )
                return math.sqrt(
                    (tire_center[0] - robot_pos[0])**2 +
                    (tire_center[1] - robot_pos[1])**2 +
                    (tire_center[2] - robot_pos[2])**2
                )
            # Sort by distance to robot, then by probability
            closest_tire = min(un_inspected, key=lambda b: (distance_to_robot(b), -b.probability))
        else:
            # Fallback: use highest probability
            closest_tire = max(un_inspected, key=lambda b: b.probability)
        
        return closest_tire

    def _log_bounding_box(self, box: BoundingBox3d, label: str):
        """Log detailed bounding box information."""
        center_x = (box.xmin + box.xmax) / 2.0
        center_y = (box.ymin + box.ymax) / 2.0
        center_z = (box.zmin + box.zmax) / 2.0
        width = box.xmax - box.xmin
        height = box.ymax - box.ymin
        depth = box.zmax - box.zmin
        
        self.get_logger().info(
            f"[{label}] Bounding Box Details:\n"
            f"  Object: {box.object_name}\n"
            f"  Probability: {box.probability:.3f}\n"
            f"  Center: ({center_x:.3f}, {center_y:.3f}, {center_z:.3f})\n"
            f"  Dimensions: W={width:.3f}, H={height:.3f}, D={depth:.3f}\n"
            f"  Bounds: X=[{box.xmin:.3f}, {box.xmax:.3f}], "
            f"Y=[{box.ymin:.3f}, {box.ymax:.3f}], "
            f"Z=[{box.zmin:.3f}, {box.zmax:.3f}]"
        )

    # ----------------------- Goals ----------------------- #
    def _dispatch_standoff_goal(self):
        """Dispatch standoff goal (legacy mode only - uses YAML file)."""
        trucks = self._load_trucks()
        if self.current_vehicle_idx >= len(trucks):
            return
        truck = trucks[self.current_vehicle_idx]
        standoff = self.get_parameter("standoff_distance").value
        yaw = truck.get("yaw", 0.0)
        goal_pose = self._pose_from_truck(truck, standoff, yaw)
        self.get_logger().info(
            f"Vehicle {self.current_vehicle_idx+1}/{len(trucks)}: navigating to standoff (legacy mode)."
        )
        self._send_nav_goal(goal_pose, self._on_standoff_done)
        # Note: Legacy mode uses old state names - this is for backward compatibility only

    def _dispatch_box_goal(self, box: BoundingBox3d, offset: float):
        center_x = (box.xmin + box.xmax) / 2.0
        center_y = (box.ymin + box.ymax) / 2.0
        center_z = (box.zmin + box.zmax) / 2.0

        # Approach from robot toward object: position goal 'offset' m back from object along robot->object line.
        # This ensures we drive toward the object from where we are, not from map origin.
        robot_pose = self._get_current_pose()
        if robot_pose is not None:
            dx = center_x - robot_pose.pose.position.x
            dy = center_y - robot_pose.pose.position.y
            dist = math.sqrt(dx * dx + dy * dy)
            if dist > 0.01:  # Avoid division by zero
                heading = math.atan2(dy, dx)
            else:
                heading = math.atan2(center_y, center_x)  # Fallback
        else:
            heading = math.atan2(center_y, center_x)  # Fallback if TF unavailable
        goal = PoseStamped()
        goal.header = Header()
        goal.header.frame_id = self.get_parameter("world_frame").value
        goal.pose.position.x = center_x - offset * math.cos(heading)
        goal.pose.position.y = center_y - offset * math.sin(heading)
        goal.pose.position.z = center_z
        goal.pose.orientation = quaternion_from_yaw(heading)

        self._send_nav_goal(goal, self._on_box_goal_done)

    def _send_nav_goal(self, pose: PoseStamped, done_cb):
        if not self.nav_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().error("Nav2 action server not available.")
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose
        self.pending_goal_handle = self.nav_client.send_goal_async(goal_msg)
        self.pending_goal_handle.add_done_callback(done_cb)

    def _get_current_yaw(self) -> Optional[float]:
        """Get current robot yaw from TF."""
        try:
            world_frame = self.get_parameter("world_frame").value
            base_frame = self.get_parameter("base_frame").value
            transform = self.tf_buffer.lookup_transform(
                world_frame, base_frame, rclpy.time.Time()
            )
            q = transform.transform.rotation
            return yaw_from_quaternion(q)
        except TransformException as ex:
            self.get_logger().warn(f"Could not get current pose: {ex}")
            return None

    def _get_current_pose(self) -> Optional[PoseStamped]:
        """Get current robot pose from TF."""
        try:
            world_frame = self.get_parameter("world_frame").value
            base_frame = self.get_parameter("base_frame").value
            transform = self.tf_buffer.lookup_transform(
                world_frame, base_frame, rclpy.time.Time()
            )
            pose = PoseStamped()
            pose.header.frame_id = world_frame
            pose.header.stamp = transform.header.stamp
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            return pose
        except TransformException as ex:
            self.get_logger().warn(f"Could not get current pose: {ex}")
            return None

    def _dispatch_rotation_goal(self, is_vehicle: bool = True, is_search: bool = False):
        """Dispatch a rotation goal to turn in place by rotation_angle.
        
        To ensure Nav2 actually executes the rotation, we add a small position offset
        in the direction of the new heading. This forces Nav2 to rotate to align
        with the goal orientation.
        """
        current_pose = self._get_current_pose()
        if current_pose is None:
            self.get_logger().error("Cannot get current pose for rotation. Aborting recovery.")
            if is_vehicle:
                self._set_state(MissionState.NEXT_VEHICLE)
            else:
                self.current_tire_idx += 1
                if self.current_tire_idx >= 4:
                    self._set_state(MissionState.NEXT_VEHICLE)
                else:
                    self._set_state(MissionState.WAIT_TIRE_BOX)
            return

        current_yaw = yaw_from_quaternion(current_pose.pose.orientation)
        rotation_angle = self.get_parameter("rotation_angle").value
        new_yaw = current_yaw + rotation_angle
        
        # Normalize yaw to [-pi, pi]
        new_yaw = math.atan2(math.sin(new_yaw), math.cos(new_yaw))
        
        # Calculate actual yaw difference (accounting for wrap-around)
        yaw_diff = new_yaw - current_yaw
        # Normalize to [-pi, pi]
        if yaw_diff > math.pi:
            yaw_diff -= 2 * math.pi
        elif yaw_diff < -math.pi:
            yaw_diff += 2 * math.pi
        
        # Add a small position offset in the direction of the new heading
        # This forces Nav2 to actually execute the rotation instead of
        # thinking it's already at the goal (due to orientation tolerance)
        offset = self.get_parameter("rotation_position_offset").value
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = current_pose.header.frame_id
        goal_pose.header.stamp = self.get_clock().now().to_msg()  # Use current time
        # Add small forward offset along new heading to force rotation execution
        goal_pose.pose.position.x = current_pose.pose.position.x + offset * math.cos(new_yaw)
        goal_pose.pose.position.y = current_pose.pose.position.y + offset * math.sin(new_yaw)
        goal_pose.pose.position.z = current_pose.pose.position.z
        goal_pose.pose.orientation = quaternion_from_yaw(new_yaw)
        
        self.rotation_attempts += 1
        current_yaw_deg = math.degrees(current_yaw)
        new_yaw_deg = math.degrees(new_yaw)
        yaw_diff_deg = math.degrees(yaw_diff)
        self.get_logger().info(
            f"Rotation goal (attempt {self.rotation_attempts}):\n"
            f"  Current yaw: {current_yaw_deg:.2f}° ({current_yaw:.3f} rad)\n"
            f"  Target yaw: {new_yaw_deg:.2f}° ({new_yaw:.3f} rad)\n"
            f"  Yaw difference: {yaw_diff_deg:.2f}° ({yaw_diff:.3f} rad)\n"
            f"  Position offset: {offset:.3f}m\n"
            f"  Goal position: ({goal_pose.pose.position.x:.3f}, {goal_pose.pose.position.y:.3f})"
        )
        
        if is_search:
            self._send_nav_goal(goal_pose, self._on_rotation_done_search)
            self._set_state(MissionState.TURN_IN_PLACE_SEARCH)
        elif is_vehicle:
            self._send_nav_goal(goal_pose, self._on_rotation_done_vehicle)
            self._set_state(MissionState.TURN_IN_PLACE_VEHICLE)
        else:
            self._send_nav_goal(goal_pose, self._on_rotation_done_tire)
            self._set_state(MissionState.TURN_IN_PLACE_TIRE)

    # ----------------------- Callbacks ----------------------- #
    def _on_standoff_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Standoff goal rejected.")
            return

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_standoff_result)

    def _on_standoff_result(self, future):
        status = future.result().status
        self.get_logger().info(f"Standoff result status: {status}")
        # Regardless of status we move to waiting for vehicle box; you may gate this on success.
        self._set_state(MissionState.WAIT_VEHICLE_BOX)

    def _on_box_goal_done(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Box goal rejected.")
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_box_result)

    def _on_box_result(self, future):
        status = future.result().status
        self.get_logger().info(f"Box goal result status: {status}")
        if self.current_state == MissionState.APPROACH_VEHICLE:
            # Reset tire tracking for this vehicle
            self.current_tire_idx = 0
            self.inspected_tire_positions = []
            self.rotation_attempts = 0  # Reset rotation attempts for tire detection
            self._set_state(MissionState.WAIT_TIRE_BOX)
            return

        if self.current_state == MissionState.INSPECT_TIRE:
            # Trigger photo capture
            from std_msgs.msg import Bool
            photo_msg = Bool()
            photo_msg.data = True
            self.photo_capture_pub.publish(photo_msg)
            self.get_logger().info(f"Photo capture triggered for tire at position {len(self.inspected_tire_positions)}")
            
            # Check if we've inspected enough tires (typically 4 per vehicle)
            if len(self.inspected_tire_positions) >= 4:
                self.get_logger().info(f"Completed inspection of {len(self.inspected_tire_positions)} tires. Moving to next vehicle.")
                self._set_state(MissionState.NEXT_VEHICLE)
            else:
                self._set_state(MissionState.WAIT_TIRE_BOX)

    def _on_rotation_done_vehicle(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Rotation goal rejected.")
            self._set_state(MissionState.NEXT_VEHICLE)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rotation_result_vehicle)

    def _on_rotation_result_vehicle(self, future):
        status = future.result().status
        self.get_logger().info(f"Rotation result status: {status}")
        # Reset wait timer to give more time after rotation
        self.wait_start_time = time.time()
        # Return to waiting for vehicle box
        self._set_state(MissionState.WAIT_VEHICLE_BOX)

    def _on_rotation_done_tire(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Rotation goal rejected.")
            # Check if we have enough tires
            if len(self.inspected_tire_positions) >= 4:
                self._set_state(MissionState.NEXT_VEHICLE)
            else:
                self._set_state(MissionState.WAIT_TIRE_BOX)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rotation_result_tire)

    def _on_rotation_result_tire(self, future):
        status = future.result().status
        self.get_logger().info(f"Rotation result status: {status}")
        # Reset wait timer to give more time after rotation
        self.wait_start_time = time.time()
        # Return to waiting for tire box
        self._set_state(MissionState.WAIT_TIRE_BOX)

    def _on_rotation_done_search(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Search rotation goal rejected.")
            self._set_state(MissionState.DONE)
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_rotation_result_search)

    def _on_rotation_result_search(self, future):
        status = future.result().status
        self.get_logger().info(f"Search rotation result status: {status}")
        self.wait_start_time = time.time()
        self._set_state(MissionState.SEARCH_VEHICLE)

    # ----------------------- Helpers ----------------------- #
    def _pose_from_truck(self, truck: dict, standoff: float, yaw: float) -> PoseStamped:
        pose = PoseStamped()
        pose.header = Header()
        pose.header.frame_id = self.get_parameter("world_frame").value

        # Stand off along -x of truck heading.
        pose.pose.position.x = truck["x"] - standoff * math.cos(yaw)
        pose.pose.position.y = truck["y"] - standoff * math.sin(yaw)
        pose.pose.position.z = truck.get("z", 0.0)

        pose.pose.orientation = quaternion_from_yaw(yaw)
        return pose


def main(args=None):
    rclpy.init(args=args)
    node = VehicleInspectionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

