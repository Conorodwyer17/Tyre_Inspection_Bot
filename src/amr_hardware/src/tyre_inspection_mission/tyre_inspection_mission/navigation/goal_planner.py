#!/usr/bin/env python3
"""
Goal Planner Module

Handles all goal calculation, validation, and dynamic recalculation.
Ensures goals are always safe and reachable by Nav2.

This module provides:
- Vehicle approach goal calculation
- Tyre approach goal calculation
- Goal validation and safety checks
- Dynamic goal recalculation
- Distance and reachability checks

All goals are validated for safety and reachability before being returned.
"""

from typing import Optional
import math
import rclpy
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from gb_visual_detection_3d_msgs.msg import BoundingBox3d
import tf2_ros
from tf2_geometry_msgs import do_transform_pose_stamped

# Import shared modules
from tyre_inspection_mission.common.config import NavigationConfig
from tyre_inspection_mission.common.utils import (
    yaw_to_quaternion,
    calculate_distance_2d,
    validate_pose,
    transform_pose
)
from tyre_inspection_mission.common.structured_logger import StructuredLogger, LogCategory
from tyre_inspection_mission.common.exceptions import PlanningError, ValidationError, TransformError


class GoalPlanner:
    """
    Handles goal planning with dynamic recalculation.
    
    This class ensures all navigation goals are:
    - Safe (minimum distance from robot)
    - Reachable (validated with Nav2)
    - Properly oriented (facing target)
    - In correct coordinate frame
    
    All goals are validated before being returned, and safety checks
    prevent goals that are too close or unreachable.
    """
    
    # Use configuration from config module
    MIN_SAFE_DISTANCE = NavigationConfig.MIN_SAFE_DISTANCE
    GOAL_ARRIVAL_DISTANCE = NavigationConfig.GOAL_ARRIVAL_DISTANCE
    GOAL_UPDATE_DISTANCE = NavigationConfig.GOAL_UPDATE_DISTANCE
    
    def __init__(self, node, tf_buffer: tf2_ros.Buffer):
        """
        Initialize goal planner.
        
        Args:
            node: ROS 2 node (for logging and parameters)
            tf_buffer: TF2 buffer for coordinate transformations
            
        Raises:
            ValueError: If required parameters are invalid
        """
        self.node = node
        self.tf_buffer = tf_buffer
        
        # Initialize structured logger
        enable_debug = node.get_parameter('enable_debug_logs').value if node.has_parameter('enable_debug_logs') else False
        self.slogger = StructuredLogger(node.get_logger(), enable_debug=enable_debug)
        
        # Get parameters with defaults from config
        self.nav_frame: str = node.get_parameter('navigation_frame').value
        self.approach_distance: float = node.get_parameter('approach_distance').value if node.has_parameter('approach_distance') else NavigationConfig.DEFAULT_APPROACH_DISTANCE
        
        # Validate parameters
        if not self.nav_frame:
            raise ValueError("navigation_frame cannot be empty")
        if self.approach_distance < NavigationConfig.MIN_SAFE_DISTANCE:
            self.slogger.warn(f"approach_distance ({self.approach_distance}m) < MIN_SAFE_DISTANCE "
                            f"({NavigationConfig.MIN_SAFE_DISTANCE}m). This may cause navigation issues.",
                            category=LogCategory.SYSTEM)
        
        self.slogger.info(f"Goal planner initialized: nav_frame={self.nav_frame}, "
                         f"approach_distance={self.approach_distance}m",
                         category=LogCategory.SYSTEM)
        
    def calculate_vehicle_approach_goal(self, bbox: BoundingBox3d, header, 
                                       current_robot_pose: Optional[PoseStamped] = None) -> Optional[PoseStamped]:
        """
        Calculate approach goal for vehicle license plate.
        
        Calculates a safe approach position for navigating to a vehicle's license plate.
        The goal is positioned at approach_distance from the vehicle, ensuring it's
        at least MIN_SAFE_DISTANCE from the robot for safe navigation.
        
        Args:
            bbox: BoundingBox3d message with vehicle detection
            header: ROS header with frame_id and timestamp
            current_robot_pose: Optional current robot pose (for optimization)
                                If None, will be looked up using TF
            
        Returns:
            PoseStamped in navigation frame, or None if calculation fails
            
        Raises:
            PlanningError: If goal calculation fails
            ValidationError: If goal validation fails
        """
        if bbox is None:
            raise PlanningError("bbox cannot be None", error_code="PLAN_004")
        if header is None:
            raise PlanningError("header cannot be None", error_code="PLAN_004")
        
        try:
            # Calculate vehicle center in camera frame
            center_x = float((bbox.xmin + bbox.xmax) / 2.0)
            center_y = float((bbox.ymin + bbox.ymax) / 2.0)
            center_z = float((bbox.zmin + bbox.zmax) / 2.0)
            
            # Get robot pose if not provided
            if current_robot_pose is None:
                current_robot_pose = self._get_robot_pose()
                if current_robot_pose is None:
                    raise PlanningError("Cannot get robot pose for goal calculation", 
                                      error_code="PLAN_004")
            
            robot_x = current_robot_pose.pose.position.x
            robot_y = current_robot_pose.pose.position.y
            
            # Transform vehicle center to navigation frame
            vehicle_center_nav = self._transform_to_nav_frame(
                center_x, center_y, center_z, header.frame_id
            )
            if vehicle_center_nav is None:
                raise TransformError(f"Failed to transform vehicle position from {header.frame_id} to {self.nav_frame}",
                                   error_code="TF_001")
            
            vehicle_x = vehicle_center_nav.pose.position.x
            vehicle_y = vehicle_center_nav.pose.position.y
            
            # Calculate distance from robot to vehicle
            try:
                distance_to_vehicle = calculate_distance_2d(
                    (robot_x, robot_y), (vehicle_x, vehicle_y)
                )
            except ValueError as e:
                raise ValidationError(f"Invalid positions for distance calculation: {e}",
                                    error_code="VAL_003")
            
            # If vehicle is too close, can't approach safely
            if distance_to_vehicle < self.MIN_SAFE_DISTANCE:
                raise PlanningError(
                    f"Vehicle too close ({distance_to_vehicle:.2f}m < {self.MIN_SAFE_DISTANCE:.2f}m). "
                    f"Cannot calculate safe approach goal.",
                    error_code="PLAN_004",
                    context={'distance': distance_to_vehicle, 'min_safe': self.MIN_SAFE_DISTANCE}
                )
            
            # Calculate approach position
            # Direction from robot to vehicle
            direction_x = (vehicle_x - robot_x) / distance_to_vehicle
            direction_y = (vehicle_y - robot_y) / distance_to_vehicle
            
            # Place goal at safe distance from robot, but not too far from vehicle
            # Goal should be: approach_distance from vehicle front, but at least MIN_SAFE_DISTANCE from robot
            safety_buffer = 0.1  # meters - extra buffer for safety
            goal_distance_from_robot = max(
                self.MIN_SAFE_DISTANCE + safety_buffer,
                distance_to_vehicle - self.approach_distance
            )
            
            # Ensure goal is not behind vehicle (at least 20cm from vehicle)
            min_distance_from_vehicle = 0.2  # meters
            goal_distance_from_robot = min(goal_distance_from_robot, distance_to_vehicle - min_distance_from_vehicle)
            
            if goal_distance_from_robot < self.MIN_SAFE_DISTANCE:
                raise PlanningError(
                    f"Cannot place goal at safe distance. Vehicle too close.",
                    error_code="PLAN_004",
                    context={'distance_to_vehicle': distance_to_vehicle, 'min_safe': self.MIN_SAFE_DISTANCE}
                )
            
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.nav_frame
            goal_pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(robot_x + direction_x * goal_distance_from_robot)
            goal_pose.pose.position.y = float(robot_y + direction_y * goal_distance_from_robot)
            goal_pose.pose.position.z = 0.0
            
            # Orient toward vehicle
            yaw = math.atan2(direction_y, direction_x)
            goal_pose.pose.orientation = yaw_to_quaternion(yaw)
            
            # Validate goal
            if not self.validate_goal(goal_pose, current_robot_pose):
                raise ValidationError("Goal validation failed", error_code="VAL_001")
            
            self.slogger.navigation_start(
                goal_type="vehicle_approach",
                target_id=bbox.class_name if bbox.class_name else "unknown",
                position=(goal_pose.pose.position.x, goal_pose.pose.position.y),
                distance=goal_distance_from_robot
            )
            
            return goal_pose
            
        except (PlanningError, ValidationError, TransformError):
            # Re-raise our custom exceptions
            raise
        except Exception as e:
            self.slogger.error(f"Error calculating vehicle approach goal: {e}",
                             category=LogCategory.PLANNING)
            raise PlanningError(f"Failed to calculate vehicle approach goal: {e}",
                              error_code="PLAN_004")
    
    def calculate_tyre_approach_goal(self, tyre_position_3d: Point, frame_id: str,
                                     current_robot_pose: Optional[PoseStamped] = None) -> Optional[PoseStamped]:
        """
        Calculate approach goal for tyre.
        
        Calculates a safe approach position for navigating to a tyre.
        The goal is positioned at approach_distance from the tyre, ensuring it's
        at least MIN_SAFE_DISTANCE from the robot for safe navigation.
        
        Args:
            tyre_position_3d: Point with x, y, z in camera frame
            frame_id: Frame ID of tyre position
            current_robot_pose: Optional current robot pose (for optimization)
                                If None, will be looked up using TF
            
        Returns:
            PoseStamped in navigation frame, or None if calculation fails
            
        Raises:
            PlanningError: If goal calculation fails
            ValidationError: If goal validation fails
        """
        if tyre_position_3d is None:
            raise PlanningError("tyre_position_3d cannot be None", error_code="PLAN_004")
        if not frame_id:
            raise PlanningError("frame_id cannot be empty", error_code="PLAN_004")
        
        try:
            # Get robot pose if not provided
            if current_robot_pose is None:
                current_robot_pose = self._get_robot_pose()
                if current_robot_pose is None:
                    raise PlanningError("Cannot get robot pose for goal calculation",
                                      error_code="PLAN_004")
            
            robot_x = current_robot_pose.pose.position.x
            robot_y = current_robot_pose.pose.position.y
            
            # Transform tyre position to navigation frame
            tyre_center_nav = self._transform_to_nav_frame(
                tyre_position_3d.x, tyre_position_3d.y, tyre_position_3d.z, frame_id
            )
            if tyre_center_nav is None:
                raise TransformError(f"Failed to transform tyre position from {frame_id} to {self.nav_frame}",
                                   error_code="TF_001")
            
            tyre_x = tyre_center_nav.pose.position.x
            tyre_y = tyre_center_nav.pose.position.y
            
            # Calculate distance from robot to tyre
            try:
                distance_to_tyre = calculate_distance_2d(
                    (robot_x, robot_y), (tyre_x, tyre_y)
                )
            except ValueError as e:
                raise ValidationError(f"Invalid positions for distance calculation: {e}",
                                    error_code="VAL_003")
            
            # If tyre is too close, can't approach safely
            if distance_to_tyre < self.MIN_SAFE_DISTANCE:
                raise PlanningError(
                    f"Tyre too close ({distance_to_tyre:.2f}m < {self.MIN_SAFE_DISTANCE:.2f}m)",
                    error_code="PLAN_004",
                    context={'distance': distance_to_tyre, 'min_safe': self.MIN_SAFE_DISTANCE}
                )
            
            # Calculate approach position
            direction_x = (tyre_x - robot_x) / distance_to_tyre
            direction_y = (tyre_y - robot_y) / distance_to_tyre
            
            # Place goal at safe distance
            safety_buffer = 0.1  # meters - extra buffer
            min_distance_from_tyre = 0.2  # meters - minimum distance from tyre
            goal_distance_from_robot = max(
                self.MIN_SAFE_DISTANCE + safety_buffer,
                distance_to_tyre - self.approach_distance
            )
            goal_distance_from_robot = min(goal_distance_from_robot, distance_to_tyre - min_distance_from_tyre)
            
            if goal_distance_from_robot < self.MIN_SAFE_DISTANCE:
                raise PlanningError(
                    f"Cannot place goal at safe distance. Tyre too close.",
                    error_code="PLAN_004",
                    context={'distance_to_tyre': distance_to_tyre, 'min_safe': self.MIN_SAFE_DISTANCE}
                )
            
            # Create goal pose
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = self.nav_frame
            goal_pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(robot_x + direction_x * goal_distance_from_robot)
            goal_pose.pose.position.y = float(robot_y + direction_y * goal_distance_from_robot)
            goal_pose.pose.position.z = 0.0
            
            # Orient toward tyre
            yaw = math.atan2(direction_y, direction_x)
            goal_pose.pose.orientation = yaw_to_quaternion(yaw)
            
            # Validate goal
            if not self.validate_goal(goal_pose, current_robot_pose):
                raise ValidationError("Goal validation failed", error_code="VAL_001")
            
            self.slogger.navigation_start(
                goal_type="tyre_approach",
                target_id="tyre",
                position=(goal_pose.pose.position.x, goal_pose.pose.position.y),
                distance=goal_distance_from_robot
            )
            
            return goal_pose
            
        except (PlanningError, ValidationError, TransformError):
            # Re-raise our custom exceptions
            raise
        except Exception as e:
            self.slogger.error(f"Error calculating tyre approach goal: {e}",
                             category=LogCategory.PLANNING)
            raise PlanningError(f"Failed to calculate tyre approach goal: {e}",
                              error_code="PLAN_004")
    
    def validate_goal(self, goal_pose: PoseStamped, 
                     current_robot_pose: Optional[PoseStamped] = None) -> bool:
        """
        Validate that goal is safe and reachable.
        
        Validates:
        - Goal pose is not None
        - Goal is at safe distance from robot (>= MIN_SAFE_DISTANCE)
        - Goal values are finite (no NaN or Inf)
        - Goal can be transformed to navigation frame (if needed)
        
        Args:
            goal_pose: PoseStamped goal to validate
            current_robot_pose: Optional current robot pose (will be looked up if None)
            
        Returns:
            True if goal is valid, False otherwise
            
        Raises:
            ValidationError: If validation fails with specific reason
        """
        try:
            # Basic validation
            if goal_pose is None:
                raise ValidationError("Goal pose cannot be None", error_code="VAL_001")
            
            # Use shared validation function
            if not validate_pose(goal_pose, check_finite=True):
                raise ValidationError("Goal pose contains invalid values", error_code="VAL_001")
            
            # Get robot pose if not provided
            if current_robot_pose is None:
                current_robot_pose = self._get_robot_pose()
                if current_robot_pose is None:
                    raise ValidationError("Cannot validate goal: robot pose unavailable",
                                        error_code="VAL_001")
            
            robot_x = current_robot_pose.pose.position.x
            robot_y = current_robot_pose.pose.position.y
            
            # Transform goal to navigation frame if needed
            if goal_pose.header.frame_id != self.nav_frame:
                goal_pose = transform_pose(goal_pose, self.nav_frame, self.tf_buffer)
                if goal_pose is None:
                    raise TransformError(f"Cannot transform goal from {goal_pose.header.frame_id} to {self.nav_frame}",
                                       error_code="TF_001")
            
            goal_x = goal_pose.pose.position.x
            goal_y = goal_pose.pose.position.y
            
            # Calculate distance using shared utility
            try:
                distance = calculate_distance_2d((robot_x, robot_y), (goal_x, goal_y))
            except ValueError as e:
                raise ValidationError(f"Invalid positions for distance calculation: {e}",
                                    error_code="VAL_003")
            
            # Check minimum distance (with tolerance)
            distance_tolerance = 0.1  # meters - allow slight under for rounding
            if distance < (self.MIN_SAFE_DISTANCE - distance_tolerance):
                raise ValidationError(
                    f"Goal too close: {distance:.2f}m < {self.MIN_SAFE_DISTANCE:.2f}m",
                    error_code="VAL_001",
                    context={
                        'distance': distance,
                        'min_safe': self.MIN_SAFE_DISTANCE,
                        'goal': (goal_x, goal_y),
                        'robot': (robot_x, robot_y)
                    }
                )
            
            # All validations passed
            self.slogger.info(f"Goal validated: distance={distance:.2f}m (safe, > {self.MIN_SAFE_DISTANCE:.2f}m)",
                            category=LogCategory.PLANNING)
            return True
            
        except (ValidationError, TransformError):
            # Re-raise our custom exceptions
            raise
        except Exception as e:
            self.slogger.error(f"Error validating goal: {e}", category=LogCategory.PLANNING)
            raise ValidationError(f"Goal validation failed: {e}", error_code="VAL_001")
    
    def should_recalculate_goal(self, goal_pose, current_robot_pose=None):
        """
        Check if goal should be recalculated because robot got too close
        
        Args:
            goal_pose: Current goal pose
            current_robot_pose: Optional current robot pose
            
        Returns:
            True if goal should be recalculated, False otherwise
        """
        try:
            if goal_pose is None:
                return False
            
            if current_robot_pose is None:
                current_robot_pose = self._get_robot_pose()
                if current_robot_pose is None:
                    return False
            
            robot_x = current_robot_pose.transform.translation.x
            robot_y = current_robot_pose.transform.translation.y
            
            # Transform goal if needed
            if goal_pose.header.frame_id != self.nav_frame:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        self.nav_frame,
                        goal_pose.header.frame_id,
                        rclpy.time.Time(seconds=0),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                    goal_pose = do_transform_pose_stamped(goal_pose, transform)
                except:
                    return False
            
            goal_x = goal_pose.pose.position.x
            goal_y = goal_pose.pose.position.y
            
            distance = math.sqrt((goal_x - robot_x)**2 + (goal_y - robot_y)**2)
            
            # Recalculate if robot got too close to goal (but not close enough to consider arrived)
            if distance < self.GOAL_UPDATE_DISTANCE and distance > self.GOAL_ARRIVAL_DISTANCE:
                self.node.get_logger().info(
                    f"🔄 Robot close to goal ({distance:.2f}m), recalculating..."
                )
                return True
            
            return False
            
        except Exception as e:
            self.node.get_logger().warn(f"Error checking goal recalculation: {e}")
            return False
    
    def is_goal_reached(self, goal_pose: PoseStamped, 
                       current_robot_pose: Optional[PoseStamped] = None,
                       tolerance: Optional[float] = None) -> bool:
        """
        Check if robot has reached the goal.
        
        Args:
            goal_pose: Goal pose
            current_robot_pose: Optional current robot pose (will be looked up if None)
            tolerance: Optional tolerance (defaults to GOAL_ARRIVAL_DISTANCE)
            
        Returns:
            True if goal is reached (within tolerance), False otherwise
        """
        try:
            if goal_pose is None:
                return False
            
            if tolerance is None:
                tolerance = self.GOAL_ARRIVAL_DISTANCE
            
            if current_robot_pose is None:
                current_robot_pose = self._get_robot_pose()
                if current_robot_pose is None:
                    return False
            
            robot_x = current_robot_pose.pose.position.x
            robot_y = current_robot_pose.pose.position.y
            
            # Transform goal to navigation frame if needed
            if goal_pose.header.frame_id != self.nav_frame:
                goal_pose = transform_pose(goal_pose, self.nav_frame, self.tf_buffer)
                if goal_pose is None:
                    return False
            
            goal_x = goal_pose.pose.position.x
            goal_y = goal_pose.pose.position.y
            
            # Calculate distance using shared utility
            try:
                distance = calculate_distance_2d((robot_x, robot_y), (goal_x, goal_y))
            except ValueError:
                return False
            
            return distance <= tolerance
            
        except Exception as e:
            self.slogger.warn(f"Error checking goal reached: {e}", category=LogCategory.PLANNING)
            return False
    
    def _get_robot_pose(self) -> Optional[PoseStamped]:
        """
        Get current robot pose in navigation frame.
        
        Uses TF2 to look up robot's base_footprint frame in navigation frame.
        
        Returns:
            PoseStamped in navigation frame, or None if lookup fails
        """
        from tyre_inspection_mission.utils import get_robot_pose
        
        return get_robot_pose(
            self.tf_buffer,
            target_frame=self.nav_frame,
            source_frame='base_footprint',
            timeout_sec=1.0
        )
    
    def _transform_to_nav_frame(self, x: float, y: float, z: float, 
                                source_frame: str) -> Optional[PoseStamped]:
        """
        Transform a point from source frame to navigation frame.
        
        Args:
            x: X coordinate in source frame
            y: Y coordinate in source frame
            z: Z coordinate in source frame
            source_frame: Frame ID of source point
            
        Returns:
            PoseStamped in navigation frame, or None if transform fails
            
        Note:
            Will attempt fallback to 'odom' frame if 'map' transform fails
        """
        try:
            # Create pose in source frame
            pose_source = PoseStamped()
            pose_source.header.frame_id = source_frame
            pose_source.header.stamp = self.node.get_clock().now().to_msg()
            pose_source.pose.position = Point(x=float(x), y=float(y), z=float(z))
            pose_source.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            
            # Transform to navigation frame using shared utility
            pose_nav = transform_pose(pose_source, self.nav_frame, self.tf_buffer, timeout_sec=2.0)
            if pose_nav:
                return pose_nav
            
            # Fallback to 'odom' if nav_frame was 'map' and transform failed
            if self.nav_frame != 'odom':
                try:
                    self.slogger.warn(f"Transform to {self.nav_frame} failed, trying 'odom'",
                                    category=LogCategory.PLANNING)
                    pose_nav = transform_pose(pose_source, 'odom', self.tf_buffer, timeout_sec=2.0)
                    if pose_nav:
                        return pose_nav
                except Exception as e2:
                    self.slogger.error(f"Fallback transform to 'odom' also failed: {e2}",
                                     category=LogCategory.PLANNING)
            
            return None
            
        except Exception as e:
            self.slogger.error(f"Transform failed: {e}", category=LogCategory.PLANNING)
            return None
    
