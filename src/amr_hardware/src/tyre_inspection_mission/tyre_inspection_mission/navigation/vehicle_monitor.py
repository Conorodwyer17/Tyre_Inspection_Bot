#!/usr/bin/env python3
"""
Vehicle Monitor Module

Monitors vehicle position and visibility during navigation.
Detects if vehicle has moved and determines if goal should be updated.

This module provides:
- Vehicle visibility monitoring during navigation
- Vehicle movement detection
- Goal update recommendations
- Vehicle position tracking
"""

import math
import time
from typing import Optional, Tuple, Dict
from geometry_msgs.msg import PoseStamped, Point
from gb_visual_detection_3d_msgs.msg import BoundingBoxes3d, BoundingBox3d


class VehicleMonitor:
    """
    Monitors vehicle during navigation to detect movement and visibility changes.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize vehicle monitor.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.vehicle_movement_threshold = parameters.get('vehicle_movement_threshold', 0.5)  # meters
        self.vehicle_visibility_timeout = parameters.get('vehicle_visibility_timeout', 5.0)  # seconds
        self.vehicle_reposition_threshold = parameters.get('vehicle_reposition_threshold', 1.0)  # meters
        self.truck_class_name = parameters.get('truck_class_name', 'truck')
        self.detection_confidence_threshold = parameters.get('detection_confidence_threshold', 0.5)
        
        # Vehicle classes to monitor (support both 'car' and 'truck')
        self.vehicle_classes = ['truck', 'car'] if self.truck_class_name.lower() == 'truck' else [self.truck_class_name.lower()]
        
        # Tracking state
        self.last_vehicle_detection_time = None
        self.last_vehicle_pose = None
        self.vehicle_visible_count = 0
        self.vehicle_not_visible_count = 0
        self.movement_detected = False
        self.last_movement_check_time = None
        
        self.logger.info(
            f"VehicleMonitor initialized: "
            f"movement_threshold={self.vehicle_movement_threshold}m, "
            f"visibility_timeout={self.vehicle_visibility_timeout}s"
        )
    
    def monitor_vehicle_during_navigation(
        self, 
        bbox_msg: BoundingBoxes3d, 
        current_truck,
        bbox_to_pose_func
    ) -> Dict:
        """
        Monitor vehicle during navigation phase.
        
        Args:
            bbox_msg: Current bounding boxes message
            current_truck: Current TruckData object with original detection_pose
            bbox_to_pose_func: Function to convert bbox to pose (from mission_controller)
            
        Returns:
            Dict with:
                - 'vehicle_visible': bool - Is vehicle currently visible?
                - 'vehicle_moved': bool - Has vehicle moved significantly?
                - 'movement_distance': float - Distance vehicle moved (meters)
                - 'should_update_goal': bool - Should navigation goal be updated?
                - 'current_vehicle_pose': PoseStamped or None - Current vehicle pose if visible
        """
        result = {
            'vehicle_visible': False,
            'vehicle_moved': False,
            'movement_distance': 0.0,
            'should_update_goal': False,
            'current_vehicle_pose': None
        }
        
        if not current_truck or not current_truck.detection_pose:
            self.logger.warn("VehicleMonitor: current_truck or detection_pose is None")
            return result
        
        current_time = time.time()
        self.last_vehicle_detection_time = current_time
        
        # Find vehicle in current detections
        vehicle_bbox = self._find_vehicle_in_detections(bbox_msg, current_truck.truck_id)
        
        if vehicle_bbox is None:
            # Vehicle not visible in current frame
            self.vehicle_not_visible_count += 1
            self.vehicle_visible_count = 0
            
            # Check if vehicle has been lost for too long
            if self.last_vehicle_detection_time:
                time_since_last_detection = current_time - self.last_vehicle_detection_time
                if time_since_last_detection > self.vehicle_visibility_timeout:
                    self.logger.warn(
                        f"VehicleMonitor: Vehicle not visible for {time_since_last_detection:.1f}s "
                        f"(timeout: {self.vehicle_visibility_timeout}s)"
                    )
            
            result['vehicle_visible'] = False
            return result
        
        # Vehicle is visible
        self.vehicle_visible_count += 1
        self.vehicle_not_visible_count = 0
        result['vehicle_visible'] = True
        
        # Calculate current vehicle pose
        try:
            current_vehicle_pose = bbox_to_pose_func(vehicle_bbox, bbox_msg.header)
            if current_vehicle_pose is None:
                self.logger.warn("VehicleMonitor: Failed to calculate current vehicle pose")
                return result
            
            result['current_vehicle_pose'] = current_vehicle_pose
            
            # Calculate movement from original detection
            movement_distance = self._calculate_movement(
                current_vehicle_pose,
                current_truck.detection_pose
            )
            
            result['movement_distance'] = movement_distance
            
            # Check if vehicle moved significantly
            if movement_distance > self.vehicle_movement_threshold:
                result['vehicle_moved'] = True
                self.movement_detected = True
                
                self.logger.info(
                    f"VehicleMonitor: Vehicle movement detected: {movement_distance:.2f}m "
                    f"(threshold: {self.vehicle_movement_threshold}m)"
                )
                
                # Recommend goal update if movement is significant
                if movement_distance > self.vehicle_reposition_threshold:
                    result['should_update_goal'] = True
                    self.logger.warn(
                        f"VehicleMonitor: Significant vehicle movement ({movement_distance:.2f}m). "
                        f"Goal update recommended."
                    )
            
            # Update last known position
            self.last_vehicle_pose = current_vehicle_pose
            
        except Exception as e:
            self.logger.error(f"VehicleMonitor: Error processing vehicle detection: {e}", exc_info=True)
        
        return result
    
    def _find_vehicle_in_detections(
        self, 
        bbox_msg: BoundingBoxes3d, 
        truck_id: str
    ) -> Optional[BoundingBox3d]:
        """
        Find vehicle matching current_truck in bounding boxes.
        
        Args:
            bbox_msg: Bounding boxes message
            truck_id: ID of truck to find
            
        Returns:
            BoundingBox3d if found, None otherwise
        """
        if not bbox_msg or not bbox_msg.bounding_boxes:
            return None
        
        for bbox in bbox_msg.bounding_boxes:
            # Check if this is a vehicle class
            if bbox.object_name.lower() not in [vc.lower() for vc in self.vehicle_classes]:
                continue
            
            # Check confidence
            if bbox.probability < self.detection_confidence_threshold:
                continue
            
            # For now, return first matching vehicle
            # TODO: Could improve by matching position to original detection
            return bbox
        
        return None
    
    def _calculate_movement(
        self, 
        current_pose: PoseStamped, 
        original_pose: PoseStamped
    ) -> float:
        """
        Calculate distance vehicle has moved from original detection.
        
        Args:
            current_pose: Current vehicle pose
            original_pose: Original detection pose
            
        Returns:
            Distance in meters
        """
        try:
            if not current_pose or not original_pose:
                return 0.0
            
            if not hasattr(current_pose, 'pose') or not hasattr(original_pose, 'pose'):
                return 0.0
            
            current_pos = current_pose.pose.position
            original_pos = original_pose.pose.position
            
            # Calculate 2D distance (x, y only)
            dx = current_pos.x - original_pos.x
            dy = current_pos.y - original_pos.y
            distance = math.sqrt(dx**2 + dy**2)
            
            return distance
            
        except Exception as e:
            self.logger.error(f"VehicleMonitor: Error calculating movement: {e}", exc_info=True)
            return 0.0
    
    def check_vehicle_visible(
        self, 
        bbox_msg: BoundingBoxes3d, 
        truck_id: str
    ) -> bool:
        """
        Quick check if vehicle is visible in current frame.
        
        Args:
            bbox_msg: Current bounding boxes
            truck_id: ID of vehicle to check
            
        Returns:
            True if vehicle visible, False otherwise
        """
        vehicle_bbox = self._find_vehicle_in_detections(bbox_msg, truck_id)
        return vehicle_bbox is not None
    
    def get_visibility_status(self) -> Dict:
        """
        Get current vehicle visibility status.
        
        Returns:
            Dict with visibility statistics
        """
        return {
            'last_detection_time': self.last_vehicle_detection_time,
            'visible_count': self.vehicle_visible_count,
            'not_visible_count': self.vehicle_not_visible_count,
            'movement_detected': self.movement_detected,
            'last_vehicle_pose': self.last_vehicle_pose
        }
    
    def reset(self):
        """Reset monitoring state."""
        self.last_vehicle_detection_time = None
        self.last_vehicle_pose = None
        self.vehicle_visible_count = 0
        self.vehicle_not_visible_count = 0
        self.movement_detected = False
        self.last_movement_check_time = None
