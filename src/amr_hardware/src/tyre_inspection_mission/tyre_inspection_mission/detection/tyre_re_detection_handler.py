#!/usr/bin/env python3
"""
Tyre Re-detection Handler Module

Handles re-detection of tyres during navigation to improve position accuracy.
Updates tyre positions if better detections are found, but maintains visit order.

This module provides:
- Tyre position updates from re-detections
- Position validation against stored positions
- Duplicate detection prevention
- Position accuracy improvement
"""

import math
from typing import Optional, Dict, List
from geometry_msgs.msg import Point
from gb_visual_detection_3d_msgs.msg import BoundingBox3d


class TyreReDetectionHandler:
    """
    Handles tyre re-detection and position updates during navigation.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre re-detection handler.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.position_update_threshold = parameters.get('tyre_position_update_threshold', 0.3)  # meters
        self.min_update_confidence = parameters.get('tyre_detection_confidence_threshold', 0.5)
        self.max_association_distance = parameters.get('tyre_duplicate_detection_distance', 0.5)  # meters
        
        # Re-detection tracking
        self.tyre_re_detections = {}  # tyre_id -> {count, last_update_time, best_position}
        self.enable_re_detection = True
        
        self.logger.info(
            f"TyreReDetectionHandler initialized: "
            f"update_threshold={self.position_update_threshold}m, "
            f"max_association_distance={self.max_association_distance}m"
        )
    
    def process_tyre_re_detection(
        self,
        detected_bbox: BoundingBox3d,
        detected_position: Point,
        existing_tyres: List,  # List of TyreData objects
        bbox_to_pose_func = None
    ) -> Optional[Dict]:
        """
        Process re-detection of a tyre and update position if better.
        
        Args:
            detected_bbox: Newly detected tyre bounding box
            detected_position: Calculated position from bbox
            existing_tyres: List of existing TyreData objects
            bbox_to_pose_func: Function to convert bbox to pose
            
        Returns:
            Dict with update information, or None if no update
        """
        if not self.enable_re_detection or not existing_tyres:
            return None
        
        try:
            # Find matching existing tyre (by position proximity)
            best_match = None
            min_distance = float('inf')
            
            for tyre in existing_tyres:
                if not hasattr(tyre, 'position_3d') or not tyre.position_3d:
                    continue
                
                # Calculate distance between detected position and stored position
                distance = math.sqrt(
                    (detected_position.x - tyre.position_3d.x)**2 +
                    (detected_position.y - tyre.position_3d.y)**2
                )
                
                # Check if this is a match (within association distance)
                if distance < self.max_association_distance and distance < min_distance:
                    min_distance = distance
                    best_match = tyre
            
            if best_match is None:
                # No matching tyre found - might be a new detection
                return None
            
            # Check if detected position is better (higher confidence or more accurate)
            current_position = best_match.position_3d
            position_improvement = math.sqrt(
                (detected_position.x - current_position.x)**2 +
                (detected_position.y - current_position.y)**2
            )
            
            # Only update if:
            # 1. Confidence is sufficient
            # 2. Position is significantly different (might be more accurate)
            # 3. Not already photographed (don't update positions of photographed tyres)
            if (detected_bbox.probability >= self.min_update_confidence and
                position_improvement > self.position_update_threshold and
                not (hasattr(best_match, 'photo_taken') and best_match.photo_taken)):
                
                # Update tyre position
                old_position = current_position
                best_match.position_3d = detected_position
                
                # Update bbox if available
                if hasattr(best_match, 'bbox_3d'):
                    best_match.bbox_3d = detected_bbox
                
                # Track re-detection
                tyre_id = best_match.tyre_id
                if tyre_id not in self.tyre_re_detections:
                    self.tyre_re_detections[tyre_id] = {
                        'count': 0,
                        'last_update_time': None,
                        'best_position': None
                    }
                
                self.tyre_re_detections[tyre_id]['count'] += 1
                import time
                self.tyre_re_detections[tyre_id]['last_update_time'] = time.time()
                self.tyre_re_detections[tyre_id]['best_position'] = detected_position
                
                self.logger.info(
                    f"✅ Updated tyre {tyre_id} position: "
                    f"improvement={position_improvement:.3f}m, "
                    f"confidence={detected_bbox.probability:.2f}, "
                    f"re-detections={self.tyre_re_detections[tyre_id]['count']}"
                )
                
                return {
                    'tyre_id': tyre_id,
                    'old_position': old_position,
                    'new_position': detected_position,
                    'position_improvement': position_improvement,
                    'confidence': detected_bbox.probability,
                    'updated': True
                }
            
            return None
            
        except Exception as e:
            self.logger.error(f"Error processing tyre re-detection: {e}", exc_info=True)
            return None
    
    def should_update_tyre_position(
        self,
        detected_position: Point,
        existing_position: Point,
        confidence: float
    ) -> bool:
        """
        Determine if tyre position should be updated.
        
        Args:
            detected_position: Newly detected position
            existing_position: Current stored position
            confidence: Detection confidence
            
        Returns:
            True if position should be updated, False otherwise
        """
        # Check confidence
        if confidence < self.min_update_confidence:
            return False
        
        # Check position difference
        distance = math.sqrt(
            (detected_position.x - existing_position.x)**2 +
            (detected_position.y - existing_position.y)**2
        )
        
        # Update if position is significantly different
        return distance > self.position_update_threshold
    
    def get_tyre_re_detection_stats(self, tyre_id: str) -> Dict:
        """
        Get re-detection statistics for a tyre.
        
        Args:
            tyre_id: ID of tyre
            
        Returns:
            Dict with re-detection statistics
        """
        if tyre_id not in self.tyre_re_detections:
            return {
                'count': 0,
                'last_update_time': None,
                'best_position': None
            }
        
        return self.tyre_re_detections[tyre_id].copy()
    
    def clear_tyre_stats(self, tyre_id: str):
        """Clear re-detection statistics for a tyre."""
        if tyre_id in self.tyre_re_detections:
            del self.tyre_re_detections[tyre_id]
    
    def reset(self):
        """Reset all re-detection tracking."""
        self.tyre_re_detections.clear()
        self.logger.debug("Tyre re-detection tracking reset")
