#!/usr/bin/env python3
"""
Tyre Identifier Module

Identifies and deduplicates tyres based on position, size, and visual features.
Prevents duplicate tyres and ensures each tyre is only photographed once.

This module provides:
- Tyre identification based on position
- Duplicate detection and prevention
- Tyre ID assignment and tracking
- Position-based matching
"""

import math
from typing import Optional, Dict, List, Tuple
from geometry_msgs.msg import Point
from gb_visual_detection_3d_msgs.msg import BoundingBox3d


class TyreIdentifier:
    """
    Identifies and deduplicates tyres to prevent duplicates.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize tyre identifier.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.duplicate_distance_threshold = parameters.get('tyre_duplicate_detection_distance', 0.5)  # meters
        self.tyre_size_tolerance = parameters.get('tyre_size_tolerance', 0.2)  # meters - size difference tolerance
        self.min_tyre_diameter = parameters.get('min_tyre_diameter', 0.3)  # meters
        self.max_tyre_diameter = parameters.get('max_tyre_diameter', 1.5)  # meters
        
        # Tyre registry (position -> tyre_id mapping)
        self.tyre_registry = {}  # position_key -> tyre_id
        self.tyre_positions = {}  # tyre_id -> position
        
        self.logger.info(
            f"TyreIdentifier initialized: "
            f"duplicate_threshold={self.duplicate_distance_threshold}m, "
            f"size_tolerance={self.tyre_size_tolerance}m"
        )
    
    def identify_tyre(
        self,
        bbox: BoundingBox3d,
        detected_position: Point,
        existing_tyres: List
    ) -> Dict:
        """
        Identify tyre and check for duplicates.
        
        Args:
            bbox: Detected tyre bounding box
            detected_position: Calculated position
            existing_tyres: List of existing TyreData objects
            
        Returns:
            Dict with:
                - 'is_duplicate': bool - Is this a duplicate?
                - 'matched_tyre_id': str or None - ID of matched tyre if duplicate
                - 'tyre_id': str - Assigned or matched tyre ID
                - 'position_key': str - Position-based key for registry
                - 'confidence': float - Identification confidence
        """
        result = {
            'is_duplicate': False,
            'matched_tyre_id': None,
            'tyre_id': None,
            'position_key': None,
            'confidence': 0.0
        }
        
        try:
            # Calculate tyre size (diameter estimate)
            width = abs(bbox.xmax - bbox.xmin)
            height = abs(bbox.ymax - bbox.ymin)
            depth = abs(bbox.zmax - bbox.zmin)
            avg_size = (width + height + depth) / 3.0
            
            # Validate tyre size
            if avg_size < self.min_tyre_diameter or avg_size > self.max_tyre_diameter:
                self.logger.debug(
                    f"Tyre size out of range: {avg_size:.2f}m "
                    f"(valid: {self.min_tyre_diameter}-{self.max_tyre_diameter}m)"
                )
                return result
            
            # Create position key (rounded to avoid floating point issues)
            position_key = self._create_position_key(detected_position)
            result['position_key'] = position_key
            
            # Check if position already registered
            if position_key in self.tyre_registry:
                # This position is already registered - duplicate
                matched_tyre_id = self.tyre_registry[position_key]
                result['is_duplicate'] = True
                result['matched_tyre_id'] = matched_tyre_id
                result['tyre_id'] = matched_tyre_id
                result['confidence'] = 1.0  # High confidence for position match
                
                self.logger.debug(
                    f"Duplicate tyre detected: position_key={position_key}, "
                    f"matched_tyre_id={matched_tyre_id}"
                )
                return result
            
            # Check against existing tyres by position proximity
            best_match = None
            min_distance = float('inf')
            
            for tyre in existing_tyres:
                if not hasattr(tyre, 'position_3d') or not tyre.position_3d:
                    continue
                
                # Calculate distance
                distance = math.sqrt(
                    (detected_position.x - tyre.position_3d.x)**2 +
                    (detected_position.y - tyre.position_3d.y)**2 +
                    (detected_position.z - tyre.position_3d.z)**2
                )
                
                # Check if within duplicate threshold
                if distance < self.duplicate_distance_threshold and distance < min_distance:
                    min_distance = distance
                    best_match = tyre
            
            if best_match:
                # Found match by proximity - duplicate
                matched_tyre_id = best_match.tyre_id
                result['is_duplicate'] = True
                result['matched_tyre_id'] = matched_tyre_id
                result['tyre_id'] = matched_tyre_id
                result['confidence'] = 1.0 - (min_distance / self.duplicate_distance_threshold)  # Higher confidence for closer match
                
                # Register position
                self.tyre_registry[position_key] = matched_tyre_id
                self.tyre_positions[matched_tyre_id] = detected_position
                
                self.logger.debug(
                    f"Duplicate tyre by proximity: distance={min_distance:.3f}m, "
                    f"matched_tyre_id={matched_tyre_id}"
                )
                return result
            
            # New tyre - generate ID
            tyre_id = self._generate_tyre_id(detected_position, bbox)
            result['tyre_id'] = tyre_id
            result['confidence'] = bbox.probability  # Use detection confidence
            
            # Register new tyre
            self.tyre_registry[position_key] = tyre_id
            self.tyre_positions[tyre_id] = detected_position
            
            self.logger.info(
                f"New tyre identified: {tyre_id} at ({detected_position.x:.2f}, {detected_position.y:.2f}, {detected_position.z:.2f})"
            )
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error identifying tyre: {e}", exc_info=True)
            return result
    
    def _create_position_key(self, position: Point) -> str:
        """
        Create position-based key for registry.
        Rounds position to avoid floating point precision issues.
        
        Args:
            position: Tyre position
            
        Returns:
            Position key string
        """
        # Round to 0.1m precision
        precision = 0.1
        x = round(position.x / precision) * precision
        y = round(position.y / precision) * precision
        z = round(position.z / precision) * precision
        
        return f"{x:.1f},{y:.1f},{z:.1f}"
    
    def _generate_tyre_id(self, position: Point, bbox: BoundingBox3d) -> str:
        """
        Generate unique tyre ID based on position and detection.
        
        Args:
            position: Tyre position
            bbox: Tyre bounding box
            
        Returns:
            Tyre ID string
        """
        # Use position-based ID (rounded)
        x = round(position.x, 1)
        y = round(position.y, 1)
        z = round(position.z, 1)
        
        # Generate ID: tyre_X_Y_Z
        tyre_id = f"tyre_{x:.1f}_{y:.1f}_{z:.1f}"
        
        # Ensure uniqueness
        counter = 1
        original_id = tyre_id
        while tyre_id in self.tyre_positions:
            tyre_id = f"{original_id}_{counter}"
            counter += 1
        
        return tyre_id
    
    def register_tyre(self, tyre_id: str, position: Point):
        """
        Register a tyre in the registry.
        
        Args:
            tyre_id: Tyre ID
            position: Tyre position
        """
        position_key = self._create_position_key(position)
        self.tyre_registry[position_key] = tyre_id
        self.tyre_positions[tyre_id] = position
        
        self.logger.debug(f"Registered tyre: {tyre_id} at {position_key}")
    
    def is_duplicate(self, position: Point, existing_tyres: List) -> Tuple[bool, Optional[str]]:
        """
        Quick check if position is a duplicate.
        
        Args:
            position: Position to check
            existing_tyres: List of existing tyres
            
        Returns:
            Tuple of (is_duplicate: bool, matched_tyre_id: str or None)
        """
        position_key = self._create_position_key(position)
        
        # Check registry
        if position_key in self.tyre_registry:
            return True, self.tyre_registry[position_key]
        
        # Check by proximity
        for tyre in existing_tyres:
            if not hasattr(tyre, 'position_3d') or not tyre.position_3d:
                continue
            
            distance = math.sqrt(
                (position.x - tyre.position_3d.x)**2 +
                (position.y - tyre.position_3d.y)**2 +
                (position.z - tyre.position_3d.z)**2
            )
            
            if distance < self.duplicate_distance_threshold:
                return True, tyre.tyre_id
        
        return False, None
    
    def get_tyre_position(self, tyre_id: str) -> Optional[Point]:
        """Get registered position for a tyre."""
        return self.tyre_positions.get(tyre_id)
    
    def clear_registry(self):
        """Clear tyre registry (call when starting new vehicle)."""
        self.tyre_registry.clear()
        self.tyre_positions.clear()
        self.logger.info("Tyre registry cleared")
    
    def get_registry_summary(self) -> Dict:
        """Get summary of registered tyres."""
        return {
            'total_tyres': len(self.tyre_positions),
            'tyre_ids': list(self.tyre_positions.keys()),
            'registry_size': len(self.tyre_registry)
        }
