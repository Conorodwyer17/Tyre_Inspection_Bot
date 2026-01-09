#!/usr/bin/env python3
"""
Mission State Manager Module

Manages mission state persistence and recovery.
Saves mission progress periodically and enables recovery from interruptions.

This module provides:
- Mission state serialization
- State persistence to disk
- State restoration on recovery
- Progress tracking
"""

import json
import time
from pathlib import Path
from typing import Optional, Dict, List
from datetime import datetime


class MissionStateManager:
    """
    Manages mission state persistence and recovery.
    """
    
    def __init__(self, logger, state_file_path: str = "~/.ros/mission_state.json"):
        """
        Initialize mission state manager.
        
        Args:
            logger: ROS 2 logger instance
            state_file_path: Path to state file (will be expanded)
        """
        self.logger = logger
        
        # Expand path
        state_path = Path(state_file_path).expanduser()
        state_path.parent.mkdir(parents=True, exist_ok=True)
        self.state_file_path = state_path
        
        # State save interval (seconds)
        self.save_interval = 30.0  # Save every 30 seconds
        self.last_save_time = None
        
        self.logger.info(f"MissionStateManager initialized: state_file={self.state_file_path}")
    
    def save_mission_state(
        self,
        current_state: str,
        current_truck_id: Optional[str] = None,
        current_tyre_index: int = 0,
        detected_trucks: Optional[Dict] = None,
        additional_data: Optional[Dict] = None
    ) -> bool:
        """
        Save current mission state to disk.
        
        Args:
            current_state: Current mission state (e.g., "navigating_to_tyre")
            current_truck_id: ID of current truck being inspected
            current_tyre_index: Current tyre index
            detected_trucks: Dict of detected trucks with their status
            additional_data: Optional additional state data
            
        Returns:
            True if saved successfully, False otherwise
        """
        try:
            current_time = time.time()
            
            # Check if we should save (throttle saves)
            if self.last_save_time:
                elapsed = current_time - self.last_save_time
                if elapsed < self.save_interval:
                    return True  # Too soon, skip save
            
            # Prepare state data
            state_data = {
                'timestamp': current_time,
                'datetime': datetime.fromtimestamp(current_time).isoformat(),
                'current_state': current_state,
                'current_truck_id': current_truck_id,
                'current_tyre_index': current_tyre_index,
                'detected_trucks': self._serialize_trucks(detected_trucks) if detected_trucks else {},
                'additional_data': additional_data or {}
            }
            
            # Save to file
            with open(self.state_file_path, 'w') as f:
                json.dump(state_data, f, indent=2, default=str)
            
            self.last_save_time = current_time
            
            self.logger.debug(
                f"Mission state saved: state={current_state}, "
                f"truck={current_truck_id}, tyre_index={current_tyre_index}"
            )
            
            return True
            
        except Exception as e:
            self.logger.error(f"Error saving mission state: {e}", exc_info=True)
            return False
    
    def load_mission_state(self) -> Optional[Dict]:
        """
        Load mission state from disk.
        
        Returns:
            Dict with mission state, or None if loading fails
        """
        try:
            if not self.state_file_path.exists():
                self.logger.info("No saved mission state found")
                return None
            
            with open(self.state_file_path, 'r') as f:
                state_data = json.load(f)
            
            self.logger.info(
                f"Loaded mission state: state={state_data.get('current_state')}, "
                f"truck={state_data.get('current_truck_id')}, "
                f"tyre_index={state_data.get('current_tyre_index')}"
            )
            
            return state_data
            
        except Exception as e:
            self.logger.error(f"Error loading mission state: {e}", exc_info=True)
            return None
    
    def clear_mission_state(self):
        """Clear saved mission state."""
        try:
            if self.state_file_path.exists():
                self.state_file_path.unlink()
                self.logger.info("Mission state cleared")
        except Exception as e:
            self.logger.error(f"Error clearing mission state: {e}", exc_info=True)
    
    def _serialize_trucks(self, detected_trucks: Dict) -> Dict:
        """
        Serialize truck data for saving.
        
        Args:
            detected_trucks: Dict of TruckData objects
            
        Returns:
            Serialized dict
        """
        serialized = {}
        
        for truck_id, truck in detected_trucks.items():
            if not hasattr(truck, '__dict__'):
                continue
            
            truck_data = {
                'truck_id': truck_id,
                'license_plate_photo_taken': getattr(truck, 'license_plate_photo_taken', False),
                'license_plate_photo_path': getattr(truck, 'license_plate_photo_path', None),
                'tyres': []
            }
            
            # Serialize tyres
            if hasattr(truck, 'tyres') and truck.tyres:
                for tyre in truck.tyres:
                    tyre_data = {
                        'tyre_id': getattr(tyre, 'tyre_id', None),
                        'photo_taken': getattr(tyre, 'photo_taken', False),
                        'photo_path': getattr(tyre, 'photo_path', None),
                        'position_3d': self._serialize_point(getattr(tyre, 'position_3d', None))
                    }
                    truck_data['tyres'].append(tyre_data)
            
            serialized[truck_id] = truck_data
        
        return serialized
    
    def _serialize_point(self, point) -> Optional[Dict]:
        """Serialize Point message."""
        if not point:
            return None
        
        return {
            'x': getattr(point, 'x', 0.0),
            'y': getattr(point, 'y', 0.0),
            'z': getattr(point, 'z', 0.0)
        }
    
    def get_state_file_path(self) -> str:
        """Get path to state file."""
        return str(self.state_file_path)
