#!/usr/bin/env python3
"""
Mission Resumer Module

Handles mission resumption from saved state.
Restores mission state and allows continuation from interruption.

This module provides:
- State restoration from saved file
- Mission resumption logic
- Progress validation
- State recovery verification
"""

import json
from pathlib import Path
from typing import Optional, Dict
from datetime import datetime


class MissionResumer:
    """
    Handles mission resumption from saved state.
    """
    
    def __init__(self, logger, state_manager):
        """
        Initialize mission resumer.
        
        Args:
            logger: ROS 2 logger instance
            state_manager: MissionStateManager instance
        """
        self.logger = logger
        self.state_manager = state_manager
        
        self.logger.info("MissionResumer initialized")
    
    def can_resume_mission(self) -> bool:
        """
        Check if mission can be resumed.
        
        Returns:
            True if saved state exists and is valid, False otherwise
        """
        if not self.state_manager:
            return False
        
        state = self.state_manager.load_mission_state()
        return state is not None
    
    def get_resume_info(self) -> Optional[Dict]:
        """
        Get information about resumable mission.
        
        Returns:
            Dict with resume information, or None if no saved state
        """
        if not self.state_manager:
            return None
        
        state = self.state_manager.load_mission_state()
        if not state:
            return None
        
        return {
            'state': state.get('current_state'),
            'truck_id': state.get('current_truck_id'),
            'tyre_index': state.get('current_tyre_index'),
            'timestamp': state.get('timestamp'),
            'datetime': state.get('datetime'),
            'trucks_detected': len(state.get('detected_trucks', {})),
            'trucks_info': state.get('detected_trucks', {})
        }
    
    def resume_mission(
        self,
        mission_controller
    ) -> Dict:
        """
        Resume mission from saved state.
        
        Args:
            mission_controller: MissionController instance to restore state to
            
        Returns:
            Dict with:
                - 'success': bool - Resume successful?
                - 'state_restored': bool - State restored?
                - 'truck_id': str or None - Current truck ID
                - 'tyre_index': int - Current tyre index
                - 'issues': List[str] - List of issues encountered
        """
        result = {
            'success': False,
            'state_restored': False,
            'truck_id': None,
            'tyre_index': 0,
            'issues': []
        }
        
        if not self.state_manager:
            result['issues'].append("State manager not available")
            return result
        
        try:
            # Load saved state
            state = self.state_manager.load_mission_state()
            if not state:
                result['issues'].append("No saved state found")
                return result
            
            self.logger.info(
                f"Resuming mission from saved state: "
                f"state={state.get('current_state')}, "
                f"truck={state.get('current_truck_id')}, "
                f"tyre_index={state.get('current_tyre_index')}"
            )
            
            # Restore state
            current_state = state.get('current_state')
            current_truck_id = state.get('current_truck_id')
            current_tyre_index = state.get('current_tyre_index', 0)
            
            result['truck_id'] = current_truck_id
            result['tyre_index'] = current_tyre_index
            
            # Restore detected trucks (simplified - would need full truck data)
            detected_trucks_info = state.get('detected_trucks', {})
            
            # Note: Full restoration would require:
            # - Reconstructing TruckData objects
            # - Reconstructing TyreData objects
            # - Restoring all metadata
            # For now, we just restore basic state information
            
            # Set mission controller state
            if hasattr(mission_controller, 'current_tyre_index'):
                mission_controller.current_tyre_index = current_tyre_index
            
            # Find current truck if ID provided
            if current_truck_id and hasattr(mission_controller, 'detected_trucks'):
                if current_truck_id in mission_controller.detected_trucks:
                    mission_controller.current_truck = mission_controller.detected_trucks[current_truck_id]
                    self.logger.info(f"Restored current truck: {current_truck_id}")
                else:
                    result['issues'].append(f"Truck {current_truck_id} not found in detected_trucks")
            
            # Restore mission state enum
            if hasattr(mission_controller, 'state') and current_state:
                # Map state string to enum
                from tyre_inspection_mission.core.mission_controller import MissionState
                try:
                    # CRITICAL: Don't restore IDLE or MISSION_COMPLETE states
                    if current_state in [MissionState.IDLE.value, MissionState.MISSION_COMPLETE.value, 'idle', 'mission_complete']:
                        self.logger.warn(
                            f"⚠️ Cannot resume to inactive state '{current_state}'. "
                            "This should have been caught earlier. Not restoring state."
                        )
                        result['issues'].append(f"Cannot resume to inactive state: {current_state}")
                    else:
                        # Find matching state
                        for state_enum in MissionState:
                            if state_enum.value == current_state:
                                mission_controller.state = state_enum
                                self.logger.info(f"Restored mission state: {current_state}")
                                break
                except Exception as e:
                    result['issues'].append(f"Error restoring state: {e}")
            
            result['state_restored'] = True
            result['success'] = len(result['issues']) == 0
            
            if result['success']:
                self.logger.info(
                    f"✅ Mission resumed successfully: "
                    f"truck={current_truck_id}, tyre_index={current_tyre_index}"
                )
            else:
                self.logger.warn(
                    f"⚠️ Mission resumed with issues: {', '.join(result['issues'])}"
                )
            
            return result
            
        except Exception as e:
            self.logger.error(f"Error resuming mission: {e}", exc_info=True)
            result['issues'].append(f"Error: {str(e)}")
            return result
    
    def clear_resume_state(self):
        """Clear saved state (call when mission complete)."""
        if self.state_manager:
            self.state_manager.clear_mission_state()
            self.logger.info("Resume state cleared")
