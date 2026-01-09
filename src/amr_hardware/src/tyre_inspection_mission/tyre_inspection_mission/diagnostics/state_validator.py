#!/usr/bin/env python3
"""
State Validator for Mission Controller

Provides validation and self-healing mechanisms for state machine transitions
and state consistency checks.
"""

from typing import Optional, Tuple, Dict, Any
from enum import Enum
import time

from tyre_inspection_mission.common.data_structures import MissionState, VehicleData, TyreData
from tyre_inspection_mission.common.structured_logger import StructuredLogger, LogCategory
from tyre_inspection_mission.common.exceptions import StateMachineError, ValidationError
from tyre_inspection_mission.common.config import (
    DetectionConfig,
    NavigationConfig,
    CaptureConfig,
    PlanningConfig
)


class StateValidationResult(Enum):
    """State validation result"""
    VALID = "valid"
    WARNING = "warning"
    INVALID = "invalid"
    RECOVERABLE = "recoverable"


class StateValidator:
    """Validates state machine state and provides self-healing suggestions"""
    
    def __init__(self, logger: StructuredLogger):
        """
        Initialize state validator
        
        Args:
            logger: Structured logger instance
        """
        self.logger = logger
        self.state_history = []  # Track state transitions
        self.max_history = 50  # Keep last 50 states
        
    def validate_state_transition(self, from_state: MissionState, to_state: MissionState,
                                 context: Optional[Dict[str, Any]] = None) -> Tuple[StateValidationResult, Optional[str]]:
        """
        Validate a state transition
        
        Args:
            from_state: Current state
            to_state: Target state
            context: Optional context dictionary
            
        Returns:
            (validation_result, reason)
        """
        # Record transition
        self.state_history.append({
            'from': from_state,
            'to': to_state,
            'time': time.time(),
            'context': context or {}
        })
        if len(self.state_history) > self.max_history:
            self.state_history.pop(0)
        
        # Check for invalid transitions
        invalid_transitions = self._get_invalid_transitions()
        if (from_state, to_state) in invalid_transitions:
            return StateValidationResult.INVALID, f"Invalid transition: {from_state.value} -> {to_state.value}"
        
        # Check for suspicious transitions (possible errors)
        suspicious = self._check_suspicious_transition(from_state, to_state)
        if suspicious:
            return StateValidationResult.WARNING, suspicious
        
        # Check for recoverable transitions (error recovery)
        if to_state == MissionState.ERROR_RECOVERY:
            return StateValidationResult.RECOVERABLE, "Entering error recovery"
        
        return StateValidationResult.VALID, None
    
    def validate_state_requirements(self, state: MissionState, 
                                   current_vehicle: Optional[VehicleData] = None,
                                   current_tyre_index: Optional[int] = None,
                                   detected_vehicles: Optional[Dict] = None) -> Tuple[StateValidationResult, Optional[str]]:
        """
        Validate that current state has required data
        
        Args:
            state: Current state
            current_vehicle: Current vehicle being processed
            current_tyre_index: Current tyre index
            detected_vehicles: Dictionary of detected vehicles
            
        Returns:
            (validation_result, reason)
        """
        # States that require current_vehicle
        vehicle_required_states = [
            MissionState.PLANNING,
            MissionState.NAVIGATING_TO_LICENSE_PLATE,
            MissionState.CAPTURING_LICENSE_PLATE,
            MissionState.SWITCHING_TO_INSPECTION,
            MissionState.DETECTING_TYRES,
            MissionState.NAVIGATING_TO_TYRE,
            MissionState.CAPTURING_TYRE,
            MissionState.CHECKING_COMPLETION
        ]
        
        if state in vehicle_required_states:
            if current_vehicle is None:
                return StateValidationResult.INVALID, f"State {state.value} requires current_vehicle but it is None"
            
            # Additional checks for specific states
            if state == MissionState.NAVIGATING_TO_TYRE:
                if current_tyre_index is None:
                    return StateValidationResult.INVALID, "NAVIGATING_TO_TYRE requires current_tyre_index"
                if current_vehicle.tyre_waypoints is None or len(current_vehicle.tyre_waypoints) == 0:
                    return StateValidationResult.WARNING, "NAVIGATING_TO_TYRE has no tyre waypoints"
            
            if state == MissionState.CAPTURING_TYRE:
                if current_tyre_index is None:
                    return StateValidationResult.INVALID, "CAPTURING_TYRE requires current_tyre_index"
                if current_vehicle.tyres is None or len(current_vehicle.tyres) == 0:
                    return StateValidationResult.WARNING, "CAPTURING_TYRE has no tyres in list (may use waypoints)"
            
            if state == MissionState.PLANNING:
                if current_vehicle.detection_pose is None:
                    return StateValidationResult.WARNING, "PLANNING state: vehicle has no detection_pose"
        
        # States that require detected vehicles
        if state == MissionState.SEARCHING_VEHICLES:
            if detected_vehicles is None:
                return StateValidationResult.INVALID, "SEARCHING_VEHICLES requires detected_vehicles dictionary"
        
        return StateValidationResult.VALID, None
    
    def check_state_timeout(self, state: MissionState, state_start_time: Optional[float],
                           timeout: Optional[float] = None) -> Tuple[bool, Optional[str]]:
        """
        Check if state has exceeded timeout
        
        Args:
            state: Current state
            state_start_time: When state was entered
            timeout: Custom timeout (uses default if None)
            
        Returns:
            (is_timeout, reason)
        """
        if state_start_time is None:
            return False, None
        
        elapsed = time.time() - state_start_time
        
        # Get default timeout for state
        if timeout is None:
            timeout = self._get_default_timeout(state)
        
        if timeout and elapsed > timeout:
            return True, f"State {state.value} exceeded timeout ({elapsed:.1f}s > {timeout:.1f}s)"
        
        return False, None
    
    def suggest_recovery(self, state: MissionState, issue: str,
                       context: Optional[Dict[str, Any]] = None) -> Optional[str]:
        """
        Suggest recovery action for a state issue
        
        Args:
            state: Current state
            issue: Description of the issue
            context: Optional context
            
        Returns:
            Recovery suggestion or None
        """
        suggestions = {
            MissionState.SEARCHING_VEHICLES: {
                'no_detections': "Check YOLO node, camera, and vehicle distance. Verify point cloud is available.",
                'timeout': "Continue searching or check if vehicles are in view. Verify detection pipeline."
            },
            MissionState.PLANNING: {
                'no_waypoints': "Use fallback waypoint generation. Verify vehicle detection pose is valid.",
                'timeout': "Proceed with available waypoints or use fallback generation."
            },
            MissionState.NAVIGATING_TO_LICENSE_PLATE: {
                'goal_rejected': "Recalculate goal with increased distance. Check Nav2 status.",
                'timeout': "Proceed if close enough, otherwise cancel and retry."
            },
            MissionState.DETECTING_TYRES: {
                'no_tyres': "Use fallback waypoint generation. Continue with standard 4-tyre layout.",
                'timeout': "Generate fallback waypoints and proceed."
            },
            MissionState.NAVIGATING_TO_TYRE: {
                'goal_rejected': "Use pre-planned waypoint or recalculate. Check distance to robot.",
                'timeout': "Proceed if close enough, otherwise skip to next tyre."
            },
            MissionState.CAPTURING_TYRE: {
                'service_unavailable': "Wait for service or retry. Check photo_capture node.",
                'timeout': "Skip to next tyre if max attempts reached."
            },
            MissionState.ERROR_RECOVERY: {
                'max_attempts': "Reset to IDLE state. Check system health and logs.",
                'timeout': "Reset to IDLE state. Review error logs for root cause."
            }
        }
        
        state_suggestions = suggestions.get(state, {})
        return state_suggestions.get(issue, "Check logs and error recovery strategy documentation.")
    
    def _get_invalid_transitions(self) -> list:
        """Get list of invalid state transitions"""
        return [
            # Can't go directly from IDLE to most states
            (MissionState.IDLE, MissionState.PLANNING),
            (MissionState.IDLE, MissionState.NAVIGATING_TO_LICENSE_PLATE),
            (MissionState.IDLE, MissionState.CAPTURING_LICENSE_PLATE),
            (MissionState.IDLE, MissionState.DETECTING_TYRES),
            (MissionState.IDLE, MissionState.NAVIGATING_TO_TYRE),
            (MissionState.IDLE, MissionState.CAPTURING_TYRE),
            (MissionState.IDLE, MissionState.CHECKING_COMPLETION),
            
            # Can't go from MISSION_COMPLETE to active states
            (MissionState.MISSION_COMPLETE, MissionState.SEARCHING_VEHICLES),
            (MissionState.MISSION_COMPLETE, MissionState.PLANNING),
            (MissionState.MISSION_COMPLETE, MissionState.NAVIGATING_TO_LICENSE_PLATE),
            
            # Can't skip planning
            (MissionState.SEARCHING_VEHICLES, MissionState.NAVIGATING_TO_LICENSE_PLATE),
            (MissionState.VEHICLE_DETECTED, MissionState.NAVIGATING_TO_LICENSE_PLATE),
        ]
    
    def _check_suspicious_transition(self, from_state: MissionState, to_state: MissionState) -> Optional[str]:
        """Check for suspicious but potentially valid transitions"""
        # Check for rapid state changes (possible error loop)
        if len(self.state_history) >= 3:
            recent = self.state_history[-3:]
            if all(s['to'] == to_state for s in recent):
                return f"Suspicious: Entered {to_state.value} 3+ times recently (possible loop)"
        
        # Check for backwards transitions (might indicate error)
        if from_state in [MissionState.NAVIGATING_TO_TYRE, MissionState.CAPTURING_TYRE] and \
           to_state == MissionState.DETECTING_TYRES:
            return f"Suspicious: Going backwards from {from_state.value} to {to_state.value}"
        
        return None
    
    def _get_default_timeout(self, state: MissionState) -> Optional[float]:
        """Get default timeout for a state"""
        timeouts = {
            MissionState.SEARCHING_VEHICLES: DetectionConfig.DETECTION_TIMEOUT,
            MissionState.PLANNING: PlanningConfig.PLANNING_TIMEOUT,
            MissionState.NAVIGATING_TO_LICENSE_PLATE: NavigationConfig.NAVIGATION_TIMEOUT,
            MissionState.NAVIGATING_TO_TYRE: NavigationConfig.NAVIGATION_TIMEOUT,
            MissionState.DETECTING_TYRES: DetectionConfig.TYRE_DETECTION_TIMEOUT,
            MissionState.CAPTURING_LICENSE_PLATE: CaptureConfig.PHOTO_CAPTURE_TIMEOUT,
            MissionState.CAPTURING_TYRE: CaptureConfig.PHOTO_CAPTURE_TIMEOUT,
            MissionState.ERROR_RECOVERY: 30.0,  # 30 seconds for recovery
        }
        return timeouts.get(state)
    
    def get_state_statistics(self) -> Dict[str, Any]:
        """
        Get statistics about state transitions
        
        Returns:
            Dictionary with state statistics
        """
        if not self.state_history:
            return {}
        
        # Count state occurrences
        state_counts = {}
        for entry in self.state_history:
            state = entry['to']
            state_counts[state.value] = state_counts.get(state.value, 0) + 1
        
        # Calculate average time in each state
        state_times = {}
        for i in range(1, len(self.state_history)):
            prev = self.state_history[i-1]
            curr = self.state_history[i]
            state = prev['to']
            duration = curr['time'] - prev['time']
            if state.value not in state_times:
                state_times[state.value] = []
            state_times[state.value].append(duration)
        
        avg_times = {state: sum(times)/len(times) if times else 0 
                     for state, times in state_times.items()}
        
        return {
            'total_transitions': len(self.state_history),
            'state_counts': state_counts,
            'average_state_times': avg_times,
            'recent_states': [s['to'].value for s in self.state_history[-10:]]
        }
