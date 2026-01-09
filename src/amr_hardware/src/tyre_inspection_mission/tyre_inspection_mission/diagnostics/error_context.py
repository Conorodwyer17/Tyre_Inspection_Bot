#!/usr/bin/env python3
"""
Error Context - Enhanced Error Reporting with Actionable Steps

Provides detailed error context and recovery actions for all error types.
"""

from typing import Dict, List, Optional, Tuple
from enum import Enum

from tyre_inspection_mission.common.exceptions import (
    TyreInspectionError,
    NavigationError,
    DetectionError,
    CaptureError,
    PlanningError,
    TransformError,
    ValidationError,
    StateMachineError
)


class ErrorSeverity(Enum):
    """Error severity levels"""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class RecoveryAction:
    """Recovery action for an error"""
    def __init__(self, description: str, steps: List[str], 
                 expected_outcome: str, timeout: Optional[float] = None):
        self.description = description
        self.steps = steps
        self.expected_outcome = expected_outcome
        self.timeout = timeout


class ErrorContext:
    """
    Enhanced error context with recovery actions
    
    Provides detailed context and actionable recovery steps for all error types.
    """
    
    # Error code to recovery action mapping
    RECOVERY_ACTIONS: Dict[str, RecoveryAction] = {
        # Navigation Errors
        'NAV_001': RecoveryAction(
            description="Goal too close to robot",
            steps=[
                "1. Check current robot position",
                "2. Recalculate goal with increased distance",
                "3. Use validate_goal_distance() to verify",
                "4. If still too close, use fallback waypoint or skip"
            ],
            expected_outcome="Goal at safe distance (>0.6m)",
            timeout=5.0
        ),
        'NAV_002': RecoveryAction(
            description="Nav2 action server not available",
            steps=[
                "1. Check if Nav2 is launched",
                "2. Wait for action server to become ready",
                "3. Check Nav2 logs for errors",
                "4. Restart Nav2 if needed"
            ],
            expected_outcome="Nav2 action server ready",
            timeout=30.0
        ),
        'NAV_003': RecoveryAction(
            description="Navigation goal rejected",
            steps=[
                "1. Check goal validity and distance",
                "2. Verify goal is not in obstacle",
                "3. Check Nav2 costmap",
                "4. Use fallback waypoint if available"
            ],
            expected_outcome="Goal accepted or fallback used",
            timeout=10.0
        ),
        'NAV_004': RecoveryAction(
            description="Navigation timeout",
            steps=[
                "1. Check if robot is stuck",
                "2. Verify goal is reachable",
                "3. Check Nav2 planner status",
                "4. Cancel and retry with different goal"
            ],
            expected_outcome="Navigation completes or retries",
            timeout=60.0
        ),
        
        # Detection Errors
        'DET_001': RecoveryAction(
            description="No vehicles detected",
            steps=[
                "1. Check YOLO node is running",
                "2. Verify camera is publishing images",
                "3. Check vehicle distance (should be 0.5m - 10m)",
                "4. Verify camera lights are on",
                "5. Check YOLO confidence threshold",
                "6. Review detection statistics"
            ],
            expected_outcome="Vehicles detected",
            timeout=120.0
        ),
        'DET_002': RecoveryAction(
            description="No tyres detected",
            steps=[
                "1. Check inspection mode is active",
                "2. Verify tyre detection model is loaded",
                "3. Check vehicle is in view",
                "4. Use fallback waypoint generation",
                "5. Continue with standard 4-tyre layout"
            ],
            expected_outcome="Tyres detected or fallback waypoints generated",
            timeout=60.0
        ),
        'DET_003': RecoveryAction(
            description="Point cloud not available",
            steps=[
                "1. Check point cloud topic is publishing",
                "2. Verify camera depth stream",
                "3. Check QoS settings match",
                "4. Restart point cloud node if needed"
            ],
            expected_outcome="Point cloud available",
            timeout=10.0
        ),
        
        # Capture Errors
        'CAP_001': RecoveryAction(
            description="Photo capture service unavailable",
            steps=[
                "1. Check photo_capture node is running",
                "2. Verify service is advertised",
                "3. Wait for service to become available",
                "4. Restart photo_capture node if needed"
            ],
            expected_outcome="Photo capture service available",
            timeout=10.0
        ),
        'CAP_002': RecoveryAction(
            description="Photo capture failed",
            steps=[
                "1. Check camera is initialized",
                "2. Verify file permissions",
                "3. Check disk space",
                "4. Retry capture (max 3 attempts)"
            ],
            expected_outcome="Photo captured successfully",
            timeout=10.0
        ),
        
        # Planning Errors
        'PLAN_001': RecoveryAction(
            description="Waypoint calculation failed",
            steps=[
                "1. Check vehicle detection pose is valid",
                "2. Verify TF transforms are available",
                "3. Check goal distance validation",
                "4. Use fallback waypoint generation"
            ],
            expected_outcome="Waypoints calculated or fallback used",
            timeout=30.0
        ),
        'PLAN_002': RecoveryAction(
            description="Invalid vehicle pose",
            steps=[
                "1. Check vehicle detection is recent",
                "2. Verify TF transform succeeded",
                "3. Use LiDAR position if available",
                "4. Re-detect vehicle if needed"
            ],
            expected_outcome="Valid vehicle pose",
            timeout=10.0
        ),
        'PLAN_003': RecoveryAction(
            description="Fallback waypoint generation failed",
            steps=[
                "1. Check vehicle detection pose exists",
                "2. Verify vehicle dimensions are reasonable",
                "3. Use default 4-tyre layout",
                "4. Skip to next vehicle if critical"
            ],
            expected_outcome="Fallback waypoints generated",
            timeout=10.0
        ),
        
        # Transform Errors
        'TF_001': RecoveryAction(
            description="TF frame not found",
            steps=[
                "1. Wait for frame to become available",
                "2. Check TF broadcaster is running",
                "3. Verify frame names in config",
                "4. Check TF tree connectivity"
            ],
            expected_outcome="TF frame available",
            timeout=5.0
        ),
        'TF_002': RecoveryAction(
            description="TF transform failed",
            steps=[
                "1. Check TF tree is connected",
                "2. Verify frame timestamps",
                "3. Wait and retry transform",
                "4. Use alternative frame if available"
            ],
            expected_outcome="TF transform succeeds",
            timeout=5.0
        ),
        
        # Validation Errors
        'VAL_001': RecoveryAction(
            description="Invalid pose",
            steps=[
                "1. Check pose values are finite",
                "2. Verify frame_id is set",
                "3. Check for NaN/Inf values",
                "4. Recalculate pose"
            ],
            expected_outcome="Valid pose",
            timeout=5.0
        ),
        'VAL_002': RecoveryAction(
            description="Invalid parameter",
            steps=[
                "1. Check parameter value is in valid range",
                "2. Verify parameter type",
                "3. Use default value if available",
                "4. Check configuration documentation"
            ],
            expected_outcome="Valid parameter",
            timeout=1.0
        ),
        
        # State Machine Errors
        'STM_001': RecoveryAction(
            description="Invalid state transition",
            steps=[
                "1. Check state transition logic",
                "2. Verify state requirements are met",
                "3. Review state machine flow",
                "4. Transition to ERROR_RECOVERY"
            ],
            expected_outcome="Valid state transition",
            timeout=1.0
        ),
        'STM_002': RecoveryAction(
            description="State timeout",
            steps=[
                "1. Check state execution logic",
                "2. Verify no blocking operations",
                "3. Review timeout value",
                "4. Transition to ERROR_RECOVERY if critical"
            ],
            expected_outcome="State completes or recovers",
            timeout=30.0
        ),
    }
    
    # Error severity mapping
    ERROR_SEVERITY: Dict[str, ErrorSeverity] = {
        'NAV_001': ErrorSeverity.MEDIUM,
        'NAV_002': ErrorSeverity.CRITICAL,
        'NAV_003': ErrorSeverity.MEDIUM,
        'NAV_004': ErrorSeverity.HIGH,
        'DET_001': ErrorSeverity.HIGH,
        'DET_002': ErrorSeverity.MEDIUM,
        'DET_003': ErrorSeverity.HIGH,
        'CAP_001': ErrorSeverity.HIGH,
        'CAP_002': ErrorSeverity.MEDIUM,
        'PLAN_001': ErrorSeverity.HIGH,
        'PLAN_002': ErrorSeverity.HIGH,
        'PLAN_003': ErrorSeverity.MEDIUM,
        'TF_001': ErrorSeverity.HIGH,
        'TF_002': ErrorSeverity.HIGH,
        'VAL_001': ErrorSeverity.MEDIUM,
        'VAL_002': ErrorSeverity.LOW,
        'STM_001': ErrorSeverity.CRITICAL,
        'STM_002': ErrorSeverity.HIGH,
    }
    
    @classmethod
    def get_recovery_action(cls, error_code: str) -> Optional[RecoveryAction]:
        """
        Get recovery action for an error code
        
        Args:
            error_code: Error code (e.g., 'NAV_001')
            
        Returns:
            RecoveryAction or None if not found
        """
        return cls.RECOVERY_ACTIONS.get(error_code)
    
    @classmethod
    def get_error_severity(cls, error_code: str) -> ErrorSeverity:
        """
        Get error severity for an error code
        
        Args:
            error_code: Error code (e.g., 'NAV_001')
            
        Returns:
            ErrorSeverity (defaults to MEDIUM if not found)
        """
        return cls.ERROR_SEVERITY.get(error_code, ErrorSeverity.MEDIUM)
    
    @classmethod
    def format_error_with_context(cls, error: TyreInspectionError) -> str:
        """
        Format error with full context and recovery steps
        
        Args:
            error: TyreInspectionError instance
            
        Returns:
            Formatted error message with context
        """
        error_code = getattr(error, 'error_code', 'UNKNOWN')
        message = str(error)
        
        # Get recovery action
        recovery = cls.get_recovery_action(error_code)
        severity = cls.get_error_severity(error_code)
        
        # Build formatted message
        lines = [
            f"[{error_code}] {message}",
            f"Severity: {severity.value.upper()}"
        ]
        
        if recovery:
            lines.append(f"\nRecovery Action: {recovery.description}")
            lines.append("Steps:")
            lines.extend(recovery.steps)
            lines.append(f"\nExpected Outcome: {recovery.expected_outcome}")
            if recovery.timeout:
                lines.append(f"Timeout: {recovery.timeout}s")
        
        # Add context if available
        context = getattr(error, 'context', {})
        if context:
            lines.append("\nContext:")
            for key, value in context.items():
                lines.append(f"  {key}: {value}")
        
        return "\n".join(lines)
    
    @classmethod
    def get_all_error_codes(cls) -> List[Tuple[str, str, ErrorSeverity]]:
        """
        Get all error codes with descriptions and severities
        
        Returns:
            List of (error_code, description, severity) tuples
        """
        codes = []
        for code, action in cls.RECOVERY_ACTIONS.items():
            severity = cls.ERROR_SEVERITY.get(code, ErrorSeverity.MEDIUM)
            codes.append((code, action.description, severity))
        return sorted(codes, key=lambda x: (x[2].value, x[0]))
