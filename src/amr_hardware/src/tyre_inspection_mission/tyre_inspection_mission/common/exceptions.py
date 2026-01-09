#!/usr/bin/env python3
"""
Exception Classes Module

Custom exception classes for consistent error handling throughout
the tyre inspection mission package.

All exceptions include error codes and contextual information.
"""

from typing import Optional, Dict, Any


class TyreInspectionError(Exception):
    """
    Base exception class for all tyre inspection mission errors.
    
    Attributes:
        error_code (str): Unique error code for this error type
        message (str): Human-readable error message
        context (dict): Additional context information
    """
    
    def __init__(self, message: str, error_code: Optional[str] = None, 
                 context: Optional[Dict[str, Any]] = None):
        """
        Initialize exception.
        
        Args:
            message: Human-readable error message
            error_code: Optional unique error code
            context: Optional dictionary with additional context
        """
        super().__init__(message)
        self.message = message
        self.error_code = error_code or self.__class__.__name__
        self.context = context or {}
    
    def __str__(self) -> str:
        """String representation including context."""
        if self.context:
            context_str = ", ".join(f"{k}={v}" for k, v in self.context.items())
            return f"{self.error_code}: {self.message} ({context_str})"
        return f"{self.error_code}: {self.message}"


class NavigationError(TyreInspectionError):
    """
    Navigation-related errors.
    
    Error codes:
    - NAV_001: Goal validation failed
    - NAV_002: Navigation timeout
    - NAV_003: Nav2 server unavailable
    - NAV_004: Goal rejected by Nav2
    - NAV_005: Transform failed
    - NAV_006: Goal too close to robot
    """
    
    def __init__(self, message: str, error_code: str = "NAV_001", 
                 context: Optional[Dict[str, Any]] = None):
        super().__init__(message, error_code, context)


class DetectionError(TyreInspectionError):
    """
    Detection-related errors.
    
    Error codes:
    - DET_001: Detection timeout
    - DET_002: Invalid detection data
    - DET_003: No vehicles detected
    - DET_004: No tyres detected
    - DET_005: Detection pipeline broken
    """
    
    def __init__(self, message: str, error_code: str = "DET_001", 
                 context: Optional[Dict[str, Any]] = None):
        super().__init__(message, error_code, context)


class CaptureError(TyreInspectionError):
    """
    Photo capture-related errors.
    
    Error codes:
    - CAP_001: Capture timeout
    - CAP_002: No image available
    - CAP_003: Save failed
    - CAP_004: Service unavailable
    - CAP_005: Centering failed
    """
    
    def __init__(self, message: str, error_code: str = "CAP_001", 
                 context: Optional[Dict[str, Any]] = None):
        super().__init__(message, error_code, context)


class PlanningError(TyreInspectionError):
    """
    Waypoint planning-related errors.
    
    Error codes:
    - PLAN_001: Planning timeout
    - PLAN_002: Invalid waypoint
    - PLAN_003: Transform failed during planning
    - PLAN_004: Cannot calculate waypoint
    - PLAN_005: Fallback waypoint generation failed
    """
    
    def __init__(self, message: str, error_code: str = "PLAN_001", 
                 context: Optional[Dict[str, Any]] = None):
        super().__init__(message, error_code, context)


class TransformError(TyreInspectionError):
    """
    TF2 transform-related errors.
    
    Error codes:
    - TF_001: Lookup failed
    - TF_002: Connectivity error
    - TF_003: Extrapolation error
    - TF_004: Timeout
    - TF_005: Invalid frame
    """
    
    def __init__(self, message: str, error_code: str = "TF_001", 
                 context: Optional[Dict[str, Any]] = None):
        super().__init__(message, error_code, context)


class ValidationError(TyreInspectionError):
    """
    Data validation errors.
    
    Error codes:
    - VAL_001: Invalid pose
    - VAL_002: Invalid parameter
    - VAL_003: Invalid distance
    - VAL_004: Invalid angle
    - VAL_005: Missing required data
    """
    
    def __init__(self, message: str, error_code: str = "VAL_001", 
                 context: Optional[Dict[str, Any]] = None):
        super().__init__(message, error_code, context)


class StateMachineError(TyreInspectionError):
    """
    State machine-related errors.
    
    Error codes:
    - SM_001: Invalid state transition
    - SM_002: State timeout
    - SM_003: Unexpected state
    - SM_004: State machine stuck
    """
    
    def __init__(self, message: str, error_code: str = "SM_001", 
                 context: Optional[Dict[str, Any]] = None):
        super().__init__(message, error_code, context)
