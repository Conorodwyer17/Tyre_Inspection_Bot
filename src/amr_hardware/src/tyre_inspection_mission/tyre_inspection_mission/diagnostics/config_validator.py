#!/usr/bin/env python3
"""
Configuration Validator

Validates all configuration parameters at startup to catch issues early.
"""

from typing import Dict, List, Tuple, Optional
import rclpy
from rclpy.node import Node

from tyre_inspection_mission.common.config import (
    NavigationConfig,
    DetectionConfig,
    CameraConfig,
    CaptureConfig,
    PlanningConfig,
    SystemHealthConfig,
    FrameConfig,
)
from tyre_inspection_mission.common.structured_logger import StructuredLogger, LogCategory


class ValidationResult:
    """Result of configuration validation"""
    def __init__(self, is_valid: bool, errors: List[str], warnings: List[str]):
        self.is_valid = is_valid
        self.errors = errors
        self.warnings = warnings
    
    def __bool__(self):
        return self.is_valid


class ConfigValidator:
    """Validates configuration parameters"""
    
    def __init__(self, logger: StructuredLogger):
        self.logger = logger
    
    def validate_all(self, node: Node) -> ValidationResult:
        """
        Validate all configuration parameters
        
        Args:
            node: ROS 2 node for parameter access
            
        Returns:
            ValidationResult with validation status
        """
        errors = []
        warnings = []
        
        # Validate navigation config
        nav_result = self._validate_navigation(node)
        errors.extend(nav_result.errors)
        warnings.extend(nav_result.warnings)
        
        # Validate detection config
        det_result = self._validate_detection(node)
        errors.extend(det_result.errors)
        warnings.extend(det_result.warnings)
        
        # Validate camera config
        cam_result = self._validate_camera(node)
        errors.extend(cam_result.errors)
        warnings.extend(cam_result.warnings)
        
        # Validate capture config
        cap_result = self._validate_capture(node)
        errors.extend(cap_result.errors)
        warnings.extend(cap_result.warnings)
        
        # Validate frame config
        frame_result = self._validate_frames(node)
        errors.extend(frame_result.errors)
        warnings.extend(frame_result.warnings)
        
        # Validate system health config
        health_result = self._validate_system_health(node)
        errors.extend(health_result.errors)
        warnings.extend(health_result.warnings)
        
        is_valid = len(errors) == 0
        
        return ValidationResult(is_valid, errors, warnings)
    
    def _validate_navigation(self, node: Node) -> ValidationResult:
        """Validate navigation configuration"""
        errors = []
        warnings = []
        
        try:
            approach_dist = node.get_parameter('approach_distance').value
            if approach_dist < NavigationConfig.MIN_SAFE_DISTANCE:
                errors.append(
                    f"approach_distance ({approach_dist}m) is less than minimum safe distance "
                    f"({NavigationConfig.MIN_SAFE_DISTANCE}m). Navigation goals will be rejected."
                )
            elif approach_dist < NavigationConfig.MIN_SAFE_DISTANCE * 1.2:
                warnings.append(
                    f"approach_distance ({approach_dist}m) is close to minimum safe distance. "
                    f"Consider increasing to at least {NavigationConfig.MIN_SAFE_DISTANCE * 1.2:.1f}m."
                )
            
            nav_timeout = node.get_parameter('navigation_timeout').value
            if nav_timeout < 10.0:
                errors.append(f"navigation_timeout ({nav_timeout}s) is too short. Minimum recommended: 10s")
            elif nav_timeout > 300.0:
                warnings.append(f"navigation_timeout ({nav_timeout}s) is very long. Consider reducing if navigation is slow.")
            
        except Exception as e:
            errors.append(f"Error validating navigation config: {e}")
        
        return ValidationResult(len(errors) == 0, errors, warnings)
    
    def _validate_detection(self, node: Node) -> ValidationResult:
        """Validate detection configuration"""
        errors = []
        warnings = []
        
        try:
            det_timeout = node.get_parameter('detection_timeout').value
            if det_timeout < 5.0:
                errors.append(f"detection_timeout ({det_timeout}s) is too short. Minimum recommended: 5s")
            elif det_timeout > 300.0:
                warnings.append(f"detection_timeout ({det_timeout}s) is very long. Consider reducing if detection is slow.")
            
            # Check vehicle class names
            vehicle_classes = node.get_parameter('vehicle_class_names').value
            if isinstance(vehicle_classes, str):
                vehicle_classes = [vehicle_classes]
            if not vehicle_classes:
                errors.append("vehicle_class_names is empty. No vehicles will be detected.")
            elif len(vehicle_classes) > 5:
                warnings.append(f"Many vehicle classes ({len(vehicle_classes)}). This may slow detection.")
            
        except Exception as e:
            errors.append(f"Error validating detection config: {e}")
        
        return ValidationResult(len(errors) == 0, errors, warnings)
    
    def _validate_camera(self, node: Node) -> ValidationResult:
        """Validate camera configuration"""
        errors = []
        warnings = []
        
        # Camera config is mostly from config.py, but we can validate values
        if CameraConfig.IMAGE_WIDTH <= 0 or CameraConfig.IMAGE_HEIGHT <= 0:
            errors.append("Camera image dimensions are invalid")
        
        if CameraConfig.FX <= 0 or CameraConfig.FY <= 0:
            warnings.append("Camera focal lengths may not be calibrated. Tyre centering may be inaccurate.")
        
        if CameraConfig.TYRE_CENTERING_TOLERANCE_PX > CameraConfig.IMAGE_WIDTH / 4:
            warnings.append(
                f"Tyre centering tolerance ({CameraConfig.TYRE_CENTERING_TOLERANCE_PX}px) is large. "
                f"Consider reducing for better centering accuracy."
            )
        
        return ValidationResult(len(errors) == 0, errors, warnings)
    
    def _validate_capture(self, node: Node) -> ValidationResult:
        """Validate capture configuration"""
        errors = []
        warnings = []
        
        try:
            capture_timeout = node.get_parameter('photo_capture_timeout').value
            if capture_timeout < 1.0:
                errors.append(f"photo_capture_timeout ({capture_timeout}s) is too short. Minimum recommended: 1s")
            elif capture_timeout > 60.0:
                warnings.append(f"photo_capture_timeout ({capture_timeout}s) is very long. Consider reducing.")
            
            if CaptureConfig.MAX_CENTERING_ATTEMPTS < 1:
                errors.append("MAX_CENTERING_ATTEMPTS must be at least 1")
            elif CaptureConfig.MAX_CENTERING_ATTEMPTS > 20:
                warnings.append(f"MAX_CENTERING_ATTEMPTS ({CaptureConfig.MAX_CENTERING_ATTEMPTS}) is high. May cause delays.")
            
        except Exception as e:
            errors.append(f"Error validating capture config: {e}")
        
        return ValidationResult(len(errors) == 0, errors, warnings)
    
    def _validate_frames(self, node: Node) -> ValidationResult:
        """Validate frame configuration"""
        errors = []
        warnings = []
        
        try:
            nav_frame = node.get_parameter('navigation_frame').value
            if nav_frame not in ['map', 'odom']:
                errors.append(
                    f"navigation_frame '{nav_frame}' is invalid. Must be 'map' or 'odom'. "
                    f"Using 'map' requires SLAM to be running."
                )
            
            if nav_frame == 'map':
                warnings.append(
                    "Using 'map' frame. Ensure SLAM is running and map frame is available. "
                    "If SLAM is not available, use 'odom' frame instead."
                )
            
        except Exception as e:
            errors.append(f"Error validating frame config: {e}")
        
        return ValidationResult(len(errors) == 0, errors, warnings)
    
    def _validate_system_health(self, node: Node) -> ValidationResult:
        """Validate system health configuration"""
        errors = []
        warnings = []
        
        if SystemHealthConfig.TF_ODOM_TIMEOUT <= 0:
            errors.append("TF_ODOM_TIMEOUT must be positive")
        
        if SystemHealthConfig.TF_MAP_TIMEOUT <= 0:
            errors.append("TF_MAP_TIMEOUT must be positive")
        
        return ValidationResult(len(errors) == 0, errors, warnings)
    
    def log_validation_result(self, result: ValidationResult):
        """Log validation results"""
        if result.is_valid:
            if result.warnings:
                self.logger.warn(
                    f"Configuration validation passed with {len(result.warnings)} warning(s)",
                    category=LogCategory.SYSTEM
                )
                for warning in result.warnings:
                    self.logger.warn(f"  ⚠️  {warning}", category=LogCategory.SYSTEM)
            else:
                self.logger.info("Configuration validation passed", category=LogCategory.SYSTEM)
        else:
            self.logger.error(
                f"Configuration validation failed with {len(result.errors)} error(s)",
                category=LogCategory.ERROR
            )
            for error in result.errors:
                self.logger.error(f"  ❌ {error}", category=LogCategory.ERROR)
            if result.warnings:
                for warning in result.warnings:
                    self.logger.warn(f"  ⚠️  {warning}", category=LogCategory.SYSTEM)


def validate_configuration(node: Node, logger: StructuredLogger) -> bool:
    """
    Validate configuration and log results
    
    Args:
        node: ROS 2 node
        logger: Structured logger
        
    Returns:
        True if configuration is valid, False otherwise
    """
    validator = ConfigValidator(logger)
    result = validator.validate_all(node)
    validator.log_validation_result(result)
    return result.is_valid
