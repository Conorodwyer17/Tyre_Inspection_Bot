#!/usr/bin/env python3
"""
Startup Validator - Comprehensive System Validation Before Mission Start

Validates all system components, configuration, and dependencies before
allowing mission to start. Ensures mission can complete successfully.
"""

import rclpy
from rclpy.node import Node
from typing import Dict, List, Tuple, Optional
import time

from tyre_inspection_mission.common.config import (
    TopicConfig,
    FrameConfig,
    SystemHealthConfig
)
from tyre_inspection_mission.common.structured_logger import StructuredLogger, LogCategory
from tyre_inspection_mission.diagnostics.config_validator import ConfigValidator, ValidationResult
from tyre_inspection_mission.diagnostics.diagnostics import SystemDiagnostics, HealthStatus


class StartupValidationResult:
    """Result of startup validation"""
    def __init__(self, is_valid: bool, errors: List[str], warnings: List[str], 
                 checks_performed: Dict[str, bool]):
        self.is_valid = is_valid
        self.errors = errors
        self.warnings = warnings
        self.checks_performed = checks_performed
    
    def __bool__(self):
        return self.is_valid
    
    def get_summary(self) -> str:
        """Get human-readable summary"""
        if self.is_valid:
            status = "✅ VALID"
            if self.warnings:
                status += f" ({len(self.warnings)} warnings)"
        else:
            status = f"❌ INVALID ({len(self.errors)} errors)"
        
        return f"Startup Validation: {status}"


class StartupValidator:
    """
    Comprehensive startup validation
    
    Validates all system components before mission start to ensure
    mission can complete successfully.
    """
    
    def __init__(self, node: Node, logger: StructuredLogger):
        """
        Initialize startup validator
        
        Args:
            node: ROS 2 node for topic/service access
            logger: Structured logger
        """
        self.node = node
        self.logger = logger
        self.config_validator = ConfigValidator(logger)
        self.diagnostics = None
        
        try:
            self.diagnostics = SystemDiagnostics(node, logger)
        except Exception as e:
            logger.debug(f"Could not initialize diagnostics: {e}", category=LogCategory.SYSTEM)
    
    def validate_all(self, timeout: float = 10.0) -> StartupValidationResult:
        """
        Perform comprehensive startup validation
        
        Args:
            timeout: Maximum time to wait for checks (seconds)
            
        Returns:
            StartupValidationResult with validation status
        """
        errors = []
        warnings = []
        checks_performed = {}
        
        self.logger.info("Starting comprehensive startup validation...", category=LogCategory.SYSTEM)
        
        # 1. Configuration Validation
        self.logger.info("Validating configuration...", category=LogCategory.SYSTEM)
        config_result = self.config_validator.validate_all(self.node)
        errors.extend(config_result.errors)
        warnings.extend(config_result.warnings)
        checks_performed['configuration'] = config_result.is_valid
        
        if not config_result.is_valid:
            self.logger.error("Configuration validation failed. Mission cannot start.", 
                            category=LogCategory.ERROR)
            return StartupValidationResult(False, errors, warnings, checks_performed)
        
        # 2. System Health Check
        if self.diagnostics:
            self.logger.info("Checking system health...", category=LogCategory.SYSTEM)
            health_result = self._check_system_health(timeout)
            if not health_result[0]:
                errors.extend(health_result[1])
                warnings.extend(health_result[2])
            checks_performed['system_health'] = health_result[0]
        else:
            warnings.append("System diagnostics not available. Skipping health check.")
            checks_performed['system_health'] = None
        
        # 3. Topic Availability
        self.logger.info("Checking topic availability...", category=LogCategory.SYSTEM)
        topic_result = self._check_topics(timeout)
        if not topic_result[0]:
            errors.extend(topic_result[1])
            warnings.extend(topic_result[2])
        checks_performed['topics'] = topic_result[0]
        
        # 4. Service Availability
        self.logger.info("Checking service availability...", category=LogCategory.SYSTEM)
        service_result = self._check_services(timeout)
        if not service_result[0]:
            errors.extend(service_result[1])
            warnings.extend(service_result[2])
        checks_performed['services'] = service_result[0]
        
        # 5. Nav2 Availability
        self.logger.info("Checking Nav2 availability...", category=LogCategory.SYSTEM)
        nav_result = self._check_nav2(timeout)
        if not nav_result[0]:
            errors.extend(nav_result[1])
            warnings.extend(nav_result[2])
        checks_performed['nav2'] = nav_result[0]
        
        # 6. TF Frames
        self.logger.info("Checking TF frames...", category=LogCategory.SYSTEM)
        tf_result = self._check_tf_frames(timeout)
        if not tf_result[0]:
            errors.extend(tf_result[1])
            warnings.extend(tf_result[2])
        checks_performed['tf_frames'] = tf_result[0]
        
        is_valid = len(errors) == 0
        
        result = StartupValidationResult(is_valid, errors, warnings, checks_performed)
        self._log_validation_result(result)
        
        return result
    
    def _check_system_health(self, timeout: float) -> Tuple[bool, List[str], List[str]]:
        """Check system health using diagnostics"""
        errors = []
        warnings = []
        
        try:
            health_report = self.diagnostics.get_system_health_report()
            overall = health_report.get('overall')
            
            if overall == HealthStatus.CRITICAL:
                errors.append("System health is CRITICAL. Mission may fail.")
            elif overall == HealthStatus.WARNING:
                warnings.append("System health has warnings. Some features may not work correctly.")
            
            # Check specific components
            for component, statuses in health_report.items():
                if isinstance(statuses, dict):
                    for name, status in statuses.items():
                        if status == HealthStatus.CRITICAL:
                            errors.append(f"CRITICAL: {component} {name} is not available")
                        elif status == HealthStatus.WARNING:
                            warnings.append(f"WARNING: {component} {name} has issues")
            
            return (overall != HealthStatus.CRITICAL, errors, warnings)
        except Exception as e:
            warnings.append(f"Could not check system health: {e}")
            return (True, errors, warnings)  # Don't block on diagnostics failure
    
    def _check_topics(self, timeout: float) -> Tuple[bool, List[str], List[str]]:
        """Check required topics are available"""
        errors = []
        warnings = []
        
        required_topics = [
            TopicConfig.BOUNDING_BOXES_TOPIC,
            TopicConfig.LIDAR_SCAN_TOPIC,
            '/oak/points',
            '/oak/rgb/image_rect'
        ]
        
        start_time = time.time()
        available_topics = []
        
        while (time.time() - start_time) < timeout:
            try:
                topic_info = self.node.get_topic_names_and_types()
                available_topics = [t[0] for t in topic_info]
                break
            except Exception:
                time.sleep(0.1)
        
        missing_topics = [t for t in required_topics if t not in available_topics]
        
        if missing_topics:
            errors.append(f"Required topics not available: {', '.join(missing_topics)}")
            errors.append("Check if YOLO, segmentation processor, camera, or LiDAR nodes are running")
        
        return (len(missing_topics) == 0, errors, warnings)
    
    def _check_services(self, timeout: float) -> Tuple[bool, List[str], List[str]]:
        """Check required services are available"""
        errors = []
        warnings = []
        
        required_services = [
            TopicConfig.PHOTO_CAPTURE_SERVICE,
            TopicConfig.MISSION_START_SERVICE,
            TopicConfig.MISSION_STOP_SERVICE
        ]
        
        start_time = time.time()
        available_services = []
        
        while (time.time() - start_time) < timeout:
            try:
                service_info = self.node.get_service_names_and_types()
                available_services = [s[0] for s in service_info]
                break
            except Exception:
                time.sleep(0.1)
        
        missing_services = [s for s in required_services if s not in available_services]
        
        if missing_services:
            errors.append(f"Required services not available: {', '.join(missing_services)}")
            errors.append("Check if mission_controller and photo_capture nodes are running")
        
        return (len(missing_services) == 0, errors, warnings)
    
    def _check_nav2(self, timeout: float) -> Tuple[bool, List[str], List[str]]:
        """Check Nav2 action server is available"""
        errors = []
        warnings = []
        
        try:
            from rclpy.action import ActionClient
            from nav2_msgs.action import NavigateToPose
            
            nav_client = ActionClient(self.node, NavigateToPose, TopicConfig.NAV_TO_POSE_ACTION)
            
            start_time = time.time()
            while (time.time() - start_time) < timeout:
                if nav_client.server_is_ready():
                    return (True, errors, warnings)
                time.sleep(0.1)
            
            errors.append(f"Nav2 action server not ready: {TopicConfig.NAV_TO_POSE_ACTION}")
            errors.append("Check if Nav2 is launched and action server is running")
            return (False, errors, warnings)
        except Exception as e:
            errors.append(f"Error checking Nav2: {e}")
            return (False, errors, warnings)
    
    def _check_tf_frames(self, timeout: float) -> Tuple[bool, List[str], List[str]]:
        """Check required TF frames are available"""
        errors = []
        warnings = []
        
        try:
            import tf2_ros
            import rclpy.time
            
            tf_buffer = tf2_ros.Buffer()
            tf_listener = tf2_ros.TransformListener(tf_buffer, self.node)
            
            # Wait a bit for TF to populate
            start_time = time.time()
            while (time.time() - start_time) < min(timeout, 2.0):
                rclpy.spin_once(self.node, timeout_sec=0.1)
            
            required_frames = [
                FrameConfig.BASE_FOOTPRINT_FRAME,
                FrameConfig.NAVIGATION_FRAME
            ]
            
            missing_frames = []
            for frame in required_frames:
                try:
                    transform = tf_buffer.lookup_transform(
                        FrameConfig.NAVIGATION_FRAME,
                        frame,
                        rclpy.time.Time(seconds=0),
                        timeout=rclpy.duration.Duration(seconds=1.0)
                    )
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException):
                    missing_frames.append(frame)
            
            if missing_frames:
                errors.append(f"Required TF frames not available: {', '.join(missing_frames)}")
                errors.append("Check if robot description, SLAM, or odometry nodes are running")
            
            return (len(missing_frames) == 0, errors, warnings)
        except Exception as e:
            warnings.append(f"Could not check TF frames: {e}")
            return (True, errors, warnings)  # Don't block on TF check failure
    
    def _log_validation_result(self, result: StartupValidationResult):
        """Log validation results"""
        summary = result.get_summary()
        
        if result.is_valid:
            self.logger.info(summary, category=LogCategory.SYSTEM)
            if result.warnings:
                self.logger.warn(f"Startup validation has {len(result.warnings)} warning(s):", 
                               category=LogCategory.SYSTEM)
                for warning in result.warnings:
                    self.logger.warn(f"  ⚠️  {warning}", category=LogCategory.SYSTEM)
        else:
            self.logger.error(summary, category=LogCategory.ERROR)
            self.logger.error("Mission cannot start. Fix errors and retry.", category=LogCategory.ERROR)
            for error in result.errors:
                self.logger.error(f"  ❌ {error}", category=LogCategory.ERROR)
            if result.warnings:
                for warning in result.warnings:
                    self.logger.warn(f"  ⚠️  {warning}", category=LogCategory.SYSTEM)
        
        # Log check summary
        self.logger.info("Validation checks performed:", category=LogCategory.SYSTEM)
        for check, status in result.checks_performed.items():
            if status is True:
                self.logger.info(f"  ✅ {check}", category=LogCategory.SYSTEM)
            elif status is False:
                self.logger.error(f"  ❌ {check}", category=LogCategory.ERROR)
            else:
                self.logger.warn(f"  ⚠️  {check} (skipped)", category=LogCategory.SYSTEM)


def validate_startup(node: Node, logger: StructuredLogger, timeout: float = 10.0) -> bool:
    """
    Validate system startup and log results
    
    Args:
        node: ROS 2 node
        logger: Structured logger
        timeout: Maximum time for checks (seconds)
        
    Returns:
        True if validation passed, False otherwise
    """
    validator = StartupValidator(node, logger)
    result = validator.validate_all(timeout)
    return result.is_valid
