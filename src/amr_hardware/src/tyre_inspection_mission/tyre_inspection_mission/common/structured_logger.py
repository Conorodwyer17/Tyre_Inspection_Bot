#!/usr/bin/env python3
"""
Structured Logger for Mission Controller
Provides contextual, structured logging to reduce log noise
"""

from enum import Enum
import logging
from typing import Optional, Dict, Any
from datetime import datetime


class LogCategory(Enum):
    """Log categories for filtering"""
    STATE_TRANSITION = "STATE"
    NAVIGATION = "NAV"
    DETECTION = "DET"
    CAPTURE = "CAP"
    PLANNING = "PLAN"
    ERROR = "ERR"
    SYSTEM = "SYS"


class StructuredLogger:
    """Structured logger that provides contextual, filtered logging"""
    
    def __init__(self, ros_logger, enable_debug: bool = False):
        """
        Args:
            ros_logger: ROS 2 logger instance
            enable_debug: If True, include debug messages
        """
        self.ros_logger = ros_logger
        self.enable_debug = enable_debug
        self.log_counts = {cat: 0 for cat in LogCategory}
        
    def _log(self, category: LogCategory, level: str, message: str, context: Optional[Dict[str, Any]] = None):
        """Internal log method with category and context"""
        self.log_counts[category] += 1
        
        # Build structured message
        timestamp = datetime.now().strftime("%H:%M:%S")
        prefix = f"[{timestamp}] [{category.value}]"
        
        if context:
            ctx_str = " ".join([f"{k}={v}" for k, v in context.items()])
            full_message = f"{prefix} {message} | {ctx_str}"
        else:
            full_message = f"{prefix} {message}"
        
        # Route to appropriate ROS logger method
        if level == "info":
            self.ros_logger.info(full_message)
        elif level == "warn":
            self.ros_logger.warn(full_message)
        elif level == "error":
            self.ros_logger.error(full_message)
        elif level == "debug":
            if self.enable_debug:
                self.ros_logger.debug(full_message)
    
    def state_transition(self, from_state: str, to_state: str, reason: Optional[str] = None):
        """Log state transition"""
        context = {"from": from_state, "to": to_state}
        if reason:
            context["reason"] = reason
        self._log(LogCategory.STATE_TRANSITION, "info", 
                 f"→ {to_state}", context)
    
    def navigation_start(self, goal_type: str, target_id: str, position: tuple, 
                        distance: Optional[float] = None):
        """Log navigation start"""
        context = {"type": goal_type, "target": target_id, 
                  "pos": f"({position[0]:.2f}, {position[1]:.2f})"}
        if distance is not None:
            context["dist"] = f"{distance:.2f}m"
        self._log(LogCategory.NAVIGATION, "info", 
                 f"🚀 Navigating to {goal_type}: {target_id}", context)
    
    def navigation_complete(self, target_id: str, success: bool, reason: Optional[str] = None):
        """Log navigation completion"""
        status = "✅ Arrived" if success else "❌ Failed"
        context = {"target": target_id}
        if reason:
            context["reason"] = reason
        self._log(LogCategory.NAVIGATION, "info" if success else "warn",
                 f"{status} at {target_id}", context)
    
    def navigation_goal_rejected(self, target_id: str, reason: str, 
                                retry_count: int, max_retries: int):
        """Log goal rejection"""
        context = {"target": target_id, "reason": reason, 
                  "retry": f"{retry_count}/{max_retries}"}
        self._log(LogCategory.NAVIGATION, "warn",
                 f"⚠️ Goal rejected for {target_id}", context)
    
    def navigation_too_close(self, target_id: str, distance: float, min_safe: float):
        """Log when goal is too close"""
        context = {"target": target_id, "dist": f"{distance:.2f}m", 
                  "min": f"{min_safe:.2f}m"}
        self._log(LogCategory.NAVIGATION, "warn",
                 f"⚠️ Too close to {target_id}", context)
    
    def detection_found(self, object_type: str, object_id: str, position: tuple, 
                       confidence: Optional[float] = None):
        """Log object detection"""
        context = {"type": object_type, "id": object_id,
                  "pos": f"({position[0]:.2f}, {position[1]:.2f})"}
        if confidence:
            context["conf"] = f"{confidence:.2f}"
        self._log(LogCategory.DETECTION, "info",
                 f"🎯 Detected {object_type}: {object_id}", context)
    
    def detection_lost(self, object_type: str, object_id: str, time_since: float,
                      fallback_used: Optional[str] = None):
        """Log when detection is lost"""
        context = {"type": object_type, "id": object_id, "since": f"{time_since:.1f}s"}
        if fallback_used:
            context["fallback"] = fallback_used
        self._log(LogCategory.DETECTION, "warn",
                 f"⚠️ Lost {object_type}: {object_id}", context)
    
    def detection_event(self, object_type: str, object_id: str, action: str, **kwargs):
        """Log detection event (detected, updated, etc.)"""
        context = {"type": object_type, "id": object_id, "action": action}
        context.update(kwargs)
        self._log(LogCategory.DETECTION, "info",
                 f"📦 {object_type.capitalize()} {object_id} {action}", context)
    
    def planning_event(self, action: str, target: str, **kwargs):
        """Log planning event"""
        context = {"action": action, "target": target}
        context.update(kwargs)
        self._log(LogCategory.PLANNING, "info",
                 f"📋 {action} {target}", context)
    
    def capture_success(self, object_type: str, object_id: str, path: str, **kwargs):
        """Log successful capture"""
        context = {"type": object_type, "id": object_id, "path": path}
        context.update(kwargs)
        self._log(LogCategory.CAPTURE, "info",
                 f"✅ {object_type.capitalize()} photo saved: {path}", context)
    
    def capture_failure(self, object_type: str, object_id: str, reason: str, **kwargs):
        """Log capture failure"""
        context = {"type": object_type, "id": object_id, "reason": reason}
        context.update(kwargs)
        self._log(LogCategory.CAPTURE, "error",
                 f"❌ Failed to capture {object_type} {object_id}: {reason}", context)
    
    def capture_start(self, target_type: str, target_id: str, centered: bool = False):
        """Log photo capture start"""
        context = {"type": target_type, "target": target_id, "centered": centered}
        centering_status = "✅ Centered" if centered else "⏳ Centering"
        self._log(LogCategory.CAPTURE, "info",
                 f"📸 {centering_status} - Capturing {target_type}: {target_id}", context)
    
    def capture_complete(self, target_id: str, success: bool, path: Optional[str] = None):
        """Log photo capture completion"""
        status = "✅ Captured" if success else "❌ Failed"
        context = {"target": target_id}
        if path:
            context["path"] = path
        self._log(LogCategory.CAPTURE, "info" if success else "error",
                 f"{status}: {target_id}", context)
    
    def planning_start(self, vehicle_id: str, num_vehicles: int):
        """Log planning start"""
        context = {"vehicle": vehicle_id, "total": num_vehicles}
        self._log(LogCategory.PLANNING, "info",
                 f"📋 Planning waypoints for {vehicle_id}", context)
    
    def planning_complete(self, vehicle_id: str, license_waypoint: bool, 
                         tyre_waypoints: int):
        """Log planning completion"""
        context = {"vehicle": vehicle_id, "license": license_waypoint,
                  "tyres": tyre_waypoints}
        self._log(LogCategory.PLANNING, "info",
                 f"✅ Planning complete for {vehicle_id}", context)
    
    def error(self, error_type: str, message: str, context: Optional[Dict[str, Any]] = None):
        """Log error"""
        ctx = context or {}
        ctx["type"] = error_type
        self._log(LogCategory.ERROR, "error", f"❌ {message}", ctx)
    
    def error_recovery(self, message: str, **kwargs):
        """Log error recovery attempt"""
        context = kwargs.copy()
        self._log(LogCategory.ERROR, "warn", f"⚠️ Error recovery: {message}", context)
    
    def system_health(self, component: str, status: bool, details: Optional[str] = None):
        """Log system health check"""
        status_str = "✅ OK" if status else "❌ FAIL"
        context = {"component": component}
        if details:
            context["details"] = details
        self._log(LogCategory.SYSTEM, "info" if status else "warn",
                 f"{status_str}: {component}", context)
    
    def get_stats(self) -> Dict[str, int]:
        """Get log statistics"""
        return self.log_counts.copy()
    
    def info(self, message: str, category: LogCategory = LogCategory.SYSTEM):
        """Generic info log"""
        self._log(category, "info", message)
    
    def warn(self, message: str, category: LogCategory = LogCategory.SYSTEM):
        """Generic warn log"""
        self._log(category, "warn", message)
    
    def error_msg(self, message: str, category: LogCategory = LogCategory.ERROR):
        """Generic error log"""
        self._log(category, "error", message)
    
    def debug(self, message: str, category: LogCategory = LogCategory.SYSTEM):
        """Generic debug log"""
        self._log(category, "debug", message)
