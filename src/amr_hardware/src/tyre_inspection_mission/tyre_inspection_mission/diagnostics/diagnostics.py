#!/usr/bin/env python3
"""
Diagnostic Tools for Tyre Inspection Mission

Provides system health monitoring and diagnostic capabilities
to help identify and resolve issues quickly.
"""

from typing import Dict, List, Optional, Tuple
import time
from enum import Enum
from rclpy.node import Node
import rclpy
import tf2_ros
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from tyre_inspection_mission.common.data_structures import MissionState
from tyre_inspection_mission.common.structured_logger import StructuredLogger, LogCategory
from tyre_inspection_mission.common.config import (
    TopicConfig,
    FrameConfig
)


class HealthStatus(Enum):
    """System health status"""
    HEALTHY = "healthy"
    WARNING = "warning"
    CRITICAL = "critical"
    UNKNOWN = "unknown"


class SystemDiagnostics:
    """System diagnostic and health monitoring"""
    
    def __init__(self, node: Node, logger: StructuredLogger):
        """
        Initialize diagnostics
        
        Args:
            node: ROS 2 node instance
            logger: Structured logger instance
        """
        self.node = node
        self.logger = logger
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, node)
        
        # Health status tracking
        self.health_status: Dict[str, HealthStatus] = {}
        self.last_check_time: Dict[str, float] = {}
        self.check_intervals: Dict[str, float] = {
            'tf_frames': 5.0,
            'topics': 10.0,
            'services': 10.0,
            'nav2': 5.0
        }
        
    def check_tf_frames(self) -> Dict[str, HealthStatus]:
        """
        Check TF frame availability
        
        Returns:
            Dictionary mapping frame names to health status
        """
        results = {}
        required_frames = [
            FrameConfig.BASE_FOOTPRINT_FRAME,
            FrameConfig.NAVIGATION_FRAME,
            FrameConfig.CAMERA_FRAME
        ]
        
        for frame in required_frames:
            try:
                # Try to get transform
                transform = self.tf_buffer.lookup_transform(
                    FrameConfig.NAVIGATION_FRAME,
                    frame,
                    rclpy.time.Time(seconds=0),
                    timeout=rclpy.duration.Duration(seconds=1.0)  # Use config value if available
                )
                results[frame] = HealthStatus.HEALTHY
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException) as e:
                results[frame] = HealthStatus.CRITICAL
                self.logger.warn(
                    f"TF frame {frame} not available: {e}",
                    category=LogCategory.SYSTEM
                )
            except tf2_ros.ExtrapolationException:
                results[frame] = HealthStatus.WARNING
            except Exception as e:
                results[frame] = HealthStatus.UNKNOWN
                self.logger.error(
                    f"Error checking TF frame {frame}: {e}",
                    category=LogCategory.SYSTEM
                )
        
        return results
    
    def check_topics(self, topic_list: List[str]) -> Dict[str, HealthStatus]:
        """
        Check if required topics are publishing
        
        Args:
            topic_list: List of topic names to check
            
        Returns:
            Dictionary mapping topic names to health status
        """
        results = {}
        
        for topic in topic_list:
            try:
                # Check if topic exists
                topic_info = self.node.get_topic_names_and_types()
                topic_exists = any(t[0] == topic for t in topic_info)
                
                if topic_exists:
                    # Try to get one message (non-blocking)
                    # In a real implementation, you'd use a subscriber with timeout
                    results[topic] = HealthStatus.HEALTHY
                else:
                    results[topic] = HealthStatus.CRITICAL
                    self.logger.warn(
                        f"Topic {topic} not found",
                        category=LogCategory.SYSTEM
                    )
            except Exception as e:
                results[topic] = HealthStatus.UNKNOWN
                self.logger.error(
                    f"Error checking topic {topic}: {e}",
                    category=LogCategory.SYSTEM
                )
        
        return results
    
    def check_services(self, service_list: List[str]) -> Dict[str, HealthStatus]:
        """
        Check if required services are available
        
        Args:
            service_list: List of service names to check
            
        Returns:
            Dictionary mapping service names to health status
        """
        results = {}
        
        for service in service_list:
            try:
                # Check if service exists
                service_info = self.node.get_service_names_and_types()
                service_exists = any(s[0] == service for s in service_info)
                
                if service_exists:
                    results[service] = HealthStatus.HEALTHY
                else:
                    results[service] = HealthStatus.CRITICAL
                    self.logger.warn(
                        f"Service {service} not available",
                        category=LogCategory.SYSTEM
                    )
            except Exception as e:
                results[service] = HealthStatus.UNKNOWN
                self.logger.error(
                    f"Error checking service {service}: {e}",
                    category=LogCategory.SYSTEM
                )
        
        return results
    
    def check_nav2_health(self) -> HealthStatus:
        """
        Check Nav2 action server health
        
        Returns:
            Health status of Nav2
        """
        try:
            from rclpy.action import ActionClient
            from nav2_msgs.action import NavigateToPose
            
            nav_client = ActionClient(self.node, NavigateToPose, TopicConfig.NAV_TO_POSE_ACTION)
            
            if nav_client.server_is_ready():
                return HealthStatus.HEALTHY
            else:
                return HealthStatus.WARNING
        except Exception as e:
            self.logger.error(
                f"Error checking Nav2 health: {e}",
                category=LogCategory.SYSTEM
            )
            return HealthStatus.UNKNOWN
    
    def get_system_health_report(self) -> Dict:
        """
        Generate comprehensive system health report
        
        Returns:
            Dictionary containing health status of all components
        """
        report = {
            'timestamp': time.time(),
            'tf_frames': self.check_tf_frames(),
            'topics': self.check_topics([
                TopicConfig.BOUNDING_BOXES_TOPIC,
                TopicConfig.LIDAR_SCAN_TOPIC,
                '/oak/points',
                '/oak/rgb/image_rect'
            ]),
            'services': self.check_services([
                TopicConfig.PHOTO_CAPTURE_SERVICE,
                TopicConfig.MISSION_START_SERVICE,
                TopicConfig.MISSION_STOP_SERVICE
            ]),
            'nav2': self.check_nav2_health()
        }
        
        # Calculate overall health
        all_statuses = []
        for component in report.values():
            if isinstance(component, dict):
                all_statuses.extend(component.values())
            elif isinstance(component, HealthStatus):
                all_statuses.append(component)
        
        if HealthStatus.CRITICAL in all_statuses:
            report['overall'] = HealthStatus.CRITICAL
        elif HealthStatus.WARNING in all_statuses:
            report['overall'] = HealthStatus.WARNING
        elif HealthStatus.UNKNOWN in all_statuses:
            report['overall'] = HealthStatus.UNKNOWN
        else:
            report['overall'] = HealthStatus.HEALTHY
        
        return report
    
    def log_health_report(self, report: Optional[Dict] = None):
        """
        Log system health report
        
        Args:
            report: Optional health report (generates if None)
        """
        if report is None:
            report = self.get_system_health_report()
        
        overall = report['overall']
        status_str = overall.value.upper()
        
        self.logger.system_health(
            component="system",
            status=(overall == HealthStatus.HEALTHY),
            details=f"Overall: {status_str}"
        )
        
        # Log critical issues
        for component, statuses in report.items():
            if isinstance(statuses, dict):
                for name, status in statuses.items():
                    if status == HealthStatus.CRITICAL:
                        self.logger.error(
                            f"CRITICAL: {component} {name} is not available",
                            category=LogCategory.SYSTEM
                        )


class ErrorRecoveryHelper:
    """Helper class for error recovery strategies"""
    
    @staticmethod
    def get_recovery_strategy(error_code: str, context: Dict) -> Optional[str]:
        """
        Get recovery strategy for a given error code
        
        Args:
            error_code: Error code (e.g., "NAV_001", "DET_002")
            context: Error context dictionary
            
        Returns:
            Recovery strategy description or None
        """
        strategies = {
            # Navigation errors
            "NAV_001": "Recalculate goal with increased distance from robot",
            "NAV_002": "Wait for Nav2 action server to become available",
            "NAV_003": "Use pre-planned waypoint or recalculate goal",
            "NAV_004": "Proceed if close enough, otherwise cancel and retry",
            "NAV_005": "Fallback to alternative frame (odom if map unavailable)",
            
            # Detection errors
            "DET_001": "Continue searching, check YOLO and segmentation processor",
            "DET_002": "Retry LiDAR processing, check scan topic",
            "DET_003": "Use fallback waypoint generation",
            "DET_004": "Wait for point cloud, check camera node",
            
            # Planning errors
            "PLAN_001": "Use vehicle detection pose as fallback",
            "PLAN_002": "Use fallback tyre waypoint generation",
            "PLAN_003": "Use standard 4-tyre layout assumption",
            "PLAN_004": "Validate and sanitize pose values",
            "PLAN_005": "Check depth sensor calibration",
            
            # Capture errors
            "CAP_001": "Wait for service, check photo_capture node",
            "CAP_002": "Check file permissions and disk space",
            "CAP_003": "Initialize camera, check camera node",
            
            # Transform errors
            "TF_001": "Wait for frame to become available",
            "TF_002": "Check TF tree, restart TF broadcaster",
            "TF_003": "Use more recent timestamp or current time"
        }
        
        return strategies.get(error_code, "Check logs for details")
    
    @staticmethod
    def should_retry(error_code: str, retry_count: int, max_retries: int = 3) -> bool:
        """
        Determine if an operation should be retried
        
        Args:
            error_code: Error code
            retry_count: Current retry count
            max_retries: Maximum retries allowed
            
        Returns:
            True if should retry, False otherwise
        """
        # Non-retryable errors
        non_retryable = ["CAP_002", "PLAN_004"]  # File errors, invalid data
        
        if error_code in non_retryable:
            return False
        
        return retry_count < max_retries
