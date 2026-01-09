#!/usr/bin/env python3
"""
Mission Monitor - Runtime Monitoring and Progress Tracking

Provides real-time monitoring, progress tracking, and adaptive issue detection
for the tyre inspection mission.
"""

import rclpy
from rclpy.node import Node
import json
import time
from typing import Dict, Optional, List, Any
from datetime import datetime, timedelta
from collections import deque

from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

from tyre_inspection_mission.common.data_structures import MissionState
from tyre_inspection_mission.common.structured_logger import StructuredLogger, LogCategory
from tyre_inspection_mission.common.config import TopicConfig


class MissionProgress:
    """Tracks mission progress and statistics"""
    
    def __init__(self):
        self.start_time: Optional[float] = None
        self.current_state: Optional[str] = None
        self.state_history: deque = deque(maxlen=100)  # Last 100 state transitions
        self.vehicles_detected: int = 0
        self.vehicles_completed: int = 0
        self.tyres_detected: int = 0
        self.tyres_photographed: int = 0
        self.license_plates_captured: int = 0
        self.errors_encountered: int = 0
        self.recovery_attempts: int = 0
        self.state_times: Dict[str, float] = {}  # Time spent in each state
        self.last_state_entry: Optional[float] = None
        
    def start_mission(self):
        """Record mission start"""
        self.start_time = time.time()
        self.vehicles_detected = 0
        self.vehicles_completed = 0
        self.tyres_detected = 0
        self.tyres_photographed = 0
        self.license_plates_captured = 0
        self.errors_encountered = 0
        self.recovery_attempts = 0
        self.state_history.clear()
        self.state_times.clear()
        
    def update_state(self, new_state: str):
        """Update current state and track time"""
        if self.current_state and self.last_state_entry:
            # Record time spent in previous state
            elapsed = time.time() - self.last_state_entry
            if self.current_state not in self.state_times:
                self.state_times[self.current_state] = 0.0
            self.state_times[self.current_state] += elapsed
            
            # Record transition
            self.state_history.append({
                'from': self.current_state,
                'to': new_state,
                'time': time.time(),
                'duration': elapsed
            })
        
        self.current_state = new_state
        self.last_state_entry = time.time()
        
    def get_statistics(self) -> Dict[str, Any]:
        """Get current mission statistics"""
        current_time = time.time()
        elapsed = current_time - self.start_time if self.start_time else 0
        
        # Calculate current state duration
        current_state_duration = 0.0
        if self.current_state and self.last_state_entry:
            current_state_duration = current_time - self.last_state_entry
        
        return {
            'elapsed_time': elapsed,
            'current_state': self.current_state,
            'current_state_duration': current_state_duration,
            'vehicles_detected': self.vehicles_detected,
            'vehicles_completed': self.vehicles_completed,
            'tyres_detected': self.tyres_detected,
            'tyres_photographed': self.tyres_photographed,
            'license_plates_captured': self.license_plates_captured,
            'completion_rate': self._calculate_completion_rate(),
            'errors_encountered': self.errors_encountered,
            'recovery_attempts': self.recovery_attempts,
            'state_times': self.state_times.copy(),
            'recent_transitions': list(self.state_history)[-10:]  # Last 10 transitions
        }
    
    def _calculate_completion_rate(self) -> float:
        """Calculate overall mission completion rate"""
        if self.vehicles_detected == 0:
            return 0.0
        
        # Completion = (vehicles completed + partial progress) / vehicles detected
        vehicle_completion = self.vehicles_completed / self.vehicles_detected if self.vehicles_detected > 0 else 0.0
        
        # Tyre completion (if tyres detected)
        tyre_completion = 0.0
        if self.tyres_detected > 0:
            tyre_completion = self.tyres_photographed / self.tyres_detected
        
        # Weighted average (vehicles 60%, tyres 40%)
        return (vehicle_completion * 0.6) + (tyre_completion * 0.4)


class IssueDetector:
    """Detects potential issues during mission execution"""
    
    def __init__(self, logger: StructuredLogger):
        self.logger = logger
        self.issue_history: deque = deque(maxlen=50)
        
    def check_for_issues(self, progress: MissionProgress, 
                         current_state: str,
                         state_duration: float) -> List[Dict[str, Any]]:
        """
        Check for potential issues based on current state and progress
        
        Returns:
            List of detected issues with severity and recommendations
        """
        issues = []
        
        # Check for stuck states
        stuck_thresholds = {
            MissionState.SEARCHING_VEHICLES.value: 120.0,  # 2 minutes
            MissionState.PLANNING.value: 60.0,  # 1 minute
            MissionState.NAVIGATING_TO_LICENSE_PLATE.value: 120.0,  # 2 minutes
            MissionState.NAVIGATING_TO_TYRE.value: 120.0,  # 2 minutes
            MissionState.DETECTING_TYRES.value: 60.0,  # 1 minute
            MissionState.ERROR_RECOVERY.value: 60.0,  # 1 minute
        }
        
        if current_state in stuck_thresholds and state_duration > stuck_thresholds[current_state]:
            issues.append({
                'type': 'stuck_state',
                'severity': 'high',
                'state': current_state,
                'duration': state_duration,
                'recommendation': f'State {current_state} has been active for {state_duration:.1f}s. Check logs for errors or system health.'
            })
        
        # Check for high error rate
        if progress.errors_encountered > 5:
            error_rate = progress.errors_encountered / (progress.elapsed_time / 60.0) if progress.elapsed_time > 0 else 0
            if error_rate > 0.5:  # More than 0.5 errors per minute
                issues.append({
                    'type': 'high_error_rate',
                    'severity': 'high',
                    'error_count': progress.errors_encountered,
                    'error_rate': error_rate,
                    'recommendation': f'High error rate detected ({error_rate:.2f} errors/min). Review error logs and system health.'
                })
        
        # Check for no progress
        if progress.elapsed_time > 300 and progress.vehicles_detected == 0:  # 5 minutes, no vehicles
            issues.append({
                'type': 'no_progress',
                'severity': 'medium',
                'elapsed_time': progress.elapsed_time,
                'recommendation': 'No vehicles detected after 5 minutes. Check detection pipeline, camera, and YOLO node.'
            })
        
        # Check for repeated state transitions (possible loop)
        if len(progress.state_history) >= 5:
            recent_states = [s['to'] for s in list(progress.state_history)[-5:]]
            if len(set(recent_states)) <= 2:  # Only 2 unique states in last 5 transitions
                issues.append({
                    'type': 'possible_loop',
                    'severity': 'medium',
                    'states': recent_states,
                    'recommendation': 'Possible state loop detected. Check state machine logic and error recovery.'
                })
        
        # Record issues
        for issue in issues:
            self.issue_history.append({
                'issue': issue,
                'timestamp': time.time()
            })
        
        return issues


class MissionMonitor(Node):
    """
    Mission Monitor Node
    
    Monitors mission progress, detects issues, and provides adaptive recommendations.
    Publishes comprehensive status updates for external monitoring tools.
    """
    
    def __init__(self):
        super().__init__('mission_monitor')
        
        # Initialize logger
        from tyre_inspection_mission.structured_logger import StructuredLogger, LogCategory
        self.slogger = StructuredLogger(self.get_logger(), enable_debug=False)
        
        # Progress tracking
        self.progress = MissionProgress()
        self.issue_detector = IssueDetector(self.slogger)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            TopicConfig.MISSION_STATUS_TOPIC,
            self.status_callback,
            10
        )
        
        # Publishers
        self.monitor_pub = self.create_publisher(
            String,
            '/mission_monitor/status',
            10
        )
        
        self.issues_pub = self.create_publisher(
            String,
            '/mission_monitor/issues',
            10
        )
        
        # Timer for periodic monitoring
        self.monitor_timer = self.create_timer(2.0, self.monitor_mission)
        
        self.slogger.info("Mission monitor initialized", category=LogCategory.SYSTEM)
        
    def status_callback(self, msg: String):
        """Handle mission status updates"""
        try:
            status = json.loads(msg.data)
            
            # Update progress
            state = status.get('state')
            if state:
                if self.progress.current_state != state:
                    self.progress.update_state(state)
                
                # Update counters
                if 'vehicles_detected' in status:
                    self.progress.vehicles_detected = status['vehicles_detected']
                if 'tyres_detected' in status:
                    self.progress.tyres_detected = status['tyres_detected']
                if 'tyres_photographed' in status:
                    self.progress.tyres_photographed = status['tyres_photographed']
                    
        except (json.JSONDecodeError, KeyError) as e:
            self.slogger.debug(f"Error parsing status message: {e}", category=LogCategory.SYSTEM)
    
    def monitor_mission(self):
        """Periodic mission monitoring"""
        if not self.progress.current_state:
            return
        
        # Get current statistics
        stats = self.progress.get_statistics()
        
        # Check for issues
        current_state_duration = stats.get('current_state_duration', 0.0)
        issues = self.issue_detector.check_for_issues(
            self.progress,
            self.progress.current_state,
            current_state_duration
        )
        
        # Publish monitor status
        monitor_status = {
            'timestamp': time.time(),
            'progress': stats,
            'issues_detected': len(issues),
            'health_status': self._calculate_health_status(stats, issues)
        }
        
        msg = String()
        msg.data = json.dumps(monitor_status)
        self.monitor_pub.publish(msg)
        
        # Publish issues if any
        if issues:
            issues_msg = String()
            issues_msg.data = json.dumps({
                'timestamp': time.time(),
                'issues': issues
            })
            self.issues_pub.publish(issues_msg)
            
            # Log high severity issues
            for issue in issues:
                if issue['severity'] == 'high':
                    self.slogger.warn(
                        f"Issue detected: {issue['type']} - {issue.get('recommendation', '')}",
                        category=LogCategory.SYSTEM
                    )
    
    def _calculate_health_status(self, stats: Dict, issues: List[Dict]) -> str:
        """Calculate overall mission health status"""
        if not issues:
            return 'healthy'
        
        high_severity = sum(1 for i in issues if i['severity'] == 'high')
        if high_severity > 0:
            return 'critical'
        elif len(issues) > 2:
            return 'warning'
        else:
            return 'healthy'


def main(args=None):
    rclpy.init(args=args)
    monitor = MissionMonitor()
    
    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        monitor.get_logger().info('Mission monitor shutting down...')
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
