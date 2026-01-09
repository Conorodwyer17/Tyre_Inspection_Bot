#!/usr/bin/env python3
"""
Mission Logger - Comprehensive Mission Logging and Replay

Records all mission events for post-mortem analysis and debugging.
"""

import rclpy
from rclpy.node import Node
import json
import time
from typing import Dict, Optional, List, Any
from datetime import datetime
from pathlib import Path
from collections import deque

from std_msgs.msg import String

from tyre_inspection_mission.common.structured_logger import StructuredLogger, LogCategory
from tyre_inspection_mission.common.config import TopicConfig


class MissionLogger:
    """
    Comprehensive mission event logger
    
    Records all mission events for analysis and debugging.
    """
    
    def __init__(self, log_dir: Path):
        """
        Initialize mission logger
        
        Args:
            log_dir: Directory to save mission logs
        """
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(parents=True, exist_ok=True)
        
        # Create log file with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_file = self.log_dir / f"mission_log_{timestamp}.jsonl"
        
        # Event buffer (for quick access)
        self.event_buffer: deque = deque(maxlen=1000)
        
        # Statistics
        self.start_time: Optional[float] = None
        self.event_count = 0
        
    def start_mission(self, mission_id: str):
        """Record mission start"""
        self.start_time = time.time()
        self.event_count = 0
        self.event_buffer.clear()
        
        self.log_event('mission_start', {
            'mission_id': mission_id,
            'timestamp': self.start_time
        })
    
    def log_event(self, event_type: str, data: Dict[str, Any]):
        """
        Log an event
        
        Args:
            event_type: Type of event (e.g., 'state_transition', 'detection', 'error')
            data: Event data dictionary
        """
        event = {
            'event_type': event_type,
            'timestamp': time.time(),
            'elapsed': time.time() - self.start_time if self.start_time else 0.0,
            'data': data
        }
        
        # Add to buffer
        self.event_buffer.append(event)
        self.event_count += 1
        
        # Write to file (JSONL format - one JSON object per line)
        try:
            with open(self.log_file, 'a') as f:
                f.write(json.dumps(event) + '\n')
        except Exception as e:
            # Don't fail mission if logging fails
            pass
    
    def log_state_transition(self, from_state: str, to_state: str, reason: Optional[str] = None):
        """Log state transition"""
        self.log_event('state_transition', {
            'from': from_state,
            'to': to_state,
            'reason': reason
        })
    
    def log_detection(self, object_type: str, object_id: str, position: Dict, confidence: Optional[float] = None):
        """Log object detection"""
        self.log_event('detection', {
            'object_type': object_type,
            'object_id': object_id,
            'position': position,
            'confidence': confidence
        })
    
    def log_navigation(self, action: str, target: str, goal: Optional[Dict] = None, result: Optional[str] = None):
        """Log navigation event"""
        self.log_event('navigation', {
            'action': action,  # 'start', 'complete', 'failed', 'rejected'
            'target': target,
            'goal': goal,
            'result': result
        })
    
    def log_capture(self, target_type: str, target_id: str, success: bool, path: Optional[str] = None, error: Optional[str] = None):
        """Log photo capture"""
        self.log_event('capture', {
            'target_type': target_type,
            'target_id': target_id,
            'success': success,
            'path': path,
            'error': error
        })
    
    def log_error(self, error_type: str, error_code: str, message: str, context: Optional[Dict] = None):
        """Log error"""
        self.log_event('error', {
            'error_type': error_type,
            'error_code': error_code,
            'message': message,
            'context': context or {}
        })
    
    def log_issue(self, issue_type: str, severity: str, recommendation: str, details: Optional[Dict] = None):
        """Log detected issue"""
        self.log_event('issue', {
            'issue_type': issue_type,
            'severity': severity,
            'recommendation': recommendation,
            'details': details or {}
        })
    
    def get_recent_events(self, count: int = 50) -> List[Dict]:
        """Get recent events"""
        return list(self.event_buffer)[-count:]
    
    def get_events_by_type(self, event_type: str) -> List[Dict]:
        """Get all events of a specific type"""
        return [e for e in self.event_buffer if e['event_type'] == event_type]
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get mission statistics"""
        events_by_type = {}
        for event in self.event_buffer:
            event_type = event['event_type']
            events_by_type[event_type] = events_by_type.get(event_type, 0) + 1
        
        return {
            'total_events': self.event_count,
            'events_by_type': events_by_type,
            'log_file': str(self.log_file),
            'elapsed_time': time.time() - self.start_time if self.start_time else 0.0
        }


class MissionLogRecorder(Node):
    """
    Mission Log Recorder Node
    
    Records all mission events from various topics for post-mortem analysis.
    """
    
    def __init__(self, log_dir: Optional[Path] = None):
        super().__init__('mission_log_recorder')
        
        # Initialize logger
        from tyre_inspection_mission.structured_logger import StructuredLogger, LogCategory
        self.slogger = StructuredLogger(self.get_logger(), enable_debug=False)
        
        # Mission logger
        if log_dir is None:
            log_dir = Path.home() / 'tyre_inspection_logs'
        self.mission_logger = MissionLogger(log_dir)
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            TopicConfig.MISSION_STATUS_TOPIC,
            self.status_callback,
            10
        )
        
        self.monitor_sub = self.create_subscription(
            String,
            '/mission_monitor/status',
            self.monitor_callback,
            10
        )
        
        self.issues_sub = self.create_subscription(
            String,
            '/mission_monitor/issues',
            self.issues_callback,
            10
        )
        
        # Track mission state
        self.current_mission_id: Optional[str] = None
        self.last_state: Optional[str] = None
        
        self.slogger.info(f"Mission log recorder initialized. Logs: {log_dir}", category=LogCategory.SYSTEM)
        
    def status_callback(self, msg: String):
        """Handle mission status updates"""
        try:
            status = json.loads(msg.data)
            
            # Check for mission start
            state = status.get('state')
            if state == 'idle' and self.last_state and self.last_state != 'idle':
                # Mission ended
                pass
            elif state != 'idle' and (not self.last_state or self.last_state == 'idle'):
                # Mission started
                if not self.current_mission_id:
                    self.current_mission_id = f"mission_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
                    self.mission_logger.start_mission(self.current_mission_id)
            
            # Log state transitions
            if state != self.last_state:
                self.mission_logger.log_state_transition(
                    self.last_state or 'unknown',
                    state
                )
                self.last_state = state
            
            # Log detection counts
            if 'vehicles_detected' in status:
                # Could log individual detections if needed
                pass
            
        except (json.JSONDecodeError, KeyError) as e:
            self.slogger.debug(f"Error parsing status: {e}", category=LogCategory.SYSTEM)
    
    def monitor_callback(self, msg: String):
        """Handle monitor status updates"""
        try:
            monitor_data = json.loads(msg.data)
            # Could log progress updates if needed
        except json.JSONDecodeError:
            pass
    
    def issues_callback(self, msg: String):
        """Handle issue updates"""
        try:
            data = json.loads(msg.data)
            issues = data.get('issues', [])
            for issue in issues:
                self.mission_logger.log_issue(
                    issue.get('type', 'unknown'),
                    issue.get('severity', 'unknown'),
                    issue.get('recommendation', ''),
                    issue
                )
        except json.JSONDecodeError:
            pass


def main(args=None):
    rclpy.init(args=args)
    
    # Get log directory from parameter or use default
    node = Node('mission_log_recorder_temp')
    node.declare_parameter('log_dir', str(Path.home() / 'tyre_inspection_logs'))
    log_dir = Path(node.get_parameter('log_dir').value)
    node.destroy_node()
    
    recorder = MissionLogRecorder(log_dir)
    
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info('Mission log recorder shutting down...')
    finally:
        recorder.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
