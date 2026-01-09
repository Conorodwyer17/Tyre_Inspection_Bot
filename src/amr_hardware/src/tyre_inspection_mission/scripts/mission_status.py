#!/usr/bin/env python3
"""
Mission Status Viewer

Real-time mission status viewer with progress tracking and issue detection.
Usage: ros2 run tyre_inspection_mission mission_status
"""

import rclpy
from rclpy.node import Node
import json
import sys
from typing import Optional
from datetime import timedelta

from std_msgs.msg import String


class MissionStatusViewer(Node):
    """Viewer for mission status and progress"""
    
    def __init__(self):
        super().__init__('mission_status_viewer')
        
        # Subscribers
        self.status_sub = self.create_subscription(
            String,
            '/mission_controller/status',
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
        
        self.last_status: Optional[dict] = None
        self.last_monitor: Optional[dict] = None
        self.last_issues: Optional[list] = None
        
        # Display timer
        self.display_timer = self.create_timer(1.0, self.display_status)
        
        print("\n" + "="*70)
        print("MISSION STATUS VIEWER")
        print("="*70)
        print("Waiting for mission status...\n")
        
    def status_callback(self, msg: String):
        """Handle mission status updates"""
        try:
            self.last_status = json.loads(msg.data)
        except json.JSONDecodeError:
            pass
    
    def monitor_callback(self, msg: String):
        """Handle monitor status updates"""
        try:
            self.last_monitor = json.loads(msg.data)
        except json.JSONDecodeError:
            pass
    
    def issues_callback(self, msg: String):
        """Handle issue updates"""
        try:
            data = json.loads(msg.data)
            self.last_issues = data.get('issues', [])
        except json.JSONDecodeError:
            pass
    
    def display_status(self):
        """Display current mission status"""
        # Clear screen (ANSI escape code)
        print("\033[2J\033[H", end='')
        
        print("="*70)
        print("MISSION STATUS VIEWER")
        print("="*70)
        
        if not self.last_status:
            print("\n⏳ Waiting for mission status...")
            return
        
        # Basic status
        state = self.last_status.get('state', 'unknown')
        print(f"\n📊 Current State: {state.upper()}")
        
        # Progress information
        if self.last_monitor and 'progress' in self.last_monitor:
            progress = self.last_monitor['progress']
            
            elapsed = progress.get('elapsed_time', 0)
            elapsed_str = str(timedelta(seconds=int(elapsed)))
            print(f"⏱️  Elapsed Time: {elapsed_str}")
            
            state_duration = progress.get('current_state_duration', 0)
            print(f"⏳ State Duration: {state_duration:.1f}s")
            
            print(f"\n🚗 Vehicles: {progress.get('vehicles_detected', 0)} detected, "
                  f"{progress.get('vehicles_completed', 0)} completed")
            
            print(f"🛞 Tyres: {progress.get('tyres_detected', 0)} detected, "
                  f"{progress.get('tyres_photographed', 0)} photographed")
            
            completion = progress.get('completion_rate', 0) * 100
            print(f"✅ Completion: {completion:.1f}%")
            
            # Health status
            health = self.last_monitor.get('health_status', 'unknown')
            health_icon = '🟢' if health == 'healthy' else '🟡' if health == 'warning' else '🔴'
            print(f"\n{health_icon} Health Status: {health.upper()}")
        
        # Issues
        if self.last_issues:
            print(f"\n⚠️  Issues Detected: {len(self.last_issues)}")
            for issue in self.last_issues[:3]:  # Show first 3
                severity_icon = '🔴' if issue['severity'] == 'high' else '🟡'
                print(f"  {severity_icon} {issue['type']}: {issue.get('recommendation', '')}")
        else:
            print("\n✅ No issues detected")
        
        # Current vehicle info
        if 'current_vehicle' in self.last_status:
            vehicle = self.last_status['current_vehicle']
            if vehicle:
                print(f"\n🚗 Current Vehicle: {vehicle}")
        
        print("\n" + "="*70)
        print("Press Ctrl+C to exit")
        print("="*70)


def main(args=None):
    rclpy.init(args=args)
    viewer = MissionStatusViewer()
    
    try:
        rclpy.spin(viewer)
    except KeyboardInterrupt:
        print("\n\nMission status viewer shutting down...")
    finally:
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
