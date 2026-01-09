#!/usr/bin/env python3
"""
Quick Diagnostic Check Script

Run this script to quickly check system health and identify issues.
Usage: ros2 run tyre_inspection_mission diagnostic_check
"""

import rclpy
from rclpy.node import Node
import sys
from typing import Dict, List

try:
    from tyre_inspection_mission.diagnostics import SystemDiagnostics, HealthStatus, ErrorRecoveryHelper
    from tyre_inspection_mission.config import TopicConfig, FrameConfig
    DIAGNOSTICS_AVAILABLE = True
except ImportError:
    DIAGNOSTICS_AVAILABLE = False
    print("⚠️  Diagnostics module not available. Using basic checks.")


class DiagnosticChecker(Node):
    """Quick diagnostic checker for tyre inspection mission"""
    
    def __init__(self):
        super().__init__('diagnostic_checker')
        self.diagnostics = None
        
        if DIAGNOSTICS_AVAILABLE:
            try:
                from tyre_inspection_mission.structured_logger import StructuredLogger, LogCategory
                logger = StructuredLogger(self.get_logger(), enable_debug=True)
                self.diagnostics = SystemDiagnostics(self, logger)
            except Exception as e:
                self.get_logger().warn(f"Could not initialize diagnostics: {e}")
    
    def check_all(self) -> Dict:
        """Run all diagnostic checks"""
        results = {
            'overall': 'unknown',
            'checks': {}
        }
        
        # Check topics
        print("\n📡 Checking Topics...")
        required_topics = [
            TopicConfig.BOUNDING_BOXES_TOPIC,
            TopicConfig.LIDAR_SCAN_TOPIC,
            '/oak/points',
            '/oak/rgb/image_rect'
        ]
        
        topic_results = {}
        for topic in required_topics:
            try:
                topic_info = self.get_topic_names_and_types()
                exists = any(t[0] == topic for t in topic_info)
                status = "✅" if exists else "❌"
                topic_results[topic] = exists
                print(f"  {status} {topic}")
            except Exception as e:
                topic_results[topic] = False
                print(f"  ❌ {topic} - Error: {e}")
        
        results['checks']['topics'] = topic_results
        
        # Check services
        print("\n🔧 Checking Services...")
        required_services = [
            TopicConfig.PHOTO_CAPTURE_SERVICE,
            TopicConfig.MISSION_START_SERVICE,
            TopicConfig.MISSION_STOP_SERVICE
        ]
        
        service_results = {}
        for service in required_services:
            try:
                service_info = self.get_service_names_and_types()
                exists = any(s[0] == service for s in service_info)
                status = "✅" if exists else "❌"
                service_results[service] = exists
                print(f"  {status} {service}")
            except Exception as e:
                service_results[service] = False
                print(f"  ❌ {service} - Error: {e}")
        
        results['checks']['services'] = service_results
        
        # Check Nav2
        print("\n🗺️  Checking Nav2...")
        try:
            from rclpy.action import ActionClient
            from nav2_msgs.action import NavigateToPose
            
            nav_client = ActionClient(self, NavigateToPose, TopicConfig.NAV_TO_POSE_ACTION)
            if nav_client.server_is_ready():
                print(f"  ✅ Nav2 action server ready: {TopicConfig.NAV_TO_POSE_ACTION}")
                results['checks']['nav2'] = True
            else:
                print(f"  ⚠️  Nav2 action server not ready: {TopicConfig.NAV_TO_POSE_ACTION}")
                results['checks']['nav2'] = False
        except Exception as e:
            print(f"  ❌ Nav2 check failed: {e}")
            results['checks']['nav2'] = False
        
        # Check TF frames
        print("\n🔄 Checking TF Frames...")
        import tf2_ros
        import rclpy.time
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer, self)
        
        # Wait a bit for TF to populate
        rclpy.spin_once(self, timeout_sec=1.0)
        
        required_frames = [
            FrameConfig.BASE_FOOTPRINT_FRAME,
            FrameConfig.DEFAULT_NAV_FRAME,
            FrameConfig.DEFAULT_CAMERA_FRAME
        ]
        
        tf_results = {}
        for frame in required_frames:
            try:
                transform = tf_buffer.lookup_transform(
                    FrameConfig.DEFAULT_NAV_FRAME,
                    frame,
                    rclpy.time.Time(seconds=0),
                    timeout=rclpy.duration.Duration(seconds=1.0)
                )
                print(f"  ✅ {frame} -> {FrameConfig.DEFAULT_NAV_FRAME}")
                tf_results[frame] = True
            except tf2_ros.LookupException:
                print(f"  ❌ {frame} - Frame not found")
                tf_results[frame] = False
            except tf2_ros.ConnectivityException:
                print(f"  ⚠️  {frame} - Frame tree broken")
                tf_results[frame] = False
            except Exception as e:
                print(f"  ❌ {frame} - Error: {e}")
                tf_results[frame] = False
        
        results['checks']['tf_frames'] = tf_results
        
        # Use diagnostics module if available
        if self.diagnostics:
            print("\n🏥 Running Comprehensive Health Check...")
            try:
                health_report = self.diagnostics.get_system_health_report()
                overall = health_report['overall']
                print(f"  Overall Health: {overall.value.upper()}")
                
                if overall == HealthStatus.CRITICAL:
                    print("  ⚠️  CRITICAL issues detected!")
                elif overall == HealthStatus.WARNING:
                    print("  ⚠️  Warnings detected")
                elif overall == HealthStatus.HEALTHY:
                    print("  ✅ System is healthy")
                
                results['overall'] = overall.value
            except Exception as e:
                print(f"  ⚠️  Diagnostics check failed: {e}")
        
        # Calculate overall status
        all_checks = []
        for check_type, check_results in results['checks'].items():
            if isinstance(check_results, dict):
                all_checks.extend(check_results.values())
            else:
                all_checks.append(check_results)
        
        if all_checks:
            if all(all_checks):
                results['overall'] = 'healthy'
            elif any(all_checks):
                results['overall'] = 'warning'
            else:
                results['overall'] = 'critical'
        
        return results
    
    def print_summary(self, results: Dict):
        """Print diagnostic summary"""
        print("\n" + "="*60)
        print("📊 DIAGNOSTIC SUMMARY")
        print("="*60)
        
        overall = results.get('overall', 'unknown')
        if overall == 'healthy':
            print("✅ Overall Status: HEALTHY")
        elif overall == 'warning':
            print("⚠️  Overall Status: WARNING")
        elif overall == 'critical':
            print("❌ Overall Status: CRITICAL")
        else:
            print("❓ Overall Status: UNKNOWN")
        
        print("\n📋 Recommendations:")
        
        # Check topics
        topics = results['checks'].get('topics', {})
        missing_topics = [t for t, exists in topics.items() if not exists]
        if missing_topics:
            print(f"  ❌ Missing topics: {', '.join(missing_topics)}")
            print("     → Check if YOLO, segmentation processor, camera, or LiDAR nodes are running")
        
        # Check services
        services = results['checks'].get('services', {})
        missing_services = [s for s, exists in services.items() if not exists]
        if missing_services:
            print(f"  ❌ Missing services: {', '.join(missing_services)}")
            print("     → Check if mission_controller and photo_capture nodes are running")
        
        # Check Nav2
        if not results['checks'].get('nav2', False):
            print("  ❌ Nav2 not available")
            print("     → Check if Nav2 is launched and action server is ready")
        
        # Check TF
        tf_frames = results['checks'].get('tf_frames', {})
        missing_frames = [f for f, exists in tf_frames.items() if not exists]
        if missing_frames:
            print(f"  ❌ Missing TF frames: {', '.join(missing_frames)}")
            print("     → Check if robot description, SLAM, or odometry nodes are running")
        
        print("\n📖 For detailed troubleshooting, see:")
        print("   docs/troubleshooting/TROUBLESHOOTING_GUIDE.md")
        print("="*60)


def main(args=None):
    rclpy.init(args=args)
    
    checker = DiagnosticChecker()
    
    try:
        print("🔍 Running Diagnostic Checks...")
        print("   (This may take a few seconds)")
        
        results = checker.check_all()
        checker.print_summary(results)
        
        # Exit with appropriate code
        overall = results.get('overall', 'unknown')
        if overall == 'critical':
            sys.exit(1)
        elif overall == 'warning':
            sys.exit(2)
        else:
            sys.exit(0)
            
    except KeyboardInterrupt:
        print("\n⚠️  Diagnostic check interrupted")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ Diagnostic check failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        checker.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
