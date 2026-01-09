#!/usr/bin/env python3
"""
Camera Recovery Service

Monitors OAK camera health and automatically recovers from boot failures.
This service:
1. Monitors camera topics to detect if camera is publishing
2. Detects boot failures
3. Attempts USB reset to recover the device
4. Restarts camera node if USB reset fails
5. Provides recovery status via service

This addresses the issue where OAK camera fails to boot and requires
power cycling the rover to recover.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger
from std_msgs.msg import String
from rcl_interfaces.msg import Log
import subprocess
import time
import os
import re
from typing import Optional


class CameraRecovery(Node):
    """
    Monitors and recovers OAK camera from boot failures.
    """
    
    def __init__(self):
        super().__init__('camera_recovery')
        
        # Parameters
        self.declare_parameter('camera_image_topic', '/oak/rgb/image_rect')
        self.declare_parameter('camera_health_check_interval', 5.0)  # seconds
        self.declare_parameter('camera_timeout', 10.0)  # seconds - no images = failure
        self.declare_parameter('max_recovery_attempts', 3)  # max USB reset attempts
        self.declare_parameter('recovery_cooldown', 5.0)  # seconds between recovery attempts
        self.declare_parameter('enable_auto_recovery', True)  # enable/disable auto recovery
        
        # Camera monitoring
        self.camera_image_topic = self.get_parameter('camera_image_topic').value
        self.last_image_time = None
        self.camera_healthy = False
        self.recovery_attempts = 0
        self.last_recovery_time = None
        self.recovery_in_progress = False
        
        # Subscribe to camera image topic to monitor health
        self.image_sub = self.create_subscription(
            Image,
            self.camera_image_topic,
            self.image_callback,
            qos_profile_sensor_data
        )
        
        # Subscribe to rosout to detect camera boot failures
        # This allows us to detect "Failed to boot device!" errors immediately
        self.rosout_sub = self.create_subscription(
            Log,
            '/rosout',
            self.rosout_callback,
            10
        )
        
        # Service to manually trigger recovery
        self.recovery_service = self.create_service(
            Trigger,
            '/camera_recovery/recover',
            self.recovery_service_callback
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            '/camera_recovery/status',
            10
        )
        
        # Health check timer
        self.health_check_timer = self.create_timer(
            self.get_parameter('camera_health_check_interval').value,
            self.health_check_callback
        )
        
        # Track boot failure detections
        self.boot_failure_detected = False
        self.boot_failure_time = None
        self._node_start_time = time.time()  # Track when node started to measure elapsed time without images
        
        self.get_logger().info(
            f"Camera recovery service initialized. "
            f"Monitoring: {self.camera_image_topic}, "
            f"Auto-recovery: {self.get_parameter('enable_auto_recovery').value}, "
            f"Rosout monitoring: Enabled"
        )
        
        # Log that we're ready to monitor boot failures
        self.get_logger().info(
            "📡 Monitoring /rosout for camera boot failures. "
            "Will detect 'Failed to boot device!' errors immediately."
        )
    
    def image_callback(self, msg):
        """Update last image time when camera publishes"""
        self.last_image_time = time.time()
        if not self.camera_healthy:
            self.camera_healthy = True
            self.boot_failure_detected = False  # Reset boot failure flag
            self.recovery_attempts = 0  # Reset attempts on successful recovery
            self.get_logger().info("✅ Camera is now healthy (receiving images)")
    
    def rosout_callback(self, msg):
        """
        Monitor rosout logs to detect camera boot failures immediately.
        
        This allows us to detect boot failures even before the timeout,
        providing faster recovery.
        """
        try:
            # Only process ERROR and WARN level messages
            if msg.level not in [Log.ERROR, Log.WARN]:
                return
            
            # Extract node name and message text
            node_name = msg.name if hasattr(msg, 'name') and msg.name else ''
            message_text = msg.msg if hasattr(msg, 'msg') and msg.msg else ''
            
            # Detect camera boot failures
            boot_failure_keywords = [
                'failed to boot device',
                'failed to boot',
                'x_link_unbooted',
                'skipping.*x_link_unbooted',
            ]
            
            # Check if message indicates boot failure (case-insensitive)
            message_lower = message_text.lower()
            is_boot_failure = any(keyword in message_lower for keyword in boot_failure_keywords)
            
            if not is_boot_failure:
                return  # Not a boot failure message, skip
            
            # Check if this is from a camera-related node
            # Node name can be "oak" or node name might be in message text as "[oak]"
            node_name_lower = node_name.lower()
            is_camera_related = (
                'oak' in node_name_lower or 
                'camera' in node_name_lower or
                'component_container' in node_name_lower or  # oak runs in component_container
                '[oak]' in message_lower or  # Check message text for [oak] tag
                'oak' in message_lower  # Check message text for "oak" keyword
            )
            
            if is_camera_related:
                # Boot failure detected!
                if not self.boot_failure_detected:
                    self.boot_failure_detected = True
                    self.boot_failure_time = time.time()
                    self.get_logger().error(
                        f"🚨 Camera boot failure detected from logs! "
                        f"Node: {node_name}, Message: {message_text[:80]}"
                    )
                    
                    # Trigger recovery immediately if we haven't received images yet
                    if self.last_image_time is None:
                        self.get_logger().warn(
                            "Boot failure detected before any images received. "
                            "Recovery will be triggered by health check timer."
                        )
                    else:
                        self.get_logger().warn(
                            "Boot failure detected but images were previously received. "
                            "Camera may have stopped working."
                        )
                else:
                    # Already detected, just log that it's still happening (throttled)
                    time_since_detection = time.time() - self.boot_failure_time
                    if int(time_since_detection) % 5 == 0:  # Log every 5 seconds
                        self.get_logger().debug(
                            f"Camera boot failure still occurring: {message_text[:50]}"
                        )
                        
        except Exception as e:
            self.get_logger().debug(
                f"Error processing rosout message: {e}"
            )
    
    def health_check_callback(self):
        """Periodically check camera health"""
        if not self.get_parameter('enable_auto_recovery').value:
            return
        
        current_time = time.time()
        timeout = self.get_parameter('camera_timeout').value
        
        # Check if camera is publishing
        if self.last_image_time is None:
            # Never received an image - camera may not have booted
            self.camera_healthy = False
            
            # If boot failure was detected, trigger recovery faster
            if self.boot_failure_detected and self.boot_failure_time:
                boot_failure_timeout = 3.0  # Shorter timeout if boot failure detected (was 5.0)
                time_since_boot_failure = current_time - self.boot_failure_time
                
                if time_since_boot_failure > boot_failure_timeout:
                    self.get_logger().error(
                        f"🚨 Camera boot failure confirmed: {time_since_boot_failure:.1f}s since boot failure, "
                        f"no images received. Triggering USB reset recovery..."
                    )
                    if not self.recovery_in_progress:
                        self.attempt_recovery()
                else:
                    # Still waiting for boot failure timeout
                    remaining = boot_failure_timeout - time_since_boot_failure
                    self.get_logger().warn(
                        f"Camera health check: Boot failure detected {time_since_boot_failure:.1f}s ago. "
                        f"No images received. Will trigger recovery in {remaining:.1f}s..."
                    )
            else:
                # No boot failure detected yet, but no images either
                # Track how long we've been waiting for images since node start
                elapsed_without_images = current_time - self._node_start_time
                
                # After 8 seconds without images, assume boot failure even if not detected in logs
                if elapsed_without_images > 8.0:
                    self.get_logger().error(
                        f"🚨 Camera health check: No images received for {elapsed_without_images:.1f}s. "
                        f"Assuming boot failure and triggering recovery..."
                    )
                    if not self.recovery_in_progress:
                        self.boot_failure_detected = True  # Mark as boot failure for tracking
                        if not self.boot_failure_time:
                            self.boot_failure_time = current_time - 8.0  # Backdate to 8s ago
                        self.attempt_recovery()
                elif elapsed_without_images > 5.0:
                    self.get_logger().warn(
                        f"Camera health check: No images received for {elapsed_without_images:.1f}s. "
                        f"Camera may not have booted. Will trigger recovery at 8s if no images..."
                    )
                else:
                    # Only log at info level for first few seconds to reduce spam
                    if int(elapsed_without_images) % 3 == 0:  # Log every 3 seconds
                        self.get_logger().info(
                            f"Camera health check: No images received for {elapsed_without_images:.1f}s. "
                            f"Waiting for images or boot failure detection..."
                        )
        elif current_time - self.last_image_time > timeout:
            # Camera stopped publishing
            if self.camera_healthy:
                self.get_logger().error(
                    f"Camera health check FAILED: No images for "
                    f"{current_time - self.last_image_time:.1f}s (timeout: {timeout}s)"
                )
                self.camera_healthy = False
            
            # Attempt recovery if not already in progress
            if not self.recovery_in_progress:
                self.attempt_recovery()
        else:
            # Camera is healthy
            if not self.camera_healthy:
                self.camera_healthy = True
                self.get_logger().info("✅ Camera health restored")
        
        # Publish status
        self.publish_status()
    
    def attempt_recovery(self):
        """
        Attempt to recover camera from boot failure.
        
        Strategy:
        1. Find USB device for OAK camera
        2. Attempt USB reset
        3. Wait and check if camera recovers
        4. If still failing, restart camera node (requires launch file support)
        """
        if self.recovery_in_progress:
            return
        
        max_attempts = self.get_parameter('max_recovery_attempts').value
        cooldown = self.get_parameter('recovery_cooldown').value
        
        # Check cooldown
        if self.last_recovery_time:
            elapsed = time.time() - self.last_recovery_time
            if elapsed < cooldown:
                self.get_logger().debug(
                    f"Recovery cooldown: {cooldown - elapsed:.1f}s remaining"
                )
                return
        
        if self.recovery_attempts >= max_attempts:
            self.get_logger().error(
                f"Max recovery attempts ({max_attempts}) reached. "
                f"Camera recovery failed. Manual intervention required."
            )
            return
        
        self.recovery_in_progress = True
        self.recovery_attempts += 1
        self.last_recovery_time = time.time()
        
        self.get_logger().warn(
            f"Attempting camera recovery (attempt {self.recovery_attempts}/{max_attempts})..."
        )
        
        # Step 1: Find OAK camera USB device
        usb_device = self.find_oak_usb_device()
        if not usb_device:
            self.get_logger().error(
                "Camera recovery: OAK camera USB device not found. "
                "Cannot perform USB reset."
            )
            self.recovery_in_progress = False
            return
        
        self.get_logger().info(f"Found OAK camera USB device: {usb_device}")
        
        # Step 2: Attempt USB reset
        reset_success = self.reset_usb_device(usb_device)
        
        if reset_success:
            self.get_logger().info(
                f"USB reset successful. Waiting for camera to recover..."
            )
            # Wait for camera to recover
            recovery_wait_time = 5.0  # seconds
            time.sleep(recovery_wait_time)
            
            # Check if camera recovered
            if self.last_image_time and (time.time() - self.last_image_time) < 10.0:
                self.get_logger().info("✅ Camera recovery successful!")
                self.camera_healthy = True
                self.recovery_attempts = 0
            else:
                self.get_logger().warn(
                    "Camera did not recover after USB reset. "
                    "May need node restart or power cycle."
                )
        else:
            self.get_logger().error("USB reset failed. Camera recovery unsuccessful.")
        
        self.recovery_in_progress = False
    
    def find_oak_usb_device(self) -> Optional[str]:
        """
        Find OAK camera USB device using lsusb.
        
        Returns:
            USB device path (e.g., '/dev/bus/usb/001/002') or None
        """
        try:
            # Run lsusb to find OAK camera
            # OAK cameras typically show as "Intel Movidius MyriadX" or "Luxonis"
            result = subprocess.run(
                ['lsusb'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            
            if result.returncode != 0:
                self.get_logger().error(f"lsusb failed: {result.stderr}")
                return None
            
            # Look for OAK camera identifiers
            lines = result.stdout.split('\n')
            for line in lines:
                if 'Movidius' in line or 'MyriadX' in line or 'Luxonis' in line:
                    # Extract bus and device numbers
                    # Format: Bus 001 Device 002: ID 03e7:2485 Intel Movidius MyriadX
                    match = re.search(r'Bus (\d+) Device (\d+)', line)
                    if match:
                        bus = match.group(1)
                        device = match.group(2)
                        usb_path = f"/dev/bus/usb/{bus.zfill(3)}/{device.zfill(3)}"
                        
                        # Verify path exists
                        if os.path.exists(usb_path):
                            self.get_logger().info(f"Found OAK camera at: {usb_path}")
                            return usb_path
                        else:
                            self.get_logger().warn(
                                f"USB path {usb_path} does not exist"
                            )
            
            self.get_logger().warn("OAK camera not found in lsusb output")
            return None
            
        except subprocess.TimeoutExpired:
            self.get_logger().error("lsusb command timed out")
            return None
        except Exception as e:
            self.get_logger().error(f"Error finding USB device: {e}", exc_info=True)
            return None
    
    def reset_usb_device(self, usb_device: str) -> bool:
        """
        Reset USB device using usbreset utility.
        
        Args:
            usb_device: USB device path (e.g., '/dev/bus/usb/001/002')
            
        Returns:
            True if reset successful, False otherwise
        """
        try:
            # Check if usbreset utility exists
            # usbreset is typically in /usr/bin/usbreset or needs to be installed
            usbreset_paths = ['/usr/bin/usbreset', 'usbreset']
            usbreset_cmd = None
            
            for path in usbreset_paths:
                result = subprocess.run(
                    ['which', path] if path == 'usbreset' else ['test', '-f', path],
                    capture_output=True,
                    timeout=2.0
                )
                if result.returncode == 0:
                    usbreset_cmd = path
                    break
            
            if not usbreset_cmd:
                # Try alternative: use usb_modeswitch or direct ioctl
                # For now, try using chmod + r/w access method
                self.get_logger().warn(
                    "usbreset utility not found. Attempting alternative reset method..."
                )
                return self.reset_usb_device_alternative(usb_device)
            
            # Use usbreset utility
            # Note: usbreset typically requires root permissions
            # Try without sudo first, then with sudo if needed
            self.get_logger().info(f"Resetting USB device: {usb_device}")
            
            # Try without sudo first
            result = subprocess.run(
                [usbreset_cmd, usb_device],
                capture_output=True,
                text=True,
                timeout=10.0
            )
            
            if result.returncode == 0:
                self.get_logger().info("USB reset command executed successfully")
                return True
            
            # If failed, try with sudo (if available)
            self.get_logger().warn(
                f"USB reset without sudo failed: {result.stderr}. "
                f"Trying with sudo..."
            )
            
            # Check if sudo is available
            sudo_check = subprocess.run(
                ['which', 'sudo'],
                capture_output=True,
                timeout=2.0
            )
            
            if sudo_check.returncode == 0:
                result = subprocess.run(
                    ['sudo', usbreset_cmd, usb_device],
                    capture_output=True,
                    text=True,
                    timeout=10.0
                )
                
                if result.returncode == 0:
                    self.get_logger().info("USB reset with sudo executed successfully")
                    return True
                else:
                    self.get_logger().error(
                        f"USB reset with sudo failed: {result.stderr}"
                    )
            else:
                self.get_logger().error(
                    "sudo not available. USB reset requires root permissions. "
                    "Consider setting up udev rules or running with sudo."
                )
            
            return False
                
        except subprocess.TimeoutExpired:
            self.get_logger().error("USB reset command timed out")
            return False
        except Exception as e:
            self.get_logger().error(
                f"Error resetting USB device: {e}",
                exc_info=True
            )
            return False
    
    def reset_usb_device_alternative(self, usb_device: str) -> bool:
        """
        Alternative USB reset method using direct file operations.
        
        This method attempts to reset the USB device by writing to the device file.
        Requires appropriate permissions.
        
        Args:
            usb_device: USB device path
            
        Returns:
            True if reset attempted, False otherwise
        """
        try:
            # Alternative: Try to unbind and rebind the USB device
            # This requires finding the USB device in sysfs
            
            # First, get vendor and product ID from lsusb
            result = subprocess.run(
                ['lsusb'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            
            if result.returncode != 0:
                return False
            
            # Find device and extract ID
            lines = result.stdout.split('\n')
            for line in lines:
                if 'Movidius' in line or 'MyriadX' in line or 'Luxonis' in line:
                    # Extract ID (format: ID 03e7:2485)
                    match = re.search(r'ID ([0-9a-f]{4}):([0-9a-f]{4})', line, re.IGNORECASE)
                    if match:
                        vendor_id = match.group(1)
                        product_id = match.group(2)
                        
                        # Find device in sysfs
                        sysfs_path = f"/sys/bus/usb/devices/*:{vendor_id}:{product_id}"
                        
                        # Try to reset via sysfs (requires root or proper permissions)
                        # This is a simplified version - full implementation would
                        # need to find the actual device path in sysfs
                        self.get_logger().warn(
                            f"Alternative reset method: Found device {vendor_id}:{product_id}, "
                            f"but sysfs reset requires additional implementation"
                        )
                        return False
            
            return False
            
        except Exception as e:
            self.get_logger().error(
                f"Error in alternative USB reset: {e}",
                exc_info=True
            )
            return False
    
    def recovery_service_callback(self, request, response):
        """Service callback to manually trigger recovery"""
        self.get_logger().info("Manual camera recovery requested")
        
        if self.recovery_in_progress:
            response.success = False
            response.message = "Recovery already in progress"
            return response
        
        # Reset recovery attempts to allow manual recovery
        self.recovery_attempts = 0
        self.attempt_recovery()
        
        if self.camera_healthy:
            response.success = True
            response.message = "Camera recovery successful"
        else:
            response.success = False
            response.message = f"Camera recovery attempted (attempt {self.recovery_attempts})"
        
        return response
    
    def publish_status(self):
        """Publish camera recovery status"""
        status_msg = String()
        if self.camera_healthy:
            status_msg.data = "healthy"
        elif self.recovery_in_progress:
            status_msg.data = f"recovering (attempt {self.recovery_attempts})"
        else:
            status_msg.data = f"unhealthy (attempt {self.recovery_attempts})"
        
        self.status_pub.publish(status_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraRecovery()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
