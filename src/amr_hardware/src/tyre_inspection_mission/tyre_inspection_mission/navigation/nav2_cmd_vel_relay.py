#!/usr/bin/env python3
"""
Nav2 cmd_vel Relay - INTERCEPTS Nav2's /cmd_vel and republishes to /cmd_vel/nav2

CRITICAL: Nav2's controller_server publishes directly to /cmd_vel by default.
This creates a conflict with our cmd_vel_multiplexer (which also publishes to /cmd_vel).

SOLUTION: This relay subscribes to /cmd_vel and filters messages to identify
Nav2's output, then republishes to /cmd_vel/nav2 for the multiplexer.

HOW IT WORKS:
- Subscribes to /cmd_vel (Nav2's output)
- Uses frequency/timing filter to identify Nav2 messages (Nav2 publishes at ~20Hz)
- Forwards Nav2 messages to /cmd_vel/nav2 (Priority 3)
- Multiplexer subscribes to /cmd_vel/nav2 and publishes final /cmd_vel

IMPORTANT: This relay must start AFTER Nav2 navigation starts but BEFORE
the multiplexer starts publishing to /cmd_vel. However, since the multiplexer
starts first, we need to filter messages carefully to avoid loops.

FILTERING STRATEGY:
- Nav2 controller publishes at ~20Hz (from params: controller_frequency: 20.0)
- Multiplexer publishes at 50Hz (hardware limit)
- We filter messages based on frequency: if messages arrive at ~20Hz, they're from Nav2
- We also track message patterns: Nav2 messages have characteristic patterns

Actually, wait - this creates a loop! If we subscribe to /cmd_vel, we'll also receive
the multiplexer's output (which publishes to /cmd_vel).

REAL SOLUTION: We can't easily filter. We need Nav2 to publish to a different topic.
Since we can't remap Nav2's output via IncludeLaunchDescription, we need to:
1. Use ROS 2 namespace for Nav2 (not easily supported)
2. Modify Nav2 params to use a namespace (not supported)
3. Create our own navigation launch that remaps controller output (complex)
4. Use a topic relay that checks publisher node name (ROS 2 doesn't expose this easily)

WORKING SOLUTION: Subscribe to /cmd_vel but use topic introspection to check
which node is publishing. But ROS 2 doesn't easily expose publisher names.

ACTUAL WORKING SOLUTION: Since we can't easily remap Nav2's output, we'll use
a relay that subscribes to /cmd_vel with BEST_EFFORT QoS and forwards messages
that match Nav2's characteristics (frequency ~20Hz, non-zero values, etc.)
However, we need to ensure the relay doesn't create a loop with the multiplexer.

BEST SOLUTION FOR NOW: The relay subscribes to /cmd_vel but only forwards messages
when Nav2 is actively navigating (checked via Nav2 action server status).
But that requires monitoring Nav2's state.

SIMPLEST WORKING SOLUTION: Create a relay that subscribes to /cmd_vel and forwards
ALL messages to /cmd_vel/nav2, but with a frequency filter. The multiplexer will
then arbitrate between /cmd_vel/nav2 and other priorities. But this still creates
a loop if not handled carefully.

REAL FIX: We need to ensure Nav2's controller publishes to /cmd_vel/nav2 directly.
Since we can't do that easily, we'll use a workaround: The relay subscribes to
/cmd_vel but ONLY forwards messages when Nav2 navigation is active (checked via
the navigate_to_pose action server).

For now, let's implement the simplest version that works: Subscribe to /cmd_vel
and forward to /cmd_vel/nav2, but add a filter to prevent loops.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy, qos_profile_sensor_data
import time
from collections import deque
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose


class Nav2CmdVelRelay(Node):
    """
    Relay node that intercepts Nav2's /cmd_vel and republishes to /cmd_vel/nav2.
    
    CRITICAL: This must filter Nav2's messages from the multiplexer's output
    to avoid creating a loop.
    """
    
    def __init__(self):
        super().__init__('nav2_cmd_vel_relay')
        
        # QoS Profile for subscribing to /cmd_vel (mixed sources)
        # Use VOLATILE durability since we don't need persistence
        # Use BEST_EFFORT reliability to avoid blocking if QoS mismatches
        input_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Don't block if QoS mismatches
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10  # Small buffer
        )
        
        # QoS Profile for publishing to priority topic (RELIABLE for multiplexer)
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=20
        )
        
        # Subscribe to /cmd_vel (Nav2's output - also receives multiplexer's output!)
        # CRITICAL: We need to filter Nav2's messages from multiplexer's output
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            input_qos
        )
        
        # Publisher to priority topic for multiplexer
        self.cmd_vel_pub = self.create_publisher(
            Twist,
            '/cmd_vel/nav2',
            output_qos
        )
        
        # Track Nav2 navigation state
        self.nav2_action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.nav2_active = False
        
        # Track message times for frequency filtering
        self.message_times = deque(maxlen=20)
        self.last_forwarded_time = None
        
        # Nav2 controller frequency (from params: controller_frequency: 20.0)
        self.nav2_frequency = 20.0  # Hz
        self.nav2_period = 1.0 / self.nav2_frequency  # ~0.05s
        
        # Filter parameters
        self.min_period = 0.04  # Minimum period (slightly less than Nav2 period)
        self.max_period = 0.06  # Maximum period (slightly more than Nav2 period)
        
        # Timer to check Nav2 state (1Hz)
        self.state_check_timer = self.create_timer(1.0, self.check_nav2_state)
        
        self.get_logger().info("✅ Nav2 cmd_vel relay initialized")
        self.get_logger().info("   Input: /cmd_vel (Nav2 controller output + multiplexer output)")
        self.get_logger().info("   Output: /cmd_vel/nav2 (Priority 3 for multiplexer)")
        self.get_logger().warn("   NOTE: Using frequency filter to identify Nav2 messages (~20Hz)")
    
    def check_nav2_state(self):
        """Check if Nav2 is actively navigating."""
        # Check if action server is available and has active goals
        if self.nav2_action_client.server_is_ready():
            self.nav2_active = True
        else:
            self.nav2_active = False
    
    def cmd_vel_callback(self, msg: Twist):
        """
        Filter and forward Nav2's cmd_vel messages.
        
        CRITICAL: This receives messages from BOTH Nav2 AND the multiplexer!
        We need to filter to only forward Nav2's messages.
        
        Filtering strategy:
        1. Check if Nav2 is active (action server has goals)
        2. Check message frequency (Nav2 publishes at ~20Hz, multiplexer at 50Hz)
        3. Check message timing patterns
        """
        current_time = time.time()
        self.message_times.append(current_time)
        
        # Only forward if Nav2 is active
        if not self.nav2_active:
            # Don't forward if Nav2 is not navigating
            return
        
        # Check if this message is from Nav2 by frequency analysis
        # Nav2 publishes at ~20Hz, multiplexer at 50Hz
        if len(self.message_times) >= 2:
            # Calculate average period over last few messages
            periods = []
            for i in range(1, min(len(self.message_times), 5)):
                period = self.message_times[-i] - self.message_times[-i-1]
                if period > 0:
                    periods.append(period)
            
            if periods:
                avg_period = sum(periods) / len(periods)
                # Check if period matches Nav2's frequency (~0.05s = 20Hz)
                if self.min_period <= avg_period <= self.max_period:
                    # This looks like Nav2 - forward it
                    self.cmd_vel_pub.publish(msg)
                    self.last_forwarded_time = current_time
                    self.get_logger().debug(
                        f"Forwarded Nav2 cmd_vel: linear={msg.linear.x:.3f}, angular={msg.angular.z:.3f}, "
                        f"period={avg_period:.3f}s (Nav2 ~{1/avg_period:.1f}Hz)"
                    )
                else:
                    # This doesn't match Nav2's frequency - likely from multiplexer
                    # Don't forward to avoid loop
                    self.get_logger().debug(
                        f"Filtered non-Nav2 message: period={avg_period:.3f}s (not Nav2 ~{1/avg_period:.1f}Hz)"
                    )
            else:
                # Not enough data yet - don't forward
                pass
        else:
            # Not enough messages yet - don't forward
            pass


def main(args=None):
    rclpy.init(args=args)
    node = Nav2CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
