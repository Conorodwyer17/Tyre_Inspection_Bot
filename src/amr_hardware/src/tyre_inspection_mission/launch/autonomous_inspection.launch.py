#!/usr/bin/env python3
"""
Launch file for complete autonomous tyre inspection mission.

This launch file starts all required components:
- Camera (OAK-D-Lite)
- Point cloud generation
- Vision pipeline (YOLO segmentation + 3D processing)
- Mission controller
- Photo capture service
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    ugv_vision_pkg = get_package_share_directory('ugv_vision')
    segmentation_3d_pkg = get_package_share_directory('segmentation_3d')
    tyre_inspection_pkg = get_package_share_directory('tyre_inspection_mission')
    nav2_bringup_pkg = get_package_share_directory('nav2_bringup')
    ugv_description_pkg = get_package_share_directory('ugv_description')
    ugv_nav_pkg = get_package_share_directory('ugv_nav')  # for params only
    
    # Declare launch arguments
    use_nav2_arg = DeclareLaunchArgument(
        'use_nav2',
        default_value='False',
        description='Enable Nav2 navigation (requires map and localization)'
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='False',
        description='Launch RViz for visualization'
    )
    
    use_visualizer_arg = DeclareLaunchArgument(
        'use_visualizer',
        default_value='True',
        description='Launch detection visualizer (shows camera feed with bounding boxes)'
    )
    
    use_mapping_nav_arg = DeclareLaunchArgument(
        'use_mapping_nav',
        default_value='True',
        description='Enable LiDAR bringup and SLAM+Nav2 for on-the-go mapping'
    )
    
    mapping_method_arg = DeclareLaunchArgument(
        'mapping_method',
        default_value='cartographer',
        description='2D SLAM method: cartographer or gmapping'
    )
    
    # Camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_vision_pkg, 'launch', 'oak_d_lite.launch.py')
        )
    )
    
    # Camera recovery service - automatically recovers from boot failures
    camera_recovery_node = Node(
        package='tyre_inspection_mission',
        executable='camera_recovery',
        name='camera_recovery',
        parameters=[{
            'camera_image_topic': '/oak/rgb/image_rect',
            'camera_health_check_interval': 5.0,  # Check every 5 seconds
            'camera_timeout': 10.0,  # No images for 10s = failure
            'max_recovery_attempts': 3,  # Max USB reset attempts
            'recovery_cooldown': 5.0,  # Wait 5s between attempts
            'enable_auto_recovery': True,  # Enable automatic recovery
        }],
        output='screen'
    )
    
    # Base robot description (publishes base_link/base_footprint and joints)
    robot_description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ugv_description_pkg, 'launch', 'display.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
            'rviz_config': 'nav_2d',
            'use_joint_state_publisher_gui': 'False'
        }.items()
    )
    
    # Point cloud generation node
    # Improved synchronization settings for better point cloud generation rate
    point_cloud_node = Node(
        package='depth_image_proc',
        executable='point_cloud_xyz_node',
        name='point_cloud_xyz_node',
        parameters=[{
            'use_approximate_sync': True,
            'queue_size': 50  # Increased from 10 to handle burst of camera_info messages
        }],
        remappings=[
            ('image_rect', '/oak/stereo/image_raw'),
            ('camera_info', '/oak/stereo/camera_info'),
            ('points', '/points')
        ],
        output='screen'
    )
    
    # Vision pipeline (segmentation)
    segmentation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(segmentation_3d_pkg, 'launch', 'segment_3d.launch.py')
        )
    )
    
    # Hardware driver nodes - REQUIRED for rover movement
    # NOTE: ugv_bringup now handles both reading sensor feedback AND sending velocity commands
    # ugv_driver is NOT needed for basic movement (only needed for joint states and LED control)
    # ugv_driver_node = Node(
    #     package='ugv_bringup',
    #     executable='ugv_driver',
    #     name='ugv_driver',
    #     output='screen'
    # )
    
    # ========================================================================
    # CRITICAL: UGV Base Controller Serial Port Configuration
    # ========================================================================
    # The Waveshare UGV Rover uses GPIO UART for ESP32 communication, NOT USB serial.
    # USB serial (/dev/ttyACM0) does NOT work - ESP32 won't respond to movement commands.
    # GPIO UART is the ONLY reliable method that actually works for motor control.
    #
    # Platform Detection:
    # - Jetson (Orin/Nano/etc): Uses /dev/ttyTHS1 (hardware UART)
    # - Other platforms (Raspberry Pi/etc): Uses /dev/ttyAMA0 (GPIO UART)
    #
    # Baud Rate: 115200 is the standard Waveshare GPIO UART baud rate.
    # This matches the default in ugv_bringup.py and is what the ESP32 firmware expects.
    # ========================================================================
    
    # Detect Jetson platform (same method as ugv_bringup.py)
    is_jetson = False
    try:
        # Method 1: Check for nv_tegra_release file (reliable Jetson indicator)
        if os.path.exists('/etc/nv_tegra_release'):
            is_jetson = True
        # Method 2: Check if /dev/ttyTHS1 exists (Jetson-specific hardware UART)
        elif os.path.exists('/dev/ttyTHS1'):
            is_jetson = True
    except Exception:
        pass
    
    # Set default port and baud based on platform
    if is_jetson:
        default_ugv_port = '/dev/ttyTHS1'  # Jetson hardware UART
        default_ugv_baud = '115200'  # Standard Waveshare GPIO UART baud rate
    else:
        default_ugv_port = '/dev/ttyAMA0'  # Other platforms GPIO UART
        default_ugv_baud = '115200'  # Standard GPIO UART baud rate
    
    # Declare launch arguments FIRST, before using them in Node definitions
    # Declare launch argument for UGV serial port
    ugv_port_arg = DeclareLaunchArgument(
        'ugv_serial_port',
        default_value=default_ugv_port,
        description='Serial port for UGV base controller (GPIO UART: /dev/ttyTHS1 on Jetson, /dev/ttyAMA0 on others)'
    )
    
    # Declare launch argument for UGV serial baud rate
    # CRITICAL: Must be 115200 for GPIO UART (Waveshare standard, matches ESP32 firmware)
    ugv_baud_arg = DeclareLaunchArgument(
        'ugv_serial_baud',
        default_value=default_ugv_baud,  # 115200 for GPIO UART
        description='Serial baud rate for UGV base controller (115200 for GPIO UART)'
    )
    
    # Create LaunchConfiguration objects AFTER declaring the arguments
    ugv_serial_port_config = LaunchConfiguration('ugv_serial_port')
    ugv_serial_baud_config = LaunchConfiguration('ugv_serial_baud')
    
    # ugv_bringup: Reads sensor feedback (IMU, odometry) AND sends velocity commands via serial
    # Pass serial port and baud rate parameters to the node
    # NOTE: The node handles string-to-int conversion for baud rate, so we can pass LaunchConfiguration directly
    ugv_bringup_node = Node(
        package='ugv_bringup',
        executable='ugv_bringup',
        name='ugv_bringup',
        output='screen',
        parameters=[{
            'serial_port': ugv_serial_port_config,
            'serial_baud': ugv_serial_baud_config  # Node will convert string to int if needed
        }]
    )
    
    # IMU complementary filter: Converts imu/data_raw to imu/data for base_node
    imu_filter_node = Node(
        package='imu_complementary_filter',
        executable='complementary_filter_node',
        name='complementary_filter_node',
        output='screen',
        parameters=[
            {'do_bias_estimation': True},
            {'do_adaptive_gain': True},
            {'use_mag': False},
            {'gain_acc': 0.01},
            {'gain_mag': 0.01},
        ]
    )
    
    # base_node: Processes odometry and publishes /odom topic
    # Note: base_node subscribes to imu/data (not imu/data_raw)
    base_node = Node(
        package='ugv_base_node',
        executable='base_node',
        name='base_node',
        parameters=[{'pub_odom_tf': True}],
        arguments=['--ros-args', '--log-level', 'ERROR'],  # Reduce IMU NaN warnings
        output='screen'
    )
    
    # Minimal LiDAR + odometry bringup
    # LiDAR Serial Port Configuration
    # ============================================================
    # Note: ESP32 base controller uses GPIO UART (/dev/ttyTHS1 on Jetson, /dev/ttyAMA0 on others),
    # so there's NO conflict with LiDAR which uses USB serial (/dev/ttyACM0 or /dev/ttyACM1).
    # LiDAR typically uses USB serial - check which port your LiDAR is connected to.
    # ============================================================
    default_lidar_port = '/dev/ttyACM1' if os.path.exists('/dev/ttyACM1') else '/dev/ttyACM0'
    
    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value=default_lidar_port,
        description='Serial port for LiDAR (typically USB serial: /dev/ttyACM0 or /dev/ttyACM1)'
    )
    ldlidar_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar'), 'launch', 'ldlidar.launch.py')
        ),
        launch_arguments={
            'lidar_port': LaunchConfiguration('lidar_port')
        }.items(),
        condition=IfCondition(LaunchConfiguration('use_mapping_nav'))
    )
    rf2o_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('rf2o_laser_odometry'), 'launch', 'rf2o_laser_odometry.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_mapping_nav'))
    )
    
    # SLAM + Nav2 (mapping and navigation simultaneously) using nav2_bringup
    # Note: Log levels are set via runtime script (quiet_logs.sh) for better compatibility
    nav2_params = os.path.join(ugv_nav_pkg, 'param', 'slam_nav.yaml')
    nav2_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_pkg, 'launch', 'slam_launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_mapping_nav')),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': nav2_params,
            'use_respawn': 'False'
        }.items()
    )
    # CRITICAL: Use custom Nav2 navigation launch that remaps controller cmd_vel output
    # Nav2's controller publishes to /cmd_vel by default, which conflicts with our multiplexer.
    # This custom launch remaps it to /cmd_vel/nav2 so the multiplexer can handle it as Priority 3.
    # 
    # REMAPPING: controller_server, behavior_server, and velocity_smoother all publish cmd_vel.
    # They are remapped from /cmd_vel to /cmd_vel/nav2 so the multiplexer receives them as Priority 3.
    nav2_navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tyre_inspection_pkg, 'launch', 'nav2_navigation_with_remap.launch.py')
        ),
        condition=IfCondition(LaunchConfiguration('use_mapping_nav')),
        launch_arguments={
            'use_sim_time': 'False',
            'autostart': 'True',
            'params_file': nav2_params,  # Pass the params file path as a string
            'use_composition': 'False',
            'use_respawn': 'False',
            'namespace': '',
            'use_namespace': 'false',
            'log_level': 'info',
        }.items()
    )
    
    # CRITICAL: cmd_vel_multiplexer - Priority-based command arbitration
    # MUST start BEFORE any nodes that publish to priority topics
    # This node arbitrates between multiple cmd_vel sources and publishes highest-priority command
    # CRITICAL: cmd_vel_multiplexer - MUST have respawn=True to auto-restart if crashes
    # This node is CRITICAL for robot movement - if it dies, robot cannot move
    cmd_vel_multiplexer_node = Node(
        package='tyre_inspection_mission',
        executable='cmd_vel_multiplexer',
        name='cmd_vel_multiplexer',
        output='screen',
        respawn=True,  # CRITICAL: Auto-restart if crashes (robot cannot move without this node)
        respawn_delay=2.0,  # Wait 2 seconds before restarting
        parameters=[{
            'use_sim_time': False,  # CRITICAL: Must be boolean, not string
        }]
    )
    
    # CRITICAL FIX: Nav2's cmd_vel output is remapped to /cmd_vel/nav2 via nav2_navigation_with_remap.launch.py
    # The remapping uses absolute topic names ('/cmd_vel' -> '/cmd_vel/nav2') to ensure proper remapping.
    # The multiplexer subscribes to /cmd_vel/nav2 as Priority 3 and arbitrates correctly.
    # No conflicts - Nav2 commands are handled by multiplexer as Priority 3.
    #
    # NOTE: nav2_cmd_vel_relay node is OBSOLETE and not needed - remapping handles it directly.
    # The relay was previously attempted but created a feedback loop and has been removed.
    
    # Movement diagnostic node - CRITICAL: Monitors cmd_vel and robot movement
    # CRITICAL: movement_diagnostic - Add respawn=True to auto-restart if crashes
    # This node is useful for diagnostics but not critical for robot movement
    movement_diagnostic_node = Node(
        package='tyre_inspection_mission',
        executable='movement_diagnostic',
        name='movement_diagnostic',
        output='screen',
        respawn=True,  # Auto-restart if crashes (helpful for diagnostics)
        respawn_delay=2.0,  # Wait 2 seconds before restarting
        parameters=[{
            'use_sim_time': False,  # CRITICAL: Must be boolean, not string
        }]
    )
    
    # Photo capture service
    photo_capture_node = Node(
        package='tyre_inspection_mission',
        executable='photo_capture_service',
        name='photo_capture_service',
        parameters=[{
            'camera_topic': '/oak/rgb/image_rect',
            'default_storage_dir': '~/tyre_inspection_photos',
            'image_format': 'jpg',
            'image_quality': 95
        }],
        output='screen'
    )
    
    # Mission controller
    mission_controller_node = Node(
        package='tyre_inspection_mission',
        executable='mission_controller',
        name='mission_controller',
        parameters=[{
            'photo_storage_dir': '~/tyre_inspection_photos',
            'approach_distance': 1.5,  # meters from object (safer distance for first test)
            'navigation_timeout': 60.0,  # seconds
            'detection_timeout': 60.0,  # seconds (increased from 30.0 to match code default - better for first test)
            'photo_capture_timeout': 10.0,  # seconds
            'vehicle_class_names': ['truck', 'car'],  # List of vehicle types to detect
            'tyre_class_name': 'tyre',
            'license_plate_class_name': 'license_plate',  # NEW: Direct license plate detection class
            'bounding_boxes_topic': '/darknet_ros_3d/bounding_boxes',  # Must match segmentation_processor output
            'navigation_frame': 'map'  # Frame for navigation: 'map' (with Nav2/SLAM) or 'odom' (without Nav2)
        }],
        output='screen'
    )
    
    # Optional: Nav2 launch (if using navigation)
    # Uncomment if you want Nav2 included:
    # nav2_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         os.path.join(get_package_share_directory('ugv_nav'), 'launch', 'slam_nav.launch.py')
    #     ),
    #     condition=IfCondition(LaunchConfiguration('use_nav2'))
    # )
    
    # Detection visualizer node (optional - shows camera feed with bounding boxes)
    detection_visualizer_node = Node(
        package='tyre_inspection_mission',
        executable='visualize_detections',
        name='detection_visualizer',
        parameters=[{
            'camera_topic': '/oak/rgb/image_rect',
            'bbox_topic': '/darknet_ros_3d/bounding_boxes',
            'output_topic': '/detection_visualization/image_annotated',
            'show_labels': True,
            'show_confidence': True,
            'line_thickness': 2,
            'font_scale': 0.6,
            'show_window': False  # Set to True to show OpenCV window (requires X11/display)
        }],
        condition=IfCondition(LaunchConfiguration('use_visualizer')),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_nav2_arg,
        use_rviz_arg,
        use_mapping_nav_arg,
        use_visualizer_arg,
        mapping_method_arg,
        ugv_port_arg,
        ugv_baud_arg,
        lidar_port_arg,
        # CRITICAL: cmd_vel_multiplexer MUST start FIRST (before any nodes that publish cmd_vel)
        cmd_vel_multiplexer_node,
        # Camera and hardware nodes
        camera_launch,
        camera_recovery_node,
        ugv_bringup_node,
        imu_filter_node,
        base_node,
        # Navigation stack (SLAM + Nav2)
        ldlidar_launch,
        rf2o_launch,
        robot_description_launch,
        nav2_slam_launch,
        nav2_navigation_launch,  # CRITICAL: Custom launch remaps Nav2's cmd_vel to /cmd_vel/nav2
        # Vision pipeline
        point_cloud_node,
        segmentation_launch,
        # Mission control nodes
        photo_capture_node,
        movement_diagnostic_node,  # CRITICAL: Diagnostic monitoring
        mission_controller_node,
        detection_visualizer_node,
    ])
