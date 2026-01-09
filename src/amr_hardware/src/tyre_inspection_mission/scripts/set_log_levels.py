#!/usr/bin/env python3
"""
Script to set log levels for noisy components.
Run this after launching the system to reduce log verbosity.
"""

import rclpy
from rclpy.logging import set_logger_level, get_logger
import sys


def set_log_levels():
    """Set appropriate log levels to reduce spam"""
    rclpy.init()
    
    logger = get_logger('log_level_setter')
    
    # Components to quiet down
    log_configs = {
        # Costmap/Planner - reduce out-of-bounds warnings
        'nav2_costmap_2d': 'ERROR',
        'local_costmap.local_costmap': 'ERROR',
        'global_costmap.global_costmap': 'ERROR',
        'planner_server': 'ERROR',
        'controller_server': 'ERROR',
        
        # SLAM - reduce message filter spam
        'slam_toolbox': 'WARN',
        'sync_slam_toolbox_node': 'WARN',
        
        # Odometry - reduce eigensolver warnings
        'rf2o_laser_odometry': 'ERROR',
        'rf2o_laser_odometry_node': 'ERROR',
        
        # IMU/Base - reduce NaN warnings
        'base_node': 'ERROR',
        'ugv_bringup': 'WARN',  # Keep some info for debugging
    }
    
    success_count = 0
    fail_count = 0
    
    for logger_name, level in log_configs.items():
        try:
            set_logger_level(logger_name, level)
            logger.info(f"✓ Set {logger_name} to {level}")
            success_count += 1
        except Exception as e:
            logger.warn(f"✗ Failed to set {logger_name}: {e}")
            fail_count += 1
    
    logger.info(f"\n✅ Log levels set: {success_count} succeeded, {fail_count} failed")
    logger.info("Log verbosity reduced. Important messages (mission controller, YOLO, errors) will still appear.")
    
    rclpy.shutdown()


if __name__ == '__main__':
    set_log_levels()
