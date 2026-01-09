#!/usr/bin/env python3
"""
Mission Timeout Handler Module

Handles mission-wide timeouts to prevent missions from running indefinitely.
Monitors total mission time and individual phase timeouts.

This module provides:
- Global mission timeout monitoring
- Phase-specific timeout tracking
- Timeout warnings and actions
- Graceful timeout handling
"""

import time
from typing import Optional, Dict
from enum import Enum


class MissionPhase(Enum):
    """Mission phases for timeout tracking."""
    SEARCHING = "searching"
    LICENSE_PLATE = "license_plate"
    TYRE_DETECTION = "tyre_detection"
    TYRE_NAVIGATION = "tyre_navigation"
    TYRE_CAPTURE = "tyre_capture"
    COMPLETION_CHECK = "completion_check"


class MissionTimeoutHandler:
    """
    Handles mission-wide and phase-specific timeouts.
    """
    
    def __init__(self, logger, parameters: Dict):
        """
        Initialize mission timeout handler.
        
        Args:
            logger: ROS 2 logger instance
            parameters: Dictionary of ROS parameters
        """
        self.logger = logger
        
        # Parameters
        self.max_mission_time = parameters.get('max_mission_time', 3600.0)  # 1 hour default
        self.max_phase_time = parameters.get('max_phase_time', 600.0)  # 10 minutes per phase
        self.timeout_warning_threshold = parameters.get('timeout_warning_threshold', 0.8)  # Warn at 80%
        
        # Mission timing
        self.mission_start_time = None
        self.phase_start_time = None
        self.current_phase = None
        self.phase_times = {}  # Track time spent in each phase
        
        self.logger.info(
            f"MissionTimeoutHandler initialized: "
            f"max_mission_time={self.max_mission_time}s, "
            f"max_phase_time={self.max_phase_time}s"
        )
    
    def start_mission(self):
        """Start mission timing."""
        self.mission_start_time = time.time()
        self.phase_start_time = None
        self.current_phase = None
        self.phase_times.clear()
        self.logger.info("Mission timing started")
    
    def start_phase(self, phase: MissionPhase):
        """
        Start timing a mission phase.
        
        Args:
            phase: Current mission phase
        """
        # End previous phase timing
        if self.current_phase and self.phase_start_time:
            elapsed = time.time() - self.phase_start_time
            if self.current_phase.value not in self.phase_times:
                self.phase_times[self.current_phase.value] = 0.0
            self.phase_times[self.current_phase.value] += elapsed
        
        # Start new phase
        self.current_phase = phase
        self.phase_start_time = time.time()
        self.logger.debug(f"Started phase timing: {phase.value}")
    
    def check_mission_timeout(self) -> Dict:
        """
        Check if mission has exceeded maximum time.
        
        Returns:
            Dict with:
                - 'timeout': bool - Has mission timed out?
                - 'elapsed_time': float - Time elapsed
                - 'remaining_time': float - Time remaining
                - 'warning': bool - Should warn about approaching timeout?
                - 'action': str - Recommended action
        """
        result = {
            'timeout': False,
            'elapsed_time': 0.0,
            'remaining_time': self.max_mission_time,
            'warning': False,
            'action': 'continue'
        }
        
        if not self.mission_start_time:
            return result
        
        elapsed = time.time() - self.mission_start_time
        remaining = self.max_mission_time - elapsed
        warning_threshold = self.max_mission_time * self.timeout_warning_threshold
        
        result['elapsed_time'] = elapsed
        result['remaining_time'] = remaining
        
        if elapsed >= self.max_mission_time:
            result['timeout'] = True
            result['action'] = 'abort'
            self.logger.error(
                f"Mission timeout: {elapsed:.1f}s >= {self.max_mission_time}s"
            )
        elif elapsed >= warning_threshold:
            result['warning'] = True
            result['action'] = 'warn'
            self.logger.warn(
                f"Mission approaching timeout: {elapsed:.1f}s / {self.max_mission_time}s "
                f"({remaining:.1f}s remaining)"
            )
        
        return result
    
    def check_phase_timeout(self) -> Dict:
        """
        Check if current phase has exceeded maximum time.
        
        Returns:
            Dict with timeout information
        """
        result = {
            'timeout': False,
            'elapsed_time': 0.0,
            'remaining_time': self.max_phase_time,
            'warning': False,
            'action': 'continue'
        }
        
        if not self.phase_start_time or not self.current_phase:
            return result
        
        elapsed = time.time() - self.phase_start_time
        remaining = self.max_phase_time - elapsed
        warning_threshold = self.max_phase_time * self.timeout_warning_threshold
        
        result['elapsed_time'] = elapsed
        result['remaining_time'] = remaining
        
        if elapsed >= self.max_phase_time:
            result['timeout'] = True
            result['action'] = 'skip_phase'
            self.logger.warn(
                f"Phase timeout: {self.current_phase.value} took {elapsed:.1f}s >= {self.max_phase_time}s"
            )
        elif elapsed >= warning_threshold:
            result['warning'] = True
            result['action'] = 'warn'
            self.logger.warn(
                f"Phase approaching timeout: {self.current_phase.value} "
                f"{elapsed:.1f}s / {self.max_phase_time}s ({remaining:.1f}s remaining)"
            )
        
        return result
    
    def get_mission_statistics(self) -> Dict:
        """
        Get mission timing statistics.
        
        Returns:
            Dict with timing statistics
        """
        stats = {
            'mission_elapsed': 0.0,
            'current_phase': None,
            'phase_elapsed': 0.0,
            'phase_times': self.phase_times.copy(),
            'total_phase_time': sum(self.phase_times.values())
        }
        
        if self.mission_start_time:
            stats['mission_elapsed'] = time.time() - self.mission_start_time
        
        if self.current_phase:
            stats['current_phase'] = self.current_phase.value
            if self.phase_start_time:
                stats['phase_elapsed'] = time.time() - self.phase_start_time
        
        return stats
    
    def end_mission(self):
        """End mission timing and log statistics."""
        if self.mission_start_time:
            total_time = time.time() - self.mission_start_time
            self.logger.info(
                f"Mission completed in {total_time:.1f}s. "
                f"Phase breakdown: {self.phase_times}"
            )
        
        self.mission_start_time = None
        self.phase_start_time = None
        self.current_phase = None
