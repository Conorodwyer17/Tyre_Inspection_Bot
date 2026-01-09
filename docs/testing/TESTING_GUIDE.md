# Testing Guide - Tyre Inspection Mission

## Overview

This guide provides procedures for testing the tyre inspection mission system, including unit tests, integration tests, and system tests.

## Test Categories

### 1. Unit Tests
Test individual functions and modules in isolation.

### 2. Integration Tests
Test interactions between modules.

### 3. System Tests
Test complete mission execution.

### 4. Error Scenario Tests
Test error handling and recovery.

## Test Procedures

### Detection Testing

**Test: Vehicle Detection**
1. Place vehicle in camera view
2. Start mission
3. Verify vehicle is detected
4. Check logs for `[DET]` category
5. Verify `VehicleData` is created

**Test: Tyre Detection**
1. Navigate to vehicle
2. Switch to inspection mode
3. Verify tyres are detected
4. Check logs for tyre detections
5. Verify `TyreData` is created

**Test: LiDAR Fusion**
1. Place vehicle in view
2. Cover camera
3. Verify LiDAR continues tracking
4. Check logs for LiDAR updates

### Navigation Testing

**Test: Goal Validation**
1. Calculate goal too close to robot
2. Verify goal is rejected
3. Check error code: `NAV_001`
4. Verify recovery strategy

**Test: Navigation Execution**
1. Send valid navigation goal
2. Verify Nav2 accepts goal
3. Monitor navigation progress
4. Verify arrival at goal

**Test: Navigation Timeout**
1. Send goal that takes too long
2. Verify timeout handling
3. Check recovery behavior

### Error Recovery Testing

**Test: Navigation Error Recovery**
1. Simulate goal rejection
2. Verify error recovery state
3. Check recovery attempts
4. Verify mission continues

**Test: Detection Error Recovery**
1. Disable YOLO node
2. Verify fallback waypoint generation
3. Check mission continues

**Test: System Health Check**
1. Disable critical service
2. Verify health check detects issue
3. Check error recovery behavior

## Test Scenarios

### Scenario 1: Normal Mission Flow
1. Start mission
2. Vehicle detected
3. Navigate to license plate
4. Capture license plate
5. Detect tyres
6. Navigate to each tyre
7. Capture each tyre
8. Complete mission

**Expected:** All steps complete successfully

### Scenario 2: Vehicle Too Close
1. Place vehicle <0.5m from robot
2. Start mission
3. Verify detection occurs
4. Verify navigation is skipped
5. Verify capture proceeds

**Expected:** System adapts and proceeds

### Scenario 3: No Tyres Detected
1. Start mission with vehicle
2. Navigate to vehicle
3. No tyres detected
4. Verify fallback waypoints generated
5. Verify navigation to fallback positions

**Expected:** Fallback waypoints used

### Scenario 4: Navigation Failure
1. Send invalid navigation goal
2. Verify goal rejection
3. Verify recovery attempt
4. Verify mission continues

**Expected:** Recovery succeeds, mission continues

### Scenario 5: Multiple Vehicles
1. Place multiple vehicles in view
2. Start mission
3. Verify all vehicles detected
4. Verify each vehicle processed
5. Verify mission completes

**Expected:** All vehicles processed

## Diagnostic Commands for Testing

```bash
# Monitor mission status
ros2 topic echo /mission_controller/status

# Monitor detection messages
ros2 topic echo /darknet_ros_3d/bounding_boxes

# Monitor navigation goals
ros2 topic echo /mission_controller/goal_markers

# Check system health
ros2 service call /mission_controller/health_check std_srvs/srv/Trigger
```

## Test Checklist

- [ ] Vehicle detection works
- [ ] Tyre detection works
- [ ] Navigation goals are valid
- [ ] Navigation execution succeeds
- [ ] Photo capture works
- [ ] Error recovery works
- [ ] Fallback strategies work
- [ ] System health monitoring works
- [ ] Multiple vehicles handled
- [ ] Mission completes successfully

---

**Last Updated:** Current Session  
**Maintained By:** Development Team
