# Real-World Mission Execution Flow

## Overview
This document describes how the autonomous tyre inspection mission works in a real-world scenario, from robot startup to mission completion.

## Real-World Scenario: Autonomous Vehicle Inspection Yard

### Initial Conditions
- **Robot starting position**: Random position and orientation in inspection yard
- **Robot orientation to vehicles**: Unknown - robot may face front, side, or rear of any vehicle
- **Vehicles**: Multiple vehicles (trucks) parked at various locations and orientations
- **Environment**: Real-world yard with:
  - Ground truth (real terrain, not simulation)
  - Obstacles (other vehicles, equipment, barriers)
  - Lighting conditions (day/night, shadows, reflections)
  - Physical constraints (real motors, sensors, communication delays)

---

## Complete Mission Flow

### Phase 1: System Startup and Initialization
1. **Robot powers on** → All hardware nodes start (base_node, camera, LiDAR, etc.)
2. **SLAM/Localization starts** → Robot begins mapping and localizing in yard
3. **Mission controller initializes** → Waits for all systems to be ready
4. **System readiness check** → Validates:
   - Nav2 action server ready
   - Camera publishing images
   - Odometry publishing (`/odom` topic active)
   - TF tree complete (map → base_footprint transform available)
   - License plate detector initialized
5. **Mission starts** → State machine enters `IDLE` → `DETECTING_VEHICLES`

### Phase 2: Vehicle Detection
1. **Camera captures images** → YOLO detects vehicles in frame
2. **3D processing** → Point cloud used to get vehicle 3D position
3. **Stable detection** → Vehicle must be detected for N frames (stability check)
4. **Vehicle tracking** → Each vehicle assigned unique ID based on position
5. **Vehicle stored** → Added to `detected_trucks` dictionary
6. **State transition** → `DETECTING_VEHICLES` → `TRUCK_DETECTED` (first unprocessed vehicle)

### Phase 3: Robot-Relative Goal Calculation (NEW - Works from ANY Angle)

**CRITICAL FEATURE**: Robot can approach from any angle (front, side, rear)

#### Scenario A: Robot at Front of Vehicle (0°)
- Robot position: (5m, 0m) relative to vehicle
- Vehicle position: (0m, 0m)
- **Calculation**: `approach_direction = atan2(0, 5) = 0°` (from vehicle toward robot)
- **Goal position**: Vehicle position + (approach_distance * cos(0°), approach_distance * sin(0°))
- **Goal orientation**: `approach_direction + π = 180°` (robot faces vehicle)
- **Result**: Robot approaches from front, facing vehicle

#### Scenario B: Robot at Side of Vehicle (90°)
- Robot position: (0m, 5m) relative to vehicle
- Vehicle position: (0m, 0m)
- **Calculation**: `approach_direction = atan2(5, 0) = 90°` (from vehicle toward robot)
- **Goal position**: Vehicle position + (approach_distance * cos(90°), approach_distance * sin(90°))
- **Goal orientation**: `approach_direction + π = 270°` (robot faces vehicle)
- **Result**: Robot approaches from side, facing vehicle

#### Scenario C: Robot at Rear of Vehicle (180°)
- Robot position: (-5m, 0m) relative to vehicle
- Vehicle position: (0m, 0m)
- **Calculation**: `approach_direction = atan2(0, -5) = 180°` (from vehicle toward robot)
- **Goal position**: Vehicle position + (approach_distance * cos(180°), approach_distance * sin(180°))
- **Goal orientation**: `approach_direction + π = 0°` (robot faces vehicle)
- **Result**: Robot approaches from rear, facing vehicle

#### Scenario D: Robot at Any Angle (e.g., 45°)
- Robot position: (3.5m, 3.5m) relative to vehicle
- Vehicle position: (0m, 0m)
- **Calculation**: `approach_direction = atan2(3.5, 3.5) = 45°`
- **Goal position**: Vehicle position + offset at 45°
- **Goal orientation**: `approach_direction + π = 225°` (robot faces vehicle)
- **Result**: Robot approaches from diagonal, facing vehicle

**Fallback for Very Close**: If robot < 0.5m from vehicle, uses vehicle's forward direction as fallback

### Phase 4: Navigation to License Plate

1. **Goal calculation** → License plate approach pose calculated (robot-relative)
2. **Goal validation** → Checks:
   - NaN/Inf values → Rejected
   - Extreme coordinates (> ±1000m) → Rejected
   - Maximum distance (> 100m) → Rejected
   - Minimum distance (< 0.8m) → Adjusted
   - Free space validation → Path clear of obstacles
3. **Navigation starts** → Goal sent to Nav2 AND direct navigation fallback activated
4. **Movement guarantee activated** → Ensures robot always moves
5. **State transition** → `TRUCK_DETECTED` → `NAVIGATING_TO_LICENSE_PLATE`

#### During Navigation:
- **Nav2 monitoring**: If Nav2 stops publishing cmd_vel → Direct navigation takes over
- **Odometry monitoring**: If odometry stale (> 2s old) → Degraded mode (assume movement)
- **TF transform monitoring**: If transform stale (> 1s old) → Reject, get new pose
- **Progress tracking**: Monitors distance traveled, validates minimum movement
- **Vehicle loss check**: If `current_truck` becomes None → Cancel navigation, ERROR_RECOVERY

#### Arrival Detection:
- **Distance check**: Robot within `arrival_distance_threshold` (default: 0.15m)
- **Orientation check**: Robot orientation matches goal orientation within `arrival_orientation_tolerance`
- **Minimum movement check**: Robot must have moved at least `min_movement_before_arrival_check` meters
- **All checks pass** → Navigation complete

6. **State transition** → `NAVIGATING_TO_LICENSE_PLATE` → `CAPTURING_LICENSE_PLATE`

### Phase 5: License Plate Capture
1. **Camera positioning** → Robot adjusts position for optimal license plate view
2. **Image capture** → Photo taken of license plate
3. **Photo validation** → Image saved and path stored
4. **State transition** → `CAPTURING_LICENSE_PLATE` → `DETECTING_TYRES`

### Phase 6: Tyre Detection
1. **Segmentation mode switch** → Camera switches to tyre detection mode
2. **YOLO detection** → Detects all tyres on vehicle
3. **3D positioning** → Each tyre's 3D position calculated
4. **Tyre identification** → Tyres identified (front-left, front-right, rear-left, rear-right)
5. **Completeness verification** → Checks if all expected tyres detected
6. **State transition** → `DETECTING_TYRES` → `NAVIGATING_TO_TYRE` (first tyre)

### Phase 7: Tyre Navigation and Capture (Repeated for Each Tyre)

For each tyre (typically 4 tyres per vehicle):

1. **Tyre goal calculation** → Calculate approach pose for current tyre
   - Robot-relative approach (works from any angle)
   - Optimal distance for photo (default: 1.0-1.5m)
2. **Goal validation** → Same checks as license plate navigation
3. **Navigation starts** → Goal sent to Nav2, direct navigation activated
4. **State**: `NAVIGATING_TO_TYRE`

#### During Tyre Navigation:
- Same monitoring as license plate navigation
- **Tyre centering**: Robot adjusts position to center tyre in camera frame
- **Vehicle loss check**: If vehicle lost → Cancel navigation, ERROR_RECOVERY

5. **Arrival detection** → Same checks as license plate (distance + orientation)
6. **State transition** → `NAVIGATING_TO_TYRE` → `CAPTURING_TYRE`
7. **Photo capture** → Tyre photo taken and saved
8. **Photo validation** → Image saved, tyre marked as photographed
9. **Next tyre** → `current_tyre_index` incremented, repeat from step 1

### Phase 8: Completion Check
1. **All tyres photographed?** → Check if `current_tyre_index >= len(current_truck.tyres)`
2. **State transition** → `NAVIGATING_TO_TYRE` → `CHECKING_COMPLETION`
3. **Completeness verification** → Verify all tyres and license plate captured
4. **State transition** → `CHECKING_COMPLETION` → `FINISHING_TRUCK`

### Phase 9: Truck Completion
1. **Metadata save** → Save truck inspection data (photos, positions, timestamps)
2. **State reset** → Clear current_truck, reset tyre index
3. **Find next truck** → Check `detected_trucks` for unprocessed vehicles
4. **Decision**:
   - **More trucks?** → `FINISHING_TRUCK` → `TRUCK_DETECTED` (next vehicle)
   - **All done?** → `FINISHING_TRUCK` → `MISSION_COMPLETE`

### Phase 10: Mission Complete
1. **Final verification** → All vehicles inspected, all photos saved
2. **Mission statistics** → Log total vehicles, tyres, photos
3. **State**: `MISSION_COMPLETE` (final state, mission ends)

---

## Real-World Failure Handling

### Failure Mode 1: Vehicle Detection Lost During Navigation
- **Detection**: `current_truck` becomes None or not in `detected_trucks`
- **Action**: Cancel navigation, deactivate direct navigation, transition to ERROR_RECOVERY
- **Recovery**: Find next unprocessed vehicle or complete mission

### Failure Mode 2: Navigation Timeout
- **Detection**: Navigation takes > `navigation_timeout` (default: 60s)
- **Action**: Cancel Nav2 goal, deactivate direct navigation, reset nav_start_time
- **Recovery**: Transition to ERROR_RECOVERY or skip to next tyre/vehicle

### Failure Mode 3: Odometry Stale
- **Detection**: `/odom` not updated for > 2s
- **Action**: Log CRITICAL error, enter degraded mode (assume movement if cmd_vel active)
- **Recovery**: Continue navigation best-effort, operator notified

### Failure Mode 4: TF Transform Stale
- **Detection**: Transform older than 1s
- **Action**: Reject transform, return None from `_get_robot_pose()`
- **Recovery**: Mission controller handles None gracefully, may transition to ERROR_RECOVERY

### Failure Mode 5: Goal Calculation Error (Extreme Values)
- **Detection**: Goal coordinates > ±1000m or distance > 100m
- **Action**: Goal validation rejects, error logged
- **Recovery**: Retry goal calculation or transition to ERROR_RECOVERY

### Failure Mode 6: Nav2 Stops Publishing
- **Detection**: Nav2 cmd_vel not received for > 0.5s
- **Action**: Direct navigation fallback activates immediately
- **Recovery**: Robot continues navigation using direct control

---

## Critical System Guarantees

1. **Movement Guarantee**: Robot ALWAYS moves when it should (movement guarantee system)
2. **Goal Consistency**: Nav2 and direct navigation always use same goal
3. **Frame Consistency**: All components use `base_footprint` frame
4. **Data Validity**: Stale transforms/odometry rejected, extreme goals rejected
5. **State Consistency**: Navigation state reset on completion/timeout/failure
6. **Vehicle Tracking**: Vehicle loss detected immediately, navigation cancelled
7. **Robot-Relative Approach**: Works from ANY angle to vehicle (front, side, rear, diagonal)

---

## Real-World Execution Example

### Scenario: Robot Starts at Rear of Vehicle (180° angle)

1. **Robot position**: (-3m, 0m, 0°) relative to vehicle at origin
2. **Vehicle detected** → License plate at (0m, 0m)
3. **Goal calculation**:
   - `approach_direction = atan2(0, -3) = 180°` (from vehicle toward robot)
   - Goal: (0m, 0m) + 1.0m * (cos(180°), sin(180°)) = (-1.0m, 0m)
   - Goal orientation: 180° + π = 0° (robot faces vehicle)
4. **Navigation** → Robot moves from (-3m, 0m) to (-1.0m, 0m), rotating to face vehicle
5. **Arrival** → Robot at goal position AND facing vehicle (orientation check passes)
6. **License plate capture** → Photo taken
7. **Tyre detection** → All 4 tyres detected
8. **Tyre navigation** → Each tyre approached from robot's current relative position
9. **Mission complete** → All photos saved

**Result**: Robot successfully inspected vehicle from rear approach angle ✅

---

## System Robustness Features

1. **Bottom-up validation**: Hardware → cmd_vel → odom → goals → state machine → safety
2. **Multiple fallbacks**: Nav2 → Direct navigation → Movement guarantee
3. **Staleness detection**: TF transforms, odometry, vehicle tracking
4. **Bounds validation**: Goal positions, distances, coordinates
5. **State recovery**: ERROR_RECOVERY handles all failure modes
6. **Autonomous operation**: Works from any starting angle, handles any vehicle orientation

---

## Production-Ready Features

- ✅ Real-world navigation (no simulation, no placeholders)
- ✅ Deterministic behavior (all thresholds, timeouts, parameters are real values)
- ✅ Comprehensive error handling (all failure modes detected and handled)
- ✅ Autonomous operation (works from any angle, any starting position)
- ✅ Production-grade logging (all critical events logged with context)
- ✅ Verification scripts (all fixes have proof/verification)

---

This mission execution flow ensures the robot can autonomously inspect vehicles in a real-world yard environment, regardless of starting position or vehicle orientation, with robust error handling and recovery mechanisms.
