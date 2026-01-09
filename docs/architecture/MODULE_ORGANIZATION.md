# Module Organization - Tyre Inspection Mission

## Package Structure

```
tyre_inspection_mission/
├── tyre_inspection_mission/          # Main package module
│   ├── __init__.py                   # Package exports
│   │
│   ├── core/                         # Core mission logic
│   │   ├── mission_controller.py     # Main state machine orchestrator
│   │   └── state_handlers.py         # State-specific handlers (future)
│   │
│   ├── detection/                    # Detection modules
│   │   ├── vehicle_tracker.py        # Vehicle and tyre tracking
│   │   └── license_plate_detector.py # OCR-based license plate detection
│   │
│   ├── navigation/                   # Navigation modules
│   │   ├── goal_planner.py           # Goal calculation and validation
│   │   └── navigation_manager.py     # Nav2 interaction management
│   │
│   ├── capture/                      # Capture modules
│   │   └── photo_capture.py          # Photo capture service
│   │
│   ├── common/                       # Shared utilities
│   │   ├── data_structures.py       # VehicleData, TyreData, MissionState
│   │   ├── utils.py                 # Transformations, distance calculations
│   │   ├── config.py                 # Centralized configuration
│   │   ├── exceptions.py             # Custom exception classes
│   │   └── structured_logger.py      # Structured logging system
│   │
│   └── diagnostics/                 # Diagnostic tools (future)
│       └── health_monitor.py        # System health monitoring
│
├── launch/                           # Launch files
│   └── autonomous_inspection.launch.py
│
├── config/                           # Configuration files
│   └── (YAML configs if needed)
│
└── docs/                             # Documentation
    ├── troubleshooting/
    ├── architecture/
    ├── testing/
    └── api/
```

## Module Responsibilities

### Core Module (`core/`)
**Purpose:** Mission orchestration and state management

**Files:**
- `mission_controller.py`: Main state machine, coordinates all components
  - State transitions
  - Mission lifecycle management
  - Error recovery
  - System health monitoring

**Key Functions:**
- `state_machine_step()`: Main state machine loop
- `start_mission_callback()`: Mission start handler
- `stop_mission_callback()`: Mission stop handler
- `check_system_health()`: Health monitoring

**Dependencies:**
- All other modules
- ROS 2 services and actions
- TF2 transforms

---

### Detection Module (`detection/`)
**Purpose:** Object detection and tracking

**Files:**
- `vehicle_tracker.py`: Vehicle and tyre tracking
  - Vehicle detection processing
  - Tyre detection processing
  - LiDAR fusion
  - Position tracking

- `license_plate_detector.py`: License plate OCR
  - OCR-based detection
  - 3D position calculation
  - Text extraction

**Key Functions:**
- `process_vehicle_detections()`: Process vehicle bounding boxes
- `process_tyre_detections()`: Process tyre bounding boxes
- `process_lidar_vehicle_tracking()`: LiDAR-based tracking
- `detect_license_plate()`: OCR detection

**Dependencies:**
- `common/data_structures.py`
- `common/utils.py`
- `common/config.py`
- `common/exceptions.py`

---

### Navigation Module (`navigation/`)
**Purpose:** Navigation goal planning and execution

**Files:**
- `goal_planner.py`: Goal calculation
  - License plate waypoint calculation
  - Tyre waypoint calculation
  - Goal validation
  - Distance checks

- `navigation_manager.py`: Nav2 interaction
  - Goal sending
  - Feedback handling
  - Result processing
  - Goal cancellation

**Key Functions:**
- `calculate_license_plate_waypoint()`: Calculate license plate goal
- `calculate_tyre_waypoint()`: Calculate tyre goal
- `validate_goal_distance()`: Validate goal safety
- `send_navigation_goal()`: Send goal to Nav2

**Dependencies:**
- `common/data_structures.py`
- `common/utils.py`
- `common/config.py`
- `common/exceptions.py`
- Nav2 action server

---

### Capture Module (`capture/`)
**Purpose:** Photo capture and storage

**Files:**
- `photo_capture.py`: Photo capture service
  - Image capture
  - File storage
  - Metadata management

**Key Functions:**
- `capture_photo()`: Capture and save image
- `save_metadata()`: Save inspection metadata

**Dependencies:**
- Camera service
- File system

---

### Common Module (`common/`)
**Purpose:** Shared utilities and data structures

**Files:**
- `data_structures.py`: Shared data classes
  - `MissionState`: State machine states
  - `VehicleData`: Vehicle tracking data
  - `TyreData`: Tyre tracking data

- `utils.py`: Utility functions
  - TF transformations
  - Quaternion conversions
  - Distance calculations
  - Pose validation

- `config.py`: Configuration constants
  - Navigation config
  - Detection config
  - Camera config
  - Capture config
  - Logging config

- `exceptions.py`: Custom exceptions
  - `TyreInspectionError`: Base exception
  - `NavigationError`: Navigation failures
  - `DetectionError`: Detection failures
  - `CaptureError`: Capture failures
  - `PlanningError`: Planning failures
  - `TransformError`: TF transform failures
  - `ValidationError`: Validation failures
  - `StateMachineError`: State machine errors

- `structured_logger.py`: Structured logging
  - `StructuredLogger`: Contextual logging
  - `LogCategory`: Log categories

**Dependencies:**
- ROS 2 messages
- TF2
- Standard library

---

## Data Flow

### Detection Flow
```
YOLO Node → Segmentation Processor → mission_controller.bbox_callback()
    ↓
vehicle_tracker.process_vehicle_detections()
    ↓
VehicleData created/updated
    ↓
LiDAR fusion (if available)
    ↓
Position tracking
```

### Navigation Flow
```
State Machine → goal_planner.calculate_waypoint()
    ↓
Goal validation
    ↓
navigation_manager.send_goal()
    ↓
Nav2 action server
    ↓
Feedback/Result callbacks
    ↓
State transition
```

### Capture Flow
```
State Machine → capture_photo()
    ↓
Photo capture service
    ↓
File storage
    ↓
Metadata saved
    ↓
State transition
```

---

## Error Handling Strategy

### Exception Hierarchy
```
TyreInspectionError (base)
├── NavigationError
├── DetectionError
├── CaptureError
├── PlanningError
├── TransformError
├── ValidationError
└── StateMachineError
```

### Error Recovery
1. **Detection Errors**: Retry with fallback waypoints
2. **Navigation Errors**: Recalculate goal, retry with adjusted distance
3. **Capture Errors**: Retry up to max attempts
4. **Planning Errors**: Use fallback waypoint generation
5. **Transform Errors**: Fallback to alternative frames
6. **State Machine Errors**: Transition to ERROR_RECOVERY state

---

## Configuration Management

### Centralized Config (`config.py`)
All configuration constants are centralized in `config.py`:
- `NavigationConfig`: Navigation parameters
- `DetectionConfig`: Detection parameters
- `CameraConfig`: Camera parameters
- `CaptureConfig`: Capture parameters
- `PlanningConfig`: Planning parameters
- `LoggingConfig`: Logging parameters
- `TopicConfig`: Topic names
- `FrameConfig`: Frame names
- `TransformConfig`: Transform timeouts

### Parameter Overrides
ROS 2 parameters can override config defaults:
- `approach_distance`
- `navigation_timeout`
- `detection_timeout`
- `vehicle_class_names`
- `tyre_class_name`

---

## Logging Strategy

### Structured Logging
All logging uses `StructuredLogger` with categories:
- `[SYS]`: System events
- `[DET]`: Detection events
- `[NAV]`: Navigation events
- `[PLAN]`: Planning events
- `[CAP]`: Capture events
- `[ERR]`: Errors
- `[STATE]`: State transitions

### Log Levels
- `INFO`: Normal operation
- `WARN`: Recoverable issues
- `ERROR`: Failures requiring attention
- `DEBUG`: Detailed debugging (enabled via config)

---

## Testing Strategy

### Unit Tests (Future)
- Test individual functions in isolation
- Mock ROS 2 dependencies
- Test error handling

### Integration Tests (Future)
- Test module interactions
- Test state machine transitions
- Test error recovery

### System Tests
- Full mission execution
- Error scenario testing
- Performance testing

---

## Future Improvements

1. **Split State Handlers**: Extract state-specific logic to `state_handlers.py`
2. **Diagnostic Module**: Add `diagnostics/health_monitor.py` for system health
3. **Configuration Files**: Move config to YAML files for easier tuning
4. **Unit Tests**: Add comprehensive test coverage
5. **Documentation**: Add API documentation for all modules

---

**Last Updated:** Current Session  
**Maintained By:** Development Team
