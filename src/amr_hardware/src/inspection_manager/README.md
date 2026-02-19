# Inspection Manager - Aurora Hardware Integration

Mission manager for autonomous vehicle tire inspection using **SLAMTEC Aurora**, Nav2, and 3D detections.

## Overview

The `inspection_manager` orchestrates the complete tire inspection mission:

1. **Search for vehicles** (cars/trucks) using Aurora mapping and detection
2. **Save vehicle positions** dynamically in map frame
3. **Navigate to each vehicle** using Nav2
4. **Detect tires** on each vehicle
5. **Navigate to each tire** and **capture photos**
6. **Complete inspection** of all vehicles

## Key Features

- **Dynamic Vehicle Detection:** Detects and saves vehicle positions from bounding boxes (no YAML file needed)
- **Supports Cars and Trucks:** Configurable vehicle labels (`car`, `truck`)
- **Tire Detection:** Uses inspection model (`best.pt`) for tire detection
- **Aurora Integration:** Uses Aurora map frame, point cloud, and images
- **Nav2 Integration:** Full navigation stack integration
- **Photo Capture:** Triggers photo capture at each tire location
- **Recovery Behaviors:** Rotation recovery if detection fails

## Differences from Simulation Version

This version has been adapted for Aurora hardware (not Gazebo):

- **Dynamic Detection:** Vehicles detected from bounding boxes, positions saved automatically
- **Aurora Topics:** Uses `/darknet_ros_3d/bounding_boxes` from segmentation_3d
- **Aurora Frames:** Uses `map` frame from Aurora SLAM
- **Car and Truck Support:** Detects both vehicle types
- **Tire Label:** Uses `tire` instead of `wheel`
- **No YAML Dependency:** By default, uses dynamic detection (YAML optional for legacy mode)

## Parameters

### Vehicle Detection

- **`vehicle_labels`** (string, default: `"car,truck"`):
  - Comma-separated list of vehicle class names to detect
  - Example: `"car,truck"` or `"car"` or `"truck"`

- **`tire_label`** (string, default: `"tire"`):
  - Tire class name for detection
  - Must match detection config (`interested_classes` in segment_3d)

- **`use_dynamic_detection`** (bool, default: `true`):
  - `true`: Detect vehicles from bounding boxes, save positions dynamically
  - `false`: Use YAML file with predefined vehicle positions (legacy mode)

- **`min_vehicle_probability`** (double, default: `0.5`):
  - Minimum detection probability for vehicles

- **`min_tire_probability`** (double, default: `0.35`):
  - Minimum detection probability for tires

### Navigation

- **`standoff_distance`** (double, default: `2.0`):
  - Distance [m] to stop before vehicle (not used in dynamic mode)

- **`approach_offset`** (double, default: `0.5`):
  - Offset [m] when approaching vehicle from detection box

- **`tire_offset`** (double, default: `0.4`):
  - Offset [m] when approaching tire from detection box

### Detection Topics

- **`detection_topic`** (string, default: `"/darknet_ros_3d/bounding_boxes"`):
  - Topic for `gb_visual_detection_3d_msgs/BoundingBoxes3d`
  - Should match segment_3d output topic

- **`segmentation_mode_topic`** (string, default: `"/segmentation_mode"`):
  - Topic to control YOLO model switching
  - Publishes `"navigation"` for vehicle detection, `"inspection"` for tire detection

### Frames

- **`world_frame`** (string, default: `"map"`):
  - Global frame (Aurora map frame)

- **`base_frame`** (string, default: `"base_link"`):
  - Robot base frame

### Recovery

- **`detection_timeout`** (double, default: `5.0`):
  - Seconds to wait before recovery rotation

- **`rotation_angle`** (double, default: `0.785`):
  - Rotation angle per recovery attempt (45 degrees)

- **`max_rotation_attempts`** (int, default: `8`):
  - Maximum rotation attempts (8 * 45° = 360°)

- **`tire_position_tolerance`** (double, default: `0.5`):
  - Meters - tires closer than this are considered the same

- **`max_tire_distance_from_vehicle`** (double, default: `5.0`):
  - Meters - tires beyond this distance are from other vehicles

## State Machine

### States

1. **`IDLE`**: Initial state, starts mission
2. **`SEARCH_VEHICLE`**: Search for vehicles (dynamic mode)
3. **`WAIT_VEHICLE_BOX`**: Wait for vehicle detection
4. **`TURN_IN_PLACE_VEHICLE`**: Rotate to find vehicle
5. **`APPROACH_VEHICLE`**: Navigate to detected vehicle
6. **`WAIT_TIRE_BOX`**: Wait for tire detection
7. **`TURN_IN_PLACE_TIRE`**: Rotate to find tires
8. **`INSPECT_TIRE`**: Navigate to tire and capture photo
9. **`NEXT_VEHICLE`**: Move to next vehicle
10. **`DONE`**: Mission complete

### State Transitions

```
IDLE → SEARCH_VEHICLE (dynamic) or NAV_TO_TRUCK_STANDOFF (legacy)
SEARCH_VEHICLE → WAIT_VEHICLE_BOX (when vehicles detected)
WAIT_VEHICLE_BOX → APPROACH_VEHICLE (when vehicle detected)
APPROACH_VEHICLE → WAIT_TIRE_BOX (after navigation)
WAIT_TIRE_BOX → INSPECT_TIRE (when tire detected)
INSPECT_TIRE → WAIT_TIRE_BOX (if more tires) or NEXT_VEHICLE (if done)
NEXT_VEHICLE → WAIT_VEHICLE_BOX (if more vehicles) or DONE
```

## Topics

### Subscribed

- **`detection_topic`** (`gb_visual_detection_3d_msgs/BoundingBoxes3d`):
  - 3D bounding boxes from segment_3d
  - Contains vehicles (`car`, `truck`) and tires (`tire`)

### Published

- **`inspection_state`** (`std_msgs/String`):
  - Current mission state for debugging

- **`segmentation_mode_topic`** (`std_msgs/String`):
  - Controls YOLO model switching
  - `"navigation"`: Use navigation model (yolov8m-seg.pt) for vehicles
  - `"inspection"`: Use inspection model (best.pt) for tires

- **`photo_capture_topic`** (`std_msgs/Bool`, default: `/inspection_manager/capture_photo`):
  - Triggers photo capture at tire locations

### Action Clients

- **`navigate_to_pose`** (`nav2_msgs/action/NavigateToPose`):
  - Nav2 action for navigation goals

## Usage

### Launch with Defaults (Dynamic Detection)

```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash

ros2 launch inspection_manager inspection_manager.launch.py
```

This will:
- Use dynamic vehicle detection (no YAML file)
- Detect both `car` and `truck`
- Use `tire` label for tire detection
- Subscribe to `/darknet_ros_3d/bounding_boxes`

### Customize Parameters

```bash
ros2 launch inspection_manager inspection_manager.launch.py \
  vehicle_labels:="car,truck" \
  tire_label:="tire" \
  detection_topic:="/darknet_ros_3d/bounding_boxes" \
  use_dynamic_detection:=true
```

### Legacy Mode (YAML File)

If you want to use a YAML file with predefined positions:

```bash
ros2 launch inspection_manager inspection_manager.launch.py \
  use_dynamic_detection:=false \
  trucks_file:=/path/to/vehicles.yaml
```

## Integration with Full Stack

### Required Components

1. **Aurora SDK**: Running and publishing map, odom, scan, point cloud, images
2. **Nav2**: Running with Aurora configuration
3. **Motor Driver**: ESP32 driver running
4. **Detection**: segment_3d running with YOLO models

### Full Launch Sequence

**Terminal 1 - Aurora:**
```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/slamware_ros_sdk/share/slamware_ros_sdk/local_setup.bash
ros2 launch ugv_nav aurora_bringup.launch.py ip_address:=192.168.11.1
```

**Terminal 2 - Motor Driver:**
```bash
cd ~/ugv_ws/src/Tyre_Inspection_Bot/src/amr_hardware/src/ugv_base_driver
./scripts/run_motor_driver_standalone.sh
```

**Terminal 3 - Navigation:**
```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/slamware_ros_sdk/share/slamware_ros_sdk/local_setup.bash
ros2 launch ugv_nav nav_aurora.launch.py use_rviz:=false aurora_ip:=192.168.11.1
```

**Terminal 4 - Detection:**
```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/slamware_ros_sdk/share/slamware_ros_sdk/local_setup.bash
# Ensure best.pt is in workspace root or specify path
ros2 launch segment_3d segment_3d.launch.py camera_rgb_topic:=/slamware_ros_sdk_server_node/left_image_raw
```

**Terminal 5 - Inspection Manager:**
```bash
source /opt/ros/humble/setup.bash
source ~/ugv_ws/install/setup.bash
ros2 launch inspection_manager inspection_manager.launch.py
```

## Mission Flow

1. **Start**: Robot begins in unknown location
2. **Search**: Robot searches for vehicles (car/truck) using detection
3. **Detect Vehicle**: When vehicle detected, position saved in map frame
4. **Navigate to Vehicle**: Nav2 navigates to vehicle position
5. **Switch to Tire Detection**: Segmentation mode switched to `inspection` (uses best.pt)
6. **Detect Tires**: Tires detected around vehicle
7. **Navigate to Each Tire**: Robot navigates to each tire position
8. **Capture Photo**: Photo captured at each tire location
9. **Next Vehicle**: Move to next detected vehicle
10. **Complete**: All vehicles inspected

## Debugging

### Check State

```bash
ros2 topic echo /inspection_state
```

### Check Detections

```bash
ros2 topic echo /darknet_ros_3d/bounding_boxes
```

### Check Segmentation Mode

```bash
ros2 topic echo /segmentation_mode
```

### Monitor Mission Progress

```bash
ros2 topic echo /inspection_state --once
ros2 node info /inspection_manager
```

## Configuration Files

- **Detection Config**: `segmentation_3d/config/config.yaml`
  - Ensure `interested_classes: ["person","truck","car","tire"]`
  - Ensure `point_cloud_topic: "/slamware_ros_sdk_server_node/point_cloud"`

- **Nav2 Config**: `ugv_nav/param/nav_aurora.yaml`
  - Uses Aurora topics and map frame

## Notes

- **YOLO Models**: 
  - Navigation model (`yolov8m-seg.pt`) downloaded automatically by ultralytics
  - Inspection model (`best.pt`) must be in workspace root or specified path
- **Photo Capture**: Currently publishes to `/inspection_manager/capture_photo` topic. You may need to implement a photo capture service that subscribes to this topic and saves images from Aurora camera topics.
- **Vehicle Positions**: Saved dynamically in `detected_vehicles` list. Positions are in Aurora `map` frame.
- **Tire Tracking**: Prevents inspecting the same tire twice using position tolerance.

## Troubleshooting

### No Vehicles Detected

- Check detection is running: `ros2 topic echo /darknet_ros_3d/bounding_boxes`
- Verify vehicle labels match: `vehicle_labels` parameter vs detection output
- Check minimum probability threshold: `min_vehicle_probability`

### No Tires Detected

- Verify segmentation mode switched to `inspection`: `ros2 topic echo /segmentation_mode`
- Check `best.pt` model is available
- Verify `tire_label` matches detection output
- Check minimum probability: `min_tire_probability`

### Navigation Fails

- Verify Nav2 is running: `ros2 node list | grep nav2`
- Check Aurora map is available: `ros2 topic echo /slamware_ros_sdk_server_node/map --once`
- Verify TF frames: `ros2 run tf2_ros tf2_echo map base_link`

---

**Adapted for Aurora Hardware - February 2026**
