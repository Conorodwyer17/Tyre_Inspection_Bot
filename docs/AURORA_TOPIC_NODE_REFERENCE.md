# Aurora Topic & Node Reference

**Must launch first:** `ros2 launch ugv_nav aurora_testing.launch.py ip_address:=192.168.11.1`  
**Then** run test scripts or other nodes in a separate terminal.

---

## Publishers (Aurora stack when running)

### slamware_ros_sdk_server_node (slamware_ros_sdk)

| Topic | Type | QoS |
|-------|------|-----|
| `/slamware_ros_sdk_server_node/left_image_raw` | sensor_msgs/Image | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/right_image_raw` | sensor_msgs/Image | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/scan` | sensor_msgs/LaserScan | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/odom` | nav_msgs/Odometry | RELIABLE |
| `/slamware_ros_sdk_server_node/map` | nav_msgs/OccupancyGrid | RELIABLE, TRANSIENT_LOCAL |
| `/slamware_ros_sdk_server_node/map_metadata` | nav_msgs/MapMetaData | |
| `/slamware_ros_sdk_server_node/point_cloud` | sensor_msgs/PointCloud2 | BEST_EFFORT |
| `/slamware_ros_sdk_server_node/robot_pose` | geometry_msgs/PoseStamped | |
| `/slamware_ros_sdk_server_node/imu_raw_data` | sensor_msgs/Imu | |
| `/slamware_ros_sdk_server_node/state` | std_msgs/String | |
| `/slamware_ros_sdk_server_node/system_status` | slamware_ros_sdk/SystemStatus | |
| `/slamware_ros_sdk_server_node/stereo_keypoints` | sensor_msgs/Image | |

### aurora_sdk_bridge

| Topic | Type | QoS |
|-------|------|-----|
| `/camera/depth/image` | sensor_msgs/Image | BEST_EFFORT |
| `/camera/depth/camera_info` | sensor_msgs/CameraInfo | RELIABLE |
| `/camera/depth/points` | sensor_msgs/PointCloud2 | BEST_EFFORT |
| `/stereo/navigation_permitted` | std_msgs/Bool | |

---

## Subscribers (Aurora stack)

### aurora_sdk_bridge

| Subscribes to | Notes |
|---------------|-------|
| `/slamware_ros_sdk_server_node/left_image_raw` | Param: left_image_topic |
| `/slamware_ros_sdk_server_node/right_image_raw` | Param: right_image_topic |

---

## Downstream subscribers (our nodes)

| Node | Subscribes to |
|------|---------------|
| **ultralytics_node** | `/slamware_ros_sdk_server_node/left_image_raw` (param: camera_rgb_topic) |
| **depth_to_registered_pointcloud_node** | `/camera/depth/image`, `/camera/depth/camera_info` |
| **segmentation_processor_node** | ObjectsSegment, `/segmentation_processor/registered_pointcloud` |
| **Nav2** (nav_aurora.yaml) | `/slamware_ros_sdk_server_node/scan`, `/slamware_ros_sdk_server_node/odom`, `/camera/depth/points`, map |
| **inspection_manager** | `/darknet_ros_3d/bounding_boxes` |
| **photo_capture_service** | Left image, `/inspection_manager/capture_photo` |
| **aurora_health_monitor** | odom, scan, map, left, right (params) |
| **aurora_sensor_validate** | Same (params) |

---

## Pre-flight check

Before running test scripts, verify Aurora is publishing:

```bash
ros2 topic list | grep -E "left_image|scan|map|depth"
```

Expected (when Aurora stack is running):
- /camera/depth/image
- /camera/depth/points
- /slamware_ros_sdk_server_node/left_image_raw
- /slamware_ros_sdk_server_node/right_image_raw
- /slamware_ros_sdk_server_node/scan
- /slamware_ros_sdk_server_node/map
- ...

If you only see `/parameter_events` and `/rosout`, the Aurora stack is **not** running. Launch it first.
