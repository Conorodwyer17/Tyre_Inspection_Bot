---

##  Running the Vision Pipeline

After building the workspace and installing all vision dependencies, follow the steps below to launch the OAK-D-Lite camera, generate a point cloud, and run 3D perception.

---

### Build the Workspace

From the root of your workspace:

```bash
colcon build --symlink-install
source install/setup.bash
```

---

### Launch the OAK-D-Lite Camera

Start the camera driver and stereo depth pipeline:

```bash
ros2 launch ugv_vision oak_d_lite.launch.py
```

This launches:

* RGB + stereo cameras
* IMU
* Stereo depth images

---

###  Generate Depth-Only Point Cloud

In a **new terminal** (with the workspace sourced), start the depth-to-point-cloud node:

```bash
ros2 run depth_image_proc point_cloud_xyz_node \
  --ros-args \
  -p use_approximate_sync:=true \
  -p queue_size:=10 \
  -r image_rect:=/oak/stereo/image_raw \
  -r camera_info:=/oak/stereo/camera_info \
  -r points:=/points
```

This converts the stereo depth image into a `PointCloud2` topic without using RGB (recommended for Jetson).

---

###  Verify Topics

Confirm that the required topics are available and publishing:

```bash
ros2 topic list | grep -E "imu|points"
```

You should see at least:

```
/oak/imu/data
/points
```

Optionally, check data rates:

```bash
ros2 topic hz /points
ros2 topic hz /oak/imu/data
```

---

###  Run 3D Detection / Segmentation

Once the point cloud is publishing, launch the 3D perception pipeline:

```bash
ros2 launch segmentation_3d segment_3d.launch.py
```

This node consumes the `/points` topic and performs 3D segmentation / detection.

---

###  Notes

* The point cloud uses **Best Effort QoS**.
  When visualizing in RViz, set the PointCloud2 display QoS **Reliability → Best Effort**.
* Depth-only point clouds are preferred for:

  * Navigation
  * Obstacle avoidance
  * Jetson performance
* RGB-colored point clouds can be added later if needed.

---


