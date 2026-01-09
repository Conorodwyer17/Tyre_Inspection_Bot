---

##  Dependencies


###  ROS 2 Packages (Humble)

Install required ROS packages using `apt`:

```bash
sudo apt update
sudo apt install -y \
  ros-humble-robot-localization \
  ros-humble-depth-image-proc
```

**Descriptions:**

* **`robot_localization`**
  Sensor fusion (EKF/UKF) for combining odometry, IMU, GPS, etc.
* **`depth_image_proc`**
  Converts depth images into point clouds (`PointCloud2`), used for stereo/depth cameras.

---

###  Python Dependencies

Install required Python libraries with `pip3`:

```bash
pip3 install --upgrade pip
pip3 install \
  ultralytics \
  ros2-numpy
```

**Descriptions:**

* **`ultralytics`**
  YOLO-based deep learning models for object detection and vision tasks.
* **`ros2-numpy`**
  Utilities for converting ROS 2 messages (images, point clouds) to NumPy arrays.

---

###  Verification (Optional)

You can verify the installations with:

```bash
ros2 pkg list | grep -E "robot_localization|depth_image_proc"
python3 -c "import ultralytics, ros2_numpy; print('Dependencies OK')"
```

---


