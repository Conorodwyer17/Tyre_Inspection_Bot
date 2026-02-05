# Deployment Strategy: ROS 2 Version and Native vs Docker

This document summarizes the architecture review, reviews ROS 2 distribution and host OS choices, evaluates native install vs Docker for the onboard robot, and gives a concrete recommendation and plan.

---

## 1. Architecture Review Summary

The architecture document (`ARCHITECTURE.md`) has been reviewed against the repository and updated so that it:

- **Hardware**: Covers WaveShare six-wheel chassis, ESP32, onboard computer (Pi 5 or Jetson Orin), optional Coral, and SLAMTEC Aurora with specs and roles.
- **Software packages**: Lists ugv_nav, ugv_base_driver, segment_3d (ultralytics_node + segmentation_3d C++ node), gb_visual_detection_3d_msgs, segmentation_msgs, ugv_vision, and inspection_manager, with their roles and interfaces.
- **Data flow**: Describes Aurora → Nav2 → /cmd_vel → ugv_base_driver → ESP32, and Aurora image/point_cloud → ultralytics_node → ObjectsSegment → segmentation_3d → BoundingBoxes3d → inspection_manager.
- **TF and odometry**: Documents map → odom → base_link (and sensors), Aurora as source, localization-only mode.
- **Mission flow**: Notes that inspection_manager drives waypoints, uses 3D detections for approach/standoff, and triggers photo capture.

**Conclusion**: The architecture is complete and consistent with the codebase. The only dependency not present in this repo is the `inspection_manager` package, which the README places under `amr_simulation` or a separate repo; the doc states that explicitly.

---

## 2. ROS 2 Version and Ubuntu: Research Summary

### 2.1 SLAMTEC Aurora ROS 2 SDK

- **Official documentation**: The Aurora ROS 2 SDK is documented for **Ubuntu 20.04 and 22.04** and the “Get Started” tutorial links to the **ROS 2 Humble** workspace creation guide.
- **Implication**: The vendor-supported combination is **Ubuntu 22.04 + ROS 2 Humble**. There is no official statement or tutorial for **Ubuntu 24.04 or ROS 2 Jazzy**. The SDK uses standard `colcon build` and shared libraries; it may build and run on 24.04/Jazzy, but compatibility is not guaranteed and is untested in the docs.

### 2.2 ROS 2 Humble vs Jazzy

- **Humble Hawksbill**: LTS; supported until **May 2027**. Aligned with Ubuntu 22.04. Widely used in production.
- **Jazzy Jalisco**: LTS; supported until **May 2029**. Native on Ubuntu 24.04. Newer features and longer support window; Nav2 and core stacks support both.

### 2.3 Ubuntu 22.04 vs 24.04

- **22.04 LTS**: Standard security maintenance until mid-2027; extended options to 2032+ with Ubuntu Pro. Pairs with Humble.
- **24.04 LTS**: Current LTS; pairs with Jazzy. Longer support horizon for the OS and for Jazzy (2029).

### 2.4 Nav2 and Dependencies

- Nav2 is supported on both **Humble and Jazzy**. No blocker for either from the Nav2 side.
- Python (3.10+), Ultralytics, PyTorch, and the rest of the stack are available on both 22.04 and 24.04.

**Conclusion (version)**: For **maximum compatibility with the Aurora SDK as documented**, the safe choice is **Ubuntu 22.04 + ROS 2 Humble**. If you need Ubuntu 24.04 and longer support, **Ubuntu 24.04 + ROS 2 Jazzy** is viable for Nav2 and your packages, but Aurora SDK support on Jazzy is unofficial; you must verify the SDK builds and runs (and possibly fix API/dependency issues).

---

## 3. Native vs Docker: Research Summary

### 3.1 Performance and Latency

- **Native**: Direct access to serial (`/dev/ttyUSB0`), USB (Coral), and network (Aurora). No container overhead. DDS and ROS 2 nodes run on the host with minimal extra latency. This is the usual recommendation for **production robots that are “out driving around.”**
- **Docker**: Adds a layer between the host and hardware. Serial can be passed with `--device=/dev/ttyUSB0`, and network is shared by default, but:
  - There can be small additional latency and scheduling effects.
  - USB device assignment (e.g. Coral) and, on Jetson, GPU access require explicit configuration (NVIDIA Container Runtime, `--runtime nvidia`).
  - DDS discovery and multicast/unicast must work across host and container (or everything runs inside one container); misconfiguration can cause delays or missed messages.

### 3.2 When Docker Helps

- **Reproducible environment**: Same ROS distro and deps across dev machines and robot (e.g. Humble on 22.04 inside a container on a 24.04 host).
- **Isolation**: Avoid polluting the host; multiple projects or ROS versions on one machine.
- **CI and automation**: Build and test in a fixed image.

### 3.3 When Docker Is a Drawback for This Robot

- **Deployment on the robot**: For the actual robot driving and inspecting, native install typically gives the simplest and most predictable performance (serial, network, GPU/Coral). Docker is not required for “best performance” and can introduce failure modes (device passthrough, restarts, image updates).
- **Jetson**: GPU in Docker works with the NVIDIA runtime but requires matching JetPack/container base and correct env (e.g. for RViz or any GPU-accelerated node). Native avoids that complexity on the device.
- **Raspberry Pi**: Serial and USB Coral need to be passed into the container; native avoids passthrough and permission issues.

**Conclusion (Docker)**: For **best performance and simplicity when the robot is out driving**, run ROS 2 **natively** on the Pi or Jetson. Use **Docker only** if you need a reproducible Humble (or fixed) environment on a different host OS (e.g. 24.04 host, Humble in container) or for development/CI. If you do use Docker on the robot, document device passthrough (serial, USB, network) and, on Jetson, GPU runtime clearly.

---

## 4. Recommendation

### 4.1 Preferred: Native Install, Choose One of Two Stacks

**Option A – Maximum Aurora compatibility (recommended if you want zero vendor ambiguity)**  
- **OS**: Ubuntu 22.04 LTS on the Pi or Jetson.  
- **ROS 2**: Humble.  
- **Pros**: Matches SLAMTEC’s documented environment; Humble is well proven.  
- **Cons**: Shorter ROS support (to May 2027) than Jazzy.

**Option B – Newer stack and longer support**  
- **OS**: Ubuntu 24.04 LTS.  
- **ROS 2**: Jazzy.  
- **Pros**: Longer support (Jazzy to May 2029), current LTS OS.  
- **Cons**: Aurora SDK is not officially documented for 24.04/Jazzy; you must **test** the SDK (build, connect, topics, TF) and fix any API or dependency issues.

**Recommendation**: Prefer **Option A (22.04 + Humble)** unless you have a concrete reason to stay on 24.04 (e.g. other software only on 24.04) or are willing to validate and maintain Option B.

### 4.2 Docker: When and How

- **Do not use Docker on the robot** for the main deployment if the goal is best performance and simplicity.
- **Use Docker** for:
  - Development or CI with a fixed Humble (or Jazzy) environment.
  - Running Humble on a 24.04 host without downgrading the host OS.

If you adopt Docker for dev/CI or for a Humble-on-24.04 scenario, the plan below (Section 5) gives a minimal path.

---

## 5. Plan

### 5.1 If You Stay Native (Recommended for the Robot)

1. **Choose stack**  
   - Either: **Ubuntu 22.04 + ROS 2 Humble** (Option A).  
   - Or: **Ubuntu 24.04 + ROS 2 Jazzy** (Option B), with the caveat that Aurora SDK must be verified on Jazzy.

2. **Update project docs**  
   - In README and Robot_first_time_Installation.md: state the chosen OS and ROS distro (e.g. “Ubuntu 22.04 + ROS 2 Humble” or “Ubuntu 24.04 + ROS 2 Jazzy”).  
   - Replace any “Jazzy”/“24.04”-only wording with the chosen pair so the docs match the recommended path.

3. **If you choose Option A (22.04 + Humble)**  
   - Install Ubuntu 22.04 on the Pi or Jetson (or reflash).  
   - Install ROS 2 Humble and build the workspace with `colcon build`.  
   - Install Aurora SDK per SLAMTEC docs (Humble workspace).  
   - Keep all launch files and params; only distro names (e.g. `ros-humble-*` vs `ros-jazzy-*`) and install commands change.

4. **If you choose Option B (24.04 + Jazzy)**  
   - Keep current OS/ROS.  
   - Build and run the Aurora SDK in the same workspace; confirm connection, topics, and TF.  
   - If the SDK fails to build or run, either fix compatibility (deps, API) or switch to Option A.

5. **No Docker** on the robot for production deployment.

### 5.2 If You Add Docker (Dev/CI or Humble on 24.04)

1. **Purpose**  
   - Use only for development, CI, or to run Humble on a 24.04 host.  
   - Do not use as the primary deployment on the robot.

2. **Base image**  
   - For Humble: use official `ros:humble-ros-base-jammy` (or `-desktop` if you need GUI).  
   - For Jazzy: use `ros:jazzy-ros-base-noble` (or `-desktop`).  
   - For Jetson: use NVIDIA’s ROS 2 image for your JetPack version (e.g. from `nvcr.io` or `ros2_jetson`-style images) so GPU works.

3. **Device and network**  
   - Serial: `--device=/dev/ttyUSB0` (or appropriate tty).  
   - Network: host mode (`--network host`) is simplest for DDS and for reaching the Aurora (same host).  
   - Coral (Pi): pass the USB device or use privileged/device-cgroup rules if needed.  
   - Jetson: run with `--runtime nvidia` and the correct GPU-capable base image.

4. **Workspace**  
   - Mount the repo (or workspace) as a volume; run `colcon build` inside the container and source `install/setup.bash` before launching.

5. **Deliverables**  
   - Add a `Dockerfile` (and optional `docker-compose.yml`) for the chosen distro.  
   - Add a short “Docker” section in the README or in PROCESS.md: when to use it (dev/CI), how to build/run, and that **on-robot deployment is native**.

---

## 6. Summary Table

| Question | Answer |
|----------|--------|
| Best performance when the robot is driving? | **Native** install on Pi or Jetson. |
| Use Docker on the robot? | **No** for production; optional for dev/CI or Humble on 24.04 host. |
| ROS 2 Jazzy or Humble? | **Humble** for official Aurora SDK compatibility (22.04). **Jazzy** (24.04) if you accept testing Aurora yourself and want longer support. |
| Ubuntu 24 or 22? | **22.04** with Humble for Aurora; **24.04** with Jazzy for newer stack (verify Aurora). |
| Is the architecture complete? | **Yes**; ARCHITECTURE.md is aligned with the repo and includes messages, pipeline, and mission flow. |

---

## References

- SLAMTEC Aurora ROS2 SDK: https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/  
- ROS 2 Humble / Jazzy releases and support: https://docs.ros.org/en/jazzy/Releases.html  
- ROS 2 distribution end-of-life: https://endoflife.date/ros-2  
- Docker device passthrough (e.g. serial): `--device=/dev/ttyUSB0`; cgroup rules for dynamic USB.  
- NVIDIA Containers for Jetson: https://nvidia-ai-iot.github.io/ros2_jetson/ros2-jetson-dockers/
