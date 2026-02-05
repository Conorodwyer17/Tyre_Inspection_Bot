# Deployment: ROS 2 version and native vs Docker

Notes on what I looked at when choosing ROS 2 distro and OS, and whether to run natively or in Docker. Summary at the end.

---

## 1. Architecture check

I went through ARCHITECTURE.md against the repo and filled in the gaps: hardware (chassis, ESP32, Pi/Jetson, Coral, Aurora), packages (ugv_nav, ugv_base_driver, segment_3d, message packages, ugv_vision, inspection_manager), data flow, TF/odom, and mission flow. The only thing not in this repo is the inspection_manager package itself, which the README says can live under amr_simulation or elsewhere; the doc makes that clear. So the architecture write-up is complete and matches the codebase.

---

## 2. ROS 2 version and Ubuntu

### 2.1 SLAMTEC Aurora ROS 2 SDK

Their docs say Ubuntu 20.04 and 22.04, and the “Get Started” tutorial points at the **ROS 2 Humble** workspace guide. So the officially supported combo is **Ubuntu 22.04 + ROS 2 Humble**. There’s nothing in writing for 24.04 or Jazzy. The SDK is standard colcon + shared libs, so it might build and run on 24.04/Jazzy, but that’s not guaranteed and they don’t test it in the docs.

### 2.2 Humble vs Jazzy

- **Humble**: LTS to May 2027. Goes with 22.04. Well used in production.
- **Jazzy**: LTS to May 2029. The one that ships with 24.04. Newer features, longer support. Nav2 and the rest support both.

### 2.3 Ubuntu 22.04 vs 24.04

- 22.04: security updates to mid-2027; longer with Ubuntu Pro. Pairs with Humble.
- 24.04: current LTS, pairs with Jazzy. Longer runway for both OS and Jazzy (2029).

### 2.4 Nav2 and the rest

Nav2 works on both Humble and Jazzy. Python 3.10+, Ultralytics, PyTorch, etc. are available on both 22.04 and 24.04. No blocker either way from that side.

**Conclusion (version):** For the least hassle with the Aurora SDK as documented, I’d stick with **Ubuntu 22.04 + ROS 2 Humble**. If you want 24.04 and longer support, **24.04 + Jazzy** is fine for Nav2 and my packages, but you’re on your own with the Aurora SDK on Jazzy—you’ll need to try building and running it and fix any API/dependency niggles.

---

## 3. Native vs Docker

### 3.1 Performance and latency

**Native:** Direct access to serial (`/dev/ttyUSB0`), USB (Coral), and the network (Aurora). No container layer. DDS and ROS 2 run on the host with minimal extra latency. For a robot that’s actually driving around, that’s what I’d use.

**Docker:** Extra layer between host and hardware. You can pass serial with `--device=/dev/ttyUSB0` and share the network, but you get a bit of extra latency and scheduling. Coral and, on Jetson, the GPU need explicit setup (device passthrough, NVIDIA runtime). DDS discovery and multicast/unicast have to work across host and container (or you run everything in one container); if that’s wrong, you get delays or dropped messages.

### 3.2 When Docker is useful

- Same ROS distro and deps on every dev machine and the robot (e.g. Humble in a container on a 24.04 host).
- Keeping the host clean or running several projects/ROS versions.
- CI and automated builds in a fixed image.

### 3.3 When Docker is a pain for this robot

- On the robot itself: for driving and inspecting, native is simpler and more predictable (serial, network, GPU/Coral). Docker doesn’t make it faster and adds failure modes (devices, restarts, image updates).
- Jetson: GPU in Docker works with the NVIDIA runtime but you need the right JetPack/container base and env (e.g. for RViz or any GPU node). Native avoids that.
- Pi: serial and Coral need passing into the container; native avoids permissions and passthrough hassle.

**Conclusion (Docker):** For best performance and simplicity when the robot is out and about, run ROS 2 **natively** on the Pi or Jetson. I’d only use Docker if I needed a fixed Humble (or similar) environment on a different host OS (e.g. 24.04 host, Humble in a container) or for dev/CI. If you do run Docker on the robot, write down exactly how you pass serial, USB, and network, and on Jetson the GPU runtime.

---

## 4. What I’d do

### 4.1 Native, and pick one stack

**Option A – Easiest match with Aurora docs**  
- OS: Ubuntu 22.04 on the Pi or Jetson.  
- ROS 2: Humble.  
- Pros: Matches what SLAMTEC document; Humble is well proven.  
- Cons: ROS support ends May 2027, earlier than Jazzy.

**Option B – Newer stack, longer support**  
- OS: Ubuntu 24.04.  
- ROS 2: Jazzy.  
- Pros: Jazzy supported to May 2029, current LTS.  
- Cons: Aurora SDK isn’t officially documented for 24.04/Jazzy; you have to test build, connect, topics, TF and fix any issues.

I’d choose **Option A (22.04 + Humble)** unless I had a good reason to stay on 24.04 or was happy to validate and maintain Option B.

### 4.2 Docker: when and how

- Don’t use Docker on the robot for the main deployment if you care about performance and simplicity.
- Use Docker for: dev or CI with a fixed Humble/Jazzy environment, or to run Humble on a 24.04 host without downgrading the host.

If you do add Docker for that, the plan in section 5 below is a minimal path.

---

## 5. Plan

### 5.1 If you stay native (what I recommend on the robot)

1. **Pick a stack:** either 22.04 + Humble (Option A) or 24.04 + Jazzy (Option B), with the Aurora caveat for Jazzy.
2. **Update the docs:** in the README and Robot_first_time_Installation.md, say which OS and ROS distro you’re using (e.g. “Ubuntu 22.04 + ROS 2 Humble”) and make the install steps match.
3. **Option A:** Install 22.04 on the Pi or Jetson, install Humble, build the workspace with `colcon build`, install the Aurora SDK per SLAMTEC (Humble workspace). Launch files and params stay; only distro names (e.g. `ros-humble-*`) and install commands change.
4. **Option B:** Keep 24.04 and Jazzy, build and run the Aurora SDK in the same workspace, check connection, topics, and TF. If it breaks, either fix compat or switch to Option A.
5. No Docker on the robot for production.

### 5.2 If you add Docker (dev/CI or Humble on 24.04)

1. **Use it for:** dev, CI, or running Humble on a 24.04 host. Not as the main way to run the robot.
2. **Base image:** Humble → `ros:humble-ros-base-jammy` (or `-desktop` for a GUI). Jazzy → `ros:jazzy-ros-base-noble`. Jetson → NVIDIA’s ROS 2 image for your JetPack so the GPU works.
3. **Devices and network:** Serial → `--device=/dev/ttyUSB0` (or your tty). Network → `--network host` is simplest for DDS and reaching the Aurora. Coral on Pi → pass the USB device or use device-cgroup rules. Jetson → `--runtime nvidia` and a GPU-capable base image.
4. **Workspace:** Mount the repo (or workspace) as a volume, run `colcon build` inside the container, source `install/setup.bash` before launching.
5. **Docs:** Add a Dockerfile (and optionally docker-compose), and a short “Docker” bit in the README or PROCESS.md saying when you use it (dev/CI) and that on-robot deployment is native.

---

## 6. Quick answers

| Question | Answer |
|----------|--------|
| Best performance when the robot is driving? | Native on Pi or Jetson. |
| Use Docker on the robot? | No for production; optional for dev/CI or Humble on 24.04. |
| Jazzy or Humble? | Humble for official Aurora compatibility (22.04). Jazzy (24.04) if you’re happy testing Aurora yourself and want longer support. |
| Ubuntu 24 or 22? | 22.04 with Humble for Aurora; 24.04 with Jazzy for the newer stack (verify Aurora). |
| Is the architecture write-up complete? | Yes; ARCHITECTURE.md matches the repo and covers messages, pipeline, and mission flow. |

---

## References

- SLAMTEC Aurora ROS2 SDK: https://developer.slamtec.com/docs/slamware/aurora-ros2-sdk-en/  
- ROS 2 releases/support: https://docs.ros.org/en/jazzy/Releases.html  
- ROS 2 EOL: https://endoflife.date/ros-2  
- Docker device passthrough: `--device=/dev/ttyUSB0`; cgroup rules for dynamic USB.  
- NVIDIA containers for Jetson: https://nvidia-ai-iot.github.io/ros2_jetson/ros2-jetson-dockers/
