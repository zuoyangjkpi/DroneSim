# 🚁 AVIANS ROS2: Autonomous Drone Mission Planning & SLAM Integration

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-orange) ![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green) ![Python](https://img.shields.io/badge/Python-3.12-blue) ![License](https://img.shields.io/badge/License-Apache%202.0-yellow) ![PX4](https://img.shields.io/badge/PX4-v1.16-red)

## 📖 Project Overview

**AVIANS ROS2** is an advanced autonomous drone system that integrates **semantic SLAM**, **LLM-driven mission planning**, **YOLO-based detection**, **NMPC tracking**, and **PX4 hardware support** into a unified ROS2 Jazzy framework. The system provides complete autonomy from high-level intent ("inspect the building") to low-level execution in both simulation (Gazebo Harmonic) and real hardware (Pixhawk).

### 🎯 Key Capabilities

#### Core Features
- **🗺️ Semantic SLAM Integration**: ORB-SLAM3 + ESDF mapping + scene understanding
- **🧠 LLM Mission Planning**: Natural language to executable mission plans
- **📋 Mission Execution Engine**: BehaviorTree-based mission orchestration
- **🛸 Advanced Mission Modules**: Takeoff, orbit, search, inspection, tracking, landing
- **🔍 Real-time Detection**: YOLO v12 neural network for object detection
- **🎮 NMPC Controller**: Nonlinear model predictive control for precise tracking
- **📹 Stereo Vision**: Dual camera setup for depth perception and SLAM
- **🔗 Hardware Integration**: uXRCE-DDS bridge to Pixhawk 6X flight controllers

#### SLAM & Mapping
- **Visual-Inertial SLAM**: ORB-SLAM3 with stereo cameras + IMU fusion
- **Dense 3D Reconstruction**: ESDF (Euclidean Signed Distance Field) mapping
- **Collision Detection**: Real-time obstacle avoidance using dense maps
- **Scene Graph**: Semantic understanding of the environment
- **Loop Closure**: Drift-free long-term mapping

#### Simulation & Testing
- **🌍 SITL Environment**: Software-in-the-loop testing with Gazebo Harmonic
- **🏙️ Realistic Worlds**: City blocks, lakeside scenes, custom environments
- **📊 Live Visualization**: RViz2 for state, trajectories, and maps
- **🔄 Complete Control Chain**: From SLAM commands to Gazebo actuation

## 🖥️ System Requirements

### Hardware
- **CPU**: Intel i7 / AMD Ryzen 7 or better (for SLAM processing)
- **Memory**: Minimum 16 GB RAM (32 GB recommended for SLAM)
- **GPU**: Dedicated GPU with 4GB+ VRAM (NVIDIA recommended for SLAM)
- **Storage**: 30+ GB free space (includes SLAM models and datasets)

### Software
- **OS**: Ubuntu 24.04 LTS (Noble Numbat)
- **ROS**: ROS2 Jazzy Jalopy
- **Simulator**: Gazebo Harmonic
- **Python**: 3.12+
- **C++ Compiler**: GCC 11+ with C++17 support

## 🏗️ System Architecture

### High-Level Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                      GAZEBO SIMULATION                          │
│  • Stereo Camera (left/right)  • IMU Sensor  • Odometry        │
└────────────────┬────────────────────────────────────────────────┘
                 │
                 ▼
┌─────────────────────────────────────────────────────────────────┐
│                    SLAM_SIM_BRIDGE                              │
│  Protocol Adapter: Gazebo ←→ SLAM System                       │
│  • Publishes: /camera1, /camera2, /imu, /machine_1/pose        │
│  • Subscribes: /machine_1/command → waypoint commands          │
└────────────────┬────────────────────────────────────────────────┘
                 │
        ┌────────┴────────┐
        │                 │
        ▼                 ▼
┌──────────────┐   ┌────────────────────────────────────────────┐
│   SLAM       │   │     AVIANS CONTROL STACK                   │
│   SYSTEM     │   │  • Waypoint Controller (PID)               │
│              │   │  • Velocity Adapter (TwistStamped→Twist)   │
│ • ORB-SLAM3  │   │  • NMPC Tracker (Precision tracking)       │
│ • ESDF Map   │   │  • YOLO Detector (Object detection)        │
│ • Mission    │   │  • Mission Executor (BT orchestration)     │
│   Executor   │   │                                            │
└──────────────┘   └────────────────────────────────────────────┘
```

### Data Flow: SLAM Integration

```
Gazebo → /camera/left, /camera/right, /imu/data
           ↓
      slam_sim_bridge → /camera1, /camera2, /imu
           ↓
      ORB-SLAM3 (stereo-inertial SLAM)
           ↓
      ESDF Mapping (dense 3D reconstruction)
           ↓
      Mission Executor → /machine_1/command (waypoint)
           ↓
      slam_sim_bridge → /drone/control/waypoint_command
           ↓
      Waypoint Controller → Velocity Adapter → /X3/cmd_vel
           ↓
      Gazebo (drone actuation)
```

### Core Components

| Component | Description | Status |
|-----------|-------------|--------|
| **ORB-SLAM3** | Visual-inertial SLAM with loop closure | ✅ Integrated |
| **ESDF Mapping** | Dense 3D reconstruction + collision detection | ✅ Integrated |
| **Scene Graph** | Semantic scene understanding | ✅ Integrated |
| **slam_sim_bridge** | Protocol adapter Gazebo ↔ SLAM | ✅ Integrated |
| **Mission Executor** | BehaviorTree-based task orchestration | ✅ Stable |
| **YOLO Detector** | Real-time object detection | ✅ Stable |
| **NMPC Controller** | Precision trajectory tracking | ✅ Stable |
| **Waypoint Controller** | PID-based waypoint navigation | ✅ Stable |
| **PX4 Bridge** | Hardware integration (Pixhawk) | ✅ Stable |

## 📦 Project Structure

```
AVIANS_ROS2/
├── 📁 src/
│   ├── 🗺️ slam_sim_bridge/              # SLAM-Simulation protocol adapter
│   ├── 🤖 neural_network_detector/      # YOLO v12 object detection
│   ├── 🚁 drone_description/            # Drone models, worlds, stereo config
│   ├── 🎯 drone_nmpc_tracker/           # NMPC controller
│   ├── 🎮 drone_guidance_controllers/   # Waypoint & yaw controllers
│   ├── ⚙️ drone_low_level_controllers/  # Velocity control adapters
│   ├── 🧠 manual_mission_planner/       # LLM mission planner
│   ├── 📋 mission_executor/             # BehaviorTree executor
│   ├── 🛸 mission_action_modules/       # High-level action modules
│   ├── 🔗 px4_bridge/                   # PX4 hardware bridge
│   ├── 📨 custom_msgs/                  # Custom message definitions
│   │   ├── neural_network_msgs/         # Detection messages
│   │   ├── uav_msgs/                    # UAV pose/command messages
│   │   └── px4_msgs/                    # PX4 message subset
│   └── 🔧 ros2_utils/                   # Shared utilities
├── 📁 external/
│   ├── 🏙️ city_worlds_subt/            # Urban simulation worlds
│   └── 🌊 harmonic_demo/                # Lakeside world (Harmonic)
├── 📄 SITL.sh                           # Main SITL launcher
├── 📄 SITL_ARCHITECTURE.md              # Detailed architecture docs
├── 📄 comprehensive_test_suite.sh       # Test harness
└── 📖 README.md                         # This file

External SLAM Workspace (separate):
SLAM/semantic_slam_ws/
├── src/
│   ├── orbslam3_ros2/                   # ORB-SLAM3 ROS2 wrapper
│   ├── dense_tsdf_mapping/              # ESDF/TSDF dense mapping
│   ├── scene_graph/                     # Semantic scene graph
│   └── mission_executor/                # SLAM mission executor
└── slam_sitl.sh                         # SLAM system launcher
```

## 🚀 Installation

### Quick Start (Recommended)

```bash
# 1. Clone the repository
git clone https://github.com/zuoyangjkpi/AVIANS_ROS2.git
cd AVIANS_ROS2

# 2. Run automated installer
chmod +x .setup_avians_ros2_complete.sh
./.setup_avians_ros2_complete.sh

# 3. Reload environment
source ~/.bashrc

# 4. Validate installation
./test_avians_complete.sh
```

### SLAM System Setup (Separate Workspace)

The SLAM system lives in a separate workspace due to different dependency requirements:

```bash
# Navigate to SLAM workspace
cd ~/SLAM/semantic_slam_ws

# Install SLAM dependencies
sudo apt install -y \
  libopencv-dev \
  libeigen3-dev \
  libpcl-dev \
  libboost-all-dev

# Build SLAM workspace
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release

# Source workspace
source install/setup.bash
```

## 🎮 Running the System

### SITL Simulation (Software-in-the-Loop)

#### Option 1: Basic Simulation

```bash
cd ~/AVIANS_ROS2
./SITL.sh
```

#### Option 2: City World (Urban Environment)

```bash
./SITL.sh --city
```

#### Option 3: Lake World (Harmonic Demo)

```bash
./SITL.sh --lake
```

### What SITL.sh Launches

The SITL script automatically starts:
1. **Gazebo Simulation** - Physics engine with drone model
2. **ROS-Gazebo Bridge** - Message conversion (Gazebo ↔ ROS2)
3. **RViz2** - 3D visualization (stereo camera feeds, odometry, maps)
4. **Waypoint Controller** - PID waypoint navigation
5. **Velocity Adapter** - TwistStamped → Twist conversion
6. **SLAM Bridge** - Protocol adapter for SLAM system

### Integrated SLAM + SITL

For full SLAM integration, run both systems:

```bash
# Terminal 1: Start AVIANS SITL
cd ~/AVIANS_ROS2
./SITL.sh --lake

# Terminal 2: Start SLAM System
cd ~/SLAM
./slam_sitl.sh
```

### Data Flow Verification

After launching, verify topics:

```bash
# Check camera topics (from Gazebo)
ros2 topic echo /camera/left/image_raw --once
ros2 topic echo /camera/right/image_raw --once

# Check SLAM bridge output (to SLAM system)
ros2 topic echo /camera1 --once
ros2 topic echo /camera2 --once
ros2 topic echo /imu --once

# Check SLAM command output
ros2 topic echo /machine_1/command --once

# Check control chain
ros2 topic echo /drone/control/waypoint_command --once
ros2 topic echo /drone/control/velocity_setpoint --once
ros2 topic echo /X3/cmd_vel --once
```

## 🎯 SLAM Features

### 1. Visual-Inertial SLAM (ORB-SLAM3)

**Input Topics:**
- `/camera1` - Left camera (Image)
- `/camera2` - Right camera (Image)
- `/camera1/camera_info` - Left camera calibration
- `/camera2/camera_info` - Right camera calibration
- `/imu` - IMU data (angular velocity, linear acceleration)

**Output Topics:**
- Camera pose estimates
- Sparse 3D map points
- Loop closure detections

### 2. Dense Mapping (ESDF)

**Features:**
- Euclidean Signed Distance Field (ESDF) representation
- Collision detection for safe navigation
- Dense 3D reconstruction from sparse SLAM
- Voxel-based efficient storage

**Topics:**
- `/tsdf_map` - Truncated Signed Distance Field
- `/esdf_map` - Euclidean Signed Distance Field
- `/collision_map` - Binary collision grid

### 3. Mission Execution

**Topics:**
- `/machine_1/pose` - Current UAV pose (from slam_sim_bridge)
- `/machine_1/command` - Waypoint commands (from SLAM Mission Executor)

**Mission Types:**
- Takeoff → Hover → Waypoint Navigation → Land
- Search Area → Inspect Objects → Return Home
- Orbit Target → Track → Land

## 🔧 Configuration

### Stereo Camera Setup

Camera configuration in `drone_description/models/x3_stereo/`:

```xml
<camera name="left_camera">
  <horizontal_fov>1.047</horizontal_fov>
  <image>
    <width>640</width>
    <height>480</height>
  </image>
  <camera_info_topic>camera/left/camera_info</camera_info_topic>
</camera>
```

**Stereo Baseline:** 0.12m (configurable in SDF)

### SLAM Bridge Configuration

The `slam_sim_bridge` node adapts simulation state and commands, without
touching the camera topics (which are shared between real flight and SITL).

- Input (from Gazebo / SITL):
  - `/X3/odometry` (nav_msgs/Odometry)
  - `/imu/data` (sensor_msgs/Imu)
  - `/machine_1/command` (uav_msgs/UAVPose)
- Output (towards SLAM / controllers):
  - `/machine_1/pose` (uav_msgs/UAVPose)
  - `/imu` (sensor_msgs/Imu)
  - `/drone/control/waypoint_command` (geometry_msgs/PoseStamped)

If you change any of these topics in `src/slam_sim_bridge/src/bridge_node.cpp`,
rebuild the bridge:

```bash
colcon build --symlink-install --packages-select slam_sim_bridge
source install/setup.bash
```

The EuRoC-style remappings in the SLAM workspace (e.g. `/imu` → `/imu0`)
are handled entirely by the SLAM launch files; they are independent of
AVIANS_ROS2 and this SITL bridge.

## 🧪 Testing

### System Health Check

```bash
./comprehensive_test_suite.sh
# Select Option 1: System Status Check
```

### SLAM Integration Test

```bash
# 1. Launch SITL
./SITL.sh --lake

# 2. Verify SLAM bridge is publishing
ros2 topic hz /camera1
ros2 topic hz /camera2
ros2 topic hz /imu

# Expected: ~30 Hz for cameras, ~100 Hz for IMU
```

### Full Integration Test

```bash
# Terminal 1: SITL
./SITL.sh --lake

# Terminal 2: SLAM System
cd ~/SLAM
./slam_sitl.sh

# Terminal 3: Monitor topics
ros2 topic echo /machine_1/command
```

### Visualization

RViz2 displays:
- Stereo camera feeds (left/right)
- Drone odometry (ground truth)
- SLAM trajectory estimates
- ESDF voxel map
- Target detections (YOLO bounding boxes)

## 🐛 Troubleshooting

### Common Issues

#### 1. slam_sim_bridge shows wrong topics in log

**Symptom:**
```
/camera/left/image_raw → /cam0/image_raw  (WRONG!)
```

**Expected:**
```
/camera/left/image_raw → /camera1  (CORRECT!)
```

**Fix:** Bridge not recompiled after editing
```bash
cd ~/AVIANS_ROS2
colcon build --packages-select slam_sim_bridge --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
# Restart SITL.sh
```

#### 2. ORB-SLAM3 not receiving images

**Check topics:**
```bash
ros2 topic list | grep camera
# Should see: /camera1, /camera2 (from slam_sim_bridge)
```

**Check remapping:**
```bash
ros2 node info /multi_camera_slam
# Should show remapping: /camera1 -> /cam0/image_raw
```

#### 3. IMU data missing

**Verify Gazebo bridge:**
```bash
ros2 topic echo /imu/data --once
```

**Verify SLAM bridge:**
```bash
ros2 topic echo /imu --once
```

If `/imu/data` works but `/imu` doesn't, rebuild slam_sim_bridge.

#### 4. Gazebo simulation slow

**Reduce physics rate:**
Edit world SDF file:
```xml
<max_step_size>0.004</max_step_size>  <!-- 250 Hz instead of 1000 Hz -->
```

**Reduce camera resolution:**
```xml
<width>320</width>   <!-- Down from 640 -->
<height>240</height> <!-- Down from 480 -->
```

#### 5. SLAM initialization fails

**Symptoms:**
- ORB-SLAM3 stuck in "NOT INITIALIZED" state
- No map points visible

**Fixes:**
- Ensure sufficient visual texture in environment
- Move drone to create parallax
- Check camera calibration parameters
- Verify stereo baseline is correct

### Performance Optimization

**For SLAM performance:**
```bash
# Enable CPU performance mode
sudo cpupower frequency-set -g performance

# Increase ROS2 middleware buffers
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file:///path/to/cyclonedds.xml
```

**For GPU acceleration:**
```bash
# Check NVIDIA GPU
nvidia-smi

# Enable CUDA for ONNX Runtime (YOLO)
export ORT_TENSORRT_ENGINE_CACHE_ENABLE=1
```

## 📚 Documentation

- **[SITL_ARCHITECTURE.md](SITL_ARCHITECTURE.md)** - Detailed system architecture and data flow
- **[README_SIMULATION.md](README_SIMULATION.md)** - Simulation setup and configuration
- **[MISSION_PIPELINE_PLAN.md](MISSION_PIPELINE_PLAN.md)** - Mission planning workflow
- **[PX4_INTEGRATION_GUIDE.md](PX4_INTEGRATION_GUIDE.md)** - Hardware integration guide

## 🤝 Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/slam-improvement`)
3. Commit changes with clear messages
4. Submit a pull request

**Areas for contribution:**
- SLAM performance optimization
- Additional sensor fusion (LiDAR, depth cameras)
- Multi-drone SLAM collaboration
- Improved semantic scene understanding
- Hardware testing and validation

## 📄 License

This project is licensed under **Apache License 2.0**.

## 🙏 Acknowledgements

- **ROS2 Community** - Middleware and tooling
- **ORB-SLAM3** - Visual-inertial SLAM implementation
- **Ultralytics** - YOLO object detection
- **Gazebo Team** - High-fidelity simulation
- **PX4 Development Team** - Autopilot software

## 📞 Support

- **GitHub Issues**: [Report bugs or request features](https://github.com/zuoyangjkpi/AVIANS_ROS2/issues)
- **Discussions**: [Ask questions and share ideas](https://github.com/zuoyangjkpi/AVIANS_ROS2/discussions)
- **Documentation**: See [SITL_ARCHITECTURE.md](SITL_ARCHITECTURE.md) for technical details

## 🎯 Roadmap

### Current Version (v2.1.0)
- ✅ ORB-SLAM3 stereo-inertial integration
- ✅ ESDF dense mapping
- ✅ slam_sim_bridge protocol adapter
- ✅ Complete SITL environment
- ✅ Stereo camera support
- ✅ IMU fusion

### Upcoming Features
- 🔄 LiDAR sensor integration
- 🔄 Multi-drone SLAM collaboration
- 🔄 Semantic segmentation (Mask R-CNN)
- 🔄 Real-time trajectory optimization
- 🔄 Hardware flight tests (Pixhawk + RealSense)
- 🔄 Web-based monitoring dashboard

---

<div align="center">

**🚁 Built for Autonomous Aerial Intelligence ❤️**

*AVIANS: Autonomous Vision-based Intelligent Aerial Navigation System*

</div>
