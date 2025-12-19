# 🚁 DroneSim: Autonomous Drone Simulation & Control

![ROS2](https://img.shields.io/badge/ROS2-Jazzy-blue) ![Ubuntu](https://img.shields.io/badge/Ubuntu-24.04%20LTS-orange) ![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-green) ![PX4](https://img.shields.io/badge/PX4-v1.16-red)

## 📖 Overview

**DroneSim** integrates **SLAM**, **LLM mission planning**, **YOLO detection**, **NMPC tracking**, and **PX4 hardware support** into a unified ROS2 framework for autonomous drone systems.

### Key Features
- **🗺️ SLAM Integration**: ORB-SLAM3 stereo + ESDF mapping (separate workspace)
- **🧠 LLM Planning**: Natural language → executable mission YAML
- **🔍 YOLO Detection**: Real-time person detection (YOLOv12)
- **🎯 NMPC Tracking**: Nonlinear model predictive control
- **🔗 PX4 Bridge**: uXRCE-DDS to Pixhawk 6X

---

## 🚀 Quick Installation

```bash
# 1. Clone repository and navigate to it
cd DroneSim

# 2. Run automated installer (installs ROS2 Jazzy, dependencies, ONNX Runtime)
chmod +x setup_dronesim.sh
./setup_dronesim.sh

# 3. Reload environment
source ~/.bashrc
```

### Minimal Manual Install

```bash
# Install ROS2 Jazzy + Gazebo Harmonic
sudo apt install -y ros-jazzy-desktop ros-dev-tools ros-jazzy-ros-gz

# Install ROS dependencies
sudo apt install -y \
  python3-colcon-common-extensions \
  ros-jazzy-cv-bridge \
  ros-jazzy-image-transport \
  ros-jazzy-tf2-geometry-msgs \
  ros-jazzy-nav-msgs

# Build workspace
cd ~/DroneSim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### 🔑 LLM API Key Configuration

For LLM-based mission planning (option 1 in `standalone_test.sh`), configure your Aliyun DashScope API key:

```bash
# Navigate to the manual_mission_planner package
cd ~/DroneSim/src/manual_mission_planner/manual_mission_planner

# Create .env file from template
cp .env_example .env

# Edit .env and add your API key
nano .env
# Set: DASHSCOPE_API_KEY=sk-your-api-key-here
```

**Get your API key**: [Aliyun DashScope Console](https://dashscope.console.aliyun.com/apiKey)

---

## 🎮 Launch Scripts

| Script | Purpose | Usage |
|--------|---------|-------|
| `SITL.sh` | **Main SITL environment** | `./SITL.sh [--lake\|--factory\|--factory_rc]` |
| `controller_tuning.sh` | Controller step testing | `./controller_tuning.sh` |
| `standalone_test.sh` | Comprehensive test suite | `./standalone_test.sh` |
| `pixhawk_hitl_test.sh` | Hardware-in-loop testing | `./pixhawk_hitl_test.sh` |

### SITL.sh - Launched Nodes

```
./SITL.sh [--lake|--factory|--factory_rc]
```

| Node | Package | Topic I/O |
|------|---------|-----------|
| Gazebo Harmonic | gz_sim | Publishes sensor data |
| ROS-GZ Bridge | ros_gz_bridge | Gazebo ↔ ROS2 conversion |
| waypoint_controller | drone_description | `/drone/control/waypoint_command` → `/drone/control/velocity_setpoint` |
| velocity_adapter | drone_low_level_controllers | `/drone/control/velocity_setpoint` → `/X3/cmd_vel` |
| slam_sim_bridge | slam_sim_bridge | `/X3/odometry` → `/machine_1/pose`, `/machine_1/command` → waypoint |
| RViz2 | rviz2 | Visualization |

### World Options

| Flag | World File | Description |
|------|------------|-------------|
| (none) | `drone_world_stereo.sdf` | Default stereo test scene |
| `--lake` | `x3_lake_world.sdf` | Lakeside environment |
| `--factory` | `x3_factory_world.sdf` | Indoor warehouse |
| `--factory_rc` | `x3_factory_rc_world.sdf` | Warehouse with actors |

---

## 🏗️ Architecture

```
┌────────────────────────────────────────────────────────────────┐
│                    GAZEBO HARMONIC                              │
│  Sensors: Stereo Camera, IMU, Odometry                         │
└────────────────────────┬───────────────────────────────────────┘
                         │ ros_gz_bridge
                         ▼
┌────────────────────────────────────────────────────────────────┐
│                  slam_sim_bridge                                │
│  /X3/odometry → /machine_1/pose                                │
│  /imu/data → /imu                                              │
│  /machine_1/command → /drone/control/waypoint_command          │
└────────────────────────┬───────────────────────────────────────┘
                         │
         ┌───────────────┴───────────────┐
         ▼                               ▼
┌─────────────────┐           ┌──────────────────────────────────┐
│   SLAM System   │           │      Control Stack               │
│  (separate ws)  │           │  waypoint_controller             │
│  • ORB-SLAM3    │           │  velocity_adapter                │
│  • ESDF Mapping │           │  → /X3/cmd_vel → Gazebo          │
└─────────────────┘           └──────────────────────────────────┘
```

---

## 📦 Package Structure

```
DroneSim/
├── src/
│   ├── slam_sim_bridge/           # SLAM ↔ Simulation adapter
│   ├── neural_network_detector/   # YOLO v12 detection
│   ├── drone_description/         # Models, worlds, RViz configs
│   ├── drone_nmpc_tracker/        # NMPC controller
│   ├── drone_guidance_controllers/# Waypoint & yaw controllers
│   ├── drone_low_level_controllers/# Velocity adapters
│   ├── manual_mission_planner/    # LLM mission planning
│   ├── mission_executor/          # BehaviorTree executor
│   ├── mission_action_modules/    # Takeoff, orbit, track modules
│   ├── px4_bridge/                # PX4 hardware bridge
│   └── custom_msgs/               # uav_msgs, neural_network_msgs
├── external/                      # City worlds, harmonic demo
├── SITL.sh                        # Main SITL launcher
├── controller_tuning.sh           # Controller test environment
├── standalone_test.sh             # Comprehensive test suite
├── pixhawk_hitl_test.sh          # Pixhawk HITL testing
└── setup_dronesim.sh             # Installation script
```

---

## 📡 Topic Reference

### Sensor Topics (from Gazebo)
| Topic | Type | Source |
|-------|------|--------|
| `/camera/left/image_raw` | Image | Stereo camera |
| `/camera/right/image_raw` | Image | Stereo camera |
| `/camera/left/camera_info` | CameraInfo | Stereo camera |
| `/X3/odometry` | Odometry | Drone |
| `/imu/data` | Imu | IMU sensor |

### Control Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/drone/control/waypoint_command` | PoseStamped | Target waypoint |
| `/drone/control/velocity_setpoint` | TwistStamped | From waypoint controller |
| `/X3/cmd_vel` | Twist | To Gazebo |
| `/X3/enable` | Bool | Enable drone |

### SLAM Bridge Topics
| Topic | Type | Direction |
|-------|------|-----------|
| `/machine_1/pose` | UAVPose | → SLAM |
| `/imu` | Imu | → SLAM |
| `/machine_1/command` | UAVPose | ← SLAM |

---

## 🧪 Testing

### Verify SITL Environment
```bash
# Start SITL
./SITL.sh --lake

# Check topics
ros2 topic list | grep -E "camera|imu|odometry|cmd_vel"
ros2 topic hz /camera/left/image_raw   # Expect ~30 Hz
ros2 topic hz /X3/odometry              # Expect ~100 Hz
```

### Run Comprehensive Tests
```bash
./standalone_test.sh
# Options:
#   1) Text mission test (LLM → Mission Executor)
#   2) Full integration test
#   3) Waypoint controller test
#   4) Manual velocity control test
#   5) Kill all ROS processes
```

---

## 🔗 SLAM Integration

The SLAM system runs in a **separate workspace** (`~/SLAM/semantic_slam_ws`):

```bash
# Terminal 1: Start DroneSim SITL
cd ~/DroneSim && ./SITL.sh --lake

# Terminal 2: Start SLAM System
cd ~/SLAM && ./slam_sitl.sh
```

---

## 🐛 Troubleshooting

| Issue | Solution |
|-------|----------|
| Drone doesn't move | Check `/X3/cmd_vel` publishing: `ros2 topic hz /X3/cmd_vel` |
| Camera topics missing | Verify ros_gz_bridge is running |
| SLAM not receiving data | Rebuild slam_sim_bridge: `colcon build --packages-select slam_sim_bridge` |
| Gazebo slow | Reduce camera resolution in SDF or physics rate |

---

## 📚 Documentation

- **[SITL_ARCHITECTURE.md](SITL_ARCHITECTURE.md)** - Detailed system architecture and data flow

---

## 📄 License

Apache License 2.0

---

<div align="center">

**🚁 DroneSim: Autonomous Vision-based Intelligent Aerial Navigation System**

</div>
