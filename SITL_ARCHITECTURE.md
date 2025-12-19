# DroneSim + SLAM SITL Architecture

Software-In-The-Loop (SITL) simulation architecture for integrating DroneSim autonomous control system with semantic SLAM.

## System Overview

```
┌──────────────────────────────────────────────────────────────────────────┐
│                         GAZEBO SIMULATION                                 │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  X3 Quadrotor Model (x3_stereo)                                    │  │
│  │  • Stereo Camera (left + right)                                    │  │
│  │  • IMU Sensor                                                      │  │
│  │  • Dynamics & Physics                                              │  │
│  └────────┬───────────────────────────────────────────┬───────────────┘  │
│           │ Sensors                                   │ Commands          │
│           ▼                                           ▲                   │
│  ┌────────────────────────────────┐     ┌────────────────────────────┐  │
│  │ GZ Topics (Gazebo Protocol)    │     │ GZ Topics (Gazebo Protocol) │  │
│  │ • /camera/left/image           │     │ • /X3/cmd_vel              │  │
│  │ • /camera/right/image          │     │ • /X3/enable               │  │
│  │ • /X3/odometry                 │     └────────────────────────────┘  │
│  │ • /imu                         │                                      │
│  └────────┬───────────────────────┘                                      │
└───────────┼──────────────────────────────────────────────────────────────┘
            │
            ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                      ROS-GAZEBO BRIDGE                                    │
│  Converts Gazebo messages ←→ ROS2 messages                               │
│                                                                           │
│  Bridged Topics:                                                         │
│  • /camera/left/image_raw      (sensor_msgs/Image)                       │
│  • /camera/right/image_raw     (sensor_msgs/Image)                       │
│  • /camera/left/camera_info    (sensor_msgs/CameraInfo)                  │
│  • /camera/right/camera_info   (sensor_msgs/CameraInfo)                  │
│  • /X3/odometry                (nav_msgs/Odometry)                       │
│  • /imu/data                   (sensor_msgs/Imu)                         │
│  • /X3/cmd_vel                 (geometry_msgs/Twist)                     │
│  • /X3/enable                  (std_msgs/Bool)                           │
└────────────┬─────────────────────────────────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                      SLAM_SIM_BRIDGE NODE                                 │
│  Purpose: Protocol adapter between Gazebo/SITL and SLAM system           │
│                                                                           │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  SENSOR & STATE FORWARDING (Gazebo → SLAM)                       │  │
│  │                                                                     │  │
│  │  Subscribe:                             Publish:                   │  │
│  │  • /X3/odometry                      →  /machine_1/pose (UAVPose)  │  │
│  │  • /imu/data                         →  /imu (Imu)                 │  │
│  │                                                                     │  │
│  │  Note: Stereo camera topics                                         │  │
│  │        /camera/left/right/image_raw                                 │  │
│  │        /camera/left/right/camera_info                               │  │
│  │        are provided directly by                                     │  │
│  │        - hardware drivers (real flight)                             │  │
│  │        - ros_gz_bridge (SITL)                                       │  │
│  └────────────────────────────────────────────────────────────────────┘  │
│                                                                           │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  COMMAND FORWARDING (SLAM → Controller)                           │  │
│  │                                                                     │  │
│  │  Subscribe:                             Publish:                   │  │
│  │  • /machine_1/command (UAVPose)      →  /drone/control/            │  │
│  │                                          waypoint_command           │  │
│  │                                          (PoseStamped)              │  │
│  └────────────────────────────────────────────────────────────────────┘  │
└────────────┬─────────────────────────────────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                         SLAM SYSTEM                                       │
│  (Semantic SLAM Workspace - separate system)                             │
│                                                                           │
│  Components:                                                              │
│  • ORB-SLAM3 (Visual-Inertial SLAM; SITL uses stereo-only pipeline)      │
│  • ESDF Mapping (Dense 3D reconstruction)                                 │
│  • Mission Executor (BehaviorTree)                                        │
│  • Scene Graph (Semantic understanding)                                   │
│                                                                           │
│  External Input Topics (common to real flight + SITL):                   │
│  • /camera/left/image_raw      (Image - left camera)                     │
│  • /camera/right/image_raw     (Image - right camera)                    │
│  • /camera/left/camera_info    (CameraInfo)                              │
│  • /camera/right/camera_info   (CameraInfo)                              │
│                                                                           │
│  Additional Inputs (from slam_sim_bridge / other nodes):                 │
│  • /imu                       (Imu)                                      │
│  • /machine_1/pose            (UAVPose)                                  │
│                                                                           │
│  Output Topics:                                                           │
│  • /machine_1/command         (UAVPose - waypoint commands)              │
└────────────┬─────────────────────────────────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                    GUIDANCE CONTROLLER                                    │
│  (drone_guidance_controllers/waypoint_controller)                        │
│                                                                           │
│  Input Topics:                                                            │
│  • /drone/control/waypoint_command  (PoseStamped)                        │
│  • /X3/odometry                     (nav_msgs/Odometry)                  │
│  • /drone/control/waypoint_enable   (Bool)                               │
│                                                                           │
│  Output Topics:                                                           │
│  • /drone/control/velocity_setpoint  (TwistStamped)                      │
│  • /drone/control/waypoint_reached   (Bool)                              │
└────────────┬─────────────────────────────────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                VELOCITY CONTROL ADAPTER                                   │
│  (drone_low_level_controllers/multicopter_velocity_control_adapter)     │
│                                                                           │
│  Purpose: Converts TwistStamped to Twist for Gazebo                      │
│                                                                           │
│  Input Topics:                                                            │
│  • /drone/control/velocity_setpoint  (TwistStamped)                      │
│                                                                           │
│  Output Topics:                                                           │
│  • /X3/cmd_vel                       (Twist)                             │
└────────────┬─────────────────────────────────────────────────────────────┘
             │
             ▼
         /X3/cmd_vel → Gazebo

## Data Flow 

```
Gazebo Sensors
    │
    ├─→ /camera/left/image_raw
    ├─→ /camera/right/image_raw
    ├─→ /camera/left/camera_info
    ├─→ /camera/right/camera_info
    ├─→ /imu/data
    └─→ /X3/odometry
            │
            ▼
ROS-Gazebo Bridge (parameter_bridge)
    │
    ├─→ /camera/left/right/image_raw ────────────────┐
    ├─→ /camera/left/right/camera_info ─────────────┤
    │                                               │
    ├─→ /imu/data ────────────────┐                 │
    └─→ /X3/odometry ─────────────┼─→ slam_sim_bridge ─→ /machine_1/pose
                                  │                         │
                                  └────────→ slam_sim_bridge ─→ /imu
                                                            │
                                                            ▼
                                         SLAM System (ORB-SLAM3 + mapping)
                                                            │
                                                            ▼
SLAM System ─→ /machine_1/command ─→ slam_sim_bridge ─→ /drone/control/waypoint_command
                                                           │
                                                           ▼
                                               waypoint_controller
                                                           │
                                                           ▼
                                         /drone/control/velocity_setpoint
                                                           │
                                                           ▼
                                    multicopter_velocity_control_adapter
                                                           │
                                                           ▼
                                                    /X3/cmd_vel
                                                           │
                                                           ▼
                                                        Gazebo
```

---

## Launch Sequence

### Current SITL.sh Launch Order:
1. **Gazebo Simulation** (`gz sim`)
2. **ROS-Gazebo Bridge** (`ros_gz_bridge parameter_bridge`)
3. **RViz2** (`rviz2`)
4. **Waypoint Controller** (`waypoint_controller`)
5. **Velocity Control Adapter** (`multicopter_velocity_control_adapter`)
6. **SLAM Bridge** (`slam_sim_bridge`)

### Recommended Full Integration Launch:

```bash
# Terminal 1: Start DroneSim SITL (Gazebo + Controllers)
cd ~/DroneSim
./SITL.sh --lake

# Terminal 2: Start SLAM System
cd ~/SLAM
./slam_sitl.sh

# Note: slam_sitl.sh should be modified to NOT start its own simulation
# It should only start SLAM components that subscribe to topics from DroneSim SITL
```

---

## Topic Reference Table

| Component | Input Topics | Output Topics | Message Type |
|-----------|-------------|---------------|--------------|
| **Gazebo** | `/X3/cmd_vel`<br>`/X3/enable` | `/camera/left/image_raw`<br>`/camera/right/image_raw`<br>`/camera/left/camera_info`<br>`/camera/right/camera_info`<br>`/X3/odometry`<br>`/imu/data` | Twist<br>Bool<br>Image<br>Image<br>CameraInfo<br>CameraInfo<br>Odometry<br>Imu |
| **ROS-GZ Bridge** | Gazebo → ROS | ROS → Gazebo | Protocol converter |
| **slam_sim_bridge** | `/X3/odometry`<br>`/imu/data`<br>`/machine_1/command` | `/machine_1/pose`<br>`/imu`<br>`/drone/control/waypoint_command` | Odometry→UAVPose<br>Imu<br>UAVPose→PoseStamped |
| **SLAM System (stereo pipeline)** | `/camera/left/image_raw`<br>`/camera/right/image_raw`<br>`/camera/left/camera_info`<br>`/camera/right/camera_info`<br>`/imu` (optional, pipeline-dependent)<br>`/machine_1/pose` | `/machine_1/command` | Image<br>Image<br>CameraInfo<br>CameraInfo<br>Imu<br>UAVPose<br>UAVPose |
| **waypoint_controller** | `/drone/control/waypoint_command`<br>`/X3/odometry`<br>`/drone/control/waypoint_enable` | `/drone/control/velocity_setpoint`<br>`/drone/control/waypoint_reached` | PoseStamped<br>Odometry<br>Bool<br>TwistStamped<br>Bool |
| **velocity_adapter** | `/drone/control/velocity_setpoint` | `/X3/cmd_vel` | TwistStamped→Twist |

---

## Testing Procedure

### 1. Verify Gazebo Topics
```bash
gz topic -l | grep -E "camera|imu|odometry|cmd_vel"
```

### 2. Verify ROS2 Topics After Bridge
```bash
ros2 topic list | grep -E "camera|imu|odometry|cmd_vel|machine"
```

### 3. Check Topic Data Flow
```bash
# Check stereo cameras
ros2 topic hz /camera/left/image_raw
ros2 topic hz /camera/right/image_raw

# Check IMU
ros2 topic hz /imu

# Check odometry
ros2 topic hz /X3/odometry

# Check SLAM pose output
ros2 topic hz /machine_1/pose

# Check SLAM command output
ros2 topic hz /machine_1/command

# Check controller velocity output
ros2 topic hz /X3/cmd_vel
```

### 4. Visualize in RViz
- Stereo images: `/camera/left/image_raw`, `/camera/right/image_raw`
- SLAM trajectory: `/slam/trajectory`
- ESDF map: `/esdf_pointcloud`
- Current waypoint: `/drone/control/waypoint_command`

---

## Configuration Files

### SITL.sh Environment Variables
- `GZ_SIM_RESOURCE_PATH`: Model search paths
- `GZ_SIM_SYSTEM_PLUGIN_PATH`: Gazebo plugin paths
- `GZ_RENDER_ENGINE_PATH`: Rendering engine (ogre2)
- `GZ_GUI_PLUGIN_PATH`: GUI plugins

### World Files
- **Default**: `drone_world_stereo.sdf` (stereo testing scene with vegetation + obstacles)
- **Lake**: `x3_lake_world.sdf` (Fortress lakeside scene with water and mixed robots)
- **Factory**: `x3_factory_world.sdf` (indoor depot / warehouse scene with shelves and pallets)
- **Factory (RC)**: `x3_factory_rc_world.sdf` (MovAi depot / shelves scene with walking actors)

### Camera Configuration
- **Stereo Baseline**: Defined in `x3_stereo` model
- **Image Resolution**: Typically 640x480 or 1280x720
- **Frame Rate**: 30 Hz (configurable in SDF)

---

## Future Improvements

1. **Unified Launch File**: Create single launch file that coordinates both DroneSim and SLAM systems
2. **Dynamic Reconfigure**: Add parameters for PID gains, camera settings
3. **Multi-UAV Support**: Extend to multiple drones with unique namespaces
4. **Hardware-In-Loop**: Add support for real flight controller (PX4/ArduPilot)
5. **Mission Planning GUI**: Web interface for uploading mission YAML files

---

## Troubleshooting

### Drone doesn't move
- Check `/X3/cmd_vel` is being published: `ros2 topic hz /X3/cmd_vel`
- Check `/X3/enable` is true: `ros2 topic echo /X3/enable --once`
- Verify controller is active: `ros2 topic echo /drone/control/waypoint_enable --once`

### SLAM not receiving images
- Check camera topics exist: `ros2 topic list | grep camera`
- Verify slam_sim_bridge is running: `ros2 node list | grep bridge`
- Check topic remapping: `ros2 node info /slam_sim_bridge`

### Control loop not closing
- Verify all nodes in chain are running
- Check for topic mismatches with `ros2 topic info <topic_name>`
- Use `rqt_graph` to visualize node connections

---

## Minimal Build for SITL.sh

To run `SITL.sh` (with or without world arguments like `--lake`), the
following local packages must be built in this workspace:

```bash
cd ~/DroneSim
colcon build --symlink-install \
  --packages-select \
    custom_msgs \
    uav_msgs \
    neural_network_msgs \
    ros2_utils \
    drone_description \
    drone_low_level_controllers \
    slam_sim_bridge
source install/setup.bash
```

System/apt packages such as `ros-gz-bridge` and `rviz2` are expected to
be installed separately via your ROS 2 Jazzy installation.
