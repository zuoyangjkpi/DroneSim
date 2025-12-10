# AVIANS_ROS2 + SLAM SITL Architecture

Software-In-The-Loop (SITL) simulation architecture for integrating AVIANS autonomous control system with semantic SLAM.

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
│  Purpose: Protocol adapter between Gazebo and SLAM system                │
│                                                                           │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  SENSOR DATA FORWARDING (Gazebo → SLAM)                           │  │
│  │                                                                     │  │
│  │  Subscribe:                             Publish:                   │  │
│  │  • /X3/odometry                      →  /machine_1/pose (UAVPose)  │  │
│  │  • /camera/left/image_raw            →  /camera1 (Image)           │  │
│  │  • /camera/right/image_raw           →  /camera2 (Image)           │  │
│  │  • /camera/left/camera_info          →  /camera1/camera_info       │  │
│  │  • /camera/right/camera_info         →  /camera2/camera_info       │  │
│  │  • /imu/data                         →  /imu (Imu)                 │  │
│  │                                                                     │  │
│  │  ✅ Publishes to ORB-SLAM3 internal topics (before remapping)     │  │
│  └────────────────────────────────────────────────────────────────────┘  │
│                                                                           │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  COMMAND FORWARDING (SLAM → Controller)                           │  │
│  │                                                                     │  │
│  │  Subscribe:                             Publish:                   │  │
│  │  • /machine_1/command (UAVPose)      →  /drone/control/            │  │
│  │                                          waypoint_command           │  │
│  │                                          (PoseStamped)              │  │
│  │                                                                     │  │
│  │  ✅ FIXED: Correct waypoint topic for controller                   │  │
│  └────────────────────────────────────────────────────────────────────┘  │
└────────────┬─────────────────────────────────────────────────────────────┘
             │
             ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                         SLAM SYSTEM                                       │
│  (Semantic SLAM Workspace - separate system)                             │
│                                                                           │
│  Components:                                                              │
│  • ORB-SLAM3 (Visual-Inertial SLAM)                                       │
│  • ESDF Mapping (Dense 3D reconstruction)                                 │
│  • Mission Executor (BehaviorTree)                                        │
│  • Scene Graph (Semantic understanding)                                   │
│                                                                           │
│  Input Topics (ORB-SLAM3 internal, published by slam_sim_bridge):        │
│  • /camera1                   (Image - left camera)                      │
│  • /camera2                   (Image - right camera)                     │
│  • /camera1/camera_info       (CameraInfo)                               │
│  • /camera2/camera_info       (CameraInfo)                               │
│  • /imu                       (Imu)                                      │
│  • /machine_1/pose            (UAVPose)                                  │
│                                                                           │
│  Note: Launch file remaps /camera1→/cam0/image_raw for EuRoC compat     │
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
│                                                                           │
│  ✅ FIXED: Complete control chain to Gazebo                              │
└────────────┬─────────────────────────────────────────────────────────────┘
             │
             ▼
         /X3/cmd_vel → Gazebo

```

## Data Flow Summary

### Perception Pipeline (Gazebo → SLAM)

1. **Gazebo Sensors**
   - Stereo camera images (left & right)
   - IMU data (angular velocity, linear acceleration)
   - Ground truth odometry (for debugging)

2. **ROS-Gazebo Bridge**
   - Converts Gazebo messages to ROS2 topics
   - Publishes: `/camera/left/image_raw`, `/camera/right/image_raw`, `/camera/left/camera_info`, `/camera/right/camera_info`
   - Publishes: `/X3/odometry`, `/imu/data` (IMU sensor data)

3. **SLAM Sim Bridge** (`slam_sim_bridge/bridge_node`)
   - Subscribes to Gazebo stereo camera topics (left/right + camera_info)
   - Converts odometry to UAVPose format
   - Publishes to ORB-SLAM3 internal topics: `/camera1`, `/camera2`, `/imu`
   - SLAM launch file uses remapping to connect external ROS topics to these internal topics

4. **SLAM System**
   - ORB-SLAM3 processes stereo images + IMU
   - Builds sparse map and estimates pose
   - ESDF mapping creates dense 3D map
   - Mission executor plans waypoints

### Control Pipeline (SLAM → Gazebo)

1. **SLAM Mission Executor**
   - Publishes waypoint commands: `/machine_1/command` (UAVPose)

2. **SLAM Sim Bridge**
   - Converts UAVPose → PoseStamped
   - Publishes to: `/drone/control/waypoint_command`

3. **Waypoint Controller**
   - PID controller: position error → velocity command
   - Publishes: `/drone/control/velocity_setpoint` (TwistStamped)

4. **Velocity Control Adapter**
   - Strips timestamp from TwistStamped
   - Publishes: `/X3/cmd_vel` (Twist)

5. **ROS-Gazebo Bridge**
   - Forwards `/X3/cmd_vel` to Gazebo
   - Drone actuates based on velocity commands

---

## Critical Issues - STATUS

### ✅ Issue 1: Stereo Camera Topic Mismatch - RESOLVED
**Problem**: `slam_sim_bridge` only subscribed to single camera topic

**Solution Implemented**: Rewrote `bridge_node.cpp` with full stereo support:
- Subscribes: `/camera/left/image_raw`, `/camera/right/image_raw`
- Subscribes: `/camera/left/camera_info`, `/camera/right/camera_info`
- Publishes: `/camera1`, `/camera2` (ORB-SLAM3 internal topics, before remapping)
- Publishes: `/camera1/camera_info`, `/camera2/camera_info`
- SLAM launch file handles remapping: `/camera1` → `/cam0/image_raw`, `/camera2` → `/cam1/image_raw`

### ✅ Issue 2: Waypoint Topic Mismatch - RESOLVED
**Problem**: Topic name mismatch between SLAM bridge and controller

**Solution Implemented**: Fixed `sim_waypoint_topic` parameter in bridge_node.cpp:
```cpp
declare_parameter("sim_waypoint_topic", "/drone/control/waypoint_command");
```

### ✅ Issue 3: Velocity Command Topic Mismatch - RESOLVED
**Problem**: TwistStamped → Twist conversion needed for Gazebo

**Solution Implemented**: Added `multicopter_velocity_control_adapter` to SITL.sh:
- Subscribes: `/drone/control/velocity_setpoint` (TwistStamped)
- Publishes: `/X3/cmd_vel` (Twist)
- Includes velocity filtering and limiting

### ✅ Issue 4: IMU Topic Not Bridged - RESOLVED
**Problem**: SITL.sh didn't bridge IMU data from Gazebo

**Solution Implemented**:
1. Added IMU bridging to ROS-Gazebo bridge in SITL.sh:
   ```bash
   /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU \
   ```
2. slam_sim_bridge publishes to `/imu` (ORB-SLAM3 internal topic)
3. SLAM launch file handles remapping: `/imu` → `/imu0`

Now IMU data flows: Gazebo → `/imu/data` → slam_sim_bridge → `/imu` → SLAM System (remapped from `/imu0`)

---

## Correct Data Flow (After Fixes)

```
Gazebo Sensors
    │
    ├─→ /camera/left/image_raw ──┐
    ├─→ /camera/right/image_raw ─┤
    ├─→ /camera/left/camera_info ┼─→ slam_sim_bridge ─→ /camera1 ────────┐
    ├─→ /camera/right/camera_info┤                   ─→ /camera2 ────────┤
    ├─→ /imu/data ───────────────┤                   ─→ /imu ─────────────┤
    └─→ /X3/odometry ─────────────┘                   ─→ /machine_1/pose ─┘
                                                                           │
                                                                           ▼
                                    SLAM launch remaps: /camera1→/cam0/image_raw
                                                        /camera2→/cam1/image_raw
                                                        /imu→/imu0
                                                                           │
                                                                           ▼
                                                        SLAM System (ORB-SLAM3)
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
# Terminal 1: Start AVIANS SITL (Gazebo + Controllers)
cd ~/AVIANS_ROS2
./SITL.sh --lake

# Terminal 2: Start SLAM System
cd ~/SLAM
./slam_sitl.sh

# Note: slam_sitl.sh should be modified to NOT start its own simulation
# It should only start SLAM components that subscribe to topics from AVIANS SITL
```

---

## Understanding ROS2 Remapping in SLAM Integration

### How Remapping Works

ROS2 remapping allows nodes to use internal topic names while connecting to different external topic names. The syntax in launch files is:

```python
remappings=[
    ('/internal_topic', '/external_topic'),
]
```

- **Left side** (`/internal_topic`): Topic name used inside the node's code
- **Right side** (`/external_topic`): Actual topic name on the ROS network

### SLAM System Remapping Example

In [multi_camera_euroc.launch.py](../SLAM/semantic_slam_ws/src/orbslam3_ros2/launch/multi_camera_euroc.launch.py):

```python
remappings=[
    ('/camera1', '/cam0/image_raw'),  # EuRoC left camera
    ('/camera2', '/cam1/image_raw'),  # EuRoC right camera
    ('/imu', '/imu0'),                # EuRoC IMU
]
```

**This means:**
- ORB-SLAM3 internally subscribes to `/camera1`, `/camera2`, `/imu`
- ROS2 transparently redirects data from `/cam0/image_raw`, `/cam1/image_raw`, `/imu0`
- **slam_sim_bridge must publish to `/camera1`, `/camera2`, `/imu`** (the internal topic names)
- The launch file's remapping handles the connection to external EuRoC-named topics

### Why This Design?

This architecture allows ORB-SLAM3 to:
1. Use its own internal topic convention (`/camera1`, `/camera2`, `/imu`)
2. Be compatible with EuRoC dataset format (`/cam0/*`, `/cam1/*`, `/imu0`)
3. Work with our Gazebo simulation without modifying SLAM code

The remapping layer acts as an adapter between different naming conventions.

---

## Topic Reference Table

| Component | Input Topics | Output Topics | Message Type |
|-----------|-------------|---------------|--------------|
| **Gazebo** | `/X3/cmd_vel`<br>`/X3/enable` | `/camera/left/image_raw`<br>`/camera/right/image_raw`<br>`/camera/left/camera_info`<br>`/camera/right/camera_info`<br>`/X3/odometry`<br>`/imu/data` | Twist<br>Bool<br>Image<br>Image<br>CameraInfo<br>CameraInfo<br>Odometry<br>Imu |
| **ROS-GZ Bridge** | Gazebo → ROS | ROS → Gazebo | Protocol converter |
| **slam_sim_bridge** | `/X3/odometry`<br>`/camera/left/image_raw`<br>`/camera/right/image_raw`<br>`/camera/left/camera_info`<br>`/camera/right/camera_info`<br>`/imu/data`<br>`/machine_1/command` | `/machine_1/pose`<br>`/camera1`<br>`/camera2`<br>`/camera1/camera_info`<br>`/camera2/camera_info`<br>`/imu`<br>`/drone/control/waypoint_command` | Odometry→UAVPose<br>Image (ORB-SLAM3 internal)<br>Image (ORB-SLAM3 internal)<br>CameraInfo (ORB-SLAM3 internal)<br>CameraInfo (ORB-SLAM3 internal)<br>Imu (ORB-SLAM3 internal)<br>UAVPose→PoseStamped |
| **SLAM System** | `/camera1` (remapped from `/cam0/image_raw`)<br>`/camera2` (remapped from `/cam1/image_raw`)<br>`/camera1/camera_info`<br>`/camera2/camera_info`<br>`/imu` (remapped from `/imu0`)<br>`/machine_1/pose` | `/machine_1/command` | Image<br>Image<br>CameraInfo<br>CameraInfo<br>Imu<br>UAVPose<br>UAVPose |
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
- **Default**: `drone_world.sdf` (basic testing, flat ground)
- **City**: `x3_city_world.sdf` (local city block with buildings + solid collisions)
- **Lake**: `x3_lake_world.sdf` (local lakeside scene with ground + water patch collisions)

### Camera Configuration
- **Stereo Baseline**: Defined in `x3_stereo` model
- **Image Resolution**: Typically 640x480 or 1280x720
- **Frame Rate**: 30 Hz (configurable in SDF)

---

## Future Improvements

1. **Unified Launch File**: Create single launch file that coordinates both AVIANS and SLAM systems
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

**Last Updated**: December 9, 2025 (Updated with complete SITL integration)
**Maintained By**: FRPG-AVIANS Team
**Related Docs**:
- [SLAM README](../SLAM/README.md)
- [AVIANS Controller Design](./docs/controller_design.md)
