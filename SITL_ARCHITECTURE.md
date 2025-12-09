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
│  │  Subscribe:                        Publish:                        │  │
│  │  • /X3/odometry                 →  /machine_1/pose (UAVPose)       │  │
│  │  • /camera/image_raw            →  /camera/image_raw (Image)       │  │
│  │  • /imu/data                    →  /imu (Imu)                      │  │
│  │                                                                     │  │
│  │  ⚠️  ISSUE: Single camera topic but Gazebo provides stereo!        │  │
│  │     Missing: /camera/left/image_raw, /camera/right/image_raw       │  │
│  └────────────────────────────────────────────────────────────────────┘  │
│                                                                           │
│  ┌────────────────────────────────────────────────────────────────────┐  │
│  │  COMMAND FORWARDING (SLAM → Gazebo)                               │  │
│  │                                                                     │  │
│  │  Subscribe:                        Publish:                        │  │
│  │  • /machine_1/command (UAVPose) →  /target_waypoint (PoseStamped)  │  │
│  │                                                                     │  │
│  │  ⚠️  ISSUE: Waypoint controller expects /drone/control/waypoint_command │  │
│  │     but slam_sim_bridge publishes to /target_waypoint              │  │
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
│  Input Topics:                                                            │
│  • /camera/left/image_raw     (from slam_sim_bridge)                     │
│  • /camera/right/image_raw    (from slam_sim_bridge)                     │
│  • /imu                       (from slam_sim_bridge)                     │
│  • /machine_1/pose            (from slam_sim_bridge)                     │
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
│                                                                           │
│  ⚠️  ISSUE: No connection between /drone/control/velocity_setpoint       │
│     and /X3/cmd_vel (Gazebo input)                                       │
└────────────┬─────────────────────────────────────────────────────────────┘
             │
             ▼
         Missing Controller Wrapper?
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
   - Publishes: `/X3/odometry`, `/imu` (if bridged)

3. **SLAM Sim Bridge** (`slam_sim_bridge/bridge_node`)
   - Subscribes to Gazebo topics
   - Converts to SLAM-expected format
   - Publishes sensor data for SLAM system

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
   - Publishes to: `/target_waypoint`

3. **⚠️ MISSING LINK**
   - Waypoint controller expects: `/drone/control/waypoint_command`
   - But slam_sim_bridge publishes: `/target_waypoint`
   - **Solution**: Add topic remapping or fix slam_sim_bridge

4. **Waypoint Controller**
   - PID controller: position error → velocity command
   - Publishes: `/drone/control/velocity_setpoint` (TwistStamped)

5. **⚠️ MISSING WRAPPER**
   - Gazebo expects: `/X3/cmd_vel` (Twist)
   - But controller publishes: `/drone/control/velocity_setpoint` (TwistStamped)
   - **Solution**: Add velocity_wrapper node or topic remapping

6. **ROS-Gazebo Bridge**
   - Forwards `/X3/cmd_vel` to Gazebo
   - Drone actuates based on velocity commands

---

## Critical Issues Identified

### Issue 1: Stereo Camera Topic Mismatch
**Problem**: `slam_sim_bridge` only subscribes to single camera topic `/camera/image_raw`, but Gazebo publishes stereo:
- `/camera/left/image_raw`
- `/camera/right/image_raw`

**Impact**: SLAM system won't receive stereo images

**Solution**:
```cpp
// In bridge_node.cpp, add:
left_camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/left/image_raw", 10,
    std::bind(&BridgeNode::leftCameraCallback, this, std::placeholders::_1));

right_camera_sub_ = create_subscription<sensor_msgs::msg::Image>(
    "/camera/right/image_raw", 10,
    std::bind(&BridgeNode::rightCameraCallback, this, std::placeholders::_1));
```

### Issue 2: Waypoint Topic Mismatch
**Problem**:
- `slam_sim_bridge` publishes: `/target_waypoint`
- `waypoint_controller` subscribes: `/drone/control/waypoint_command`

**Impact**: Controller won't receive SLAM waypoints

**Solution Options**:
1. **Option A**: Fix slam_sim_bridge parameter:
   ```cpp
   declare_parameter("sim_waypoint_topic", "/drone/control/waypoint_command");
   ```

2. **Option B**: Add topic remapping in SITL.sh:
   ```bash
   ros2 run slam_sim_bridge bridge_node \
       --ros-args -r /target_waypoint:=/drone/control/waypoint_command &
   ```

### Issue 3: Velocity Command Topic Mismatch
**Problem**:
- `waypoint_controller` publishes: `/drone/control/velocity_setpoint` (TwistStamped)
- Gazebo expects: `/X3/cmd_vel` (Twist)

**Impact**: Drone won't move based on controller commands

**Solution Options**:
1. **Option A**: Create velocity wrapper node:
   ```python
   # velocity_wrapper.py
   def callback(msg: TwistStamped):
       twist = Twist()
       twist.linear = msg.twist.linear
       twist.angular = msg.twist.angular
       pub.publish(twist)
   ```

2. **Option B**: Add topic remapping:
   ```bash
   ros2 run drone_guidance_controllers waypoint_controller \
       --ros-args -r /drone/control/velocity_setpoint:=/X3/cmd_vel &
   ```

### Issue 4: IMU Topic Not Bridged
**Problem**: SITL.sh doesn't bridge IMU data from Gazebo

**Solution**: Add to ROS-Gazebo bridge:
```bash
/imu@sensor_msgs/msg/Imu@gz.msgs.IMU \
```

---

## Correct Data Flow (After Fixes)

```
Gazebo Sensors
    │
    ├─→ /camera/left/image_raw ──┐
    ├─→ /camera/right/image_raw ─┼─→ slam_sim_bridge ─→ SLAM System
    ├─→ /camera/left/camera_info ┤                        │
    ├─→ /camera/right/camera_info┤                        │
    ├─→ /imu ────────────────────┘                        │
    └─→ /X3/odometry ──────────────────────────────────────┘
                                                           │
                                                           ▼
SLAM System ─→ /machine_1/command ─→ slam_sim_bridge ─→ /drone/control/waypoint_command
                                                           │
                                                           ▼
                                     waypoint_controller ─→ /X3/cmd_vel ─→ Gazebo
```

---

## Launch Sequence

### Current SITL.sh Launch Order:
1. **Gazebo Simulation** (`gz sim`)
2. **ROS-Gazebo Bridge** (`ros_gz_bridge parameter_bridge`)
3. **RViz2** (`rviz2`)
4. **Waypoint Controller** (`waypoint_controller`)
5. **SLAM Bridge** (`slam_sim_bridge`)

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

## Topic Reference Table

| Component | Input Topics | Output Topics | Message Type |
|-----------|-------------|---------------|--------------|
| **Gazebo** | `/X3/cmd_vel`<br>`/X3/enable` | `/camera/left/image_raw`<br>`/camera/right/image_raw`<br>`/X3/odometry`<br>`/imu` | Twist<br>Bool<br>Image<br>Image<br>Odometry<br>Imu |
| **ROS-GZ Bridge** | Gazebo → ROS | ROS → Gazebo | Protocol converter |
| **slam_sim_bridge** | `/X3/odometry`<br>`/camera/left/image_raw`<br>`/camera/right/image_raw`<br>`/imu`<br>`/machine_1/command` | `/machine_1/pose`<br>`/camera/left/image_raw`<br>`/camera/right/image_raw`<br>`/imu`<br>`/drone/control/waypoint_command` | Odometry→UAVPose<br>Image (pass-through)<br>Image (pass-through)<br>Imu (pass-through)<br>UAVPose→PoseStamped |
| **SLAM System** | `/camera/left/image_raw`<br>`/camera/right/image_raw`<br>`/imu`<br>`/machine_1/pose` | `/machine_1/command` | Image<br>Image<br>Imu<br>UAVPose<br>UAVPose |
| **waypoint_controller** | `/drone/control/waypoint_command`<br>`/X3/odometry`<br>`/drone/control/waypoint_enable` | `/X3/cmd_vel`<br>`/drone/control/waypoint_reached` | PoseStamped<br>Odometry<br>Bool<br>Twist<br>Bool |

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
- **Default**: `drone_world.sdf` (basic testing)
- **City**: `urban_circuit_01.sdf` (urban environment with obstacles)
- **Lake**: `harmonic.sdf` (lake house with indoor/outdoor scenes)

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

**Last Updated**: December 9, 2025
**Maintained By**: FRPG-AVIANS Team
**Related Docs**:
- [SLAM README](../SLAM/README.md)
- [AVIANS Controller Design](./docs/controller_design.md)
