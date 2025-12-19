# Drone NMPC Tracker

Nonlinear Model Predictive Control (NMPC) package for autonomous drone person tracking.

## Overview

This package implements a sophisticated NMPC controller for quadrotor drones to autonomously track and follow people. The controller uses real-time optimization to generate optimal control commands while considering drone dynamics, environmental constraints, and tracking objectives.

## Features

- **Advanced NMPC Controller**: Real-time nonlinear optimization for smooth tracking
- **Person Detection Integration**: Works with YOLO and other detection systems
- **ROS2 Jazzy Compatible**: Fully compatible with ROS2 Jazzy and Python 3.12
- **Visualization Support**: RViz integration for trajectory and target visualization
- **Configurable Parameters**: Extensive parameter tuning via YAML configuration
- **Safety Constraints**: Built-in safety limits and emergency behaviors
- **Test Framework**: Comprehensive testing utilities and simulation support

## System Requirements

- **OS**: Ubuntu 24.04 LTS
- **ROS2**: Jazzy Jalopy
- **Python**: 3.12+
- **Dependencies**: NumPy, SciPy, OpenCV (compatible versions)

## Installation

### 1. Clone to your ROS2 workspace

```bash
cd ~/your_ros2_workspace/src
# Copy the package files to: drone_nmpc_tracker/
```

### 2. Install Python dependencies

```bash
pip3 install numpy scipy matplotlib
```

### 3. Build the package

```bash
cd ~/your_ros2_workspace
colcon build --packages-select drone_nmpc_tracker
source install/setup.bash
```

## Package Structure

```
drone_nmpc_tracker/
├── drone_nmpc_tracker/
│   ├── __init__.py              # Package initialization
│   ├── config.py                # Configuration management
│   ├── nmpc_controller.py       # Core NMPC controller
│   ├── nmpc_node.py            # ROS2 node implementation
│   └── test_node.py            # Test utilities
├── config/
│   └── nmpc_params.yaml        # Parameter configuration
├── launch/
│   └── nmpc_tracker.launch.py  # Launch file
├── CMakeLists.txt              # Build configuration
├── package.xml                 # Package metadata
└── README.md                   # This file
```

## Usage

### Basic Usage

1. **Launch the NMPC tracker**:
```bash
ros2 launch drone_nmpc_tracker nmpc_tracker.launch.py
```

2. **Enable the controller**:
```bash
ros2 topic pub /nmpc/enable std_msgs/Bool "data: true"
```

3. **Monitor status**:
```bash
ros2 topic echo /nmpc/status
```

### With Your Existing System

The NMPC tracker integrates with your existing DroneSim system:

```bash
# Terminal 1: Launch your Gazebo simulation
./standalone_test.sh

# Terminal 2: Launch NMPC tracker
ros2 launch drone_nmpc_tracker nmpc_tracker.launch.py

# Terminal 3: Enable tracking
ros2 topic pub /nmpc/enable std_msgs/Bool "data: true"
```

### Testing with Simulated Data

For testing without real detection data:

```bash
# Terminal 1: Launch test node (provides simulated person detections)
ros2 run drone_nmpc_tracker nmpc_test_node

# Terminal 2: Launch NMPC tracker
ros2 run drone_nmpc_tracker nmpc_tracker_node

# Terminal 3: Monitor in RViz
rviz2
```
## Configuration

### Key Parameters

Edit `config/nmpc_params.yaml` to customize behavior:

```yaml
nmpc_tracker_node:
  ros__parameters:
    # Control frequency
    control_frequency: 10.0  # Hz
    
    # Tracking parameters
    tracking:
      optimal_distance: 4.0  # meters
      height_offset: 1.5     # meters above person
    
    # Weights (tune for your application)
    weights:
      position: [10.0, 10.0, 8.0]
      tracking_distance: 5.0
      camera_angle: 3.0
```

### Topic Configuration

| Topic | Type | Description |
|-------|------|-------------|
| `/X3/odometry` | `nav_msgs/Odometry` | Drone state input |
| `/target_detections` | `neural_network_msgs/NeuralNetworkDetectionArray` | Target detections |
| `/X3/cmd_vel` | `geometry_msgs/Twist` | Control output |
| `/nmpc/enable` | `std_msgs/Bool` | Enable/disable control |
| `/nmpc/status` | `std_msgs/Float64MultiArray` | Controller status |

## Algorithm Details

### NMPC Controller

The controller implements a discrete-time NMPC formulation:

1. **State Vector**: `[x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]`
2. **Control Vector**: `[thrust, roll_cmd, pitch_cmd, yaw_rate_cmd]`
3. **Prediction Horizon**: 8 steps (2 seconds at 0.25s timestep)
4. **Optimization**: Gradient descent with finite differences

### Cost Function

The cost function balances multiple objectives:

- **Position Tracking**: Follow optimal tracking position
- **Velocity Matching**: Match person's movement
- **Attitude Stability**: Maintain level flight
- **Control Effort**: Minimize aggressive maneuvers
- **Camera Constraints**: Keep person in camera view
- **Safety**: Maintain safe distance

### Quadrotor Dynamics

Simplified quadrotor model with:
- 6-DOF rigid body dynamics
- Thrust and attitude control inputs
- Wind disturbance modeling
- Physical constraints

## Troubleshooting

### Common Issues

1. **Controller not responding**:
   - Check if `/nmpc/enable` is set to `true`
   - Verify drone odometry is being received
   - Check parameter configuration

2. **Poor tracking performance**:
   - Tune cost function weights in config file
   - Adjust prediction horizon
   - Check person detection quality

3. **Compilation errors**:
   - Ensure all dependencies are installed
   - Check Python version (3.12+ required)
   - Verify ROS2 Jazzy installation

### Debug Mode

Enable debug logging:

```bash
ros2 run drone_nmpc_tracker nmpc_tracker_node --ros-args --log-level DEBUG
```

## Performance Tuning

### Optimization Parameters

- **Increase iterations** for better accuracy (higher CPU cost)
- **Decrease timestep** for smoother control (higher CPU cost)
- **Adjust weights** to prioritize different objectives
- **Tune constraints** for your specific drone platform

### Real-time Performance

- Target control frequency: 10-20 Hz
- Typical optimization time: 5-20ms
- Memory usage: ~50MB

## Integration with Rust

The package is designed to be easily convertible to Rust:

1. **Algorithm Structure**: Modular design with clear interfaces
2. **Data Types**: Uses standard numerical types
3. **No Python-specific Features**: Avoids Python-only constructs
4. **Documentation**: Comprehensive comments for translation

To convert to Rust:
1. Translate `nmpc_controller.py` using `nalgebra` for linear algebra
2. Use `r2r` or `rclrs` for ROS2 integration
3. Maintain the same algorithm structure and parameters

## Contributing

1. Fork the repository
2. Create a feature branch
3. Add tests for new functionality
4. Ensure all tests pass
5. Submit a pull request

## License

Apache-2.0 License - see LICENSE file for details.

## Support

For issues and questions:
1. Check the troubleshooting section
2. Review parameter configuration
3. Enable debug logging
4. Create an issue with detailed logs

## Acknowledgments

Based on the AirshipMPC project and adapted for quadrotor person tracking with ROS2 Jazzy compatibility.
