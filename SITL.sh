#!/bin/bash

# SITL Launch Script for AVIANS_ROS2 + SLAM Integration
# Launches Gazebo simulation, Guidance Controller, and SLAM Bridge

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: install/setup.bash not found. Please build the workspace first."
    exit 1
fi

echo "🚀 Starting SITL Environment..."

# 1. Launch Simulation (Gazebo + State Publisher)
echo "Starting Simulation..."
ros2 launch drone_state_publisher simulation.launch.py &
SIM_PID=$!
sleep 5 # Wait for Gazebo to start

# 2. Launch Guidance Controller
echo "Starting Guidance Controller..."
ros2 run drone_description waypoint_controller &
CTRL_PID=$!
sleep 2

# 3. Launch SLAM Bridge
echo "Starting SLAM-Sim Bridge..."
ros2 run slam_sim_bridge bridge_node &
BRIDGE_PID=$!

echo "✅ SITL Environment Ready!"
echo "   - Simulation: Running"
echo "   - Controller: Running"
echo "   - Bridge: Running (/machine_1/pose <-> /X3/odometry)"
echo "Press Ctrl+C to stop all processes."

# Cleanup function
cleanup() {
    echo "Stopping all processes..."
    kill $SIM_PID $CTRL_PID $BRIDGE_PID
    exit
}

# Trap Ctrl+C
trap cleanup INT

# Keep script running
wait
