#!/bin/bash

# SITL Launch Script for AVIANS_ROS2 + SLAM Integration
# Launches Gazebo simulation, Guidance Controller, RViz, and SLAM Bridge

# Parse arguments
WORLD_FILE="drone_world.sdf"
if [[ "$1" == "--city" ]]; then
    WORLD_FILE="city_drone_world.sdf"
    echo "🏙️  Using City urban world with stereo camera"
fi

# Source ROS2 Jazzy first
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
    source /opt/ros/jazzy/setup.bash
fi

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: install/setup.bash not found. Please build the workspace first."
    exit 1
fi

echo "🚀 Starting SITL Environment with $WORLD_FILE..."

# Get package paths
DRONE_DESC_PATH=$(ros2 pkg prefix drone_description --share)
WORLD_PATH="$DRONE_DESC_PATH/worlds/$WORLD_FILE"
RVIZ_CONFIG="$DRONE_DESC_PATH/config/drone.rviz"

# Check if world file exists
if [ ! -f "$WORLD_PATH" ]; then
    echo "❌ Error: World file not found: $WORLD_PATH"
    exit 1
fi

# Set Gazebo resource paths
export GZ_SIM_RESOURCE_PATH="$DRONE_DESC_PATH/models:$GZ_SIM_RESOURCE_PATH"

# 1. Launch Gazebo Simulation
echo "🎮 Starting Gazebo simulation..."
gz sim -v 4 -r "$WORLD_PATH" &
GZ_PID=$!
sleep 5 # Wait for Gazebo to start

# 2. Launch ROS-Gazebo Bridge
echo "🌉 Starting ROS-Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
    /camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
    /camera/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
    /camera/right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
    /X3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
    /X3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
    /X3/enable@std_msgs/msg/Bool@gz.msgs.Boolean \
    /world/city_multicopter/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V &
BRIDGE_PID=$!
sleep 2

# 3. Launch RViz2
echo "📊 Starting RViz2..."
if [ -f "$RVIZ_CONFIG" ]; then
    rviz2 -d "$RVIZ_CONFIG" &
else
    rviz2 &
fi
RVIZ_PID=$!

# 4. Launch Guidance Controller
echo "🎯 Starting Guidance Controller..."
ros2 run drone_description waypoint_controller &
CTRL_PID=$!
sleep 2

# 5. Launch SLAM Bridge
echo "🔗 Starting SLAM-Sim Bridge..."
ros2 run slam_sim_bridge bridge_node &
SLAM_BRIDGE_PID=$!

echo ""
echo "✅ SITL Environment Ready!"
echo "   - Gazebo: Running ($WORLD_FILE)"
echo "   - ROS-GZ Bridge: Running"
echo "   - RViz: Running"
echo "   - Controller: Running"
echo "   - SLAM Bridge: Running"
echo ""
echo "📷 Stereo Camera Topics:"
echo "   - /camera/left/image_raw"
echo "   - /camera/right/image_raw"
echo "   - /camera/left/camera_info"
echo "   - /camera/right/camera_info"
echo ""
echo "Press Ctrl+C to stop all processes."

# Cleanup function
cleanup() {
    echo ""
    echo "🛑 Stopping all processes..."
    kill $GZ_PID $BRIDGE_PID $RVIZ_PID $CTRL_PID $SLAM_BRIDGE_PID 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    exit
}

# Trap Ctrl+C
trap cleanup INT TERM

# Keep script running
wait
