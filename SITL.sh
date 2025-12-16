#!/bin/bash

# SITL Launch Script for AVIANS_ROS2 + SLAM Integration
# Launches Gazebo simulation, Guidance Controller, RViz, and SLAM Bridge

# Parse arguments
WORLD_FILE="drone_world_stereo_sitl.sdf"
WORLD_MODE="default"

case "$1" in
  "--lake")
    WORLD_MODE="lake"
    echo "🌊 Using X3 lakeside world with stereo camera"
    ;;
  "--factory")
    WORLD_MODE="factory"
    echo "🏭 Using X3 factory world with stereo camera"
    ;;
  "--factory_rc")
    WORLD_MODE="factory_rc"
    echo "🏭 Using X3 factory_rc world with stereo camera"
    ;;
  "" )
    echo "Using default stereo test world"
    ;;
  *)
    echo "Unknown argument: $1"
    echo "Usage: $0 [--lake|--factory|--factory_rc]"
    ;;
esac

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

# Base resource paths: include system + ROS share (materials) + any existing env
BASE_RESOURCE_PATH="/usr/share/gz/gz-sim8:/opt/ros/jazzy/share:${GZ_SIM_RESOURCE_PATH}"
# Ensure system + rendering plugins are resolvable
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/opt/gz_sim_vendor/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_RENDER_ENGINE_PATH="/opt/ros/jazzy/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins:${GZ_RENDER_ENGINE_PATH}"
export GZ_GUI_PLUGIN_PATH="/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins:${GZ_GUI_PLUGIN_PATH}"

# Use stereo RViz config for all modes
case "$WORLD_MODE" in
  lake)
    WORLD_PATH="$DRONE_DESC_PATH/worlds/x3_lake_world_sitl.sdf"
    WORLD_TOPIC_NAME="Fortress"
    ;;
  factory)
    WORLD_PATH="$DRONE_DESC_PATH/worlds/x3_factory_world_sitl.sdf"
    WORLD_TOPIC_NAME="world_demo"
    ;;
  factory_rc)
    WORLD_PATH="$DRONE_DESC_PATH/worlds/x3_factory_rc_world_sitl.sdf"
    WORLD_TOPIC_NAME="world_demo"
    ;;
  *)
    WORLD_PATH="$DRONE_DESC_PATH/worlds/$WORLD_FILE"
    WORLD_TOPIC_NAME="multicopter"
    ;;
esac

RVIZ_CONFIG="$DRONE_DESC_PATH/config/drone_stereo.rviz"
export GZ_SIM_RESOURCE_PATH="$BASE_RESOURCE_PATH:$DRONE_DESC_PATH/models"

# Check if world file exists
if [ ! -f "$WORLD_PATH" ]; then
    echo "❌ Error: World file not found: $WORLD_PATH"
    exit 1
fi

# 1. Launch Gazebo Simulation
echo "🎮 Starting Gazebo simulation..."

# Prefer Bullet so mesh collisions (terrain, rocks) are supported; DART lacks MeshShapeFeature.
BULLET_PLUGIN="/opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins/libgz-physics7-bullet-plugin.so"
if [ -f "$BULLET_PLUGIN" ]; then
    PHYSICS_ARG=(--physics-engine "$BULLET_PLUGIN")
else
    PHYSICS_ARG=()
fi

gz sim -v 4 -r "$WORLD_PATH" "${PHYSICS_ARG[@]}" &
GZ_PID=$!
sleep 5 # Wait for Gazebo to start

# 2. Launch ROS-Gazebo Bridge
echo "🌉 Starting ROS-Gazebo bridge..."
ros2 run ros_gz_bridge parameter_bridge \
    /camera/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
    /camera/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
    /camera/left/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
    /camera/right/image_raw@sensor_msgs/msg/Image@gz.msgs.Image \
    /camera/left/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
    /camera/right/camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo \
    /X3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
    /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU \
    /X3/command/motor_speed@actuator_msgs/msg/Actuators@gz.msgs.Actuators \
    /X3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
    # /X3/enable@std_msgs/msg/Bool@gz.msgs.Boolean \
    /world/${WORLD_TOPIC_NAME}/dynamic_pose/info@geometry_msgs/msg/PoseArray@gz.msgs.Pose_V &
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
echo "   - Gazebo: Running ($(basename $WORLD_PATH))"
echo "   - ROS-GZ Bridge: Running"
echo "   - RViz: Running"
echo "   - Waypoint Controller: Running"
echo "   - Velocity Adapter: Running"
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
