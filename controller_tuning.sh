#!/usr/bin/env bash

# Simple controller tuning launcher for X3 + drone_world.sdf
# - Starts Gazebo with drone_world.sdf
# - Starts ROS-GZ bridge for /X3/odometry, /X3/cmd_vel, /X3/enable, /imu/data
# This environment is intended for low-level controller / cmd_vel step tests.

set -e

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# Source ROS 2
if [ -f "/opt/ros/jazzy/setup.bash" ]; then
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
fi

# Source this workspace
if [ -f "${WS_DIR}/install/setup.bash" ]; then
  # shellcheck disable=SC1091
  source "${WS_DIR}/install/setup.bash"
else
  echo "❌ Error: ${WS_DIR}/install/setup.bash not found. Build the workspace first (colcon build)."
  exit 1
fi

echo "🚀 Starting X3 controller tuning environment (drone_world.sdf)..."

# Resolve drone_description share path and world
DRONE_DESC_PATH="$(ros2 pkg prefix drone_description --share)"
WORLD_PATH="${DRONE_DESC_PATH}/worlds/drone_world.sdf"

if [ ! -f "${WORLD_PATH}" ]; then
  echo "❌ Error: World file not found: ${WORLD_PATH}"
  exit 1
fi

# Configure Gazebo resource and plugin paths (mirrored from SITL.sh)
BASE_RESOURCE_PATH="/usr/share/gz/gz-sim8:/opt/ros/jazzy/share:${GZ_SIM_RESOURCE_PATH}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="/opt/ros/jazzy/opt/gz_sim_vendor/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
export GZ_RENDER_ENGINE_PATH="/opt/ros/jazzy/opt/gz_rendering_vendor/lib/gz-rendering-8/engine-plugins:${GZ_RENDER_ENGINE_PATH}"
export GZ_GUI_PLUGIN_PATH="/opt/ros/jazzy/opt/gz_gui_vendor/lib/gz-gui-8/plugins:${GZ_GUI_PLUGIN_PATH}"
export GZ_SIM_RESOURCE_PATH="${BASE_RESOURCE_PATH}:${DRONE_DESC_PATH}/models"

# Prefer Bullet physics engine when available
BULLET_PLUGIN="/opt/ros/jazzy/opt/gz_physics_vendor/lib/gz-physics-7/engine-plugins/libgz-physics7-bullet-plugin.so"
if [ -f "${BULLET_PLUGIN}" ]; then
  PHYSICS_ARG=(--physics-engine "${BULLET_PLUGIN}")
else
  PHYSICS_ARG=()
fi

echo "🎮 Starting Gazebo (drone_world.sdf)..."
gz sim -v 4 -r "${WORLD_PATH}" "${PHYSICS_ARG[@]}" &
GZ_PID=$!
sleep 5

echo "🌉 Starting ROS-GZ bridge (odometry, imu, cmd_vel, enable)..."
ros2 run ros_gz_bridge parameter_bridge \
  /X3/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry \
  /imu/data@sensor_msgs/msg/Imu@gz.msgs.IMU \
  /X3/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist \
  /X3/enable@std_msgs/msg/Bool@gz.msgs.Boolean &
BRIDGE_PID=$!
sleep 2

echo "🧪 Starting cmd_vel step tuning node..."
python3 "${WS_DIR}/Test/cmd_vel_step_tuning.py" &
TEST_PID=$!

echo ""
echo "✅ Controller tuning environment is up."
echo "   - Gazebo world: ${WORLD_PATH}"
echo "   - Bridge: /X3/odometry, /X3/cmd_vel, /X3/enable, /imu/data"
echo "   - Step tester: cmd_vel_step_tuning.py"
echo ""
echo "Press Ctrl+C here to stop Gazebo and the bridge."

cleanup() {
  echo ""
  echo "🛑 Stopping Gazebo and bridge..."
  kill "${GZ_PID}" "${BRIDGE_PID}" "${TEST_PID:-}" 2>/dev/null || true
  pkill -f "gz sim" 2>/dev/null || true
  echo "✅ Cleanup complete."
  exit 0
}

trap cleanup INT TERM

wait
