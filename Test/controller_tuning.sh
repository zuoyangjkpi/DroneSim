#!/bin/bash

# Controller Tuning Script
# Launches minimal simulation stack for controller tuning

# Change to DroneSim root directory (parent of Test/)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DRONESIM_ROOT="$(dirname "$SCRIPT_DIR")"
cd "$DRONESIM_ROOT" || exit 1

echo "🚀 Starting Controller Tuning Environment..."
echo "📂 DroneSim root: $DRONESIM_ROOT"
echo "📊 Plots will be saved to: $SCRIPT_DIR/"
echo "📝 Debug logs will be saved to: $SCRIPT_DIR/Debug/"

# Debug mode flag (default: false)
DEBUG=${DEBUG:-false}
LOG_DELAY=${LOG_DELAY:-12}
LOGGER_PID_FILE="/tmp/flight_debug_logger.pid"
LOGGER_SPAWNER_PID=""

# Cleanup function
cleanup() {
    echo ""
    echo "🛑 Stopping all processes..."
    if [ -f "$LOGGER_PID_FILE" ]; then
        LOGGER_PID=$(cat "$LOGGER_PID_FILE")
        kill "$LOGGER_PID" 2>/dev/null
        rm -f "$LOGGER_PID_FILE"
    fi
    kill $LOGGER_SPAWNER_PID 2>/dev/null
    kill $GZ_PID $YOLO_PID $TF_PID $RVIZ_PID $STATE_PUB_PID $TF_UAV_PID $WAYPOINT_PID $YAW_PID $CTRL_PID $MGR_PID 2>/dev/null
    pkill -f "gz sim" 2>/dev/null
    pkill -f "controller_tuning.py" 2>/dev/null
    pkill -f "flight_debug_logger.py" 2>/dev/null
    pkill -f "yolo12_detector_node" 2>/dev/null
    pkill -f "nmpc_tracker_node" 2>/dev/null
    exit
}

# Startup cleanup function (like standalone_test.sh)
startup_cleanup() {
    echo "🧹 Cleaning up any existing processes..."

    # Kill Gazebo processes
    pkill -f "gz sim" 2>/dev/null

    # Kill controller processes
    pkill -f "waypoint_controller" 2>/dev/null
    pkill -f "yaw_controller" 2>/dev/null
    pkill -f "controller_node" 2>/dev/null

    # Kill tuning-specific processes
    pkill -f "controller_tuning.py" 2>/dev/null
    pkill -f "controller_tuning_node" 2>/dev/null
    pkill -f "flight_debug_logger.py" 2>/dev/null

    # Kill other nodes that might be running
    pkill -f "yolo12_detector_node" 2>/dev/null
    pkill -f "nmpc_tracker_node" 2>/dev/null
    pkill -f "drone_tf_publisher" 2>/dev/null
    pkill -f "visualization.visualization_node" 2>/dev/null
    pkill -f "drone_state_publisher_node" 2>/dev/null
    pkill -f "tf_from_uav_pose_node" 2>/dev/null
    pkill -f "action_manager" 2>/dev/null
    pkill -f "ros2 topic pub" 2>/dev/null
    pkill -f "parameter_bridge" 2>/dev/null

    sleep 2

    # Force kill stubborn processes
    pkill -9 -f "controller_tuning.py" 2>/dev/null
    pkill -9 -f "flight_debug_logger.py" 2>/dev/null
    pkill -9 -f "waypoint_controller" 2>/dev/null
    pkill -9 -f "yaw_controller" 2>/dev/null
    pkill -9 -f "controller_node" 2>/dev/null

    sleep 1

    echo "✅ Cleanup complete"
    echo ""
}

# Trap Ctrl+C
trap cleanup INT TERM

# Perform startup cleanup before launching
startup_cleanup

# Source workspace
if [ -f "install/setup.bash" ]; then
    source install/setup.bash
else
    echo "Error: install/setup.bash not found. Please build the workspace first."
    exit 1
fi

# 1. Launch Gazebo
# 1. Launch Gazebo (Minimal Tuning Config)
echo "🎮 Starting Gazebo..."
ros2 launch drone_description gz_tuning.launch.py world_file:=$(pwd)/src/drone_description/worlds/drone_world.sdf &
GZ_PID=$!
sleep 10

# 2. Start YOLO (Disabled as per user request)
# echo "🧠 Starting YOLO..."
# ros2 run neural_network_detector yolo12_detector_node --ros-args -p use_sim_time:=false -p model_path:=$(find . -name "yolo12n.onnx" | head -1) -p labels_path:=$(find . -name "coco.names" | head -1) > /dev/null 2>&1 &
# YOLO_PID=$!
# sleep 1

# 3. TF Publisher
echo "📡 Starting TF Publisher..."
python3 -m drone_tf_publisher.drone_tf_publisher > /dev/null 2>&1 &
TF_PID=$!
sleep 1

# 4. RViz
echo "📊 Starting RViz..."
python3 -c "from visualization.visualization_node import main; main()" > /dev/null 2>&1 &
RVIZ_PID=$!
sleep 2

# 5. State Publisher Bridge
echo "🌉 Starting State Publisher..."
ros2 run drone_state_publisher drone_state_publisher_node --ros-args -p use_sim_time:=False > /dev/null 2>&1 &
STATE_PUB_PID=$!

# 6. TF from UAV Pose
echo "📍 Starting TF from UAV Pose..."
ros2 run tf_from_uav_pose tf_from_uav_pose_node --ros-args -p use_sim_time:=False -p poseTopicName:="/machine_1/pose" -p machineFrameID:="machine_1" -p worldFrameID:="world" > /dev/null 2>&1 &
TF_UAV_PID=$!
sleep 2

# 7. Controllers
echo "⚙️  Starting Controllers..."
GUIDANCE_PARAMS="$PWD/src/drone_guidance_controllers/config/controllers.yaml"
VELOCITY_PARAMS="$PWD/src/drone_low_level_controllers/config/controllers.yaml"

ros2 run drone_guidance_controllers waypoint_controller --ros-args --params-file "$GUIDANCE_PARAMS" > /dev/null 2>&1 &
WAYPOINT_PID=$!
sleep 1

ros2 run drone_guidance_controllers yaw_controller --ros-args --params-file "$GUIDANCE_PARAMS" > /dev/null 2>&1 &
YAW_PID=$!
sleep 1

ros2 run drone_low_level_controllers controller_node --ros-args --params-file "$VELOCITY_PARAMS" --log-level ERROR &
CTRL_PID=$!
sleep 1

# 8. Action Manager (needed for enable?)
echo "🎬 Starting Action Manager..."
ros2 run mission_action_modules action_manager > /dev/null 2>&1 &
MGR_PID=$!
sleep 2

# 9. Start Tuning Interface
echo "🎹 Starting Tuning Interface..."
LOGGER_SCRIPT="$SCRIPT_DIR/Debug/flight_debug_logger.py"

# Only start logger if DEBUG=true
if [ "$DEBUG" = "true" ]; then
    if [ -f "$LOGGER_SCRIPT" ]; then
        echo "⏱️  Will start flight logger after ${LOG_DELAY}s (post-takeoff)..."
        (
            sleep "$LOG_DELAY"
            echo "📝 Starting flight debug logger..."
            python3 "$LOGGER_SCRIPT" > /dev/null 2>&1 &
            echo $! > "$LOGGER_PID_FILE"
            wait
        ) &
        LOGGER_SPAWNER_PID=$!
    else
        echo "⚠️  Logger script not found: $LOGGER_SCRIPT"
    fi
else
    echo "ℹ️  Debug mode disabled. Logger will not start. (Set DEBUG=true to enable)"
fi

python3 "$SCRIPT_DIR/controller_tuning.py"

# Cleanup after python script exits
cleanup
