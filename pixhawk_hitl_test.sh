#!/bin/bash

# AVIANS Pixhawk Hardware-in-the-Loop (HITL) Test Suite
# =========================================================
# This script drives the Pixhawk 6X hardware-in-the-loop test
# All low-level controllers stay on PX4 firmware; ROS2 handles high-level logic

echo "🚁 AVIANS Pixhawk HITL Test Suite"
echo "===================================="

# Color codes for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Function to print colored output
print_status() {
    local color=$1
    local message=$2
    echo -e "${color}${message}${NC}"
}

# Function to check if a process is running
check_process() {
    local process_name=$1
    if pgrep -x "$process_name" > /dev/null 2>&1; then
        return 0
    elif pgrep -f "$process_name" > /dev/null 2>&1; then
        local pid=$(pgrep -f "$process_name" | head -1)
        if [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null; then
            return 0
        fi
    fi
    return 1
}

# Function to wait for topic to be available
wait_for_topic() {
    local topic_name=$1
    local timeout=${2:-10}
    local count=0

    print_status $YELLOW "⏳ Waiting for topic: $topic_name"

    while [ $count -lt $timeout ]; do
        if ros2 topic list | grep -q "$topic_name"; then
            print_status $GREEN "✅ Topic $topic_name is available"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done

    print_status $RED "❌ Timeout waiting for topic: $topic_name"
    return 1
}

# Check PX4 connection
check_px4_connection() {
    print_status $BLUE "🔍 Checking PX4 Connection"
    echo "============================"

    # Check if Micro-XRCE-DDS agent is running
    if check_process "MicroXRCEAgent"; then
        print_status $GREEN "✅ Micro-XRCE-DDS Agent is running"
    else
        print_status $RED "❌ Micro-XRCE-DDS Agent not running"
        print_status $YELLOW "💡 Start with: MicroXRCEAgent serial --dev /dev/ttyUSB0 -b 921600"
        return 1
    fi

    # Check for PX4 topics
    print_status $YELLOW "🔍 Checking for PX4 topics..."
    local px4_topics=$(ros2 topic list | grep -c "fmu" || echo "0")

    if [ "$px4_topics" -gt 0 ]; then
        print_status $GREEN "✅ PX4 topics detected ($px4_topics topics)"
        ros2 topic list | grep "fmu" | head -10
    else
        print_status $RED "❌ No PX4 topics found"
        print_status $YELLOW "💡 Make sure:"
        print_status $YELLOW "   1. Pixhawk is connected via USB"
        print_status $YELLOW "   2. AVIANS firmware is flashed"
        print_status $YELLOW "   3. uXRCE-DDS is configured on TELEM2"
        return 1
    fi

    return 0
}

# Start Micro-XRCE-DDS Agent
start_xrce_agent() {
    print_status $BLUE "🚀 Starting Micro-XRCE-DDS Agent"
    echo "=================================="

    if check_process "MicroXRCEAgent"; then
        print_status $YELLOW "⚠️  Micro-XRCE-DDS Agent already running"
        read -p "Restart agent? (y/n): " restart_agent
        if [ "$restart_agent" = "y" ]; then
            pkill -f "MicroXRCEAgent"
            sleep 2
        else
            return 0
        fi
    fi

    # Detect serial port
    local serial_port=""
    if [ -e "/dev/ttyUSB0" ]; then
        serial_port="/dev/ttyUSB0"
    elif [ -e "/dev/ttyACM0" ]; then
        serial_port="/dev/ttyACM0"
    else
        print_status $RED "❌ No serial port found (tried /dev/ttyUSB0, /dev/ttyACM0)"
        read -p "Enter serial port manually: " serial_port
    fi

    print_status $YELLOW "🔌 Using serial port: $serial_port"
    print_status $YELLOW "📡 Starting agent with 921600 baud..."

    MicroXRCEAgent serial --dev $serial_port -b 921600 > /tmp/xrce_agent.log 2>&1 &
    local agent_pid=$!

    sleep 3

    if check_process "MicroXRCEAgent"; then
        print_status $GREEN "✅ Micro-XRCE-DDS Agent started successfully"
        print_status $YELLOW "📋 Check log: tail -f /tmp/xrce_agent.log"
        return 0
    else
        print_status $RED "❌ Failed to start Micro-XRCE-DDS Agent"
        return 1
    fi
}

# Full Pixhawk HITL Integration Test
pixhawk_hitl_full_test() {
    print_status $BLUE "🎯 Pixhawk HITL Full Integration Test"
    echo "========================================="

    print_status $YELLOW "🔄 Starting Pixhawk HITL system..."
    print_status $YELLOW "📋 Launch sequence:"
    print_status $YELLOW "   1. Gazebo simulation (visual environment)"
    print_status $YELLOW "   2. Micro-XRCE-DDS Agent (PX4-ROS2 bridge)"
    print_status $YELLOW "   3. PX4 Bridge (NMPC→PX4 commands)"
    print_status $YELLOW "   4. YOLO person detector"
    print_status $YELLOW "   5. Detection visualizer"
    print_status $YELLOW "   6. Drone TF publisher"
    print_status $YELLOW "   7. RViz visualization"
    print_status $YELLOW "   8. drone_state_publisher bridge"
    print_status $YELLOW "   9. tf_from_uav_pose node"
    print_status $YELLOW "   10. projection_model node"
    print_status $YELLOW "   11. pose_cov_ops_interface node"
    print_status $YELLOW "   12. NMPC tracker"
    print_status $YELLOW "   13. Enable tracking"
    print_status $YELLOW ""
    print_status $YELLOW "⚡ Low-level control runs on Pixhawk PX4!"

    # Clean up existing processes
    print_status $YELLOW "🧹 Cleaning up existing processes..."
    kill_all_hitl_processes
    sleep 3

    # Step 1: Launch Gazebo (for visual environment only)
    print_status $YELLOW "Step 1/13: Starting Gazebo simulation (visual only)..."
    export HOME="$HOME"
    export USER="$USER"
    export DISPLAY="${DISPLAY:-:0}"
    ros2 launch drone_description gz.launch.py > /tmp/gazebo_hitl.log 2>&1 &
    sleep 10

    if check_process "gz sim"; then
        print_status $GREEN "✅ Gazebo started"
    else
        print_status $RED "❌ Gazebo failed to start"
        return 1
    fi

    # Step 2: Start Micro-XRCE-DDS Agent
    print_status $YELLOW "Step 2/13: Starting Micro-XRCE-DDS Agent..."
    if ! start_xrce_agent; then
        print_status $RED "❌ Cannot continue without XRCE agent"
        return 1
    fi

    # Verify PX4 connection
    print_status $YELLOW "🔍 Verifying PX4 connection..."
    sleep 3
    if ! check_px4_connection; then
        print_status $RED "❌ PX4 connection failed"
        return 1
    fi

    # Step 3: Start PX4 Bridge
    print_status $YELLOW "Step 3/13: Starting PX4 Bridge (NMPC→PX4)..."
    ros2 run px4_bridge px4_bridge_node \
        --ros-args \
        -p use_sim_time:=False \
        > /tmp/px4_bridge.log 2>&1 &
    local px4_bridge_pid=$!
    sleep 3

    if check_process "px4_bridge_node"; then
        print_status $GREEN "✅ PX4 Bridge started successfully"
    else
        print_status $RED "❌ PX4 Bridge failed to start"
        return 1
    fi

    # Step 4: Start YOLO detector
    print_status $YELLOW "Step 4/13: Starting YOLO detector..."
    local model_path=$(find . -name "yolo12n.onnx" | head -1)
    local labels_path=$(find . -name "coco.names" | head -1)

    if [ -z "$model_path" ] || [ -z "$labels_path" ]; then
        print_status $RED "❌ YOLO model files not found"
        return 1
    fi

    ros2 run neural_network_detector yolo12_detector_node \
        --ros-args \
        -p "use_sim_time:=false" \
        -p "model_path:=$model_path" \
        -p "labels_path:=$labels_path" \
        -p "use_gpu:=false" \
        -p "confidence_threshold:=0.3" \
        -p "desired_class:=0" \
        -p "iou_threshold:=0.4" \
        -p "publish_debug_image:=true" \
        -p "max_update_rate_hz:=2.0" > /tmp/yolo_hitl.log 2>&1 &
    sleep 5

    if check_process "yolo12_detector_node"; then
        print_status $GREEN "✅ YOLO detector started"
    fi

    # Step 5: Start detection visualizer
    print_status $YELLOW "Step 5/13: Starting detection visualizer..."
    ros2 run neural_network_detector detection_visualizer_node > /tmp/detection_visualizer_hitl.log 2>&1 &
    sleep 2

    # Step 6: Start drone TF publisher
    print_status $YELLOW "Step 6/13: Starting drone TF publisher..."
    python3 drone_tf_publisher.py > /tmp/drone_tf_hitl.log 2>&1 &
    sleep 1

    # Step 7: Start RViz visualization
    print_status $YELLOW "Step 7/13: Starting RViz visualization..."
    python3 visualization_node.py > /tmp/visualization_hitl.log 2>&1 &
    sleep 2

    # Step 8: Start drone_state_publisher bridge (reads from PX4)
    print_status $YELLOW "Step 8/13: Starting drone_state_publisher bridge..."
    ros2 run drone_state_publisher drone_state_publisher_node \
        --ros-args \
        -p use_sim_time:=False \
        -p source_topic:="/fmu/out/vehicle_odometry" \
        > /tmp/drone_state_publisher_hitl.log 2>&1 &
    sleep 2

    if check_process "drone_state_publisher_node"; then
        print_status $GREEN "✅ drone_state_publisher bridge started"
    else
        print_status $RED "❌ drone_state_publisher bridge failed"
        return 1
    fi

    # Step 9: Start tf_from_uav_pose node
    print_status $YELLOW "Step 9/13: Starting tf_from_uav_pose node..."
    ros2 run tf_from_uav_pose tf_from_uav_pose_node \
        --ros-args \
        -p use_sim_time:=False \
        -p poseTopicName:="/machine_1/pose" \
        -p rawPoseTopicName:="/machine_1/pose/raw" \
        -p stdPoseTopicName:="/machine_1/pose/corr/std" \
        -p stdRawPoseTopicName:="/machine_1/pose/raww/std" \
        -p machineFrameID:="machine_1" \
        -p worldFrameID:="world" \
        -p cameraFrameID:="machine_1_camera_link" \
        -p cameraRGBOpticalFrameID:="machine_1_camera_rgb_optical_link" \
        -p dontPublishTFs:=False \
        -p cameraStaticPublish.publish:=True \
        -p cameraStaticPublish.topic:="/machine_1/camera/pose" \
        -p cameraStaticPublish.pose_optical_topic:="/machine_1/camera/pose_optical" \
        > /tmp/tf_from_uav_pose_hitl.log 2>&1 &
    sleep 3

    # Step 10: Start projection_model node
    print_status $YELLOW "Step 10/13: Starting projection_model node..."
    ros2 run projection_model projection_model_node \
        --ros-args \
        -p topics.robot:="/X3/pose_with_covariance" \
        -p topics.camera:="/machine_1/camera/pose" \
        -p topics.optical:="/machine_1/camera/pose_optical" \
        -p camera.info_topic:="/camera/camera_info" \
        -p projected_object_topic:="/person_detections/world_frame" \
        -p detections_topic:="/person_detections" \
        > /tmp/projection_model_hitl.log 2>&1 &
    sleep 3

    # Step 11: Start pose_cov_ops_interface node
    print_status $YELLOW "Step 11/13: Starting pose_cov_ops_interface node..."
    ros2 run pose_cov_ops_interface pose_cov_ops_interface_node \
        --ros-args \
        -p input_pose_topic:="/X3/odometry" \
        -p output_pose_topic:="/X3/pose_with_covariance" \
        -p default_covariance.matrix:="[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]" \
        > /tmp/pose_cov_ops_interface_hitl.log 2>&1 &
    sleep 3

    # Step 12: Start NMPC tracker
    print_status $YELLOW "Step 12/13: Starting NMPC tracker..."
    ros2 run drone_nmpc_tracker nmpc_tracker_node > /tmp/nmpc_tracker_hitl.log 2>&1 &
    sleep 3

    if check_process "nmpc_tracker_node"; then
        print_status $GREEN "✅ NMPC tracker started"
    else
        print_status $RED "❌ NMPC tracker failed to start"
        return 1
    fi

    # Step 13: Enable drone and tracking
    print_status $YELLOW "Step 13/13: Enabling drone control and tracking..."

    # Enable drone control (forwarded to PX4)
    print_status $YELLOW "  - Enabling drone control (PX4)..."
    ros2 topic pub -r 1 /X3/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &

    # Enable NMPC tracking
    print_status $YELLOW "  - Enabling NMPC tracking..."
    ros2 topic pub -r 1 /nmpc/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &

    sleep 5

    # System status
    print_status $GREEN "🎉 Pixhawk HITL system startup complete!"
    print_status $BLUE "========================================"
    print_status $YELLOW "📊 System architecture:"
    print_status $YELLOW "   ROS2 (High-level):"
    print_status $YELLOW "     - YOLO Detection"
    print_status $YELLOW "     - NMPC Trajectory Planning"
    print_status $YELLOW "     - Visualization"
    print_status $YELLOW ""
    print_status $YELLOW "   PX4 Pixhawk (Low-level):"
    print_status $YELLOW "     - Waypoint Controller"
    print_status $YELLOW "     - Velocity Controller"
    print_status $YELLOW "     - Attitude Controller"
    print_status $YELLOW "     - Motor Control Allocation"
    print_status $YELLOW ""
    print_status $YELLOW "💡 Expected behavior:"
    print_status $YELLOW "   - Pixhawk receives waypoint commands from NMPC"
    print_status $YELLOW "   - PX4 firmware executes low-level control"
    print_status $YELLOW "   - Gazebo shows visual representation"
    print_status $YELLOW "   - Check PX4 console: screen /dev/ttyUSB0 57600"
    print_status $YELLOW ""
    print_status $YELLOW "📋 Useful commands:"
    print_status $YELLOW "   - Monitor NMPC: tail -f /tmp/nmpc_tracker_hitl.log"
    print_status $YELLOW "   - Monitor PX4 Bridge: tail -f /tmp/px4_bridge.log"
    print_status $YELLOW "   - Monitor XRCE Agent: tail -f /tmp/xrce_agent.log"
    print_status $YELLOW "   - List PX4 topics: ros2 topic list | grep fmu"
}

# Kill all HITL processes
kill_all_hitl_processes() {
    print_status $BLUE "🧹 Killing All HITL Processes"
    echo "=============================="

    print_status $YELLOW "🛑 Stopping all processes..."

    pkill -f "gz sim" 2>/dev/null
    pkill -f "MicroXRCEAgent" 2>/dev/null
    pkill -f "px4_bridge_node" 2>/dev/null
    pkill -f "yolo12_detector_node" 2>/dev/null
    pkill -f "nmpc_tracker_node" 2>/dev/null
    pkill -f "detection_visualizer_node" 2>/dev/null
    pkill -f "visualization_node.py" 2>/dev/null
    pkill -f "drone_tf_publisher.py" 2>/dev/null
    pkill -f "drone_state_publisher_node" 2>/dev/null
    pkill -f "projection_model_node" 2>/dev/null
    pkill -f "tf_from_uav_pose_node" 2>/dev/null
    pkill -f "pose_cov_ops_interface_node" 2>/dev/null
    pkill -f "ros2 topic pub" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null

    sleep 2

    print_status $GREEN "✅ All HITL processes stopped"

    # Clean up
    rm -f /dev/shm/sem.* 2>/dev/null

    print_status $GREEN "🧹 Cleanup complete"
}

# Main menu
show_menu() {
    echo ""
    print_status $BLUE "📋 Pixhawk HITL Test Options:"
    echo "1) 🔍 Check PX4 Connection"
    echo "2) 🚀 Start Micro-XRCE-DDS Agent"
    echo "3) 🎯 Full Pixhawk HITL Test"
    echo "4) 📡 Monitor PX4 Topics"
    echo "5) 🧹 Kill All HITL Processes"
    echo "0) 🚪 Exit"
    echo ""
    read -p "Enter your choice (0-5): " choice
}

# Monitor PX4 topics
monitor_px4_topics() {
    print_status $BLUE "📡 PX4 Topic Monitor"
    echo "===================="

    print_status $YELLOW "📋 All PX4 topics:"
    ros2 topic list | grep "fmu"

    echo ""
    print_status $YELLOW "📊 Key PX4 topics:"

    # Check vehicle odometry
    if ros2 topic list | grep -q "/fmu/out/vehicle_odometry"; then
        print_status $GREEN "✅ /fmu/out/vehicle_odometry available"
        echo "Sample data:"
        timeout 2s ros2 topic echo /fmu/out/vehicle_odometry --once 2>/dev/null | head -20 || echo "  No data"
    fi

    # Check trajectory setpoint
    if ros2 topic list | grep -q "/fmu/in/trajectory_setpoint"; then
        print_status $GREEN "✅ /fmu/in/trajectory_setpoint available (NMPC commands)"
    fi

    # Check vehicle status
    if ros2 topic list | grep -q "/fmu/out/vehicle_status"; then
        print_status $GREEN "✅ /fmu/out/vehicle_status available"
    fi
}

# Main execution
main() {
    # Check if we're in the right directory
    if [ ! -d "src" ]; then
        print_status $RED "❌ Error: Please run this script from AVIANS_ROS2 root directory"
        exit 1
    fi

    # Source ROS2
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    fi

    # Source workspace
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi

    while true; do
        show_menu

        case $choice in
            1) check_px4_connection ;;
            2) start_xrce_agent ;;
            3) pixhawk_hitl_full_test ;;
            4) monitor_px4_topics ;;
            5) kill_all_hitl_processes ;;
            0)
                print_status $GREEN "👋 Goodbye!"
                exit 0
                ;;
            *)
                print_status $RED "❌ Invalid option"
                ;;
        esac

        echo ""
        read -p "Press Enter to continue..."
    done
}

# Run main
main
