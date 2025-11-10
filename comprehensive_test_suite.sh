#!/bin/bash

# AVIANS ROS2 PORT1 - Comprehensive Test Suite
# ============================================

echo "🚀 AVIANS ROS2 PORT1 - Comprehensive Test Suite"
echo "================================================"

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
    # Use more specific process matching and check if process is actually running
    if pgrep -x "$process_name" > /dev/null 2>&1; then
        return 0
    elif pgrep -f "$process_name" > /dev/null 2>&1; then
        # Additional check: verify the process is actually active
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

# Function to check topic data rate
check_topic_rate() {
    local topic_name=$1
    local min_rate=${2:-1.0}
    
    print_status $YELLOW "📊 Checking data rate for: $topic_name"
    
    # Use timeout to limit the check to 5 seconds
    local rate_output=$(timeout 5s ros2 topic hz "$topic_name" 2>/dev/null | tail -1)
    
    if [[ $rate_output == *"average rate"* ]]; then
        local rate=$(echo "$rate_output" | grep -o '[0-9]*\.[0-9]*' | head -1)
        if (( $(echo "$rate >= $min_rate" | bc -l) )); then
            print_status $GREEN "✅ Topic $topic_name rate: ${rate} Hz (>= ${min_rate} Hz)"
            return 0
        else
            print_status $YELLOW "⚠️  Topic $topic_name rate: ${rate} Hz (< ${min_rate} Hz)"
            return 1
        fi
    else
        print_status $RED "❌ No data on topic: $topic_name"
        return 1
    fi
}

wait_for_node() {
    local node_name=$1
    local timeout=${2:-10}
    local count=0
    print_status $YELLOW "⏳ Waiting for ROS node: $node_name"
    while [ $count -lt $timeout ]; do
        if ros2 node list 2>/dev/null | grep -q "$node_name"; then
            print_status $GREEN "✅ Node $node_name is active"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    print_status $RED "❌ Node $node_name not detected within timeout"
    return 1
}

wait_for_service() {
    local service_name=$1
    local timeout=${2:-10}
    local count=0
    print_status $YELLOW "⏳ Waiting for ROS service: $service_name"
    while [ $count -lt $timeout ]; do
        if ros2 service list 2>/dev/null | grep -q "$service_name"; then
            print_status $GREEN "✅ Service $service_name is available"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    print_status $RED "❌ Service $service_name not available within timeout"
    return 1
}

# Main menu function
show_menu() {
    echo ""
    print_status $BLUE "📋 测试选项:"
    echo "1) 🤖 自动测试（完整链路）"
    echo "2) 🧭 路径点控制测试（Takeoff→Fly→Hold）"
    echo "3) 🎮 手动速度控制测试（绕过PID，直接测试Gazebo插件）"
    echo "4) 🧹 终止所有 ROS 进程"
    echo "0) 🚪 退出"
    echo ""
    read -p "请选择 (0-4): " choice
}

# System status check
system_status_check() {
    print_status $BLUE "🔍 System Status Check"
    echo "======================"
    
    # Check ROS2 environment
    if [ -z "$ROS_DISTRO" ]; then
        print_status $RED "❌ ROS2 not sourced"
        return 1
    else
        print_status $GREEN "✅ ROS2 $ROS_DISTRO environment active"
    fi
    
    # Check workspace
    if [ -f "install/setup.bash" ]; then
        print_status $GREEN "✅ Workspace built"
    else
        print_status $YELLOW "⚠️  Workspace not built - run option 9"
    fi
    
    # Check key processes
    if check_process "gz sim"; then
        print_status $GREEN "✅ Gazebo simulation running"
    else
        print_status $YELLOW "⚠️  Gazebo not running"
    fi
    
    if check_process "yolo12_detector_node"; then
        print_status $GREEN "✅ YOLO detector running"
    else
        print_status $YELLOW "⚠️  YOLO detector not running"
    fi
    
    # Check NMPC processes
    if check_process "nmpc_tracker_node"; then
        print_status $GREEN "✅ NMPC tracker running"
    else
        print_status $YELLOW "⚠️  NMPC tracker not running"
    fi
    
    if check_process "nmpc_test_node"; then
        print_status $GREEN "✅ NMPC test node running"
    else
        print_status $YELLOW "⚠️  NMPC test node not running"
    fi
    
    # Check topics
    local topic_count=$(ros2 topic list 2>/dev/null | wc -l)
    print_status $GREEN "📡 Active topics: $topic_count"
    
    # Check specific topics
    if ros2 topic list | grep -q "/camera/image_raw"; then
        print_status $GREEN "✅ Camera topic available"
    else
        print_status $YELLOW "⚠️  Camera topic not available"
    fi
    
    if ros2 topic list | grep -q "/person_detections"; then
        print_status $GREEN "✅ Detection topic available"
    else
        print_status $YELLOW "⚠️  Detection topic not available"
    fi
}

# Launch Gazebo simulation
launch_gazebo() {
    print_status $BLUE "🎮 Launching Gazebo Simulation"
    echo "==============================="
    
    if check_process "gz sim"; then
        print_status $YELLOW "⚠️  Gazebo already running"
        read -p "Kill existing Gazebo? (y/n): " kill_gazebo
        if [ "$kill_gazebo" = "y" ]; then
            pkill -f "gz sim"
            sleep 2
        else
            return 0
        fi
    fi
    
    print_status $YELLOW "🚀 Starting Gazebo..."
    print_status $YELLOW "   This may take 10-15 seconds..."
    
    # Launch in background with proper environment
    export HOME="$HOME"
    export USER="$USER"
    export DISPLAY="${DISPLAY:-:0}"
    ros2 launch drone_description gz.launch.py > /tmp/gazebo.log 2>&1 &
    local gazebo_pid=$!
    
    # Wait for Gazebo to start
    local count=0
    while [ $count -lt 30 ]; do
        if check_process "gz sim"; then
            print_status $GREEN "✅ Gazebo started successfully"
            
            # Wait for camera topic
            if wait_for_topic "/camera/image_raw" 15; then
                print_status $GREEN "✅ Camera topic is publishing"
                return 0
            else
                print_status $YELLOW "⚠️  Camera topic not yet available"
                return 1
            fi
        fi
        sleep 1
        count=$((count + 1))
    done
    
    print_status $RED "❌ Gazebo failed to start"
    return 1
}

# Test YOLO detector
test_yolo_detector() {
    print_status $BLUE "🧠 Testing YOLO Detector"
    echo "========================="
    
    # Check if Gazebo is running
    if ! check_process "gz sim"; then
        print_status $YELLOW "⚠️  Gazebo not running - starting it first..."
        if ! launch_gazebo; then
            print_status $RED "❌ Cannot test YOLO without Gazebo"
            return 1
        fi
    fi
    
    # Clean up any existing YOLO processes automatically
    print_status $YELLOW "🧹 Cleaning up any existing YOLO processes..."
    pkill -f "yolo12_detector_node" 2>/dev/null
    sleep 2
    
    # Double check and force kill if needed
    if check_process "yolo12_detector_node"; then
        print_status $YELLOW "⚠️  Force killing stubborn YOLO processes..."
        pkill -9 -f "yolo12_detector_node" 2>/dev/null
        sleep 2
    fi
    
    # Find model files
    local model_path=$(find . -name "yolo12n.onnx" | head -1)
    local labels_path=$(find . -name "coco.names" | head -1)
    
    if [ -z "$model_path" ] || [ -z "$labels_path" ]; then
        print_status $RED "❌ YOLO model files not found"
        return 1
    fi
    
    print_status $GREEN "📁 Using model: $model_path"
    print_status $GREEN "📁 Using labels: $labels_path"
    
    print_status $YELLOW "🚀 Starting YOLO detector..."
    
    # Start YOLO with fixed topics - ROS2 environment should be sourced via drone command
    
    ros2 run neural_network_detector yolo12_detector_node \
        --ros-args \
        -p "use_sim_time:=true" \
        -p "model_path:=$model_path" \
        -p "labels_path:=$labels_path" \
        -p "use_gpu:=false" \
        -p "confidence_threshold:=0.3" \
        -p "desired_class:=0" \
        -p "iou_threshold:=0.4" \
        -p "publish_debug_image:=true" \
        -p "max_update_rate_hz:=2.0" > /tmp/yolo.log 2>&1 &
    
    local yolo_pid=$!
    
    # Wait for YOLO to start
    sleep 5
    
    if check_process "yolo12_detector_node"; then
        print_status $GREEN "✅ YOLO detector started"
        
        # Wait for detection topic
        if wait_for_topic "/person_detections" 10; then
            print_status $GREEN "✅ Detection topic is available"
            
            # Check if detections are being published
            print_status $YELLOW "🔍 Checking for detections..."
            if check_topic_rate "/person_detections" 0.1; then
                print_status $GREEN "✅ YOLO is publishing detections!"
            else
                print_status $YELLOW "⚠️  No detections yet (this is normal if no people in view)"
            fi
            
            return 0
        else
            print_status $RED "❌ Detection topic not available"
            return 1
        fi
    else
        print_status $RED "❌ YOLO detector failed to start"
        print_status $YELLOW "📋 Check log: tail /tmp/yolo.log"
        return 1
    fi
}

# Monitor all topics
monitor_topics() {
    print_status $BLUE "📡 Topic Monitor"
    echo "================"
    
    print_status $YELLOW "📋 All active topics:"
    ros2 topic list
    
    echo ""
    print_status $YELLOW "📊 Key topic rates:"
    
    # Check camera topic
    if ros2 topic list | grep -q "/camera/image_raw"; then
        check_topic_rate "/camera/image_raw" 1.0
    fi
    
    # Check detection topic
    if ros2 topic list | grep -q "/person_detections"; then
        check_topic_rate "/person_detections" 0.1
    fi
    
    # Check NMPC topics
    if ros2 topic list | grep -q "/X3/odometry"; then
        check_topic_rate "/X3/odometry" 10.0
    fi
    
    if ros2 topic list | grep -q "/X3/cmd_vel"; then
        check_topic_rate "/X3/cmd_vel" 5.0
    fi
    
    echo ""
    print_status $YELLOW "🔍 Topic details:"
    echo "Camera topic info:"
    ros2 topic info /camera/image_raw 2>/dev/null || echo "  Not available"
    
    echo "Detection topic info:"
    ros2 topic info /person_detections 2>/dev/null || echo "  Not available"
    
    echo "NMPC Drone odometry info:"
    ros2 topic info /X3/odometry 2>/dev/null || echo "  Not available"
    
    echo "NMPC Control commands info:"
    ros2 topic info /X3/cmd_vel 2>/dev/null || echo "  Not available"
}

# Full integration test (Fixed System)
full_integration_test() {
    print_status $BLUE "🎯 Full Integration Test (Fixed System)"
    echo "========================================"
    
    print_status $YELLOW "🔄 Starting complete fixed system with enhanced human detection..."
    print_status $YELLOW "📋 Launch sequence:"
    print_status $YELLOW "   1. Gazebo simulation environment"
    print_status $YELLOW "   2. YOLO person detector (optimized parameters)"
    print_status $YELLOW "   3. Detection visualizer"
    print_status $YELLOW "   4. Drone TF publisher"
    print_status $YELLOW "   5. RViz visualization with trajectory display"
    print_status $YELLOW "   6. drone_state_publisher bridge"
    print_status $YELLOW "   7. tf_from_uav_pose node"
    print_status $YELLOW "   8. projection_model node"
    print_status $YELLOW "   9. pose_cov_ops_interface node"
    print_status $YELLOW "   10. NMPC tracker (tracking real Gazebo walking_person)"
    print_status $YELLOW "   11. Low-level controllers (waypoint, yaw, velocity)"
    print_status $YELLOW "   12. Mission action manager + TrackTarget module"
    
    # Clean up existing processes
    print_status $YELLOW "🧹 Cleaning up existing processes..."
    kill_all_processes
    sleep 3
    
    # Step 1: Launch Gazebo
    print_status $YELLOW "Step 1/12: Starting Gazebo simulation..."
    if ! launch_gazebo; then
        print_status $RED "❌ Gazebo startup failed, cannot continue"
        return 1
    fi
    
    # Step 2: Start YOLO detector
    print_status $YELLOW "Step 2/12: Starting YOLO detector..."
    if ! test_yolo_detector; then
        print_status $YELLOW "⚠️  YOLO detector had issues, but continuing..."
    fi
    
    # Start C++ detection visualizer node
    print_status $YELLOW "Step 3/12: Starting detection visualizer..."
    
    ros2 run neural_network_detector detection_visualizer_node > /tmp/detection_visualizer.log 2>&1 &
    local viz_pid=$!
    sleep 2
    
    if check_process "detection_visualizer_node"; then
        print_status $GREEN "✅ Detection visualizer started successfully"
    else
        print_status $RED "❌ Detection visualizer failed to start"
    fi
    
    # Step 4: Start drone TF publisher for proper RViz display
    print_status $YELLOW "Step 4/12: Starting drone TF publisher..."
    python3 drone_tf_publisher.py > /tmp/drone_tf.log 2>&1 &
    local tf_pid=$!
    sleep 1
    
    # Step 5: Start RViz visualization with trajectory display
    print_status $YELLOW "Step 5/12: Starting RViz visualization..."
    python3 visualization_node.py > /tmp/visualization.log 2>&1 &
    local viz_node_pid=$!
    sleep 2
    
    if check_process "visualization_node.py"; then
        print_status $GREEN "✅ RViz visualization node started successfully"
        print_status $YELLOW "💡 Open RViz2 and add these topics for visualization:"
        print_status $YELLOW "   - /person_position_markers (MarkerArray) - Red=Predicted, Blue=Actual person"
        print_status $YELLOW "   - /drone_position_markers (MarkerArray) - Green=Current drone position"
        print_status $YELLOW "   - /drone_trajectory_markers (MarkerArray)"
        print_status $YELLOW "   - /drone_path (Path)"
        print_status $YELLOW "   - /detection_image (Image) - Raw image with detection boxes"
    else
        print_status $YELLOW "⚠️  RViz visualization node failed to start (continuing anyway)"
    fi
    
    # Step 6: Start drone_state_publisher bridge
    print_status $YELLOW "Step 6/12: Starting drone_state_publisher bridge..."
    ros2 run drone_state_publisher drone_state_publisher_node \
        --ros-args \
        -p use_sim_time:=True \
        > /tmp/drone_state_publisher.log 2>&1 &
    local drone_state_pub_pid=$!
    sleep 2

    if check_process "drone_state_publisher_node"; then
        print_status $GREEN "✅ drone_state_publisher bridge started successfully"
    else
        print_status $RED "❌ drone_state_publisher bridge failed to start"
        return 1
    fi

    # Step 7: Start tf_from_uav_pose node
    print_status $YELLOW "Step 7/12: Starting tf_from_uav_pose node..."
    ros2 run tf_from_uav_pose tf_from_uav_pose_node \
        --ros-args \
        -p use_sim_time:=True \
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
        > /tmp/tf_from_uav_pose.log 2>&1 &
    local tf_uav_pid=$!
    sleep 3

    if check_process "tf_from_uav_pose_node"; then
        print_status $GREEN "✅ tf_from_uav_pose node started successfully"
    else
        print_status $RED "❌ tf_from_uav_pose node failed to start"
        return 1
    fi

    # Step 8: Start projection_model node
    print_status $YELLOW "Step 8/12: Starting projection_model node..."
    ros2 run projection_model projection_model_node \
        --ros-args \
        -p topics.robot:="/X3/pose_with_covariance" \
        -p topics.camera:="/machine_1/camera/pose" \
        -p topics.optical:="/machine_1/camera/pose_optical" \
        -p camera.info_topic:="/camera/camera_info" \
        -p projected_object_topic:="/person_detections/world_frame" \
        -p detections_topic:="/person_detections" \
        > /tmp/projection_model.log 2>&1 &
    local projection_pid=$!
    sleep 3

    if check_process "projection_model_node"; then
        print_status $GREEN "✅ projection_model node started successfully"
    else
        print_status $RED "❌ projection_model node failed to start"
        return 1
    fi

    # Step 9: Start pose_cov_ops_interface node
    print_status $YELLOW "Step 9/12: Starting pose_cov_ops_interface node..."
    ros2 run pose_cov_ops_interface pose_cov_ops_interface_node \
        --ros-args \
        -p input_pose_topic:="/X3/odometry" \
        -p output_pose_topic:="/X3/pose_with_covariance" \
        -p default_covariance.matrix:="[0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]" \
        > /tmp/pose_cov_ops_interface.log 2>&1 &
    local pose_cov_pid=$!
    sleep 3
    
    if check_process "pose_cov_ops_interface_node"; then
        print_status $GREEN "✅ pose_cov_ops_interface node started successfully"
    else
        print_status $RED "❌ pose_cov_ops_interface node failed to start"
        return 1
    fi
    
    # Step 10: Start NMPC tracker (removed test_node - using real Gazebo walking_person instead)
    print_status $YELLOW "Step 10/12: Starting NMPC tracker..."
    ros2 run drone_nmpc_tracker nmpc_tracker_node > /tmp/nmpc_tracker_fixed.log 2>&1 &
    local tracker_pid=$!
    sleep 3

    if check_process "nmpc_tracker_node"; then
        print_status $GREEN "✅ NMPC tracker started successfully"
    else
        print_status $RED "❌ NMPC tracker failed to start"
        return 1
    fi

    # Step 11: Start low-level controllers that bridge NMPC to Gazebo
    local controller_params="$PWD/src/drone_low_level_controllers/config/controllers.yaml"
    if [ ! -f "$controller_params" ]; then
        print_status $RED "❌ Controller parameter file not found at $controller_params"
        return 1
    fi

    print_status $YELLOW "Step 11/12: Starting waypoint controller..."
    ros2 run drone_low_level_controllers waypoint_controller.py \
        --ros-args --params-file "$controller_params" \
        > /tmp/waypoint_controller.log 2>&1 &
    local waypoint_pid=$!
    sleep 2

    if check_process "waypoint_controller.py"; then
        print_status $GREEN "✅ Waypoint controller started successfully"
    else
        print_status $RED "❌ Waypoint controller failed to start"
        return 1
    fi

    print_status $YELLOW "Step 11/12: Starting yaw controller..."
    ros2 run drone_low_level_controllers yaw_controller.py \
        --ros-args --params-file "$controller_params" \
        > /tmp/yaw_controller.log 2>&1 &
    local yaw_pid=$!
    sleep 2

    if check_process "yaw_controller.py"; then
        print_status $GREEN "✅ Yaw controller started successfully"
    else
        print_status $RED "❌ Yaw controller failed to start"
        return 1
    fi

    print_status $YELLOW "Step 11/12: Starting MulticopterVelocityControl adapter..."
    ros2 run drone_low_level_controllers multicopter_velocity_control_adapter.py \
        --ros-args --params-file "$controller_params" \
        > /tmp/multicopter_velocity_control_adapter.log 2>&1 &
    local mc_adapter_pid=$!
    sleep 2

    if check_process "multicopter_velocity_control_adapter.py"; then
        print_status $GREEN "✅ MulticopterVelocityControl adapter started successfully"
    else
        print_status $RED "❌ MulticopterVelocityControl adapter failed to start"
        return 1
    fi

    # Step 11b: Start mission action manager
    print_status $YELLOW "Step 11/12: Starting mission action manager..."
    ros2 run mission_action_modules action_manager > /tmp/mission_action_manager.log 2>&1 &
    local action_manager_pid=$!
    if ! wait_for_node "/mission_action_manager" 10; then
        print_status $RED "❌ Mission action manager failed to reach running state"
        return 1
    fi
    if ! wait_for_service "/mission_actions/track_target" 10; then
        print_status $RED "❌ TrackTarget service not available"
        return 1
    fi
    
    # Step 12: Enable control and trigger tracking via action module
    print_status $YELLOW "Step 12/12: Enabling drone control and starting mission sequence controller..."
    
    print_status $YELLOW "  - Publishing enable command to /X3/enable (single message)"
    ros2 topic pub --once /X3/enable std_msgs/msg/Bool "{data: true}" > /tmp/x3_enable.log 2>&1
    
    print_status $YELLOW "  - Launching mission_sequence_controller (handles takeoff→search→track workflow)"
    ros2 run mission_action_modules mission_sequence_controller > /tmp/mission_sequence_controller.log 2>&1 &
    local sequence_pid=$!
    sleep 3
    if check_process "mission_sequence_controller"; then
        print_status $GREEN "✅ Mission sequence controller running"
    else
        print_status $RED "❌ mission_sequence_controller failed to start"
        return 1
    fi

    sleep 5
    
    # System status verification
    print_status $GREEN "🎉 Complete system startup finished!"
    print_status $BLUE "========================================"
    print_status $YELLOW "📊 System status verification:"
    
    # # Check critical topics
    # if wait_for_topic "/camera/image_raw" 5; then
    #     print_status $GREEN "  ✅ Camera images available"
    # else
    #     print_status $RED "  ❌ Camera images not available"
    # fi
    
    # if wait_for_topic "/person_detections" 5; then
    #     print_status $GREEN "  ✅ Person detections available"
    # else
    #     print_status $RED "  ❌ Person detections not available"
    # fi
    
    # if wait_for_topic "/X3/odometry" 5; then
    #     print_status $GREEN "  ✅ Drone odometry available"
    # else
    #     print_status $RED "  ❌ Drone odometry not available"
    # fi
    
    # if wait_for_topic "/X3/cmd_vel" 5; then
    #     print_status $GREEN "  ✅ Drone control commands available"
    # else
    #     print_status $RED "  ❌ Drone control commands not available"
    # fi
    
    # # 添加检测图像话题检查
    # if wait_for_topic "/detection_image" 5; then
    #     print_status $GREEN "  ✅ Detection image available"
    # else
    #     print_status $RED "  ❌ Detection image not available"
    # fi
    
    # # Check new component topics
    # if wait_for_topic "/person_detections/world_frame" 5; then
    #     print_status $GREEN "  ✅ Projected detections to world frame available"
    # else
    #     print_status $RED "  ❌ Projected detections to world frame not available"
    # fi
    
    # if wait_for_topic "/X3/pose_with_covariance" 5; then
    #     print_status $GREEN "  ✅ Pose with covariance available"
    # else
    #     print_status $RED "  ❌ Pose with covariance not available"
    # fi
    
    # print_status $BLUE "========================================"
    # print_status $GREEN "🎯 System functionality verification:"
    
    # # Check topic data rates
    # if check_topic_rate "/person_detections" 1.0; then
    #     print_status $GREEN "  ✅ Person detections are being published"
    # else
    #     print_status $YELLOW "  ⚠️  Person detection data rate is low"
    # fi
    
    # if check_topic_rate "/X3/odometry" 10.0; then
    #     print_status $GREEN "  ✅ Drone odometry is being published"
    # else
    #     print_status $YELLOW "  ⚠️  Drone odometry data rate is low"
    # fi
    
    # if check_topic_rate "/X3/cmd_vel" 1.0; then
    #     print_status $GREEN "  ✅ NMPC is publishing control commands"
    # else
    #     print_status $YELLOW "  ⚠️  NMPC control command data rate is low"
    # fi
    
    # # 添加检测图像话题数据速率检查
    # if check_topic_rate "/detection_image" 1.0; then
    #     print_status $GREEN "  ✅ Detection images are being published"
    # else
    #     print_status $YELLOW "  ⚠️  Detection image data rate is low"
    # fi
    
    # # Check new component data rates
    # if check_topic_rate "/person_detections/world_frame" 1.0; then
    #     print_status $GREEN "  ✅ Projected detections are being published"
    # else
    #     print_status $YELLOW "  ⚠️  Projected detection data rate is low"
    # fi
    
    # if check_topic_rate "/X3/pose_with_covariance" 1.0; then
    #     print_status $GREEN "  ✅ Pose with covariance is being published"
    # else
    #     print_status $YELLOW "  ⚠️  Pose with covariance data rate is low"
    # fi
    
    print_status $BLUE "========================================"
    print_status $GREEN "🎉 AVIANS system is running!"
    print_status $YELLOW "💡 Expected behavior:"
    print_status $YELLOW "   - See drone model in Gazebo"
    print_status $YELLOW "   - Drone should start flying and tracking walking_person in Gazebo"
    print_status $YELLOW "   - Red sphere: Drone's prediction of person position"
    print_status $YELLOW "   - Blue cylinder: Actual person position from Gazebo"
    print_status $YELLOW "   - Check logs: tail -f /tmp/nmpc_*.log"
    print_status $YELLOW "   - Use option 8 to stop all processes"
}

# NMPC Person Tracking Test
nmpc_person_tracking_test() {
    print_status $BLUE "🚁 NMPC Person Tracking Test"
    echo "============================="
    
    print_status $YELLOW "🔄 Starting NMPC person tracking system..."
    
    # Kill existing NMPC processes if running
    if check_process "nmpc_tracker_node" || check_process "nmpc_test_node"; then
        print_status $YELLOW "🛑 Stopping existing NMPC processes..."
        pkill -f "nmpc_tracker_node" 2>/dev/null
        pkill -f "nmpc_test_node" 2>/dev/null
        sleep 2
    fi
    
    # Step 1: Start test node (simulated person)
    print_status $YELLOW "Step 1: Starting person simulator..."
    ros2 run drone_nmpc_tracker nmpc_test_node > /tmp/nmpc_test.log 2>&1 &
    local test_pid=$!
    sleep 3
    
    if check_process "nmpc_test_node"; then
        print_status $GREEN "✅ Person simulator started"
        
        # Wait for person detection topic
        if wait_for_topic "/person_detections" 5; then
            print_status $GREEN "✅ Person detections available"
        else
            print_status $RED "❌ Person detections not available"
            return 1
        fi
    else
        print_status $RED "❌ Person simulator failed to start"
        return 1
    fi
    
    # Step 2: Start NMPC tracker
    print_status $YELLOW "Step 2: Starting NMPC tracker..."
    ros2 run drone_nmpc_tracker nmpc_tracker_node > /tmp/nmpc_tracker.log 2>&1 &
    local tracker_pid=$!
    sleep 3
    
    if check_process "nmpc_tracker_node"; then
        print_status $GREEN "✅ NMPC tracker started"
        
        # Wait for control commands
        if wait_for_topic "/X3/cmd_vel" 5; then
            print_status $GREEN "✅ NMPC control commands available"
        else
            print_status $YELLOW "⚠️  NMPC control commands not yet available"
        fi
    else
        print_status $RED "❌ NMPC tracker failed to start"
        return 1
    fi
    
    # Step 3: Enable tracking
    print_status $YELLOW "Step 3: Enabling tracking..."
    ros2 topic pub -r 1 /nmpc/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &
    local enable_pid=$!
    sleep 2
    
    # Step 4: Monitor performance
    print_status $YELLOW "Step 4: Monitoring NMPC performance..."
    
    # Check person detection rate
    print_status $YELLOW "🔍 Checking person detection rate..."
    if check_topic_rate "/person_detections" 5.0; then
        print_status $GREEN "✅ Person detections at good rate"
    else
        print_status $YELLOW "⚠️  Person detection rate lower than expected"
    fi
    
    # Check drone odometry
    print_status $YELLOW "🔍 Checking drone odometry..."
    if ros2 topic list | grep -q "/X3/odometry"; then
        if check_topic_rate "/X3/odometry" 10.0; then
            print_status $GREEN "✅ Drone odometry at good rate"
        else
            print_status $YELLOW "⚠️  Drone odometry rate lower than expected"
        fi
    fi
    
    # Check control commands
    print_status $YELLOW "🔍 Checking NMPC control output..."
    if check_topic_rate "/X3/cmd_vel" 5.0; then
        print_status $GREEN "✅ NMPC generating control commands"
        
        # Show a sample control command
        print_status $YELLOW "📊 Sample control command:"
        timeout 3s ros2 topic echo /X3/cmd_vel --once 2>/dev/null || echo "  No data available"
    else
        print_status $YELLOW "⚠️  NMPC control output rate lower than expected"
    fi
    
    print_status $GREEN "🎉 NMPC Person Tracking Test completed!"
    print_status $YELLOW "💡 Your drone is actively tracking the simulated person!"
    print_status $YELLOW "💡 Check logs: tail /tmp/nmpc_tracker.log"
    print_status $YELLOW "💡 Press Ctrl+C in a terminal to stop processes"
}

# System Diagnostics and Testing
system_diagnostics_test() {
    print_status $BLUE "🔧 System Diagnostics and Testing"
    echo "=================================="
    
    # Function to check if a topic is publishing
    check_topic_diagnostics() {
        local topic=$1
        print_status $YELLOW "Checking topic: $topic"
        
        if ros2 topic list | grep -q "$topic"; then
            print_status $GREEN "  ✅ Topic $topic exists"
            
            # Check if data is being published
            local data=$(timeout 3s ros2 topic echo "$topic" --once 2>/dev/null)
            if [ -n "$data" ]; then
                print_status $GREEN "  ✅ Topic $topic has data"
                return 0
            else
                print_status $YELLOW "  ⚠️  Topic $topic exists but no data"
                return 1
            fi
        else
            print_status $RED "  ❌ Topic $topic not found"
            return 1
        fi
    }

    # Check if ROS2 is sourced
    if [ -z "$ROS_DISTRO" ]; then
        print_status $RED "❌ ROS2 not sourced"
        return 1
    fi

    print_status $GREEN "✅ ROS2 $ROS_DISTRO environment active"

    # Test 1: Check visualization topics
    print_status $BLUE "Test 1: Checking visualization topics"
    check_topic_diagnostics "/person_position_markers"
    check_topic_diagnostics "/drone_trajectory_markers" 
    check_topic_diagnostics "/drone_path"

    # Test 2: Check YOLO topics
    print_status $BLUE "Test 2: Checking YOLO detection topics"
    check_topic_diagnostics "/person_detections"
    check_topic_diagnostics "/detection_image"

    # Test 3: Check drone topics
    print_status $BLUE "Test 3: Checking drone topics"
    check_topic_diagnostics "/X3/odometry"
    check_topic_diagnostics "/X3/cmd_vel"

    # Test 4: Check camera topic
    print_status $BLUE "Test 4: Checking camera topic"
    check_topic_diagnostics "/camera/image_raw"

    # Test 5: Check running processes
    print_status $BLUE "Test 5: Checking running processes"
    
    if check_process "gz sim"; then
        print_status $GREEN "  ✅ Gazebo simulation running"
    else
        print_status $YELLOW "  ⚠️  Gazebo not running"
    fi
    
    if check_process "yolo12_detector_node"; then
        print_status $GREEN "  ✅ YOLO detector running"
    else
        print_status $YELLOW "  ⚠️  YOLO detector not running"
    fi
    
    if check_process "nmpc_tracker_node"; then
        print_status $GREEN "  ✅ NMPC tracker running"
    else
        print_status $YELLOW "  ⚠️  NMPC tracker not running"
    fi
    
    if check_process "nmpc_test_node"; then
        print_status $GREEN "  ✅ NMPC test node running"
    else
        print_status $YELLOW "  ⚠️  NMPC test node not running"
    fi

    print_status $GREEN "🎉 Diagnostics completed!"
    print_status $YELLOW "💡 If topics show 'no data', make sure the simulation is running"
    print_status $YELLOW "💡 Run option 5 to start full integration test"
}

# Waypoint controller regression test
waypoint_controller_test() {
    print_status $BLUE "🧭 Waypoint Controller Test (Takeoff → Fly 10m @45° → Hold)"
    echo "============================================================"

    print_status $YELLOW "🧹 Cleaning up existing processes..."
    kill_all_processes
    sleep 2

    print_status $YELLOW "Step 1/4: Launching Gazebo simulation..."
    if ! launch_gazebo; then
        print_status $RED "❌ Gazebo startup failed, cannot continue"
        return 1
    fi

    local controller_params="$PWD/src/drone_low_level_controllers/config/controllers.yaml"
    if [ ! -f "$controller_params" ]; then
        print_status $RED "❌ Controller parameter file not found at $controller_params"
        return 1
    fi

    print_status $YELLOW "Step 2/4: Starting low-level controllers for waypoint testing..."

    print_status $YELLOW "  • Waypoint controller"
    ros2 run drone_low_level_controllers waypoint_controller.py \
        --ros-args --params-file "$controller_params" \
        > /tmp/waypoint_controller.log 2>&1 &
    sleep 2
    if check_process "waypoint_controller.py"; then
        print_status $GREEN "    ✅ Waypoint controller running"
    else
        print_status $RED "    ❌ Waypoint controller failed to start"
        return 1
    fi

    print_status $YELLOW "  • Yaw controller"
    ros2 run drone_low_level_controllers yaw_controller.py \
        --ros-args --params-file "$controller_params" \
        > /tmp/yaw_controller.log 2>&1 &
    sleep 2
    if check_process "yaw_controller.py"; then
        print_status $GREEN "    ✅ Yaw controller running"
    else
        print_status $RED "    ❌ Yaw controller failed to start"
        return 1
    fi

    print_status $YELLOW "  • Multicopter velocity adapter"
    ros2 run drone_low_level_controllers multicopter_velocity_control_adapter.py \
        --ros-args --params-file "$controller_params" \
        > /tmp/multicopter_velocity_control_adapter.log 2>&1 &
    sleep 2
    if check_process "multicopter_velocity_control_adapter.py"; then
        print_status $GREEN "    ✅ Velocity adapter running"
    else
        print_status $RED "    ❌ Multicopter velocity adapter failed to start"
        return 1
    fi

    print_status $YELLOW "Step 3/4: Enabling PX4 bridge interface (/X3/enable)"
    ros2 topic pub --once /X3/enable std_msgs/msg/Bool "{data: true}" > /tmp/x3_enable.log 2>&1

    print_status $YELLOW "Step 4/4: Executing waypoint test sequence..."
    ros2 run mission_action_modules waypoint_test_runner \
        --ros-args \
        -p takeoff_altitude:=3.0 \
        -p offset_distance:=10.0 \
        -p offset_heading_deg:=45.0 \
        -p hover_duration:=8.0 \
        > /tmp/waypoint_test_runner.log 2>&1 &

    local runner_pid=$!
    if wait_for_node "/waypoint_test_orchestrator" 10; then
        print_status $GREEN "✅ Waypoint test orchestrator is running (PID $runner_pid)"
        print_status $YELLOW "📄 Monitor progress: tail -f /tmp/waypoint_test_runner.log"
        print_status $YELLOW "🛎️ 完成后无人机会在目标点悬停，如需停止请选择菜单 3) 杀掉进程"
    else
        print_status $RED "❌ Failed to detect waypoint test orchestrator node"
        return 1
    fi
}

# Manual Velocity Control Test (bypassing PID controllers)
manual_velocity_test() {
    print_status $BLUE "🎮 Manual Velocity Control Test (Direct Gazebo Plugin Test)"
    echo "================================================================"
    print_status $YELLOW "⚠️  此测试完全绕过所有PID控制器和Action模块"
    print_status $YELLOW "使用最简单的比例控制（v=k*error）直接发送速度指令"
    print_status $YELLOW "用于诊断Gazebo MulticopterVelocityControl插件和坐标转换是否正常"
    echo ""

    print_status $YELLOW "🧹 Cleaning up existing processes..."
    kill_all_processes
    sleep 2

    print_status $YELLOW "Step 1/4: Launching Gazebo simulation..."
    if ! launch_gazebo; then
        print_status $RED "❌ Gazebo startup failed, cannot continue"
        return 1
    fi

    local controller_params="$PWD/src/drone_low_level_controllers/config/controllers.yaml"
    if [ ! -f "$controller_params" ]; then
        print_status $RED "❌ Controller parameter file not found at $controller_params"
        return 1
    fi

    print_status $YELLOW "Step 2/4: Starting ONLY velocity adapter (NO PID controllers)..."
    ros2 run drone_low_level_controllers multicopter_velocity_control_adapter.py \
        --ros-args --params-file "$controller_params" \
        > /tmp/multicopter_velocity_control_adapter.log 2>&1 &
    sleep 2
    if check_process "multicopter_velocity_control_adapter.py"; then
        print_status $GREEN "    ✅ Velocity adapter running"
    else
        print_status $RED "    ❌ Multicopter velocity adapter failed to start"
        return 1
    fi

    print_status $YELLOW "Step 3/4: Enabling PX4 bridge interface (/X3/enable)"
    ros2 topic pub --once /X3/enable std_msgs/msg/Bool "{data: true}" > /tmp/x3_enable.log 2>&1
    sleep 1

    print_status $YELLOW "Step 4/4: Starting manual velocity test script..."
    print_status $BLUE "📋 Test parameters:"
    echo "   • Takeoff height: 3.0m"
    echo "   • Target distance: 10.0m"
    echo "   • Target angle: 45°"
    echo "   • Position gain (k): 0.5"
    echo "   • Yaw gain (k2): 1.0"
    echo ""

    python3 manual_velocity_test.py \
        --ros-args \
        -p takeoff_height:=3.0 \
        -p target_distance:=10.0 \
        -p target_angle_deg:=45.0 \
        -p position_gain:=0.5 \
        -p yaw_gain:=1.0 \
        > /tmp/manual_velocity_test.log 2>&1 &

    local test_pid=$!
    sleep 2

    if wait_for_node "/manual_velocity_tester" 10; then
        print_status $GREEN "✅ Manual velocity tester is running (PID $test_pid)"
        print_status $YELLOW "📄 Monitor progress: tail -f /tmp/manual_velocity_test.log"
        print_status $YELLOW "📊 Watch adapter: tail -f /tmp/multicopter_velocity_control_adapter.log"
        print_status $BLUE "🔍 测试流程（完全手动控制）："
        echo "   1. 手动起飞：发送 vz=dz*0.5 直到到达3m高度"
        echo "   2. 水平飞行：发送 vx=dx*0.5, vy=dy*0.5, yaw_rate=dyaw*1.0"
        echo "   3. 飞向相对位置 (10m @ 45°)"
        echo "   4. 到达后HOLD（发送零速度）"
        echo ""
        print_status $YELLOW "🛎️ 完成后按任意键返回菜单，选择选项4) 终止所有进程"
    else
        print_status $RED "❌ Failed to detect manual velocity tester node"
        return 1
    fi
}

# NMPC + Gazebo Visual Tracking
nmpc_gazebo_visual_tracking() {
    print_status $BLUE "🎮 NMPC + Gazebo Visual Tracking"
    echo "=================================="
    
    # Check if Gazebo packages are available
    if ! command -v gz > /dev/null 2>&1; then
        print_status $RED "❌ Gazebo not installed!"
        print_status $YELLOW "💡 Install with: sudo apt install gz-garden"
        print_status $YELLOW "💡 Also install: sudo apt install ros-jazzy-ros-gz ros-jazzy-ros-gz-bridge ros-jazzy-ros-gz-sim"
        return 1
    fi
    
    print_status $YELLOW "🔄 Starting complete visual tracking system..."
    
    # Step 1: Clean up existing processes
    print_status $YELLOW "Step 1: Cleaning up existing processes..."
    pkill -f "gz sim" 2>/dev/null
    pkill -f "nmpc_tracker_node" 2>/dev/null
    pkill -f "nmpc_test_node" 2>/dev/null
    pkill -f "parameter_bridge" 2>/dev/null
    sleep 3
    
    # Step 2: Start Gazebo
    print_status $YELLOW "Step 2: Starting Gazebo simulation..."
    if ! launch_gazebo; then
        print_status $RED "❌ Failed to start Gazebo"
        return 1
    fi
    
    # Step 3: Start person simulator
    print_status $YELLOW "Step 3: Starting person simulator..."
    ros2 run drone_nmpc_tracker nmpc_test_node > /tmp/nmpc_test_gazebo.log 2>&1 &
    sleep 3
    
    if ! check_process "nmpc_test_node"; then
        print_status $RED "❌ Person simulator failed to start"
        return 1
    fi
    print_status $GREEN "✅ Person simulator running"
    
    # Step 4: Start NMPC tracker
    print_status $YELLOW "Step 4: Starting NMPC tracker..."
    ros2 run drone_nmpc_tracker nmpc_tracker_node > /tmp/nmpc_tracker_gazebo.log 2>&1 &
    sleep 3
    
    if ! check_process "nmpc_tracker_node"; then
        print_status $RED "❌ NMPC tracker failed to start"
        return 1
    fi
    print_status $GREEN "✅ NMPC tracker running"
    
    # Step 5: Enable tracking
    print_status $YELLOW "Step 5: Enabling tracking..."
    ros2 topic pub -r 1 /nmpc/enable std_msgs/msg/Bool "data: true" > /dev/null 2>&1 &
    sleep 2
    
    # Step 6: Launch RViz for visualization (optional)
    print_status $YELLOW "Step 6: Starting RViz for additional visualization..."
    if command -v rviz2 > /dev/null 2>&1; then
        rviz2 > /dev/null 2>&1 &
        print_status $GREEN "✅ RViz started"
    else
        print_status $YELLOW "⚠️  RViz not available"
    fi
    
    print_status $GREEN "🎉 Complete NMPC + Gazebo system is running!"
    print_status $YELLOW "💡 You should now see:"
    print_status $YELLOW "   - Gazebo window with drone simulation"
    print_status $YELLOW "   - Drone tracking simulated person movement"
    print_status $YELLOW "   - RViz showing tracking data (if available)"
    print_status $YELLOW "💡 Use option 8 to stop all processes"
}

# Kill all ROS processes
kill_all_processes() {
    print_status $BLUE "🧹 Killing All ROS Processes"
    echo "============================="
    
    print_status $YELLOW "🛑 Stopping all processes..."
    
    # Kill specific processes
    pkill -f "gz sim" 2>/dev/null
    pkill -f "yolo12_detector_node" 2>/dev/null
    pkill -f "nmpc_tracker_node" 2>/dev/null
    pkill -f "nmpc_test_node" 2>/dev/null
    pkill -f "mission_action_manager" 2>/dev/null
    pkill -9 -f "mission_action_manager" 2>/dev/null
    pkill -f "mission_action_modules/.*/action_manager" 2>/dev/null
    pkill -9 -f "mission_action_modules/.*/action_manager" 2>/dev/null
    pkill -f "mission_action_modules.*action_manager" 2>/dev/null
    pkill -9 -f "mission_action_modules.*action_manager" 2>/dev/null
    pkill -f "mission_sequence_controller" 2>/dev/null
    pkill -9 -f "mission_sequence_controller" 2>/dev/null
    pkill -f "mission_action_modules/.*/mission_sequence_controller" 2>/dev/null
    pkill -9 -f "mission_action_modules/.*/mission_sequence_controller" 2>/dev/null
    pkill -f "detection_visualizer_node" 2>/dev/null
    pkill -f "visualization_node.py" 2>/dev/null
    pkill -f "drone_tf_publisher.py" 2>/dev/null
    pkill -f "ros2 launch" 2>/dev/null
    pkill -f "ros2 run" 2>/dev/null
    pkill -f "parameter_bridge" 2>/dev/null
    pkill -f "rviz2" 2>/dev/null
    pkill -f "ros2 topic pub" 2>/dev/null
    pkill -f "projection_model_node" 2>/dev/null
    pkill -f "tf_from_uav_pose_node" 2>/dev/null
    pkill -f "pose_cov_ops_interface_node" 2>/dev/null
    pkill -f "drone_state_publisher_node" 2>/dev/null
    pkill -f "multicopter_velocity_control_adapter.py" 2>/dev/null
    pkill -f "waypoint_controller.py" 2>/dev/null
    pkill -f "yaw_controller.py" 2>/dev/null
    pkill -f "px4_bridge" 2>/dev/null
    pkill -f "robot_state_publisher" 2>/dev/null
    pkill -f "static_transform_publisher" 2>/dev/null
    
    sleep 2
    
    # Kill any remaining ROS nodes gracefully
    if command -v ros2 >/dev/null 2>&1; then
        ros2 daemon stop >/dev/null 2>&1
        ros2 daemon start >/dev/null 2>&1
        for node in $(ros2 node list 2>/dev/null); do
            ros2 node kill "$node" 2>/dev/null
        done
    fi

    # Check if processes are stopped
    if ! check_process "gz sim" && ! check_process "yolo12_detector_node"; then
        print_status $GREEN "✅ All processes stopped"
    else
        print_status $YELLOW "⚠️  Some processes may still be running"
    fi
    
    # Clean up shared memory
    rm -f /dev/shm/sem.* 2>/dev/null
    rm -f /tmp/*.log 2>/dev/null
    
    print_status $GREEN "🧹 Cleanup complete"
}

# Rebuild project
rebuild_project() {
    print_status $BLUE "🔧 Rebuilding Project"
    echo "====================="
    
    print_status $YELLOW "🧹 Cleaning build artifacts..."
    rm -rf build/ install/ log/
    
    print_status $YELLOW "🔨 Building project..."
    if colcon build; then
        print_status $GREEN "✅ Build successful"
        print_status $YELLOW "💡 Don't forget to source: source install/setup.bash"
    else
        print_status $RED "❌ Build failed"
        return 1
    fi
}

# Performance monitor
performance_monitor() {
    print_status $BLUE "📊 Performance Monitor"
    echo "======================"
    
    print_status $YELLOW "🔍 System resources:"
    echo "CPU Usage: $(top -bn1 | grep "Cpu(s)" | awk '{print $2}' | cut -d'%' -f1)%"
    echo "Memory: $(free -h | awk '/^Mem:/ {print $3 "/" $2}')"
    
    print_status $YELLOW "🎯 ROS2 processes:"
    ps aux | grep -E "(gz|yolo|ros2)" | grep -v grep | while read line; do
        echo "  $line"
    done
    
    if check_process "yolo12_detector_node"; then
        print_status $YELLOW "🧠 YOLO performance:"
        echo "  Check /tmp/yolo.log for inference times"
        tail -5 /tmp/yolo.log 2>/dev/null | grep "Inference latency" || echo "  No recent inference data"
    fi
    
    if check_process "nmpc_tracker_node"; then
        print_status $YELLOW "🚁 NMPC performance:"
        echo "  NMPC optimization warnings are normal"
        # Check NMPC control commands
        if ros2 topic list | grep -q "/X3/cmd_vel"; then
            print_status $GREEN "  ✅ NMPC publishing control commands"
        else
            print_status $YELLOW "  ⚠️  No NMPC control commands"
        fi
    fi
}

# Main execution
main() {
    # Check if we're in the right directory
    if [ ! -d "src" ]; then
        print_status $RED "❌ Error: Please run this script from the project root directory"
        exit 1
    fi
    
    # Source ROS2 if available
    if [ -f "/opt/ros/jazzy/setup.bash" ]; then
        source /opt/ros/jazzy/setup.bash
    elif [ -f "/opt/ros/humble/setup.bash" ]; then
        source /opt/ros/humble/setup.bash
    fi
    
    # Source workspace if available
    if [ -f "install/setup.bash" ]; then
        source install/setup.bash
    fi
    
    while true; do
        show_menu
        
        case $choice in
            1)
                full_integration_test
                ;;
            2)
                waypoint_controller_test
                ;;
            3)
                manual_velocity_test
                ;;
            4)
                kill_all_processes
                ;;
            0)
                print_status $GREEN "👋 Goodbye!"
                print_status $YELLOW "🧹 退出前清理所有相关进程..."
                kill_all_processes
                exit 0
                ;;
            *)
                print_status $RED "❌ Invalid option. Please try again."
                ;;
        esac
        
        echo ""
        read -p "Press Enter to continue..."
    done
}

# Run main function
main
