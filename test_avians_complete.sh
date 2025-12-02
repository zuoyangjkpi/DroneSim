#!/bin/bash

# AVIANS ROS2 Complete Test Script

# Source environments
source /opt/ros/jazzy/setup.bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate airship_ros2
source install/setup.bash

echo "==================== AVIANS ROS2 System Test ===================="

echo "✅ ROS2 Version:"
ros2 --version

echo ""
echo "✅ Gazebo Version:"
gz --version

echo ""
echo "✅ Available AVIANS packages:"
ros2 pkg list | grep -E "(neural_network|uav_msgs|drone|nmpc)" || echo "No AVIANS packages found"

echo ""
echo "✅ Testing Gazebo launch:"
timeout 10s ros2 launch drone_description gz.launch.py > /dev/null 2>&1 &
sleep 5
if pgrep -f "gz sim" > /dev/null; then
    echo "✅ Gazebo starts successfully"
    pkill -f "gz sim"
else
    echo "❌ Gazebo failed to start"
fi

echo ""
echo "✅ Testing comprehensive suite:"
if [ -f "./comprehensive_test_suite.sh" ]; then
    echo "✅ comprehensive_test_suite.sh found"
    echo "✅ You can now run: ./comprehensive_test_suite.sh"
else
    echo "❌ comprehensive_test_suite.sh not found"
fi

echo ""
echo "==================== Test Complete ===================="
echo "🚁 AVIANS ROS2 system is ready!"
echo ""
echo "📋 Next Steps:"
echo "1. Run: source ~/.bashrc"
echo "2. Run: cd ~/AVIANS_ROS2_PORT1"
echo "3. For drone tracking: ./comprehensive_test_suite.sh (choose option 5)"
echo "4. If drone doesn't move, run: python3 ./pose_to_odom.py &"
echo "==================== ==================== ===================="
