#!/bin/bash

echo "====== 诊断 /drone/control/attitude_command 话题发布者 ======"

echo ""
echo "1. 检查当前话题信息："
ros2 topic info /drone/control/attitude_command -v 2>/dev/null || echo "话题不存在或无发布者"

echo ""
echo "2. 检查所有ROS节点："
ros2 node list 2>/dev/null

echo ""
echo "3. 检查所有action_manager相关进程："
ps aux | grep -E "action_manager|mission_sequence" | grep -v grep

echo ""
echo "4. 强制终止所有可能的发布者："
pkill -9 -f "action_manager"
pkill -9 -f "mission_sequence_controller"
pkill -9 -f "mission_action"
pkill -9 -f "waypoint_test"
pkill -9 -f "nmpc_tracker"
sleep 2

echo ""
echo "5. 重启ROS2 daemon："
ros2 daemon stop
sleep 1
ros2 daemon start
sleep 1

echo ""
echo "6. 清理完成，现在可以重新测试"
