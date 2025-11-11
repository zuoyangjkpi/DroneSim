# 节点清理覆盖总结

## 选项1启动的所有节点及清理状态

| # | 节点/进程名 | 启动方式 | kill_all_processes覆盖 |
|---|-----------|---------|----------------------|
| 1 | gz sim | ros2 launch gz.launch.py | ✅ `pkill -f "gz sim"` |
| 2 | yolo12_detector_node | ros2 run | ✅ `pkill -f "yolo12_detector_node"` |
| 3 | detection_visualizer_node | ros2 run | ✅ `pkill -f "detection_visualizer_node"` |
| 4 | drone_tf_publisher.py | python3 script | ✅ `pkill -f "drone_tf_publisher.py"` |
| 5 | visualization_node.py | python3 script | ✅ `pkill -f "visualization_node.py"` |
| 6 | drone_state_publisher_node | ros2 run | ✅ `pkill -f "drone_state_publisher_node"` |
| 7 | tf_from_uav_pose_node | ros2 run | ✅ `pkill -f "tf_from_uav_pose_node"` |
| 8 | projection_model_node | ros2 run | ✅ `pkill -f "projection_model_node"` |
| 9 | pose_cov_ops_interface_node | ros2 run | ✅ `pkill -f "pose_cov_ops_interface_node"` |
| 10 | nmpc_tracker_node | ros2 run | ✅ `pkill -f "nmpc_tracker_node"` + `pkill -9` |
| 11 | waypoint_controller | ros2 run | ✅ `pkill -f "waypoint_controller"` + `pkill -9` |
| 12 | yaw_controller | ros2 run | ✅ `pkill -f "yaw_controller"` + `pkill -9` |
| 13 | multicopter_velocity_control_adapter.py | ros2 run | ✅ `pkill -f "multicopter_velocity_control_adapter"` + `pkill -9` |
| 14 | action_manager | ros2 run | ✅ `pkill -f "mission_action_manager"` + `pkill -9` |
| 15 | mission_sequence_controller | ros2 run | ✅ `pkill -f "mission_sequence_controller"` + `pkill -9` |
| 16 | ros2 launch | background process | ✅ `pkill -f "ros2 launch"` |
| 17 | ros2 topic pub | command | ✅ `pkill -f "ros2 topic pub"` |
| 18 | ros2 run | generic | ✅ `pkill -f "ros2 run"` |

## 选项2启动的所有节点及清理状态

| # | 节点/进程名 | 启动方式 | kill_all_processes覆盖 |
|---|-----------|---------|----------------------|
| 1 | gz sim | ros2 launch | ✅ `pkill -f "gz sim"` |
| 2 | waypoint_controller | ros2 run | ✅ `pkill -f "waypoint_controller"` + `pkill -9` |
| 3 | yaw_controller | ros2 run | ✅ `pkill -f "yaw_controller"` + `pkill -9` |
| 4 | multicopter_velocity_control_adapter.py | ros2 run | ✅ `pkill -f "multicopter_velocity_control_adapter"` + `pkill -9` |
| 5 | waypoint_test_runner | ros2 run | ✅ `pkill -f "waypoint_test_runner"` + `pkill -9` |

## 选项3启动的所有节点及清理状态

| # | 节点/进程名 | 启动方式 | kill_all_processes覆盖 |
|---|-----------|---------|----------------------|
| 1 | gz sim | ros2 launch | ✅ `pkill -f "gz sim"` |
| 2 | multicopter_velocity_control_adapter.py | ros2 run | ✅ `pkill -f "multicopter_velocity_control_adapter"` + `pkill -9` |
| 3 | manual_velocity_test | ros2 run | ✅ `pkill -f "manual_velocity_test"` + `pkill -9` |

## kill_all_processes 清理策略

### 第一阶段：优雅清理
使用 `pkill -f "进程名"` 发送 SIGTERM 信号，允许进程正常退出

### 第二阶段：强制清理
使用 `pkill -9 -f "进程名"` 发送 SIGKILL 信号，强制终止顽固进程

### 第三阶段：ROS2 Daemon清理
- 停止并重启 ros2 daemon
- 遍历所有ROS节点并使用 `ros2 node kill` 清理

### 第四阶段：系统清理
- 清理共享内存: `rm -f /dev/shm/sem.*`
- 清理临时日志: `rm -f /tmp/*.log`

## 验证结果

✅ **所有节点100%覆盖**

所有三个测试选项启动的节点都已在 `kill_all_processes` 函数中被完整覆盖，确保：
1. 启动前环境干净（通过 `startup_cleanup`）
2. 退出时完整清理（选项0或选项4）
3. 两阶段清理确保顽固进程也能被终止

## 使用建议

1. **正常测试流程**：直接运行测试选项，`startup_cleanup` 会自动清理
2. **手动清理**：使用选项4手动触发清理
3. **紧急清理**：运行 `./diagnose_attitude_publishers.sh`
4. **完全清理**：先选项4，再重启ROS2 daemon

