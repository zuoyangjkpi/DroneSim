# NMPC 控制器工作原理与路径点生成说明

## 概述

本项目中的 NMPC (Nonlinear Model Predictive Control，非线性模型预测控制) 系统专门用于无人机人员跟踪任务。当选择测试套件中的选项5时，系统会启动完整的集成测试，实现无人机对移动人员的环绕跟踪，确保摄像头始终对准目标人员。

## NMPC 系统架构

### 1. 系统组件结构

```
NMPC跟踪系统
├── 感知层
│   ├── Gazebo仿真环境 (提供无人机状态和人员位置)
│   ├── YOLO人员检测器 (识别摄像头中的人员)
│   └── 投影模型 (将2D检测转换为3D世界坐标)
├── 控制层
│   ├── NMPC优化器 (生成最优控制序列)
│   ├── 轨迹预测器 (预测未来状态序列)
│   └── 成本函数计算器 (评估控制质量)
└── 执行层
    ├── 低级位置控制器 (waypoint_controller.py)
    ├── 低级姿态控制器 (attitude_controller.py)
    └── Gazebo MulticopterVelocityControl 适配器 (multicopter_velocity_control_adapter.py)
```

### 2. 核心文件说明

- **nmpc_controller.py**: NMPC优化算法实现
- **nmpc_node.py**: ROS2节点，处理传感器数据和控制指令
- **config.py**: 系统参数配置
- **nmpc_params.yaml**: ROS2参数文件

## NMPC 算法工作原理

### 1. 状态向量定义

NMPC控制器使用12维状态向量描述无人机状态：

```python
状态向量 = [x, y, z, vx, vy, vz, roll, pitch, yaw, wx, wy, wz]
```

**状态分量说明**：
- `[x, y, z]`: 位置坐标 (米)
- `[vx, vy, vz]`: 线速度 (米/秒)
- `[roll, pitch, yaw]`: 姿态角 (弧度)
- `[wx, wy, wz]`: 角速度 (弧度/秒)

### 2. 控制输入定义

控制输入为4维向量：

```python
控制输入 = [thrust, roll_cmd, pitch_cmd, yaw_rate_cmd]
```

**控制分量说明**：
- `thrust`: 推力指令 (牛顿)
- `roll_cmd`: 滚转角指令 (弧度)
- `pitch_cmd`: 俯仰角指令 (弧度)
- `yaw_rate_cmd`: 偏航角速度指令 (弧度/秒)

### 3. 无人机动力学模型

NMPC使用四旋翼动力学模型进行状态预测：

```python
def drone_dynamics(self, state, control):
    # 位置动力学: dx/dt = v
    state_dot[0:3] = velocity

    # 速度动力学: dv/dt = (R * thrust_body + gravity) / mass
    R = rotation_matrix(roll, pitch, yaw)
    thrust_world = R @ [0, 0, thrust]
    acceleration = (thrust_world + [0, 0, -mg]) / mass
    state_dot[3:6] = acceleration

    # 姿态动力学: d(attitude)/dt = angular_velocity
    state_dot[6:9] = angular_velocity

    # 角速度动力学 (简化PD控制)
    state_dot[9] = 5.0 * (roll_cmd - roll)
    state_dot[10] = 5.0 * (pitch_cmd - pitch)
    state_dot[11] = yaw_rate_cmd
```

## 人员跟踪策略

### 1. 环绕跟踪算法

NMPC实现基于相位控制的环绕跟踪：

```python
def _update_tracking_target(self):
    # 计算当前相对于人员的相位角
    drone_pos = self.current_state.data[0:3]
    relative_pos = drone_pos[:2] - person_pos[:2]
    current_phase = atan2(relative_pos[1], relative_pos[0])

    # 自适应角速度控制
    person_speed = norm(person_velocity[:2])
    base_angular_velocity = 0.5  # 基础轨道速度
    adaptive_angular_velocity = base_angular_velocity + 0.3 * person_speed

    # 更新期望相位 (积分运动学)
    self._desired_phase += adaptive_angular_velocity * dt

    # 计算圆轨道目标位置
    radius = self.config.OPTIMAL_TRACKING_DISTANCE  # 4.0米
    target_x = person_x + radius * cos(desired_phase)
    target_y = person_y + radius * sin(desired_phase)
    target_z = person_z + height_offset  # 1.5米高度偏移

    # 计算切向速度以实现平滑圆周运动
    tangential_speed = radius * adaptive_angular_velocity
    target_vx = person_vx + (-sin(desired_phase) * tangential_speed)
    target_vy = person_vy + (cos(desired_phase) * tangential_speed)
```

### 2. 摄像头指向控制

确保无人机摄像头始终朝向被跟踪人员：

```python
# 计算朝向人员的期望偏航角
to_person = person_position - drone_position
desired_yaw = atan2(to_person[1], to_person[0]) + pi  # +π确保面向人员
```

## 成本函数设计

### 1. 多目标成本函数

NMPC使用加权多目标成本函数：

```python
def compute_stage_cost(self, state, control, time_step):
    cost = 0.0

    # 1. 位置跟踪成本
    pos_error = state[0:3] - target_position
    cost += sum(W_POSITION * pos_error²)

    # 2. 速度跟踪成本
    vel_error = state[3:6] - target_velocity
    cost += sum(W_VELOCITY * vel_error²)

    # 3. 姿态稳定成本
    att_error = state[6:8]  # roll, pitch (不包括yaw)
    cost += sum(W_ATTITUDE[0:2] * att_error²)

    # 4. 偏航角跟踪成本
    yaw_error = wrap_angle(state[8] - desired_yaw)
    cost += W_ATTITUDE[2] * yaw_error²

    # 5. 角速度稳定成本
    omega_error = state[9:12]
    cost += sum(W_ANGULAR_RATE * omega_error²)

    # 6. 控制能耗成本
    cost += sum(W_CONTROL * control²)

    # 7. 距离保持成本
    distance_to_person = norm(state[0:3] - person_position)
    distance_error = abs(distance_to_person - OPTIMAL_TRACKING_DISTANCE)
    cost += W_TRACKING_DISTANCE * distance_error²

    # 8. 视角保持成本
    drone_facing = [cos(yaw), sin(yaw)]
    to_person_dir = normalize(to_person[0:2])
    angle_error = acos(dot(drone_facing, to_person_dir))
    cost += W_CAMERA_ANGLE * angle_error²

    return cost
```

### 2. 成本权重配置

```python
# 位置跟踪权重
W_POSITION = [10.0, 10.0, 8.0]     # [x, y, z]
W_VELOCITY = [2.0, 2.0, 2.0]       # [vx, vy, vz]

# 姿态控制权重
W_ATTITUDE = [1.0, 1.0, 3.0]       # [roll, pitch, yaw]
W_ANGULAR_RATE = [0.5, 0.5, 1.0]   # [wx, wy, wz]

# 控制努力权重
W_CONTROL = [0.1, 0.5, 0.5, 0.3]   # [thrust, roll, pitch, yaw_rate]

# 跟踪专用权重
W_TRACKING_DISTANCE = 5.0          # 距离保持
W_CAMERA_ANGLE = 3.0               # 视角保持
```

## 优化算法实现

### 1. 梯度下降优化

NMPC使用梯度下降法求解最优控制序列：

```python
def optimize(self):
    # 初始化控制序列 (热启动)
    controls = [ctrl.copy() for ctrl in self.control_history]

    # 梯度下降迭代
    for iteration in range(max_iterations):
        # 计算成本函数梯度
        gradient = self.compute_gradient(controls)

        # 更新控制输入
        for i in range(len(controls)):
            controls[i] -= step_size * gradient[i]
            controls[i] = clip_control(controls[i])

        # 检查收敛性
        grad_norm = norm(concatenate(gradient))
        if grad_norm < tolerance:
            break

    # 返回第一个控制输入作为当前最优控制
    return controls[0], optimization_info
```

### 2. 梯度计算 (有限差分法)

```python
def compute_gradient(self, controls):
    gradient = []
    base_trajectory = self.predict_trajectory(current_state, controls)
    base_cost = self.compute_total_cost(base_trajectory, controls)

    # 对每个控制输入的每个分量计算偏导数
    for i in range(len(controls)):
        control_grad = zeros_like(controls[i])

        for j in range(len(controls[i])):
            # 扰动控制输入
            perturbed_controls = copy(controls)
            perturbed_controls[i][j] += regularization

            # 计算扰动后的成本
            perturbed_trajectory = self.predict_trajectory(current_state, perturbed_controls)
            perturbed_cost = self.compute_total_cost(perturbed_trajectory, perturbed_controls)

            # 有限差分梯度
            control_grad[j] = (perturbed_cost - base_cost) / regularization

        gradient.append(control_grad)

    return gradient
```

## 路径点生成过程

### 1. 轨迹预测

NMPC通过欧拉积分预测未来轨迹：

```python
def predict_trajectory(self, initial_state, controls):
    trajectory = [initial_state.copy()]
    current_state = initial_state.copy()

    # 预测N步未来状态
    for i in range(N):  # N = 8步
        control = controls[i] if i < len(controls) else hover_control

        # 使用动力学模型积分
        state_dot = self.drone_dynamics(current_state, control)
        current_state = current_state + state_dot * timestep  # dt = 0.25s

        # 状态约束裁剪
        current_state.data = clip_state(current_state.data)
        trajectory.append(current_state.copy())

    return trajectory
```

### 2. 预测时域配置

```python
# 时域参数
TIMESTEP = 0.25      # 时间步长: 0.25秒
LOOKAHEAD = 2.0      # 预测时域: 2.0秒
N = 8                # 预测步数: 8步

# 轨迹包含9个状态点 (当前 + 8个未来)
trajectory_points = [
    current_state,     # t = 0.0s (当前状态)
    state_1,          # t = 0.25s
    state_2,          # t = 0.50s
    state_3,          # t = 0.75s
    state_4,          # t = 1.0s
    state_5,          # t = 1.25s
    state_6,          # t = 1.5s
    state_7,          # t = 1.75s
    state_8           # t = 2.0s (预测终点)
]
```

### 3. 路径点输出格式

NMPC输出的路径点包含完整的状态信息：

```python
# 每个路径点包含12维状态向量
path_point = {
    'position': [x, y, z],              # 位置 (米)
    'velocity': [vx, vy, vz],           # 速度 (m/s)
    'attitude': [roll, pitch, yaw],     # 姿态 (弧度)
    'angular_velocity': [wx, wy, wz],   # 角速度 (rad/s)
    'timestamp': t                      # 时间戳 (秒)
}
```

## 控制指令转换

### 1. 高级控制到低级控制

NMPC输出经过转换发送给低级控制器：

```python
def _send_tracking_commands(self, optimal_control):
    # 提取目标位置
    target_position = self.controller.target_position

    # 生成航点指令
    waypoint_msg = PoseStamped()
    waypoint_msg.pose.position.x = target_position[0]
    waypoint_msg.pose.position.y = target_position[1]
    waypoint_msg.pose.position.z = target_position[2]
    self.waypoint_pub.publish(waypoint_msg)

    # 生成姿态指令 (面向人员)
    to_person = person_position - drone_position
    desired_yaw = atan2(to_person[1], to_person[0])

    attitude_msg = Vector3Stamped()
    attitude_msg.vector.x = 0.0        # roll
    attitude_msg.vector.y = 0.0        # pitch
    attitude_msg.vector.z = desired_yaw # yaw
    self.attitude_pub.publish(attitude_msg)
```

### 2. 控制流水线

```
NMPC优化器
    ↓ [thrust, roll_cmd, pitch_cmd, yaw_rate_cmd]
目标生成器
    ↓ [waypoint, attitude_command]
低级控制器
    ↓ [cmd_vel, rotor_commands]
Gazebo仿真器
```

## 系统约束

### 1. 状态约束

```python
# 位置约束 (米)
position_limits = {
    'x': [-50.0, 50.0],
    'y': [-50.0, 50.0],
    'z': [0.5, 20.0]
}

# 速度约束 (m/s)
velocity_limits = {
    'vx': [-3.0, 3.0],
    'vy': [-3.0, 3.0],
    'vz': [-3.0, 3.0]
}

# 姿态约束 (弧度)
attitude_limits = {
    'roll': [-π/4, π/4],    # ±45度
    'pitch': [-π/4, π/4],   # ±45度
    'yaw': [-π, π]          # ±180度
}
```

### 2. 控制约束

```python
# 控制输入约束
control_limits = {
    'thrust': [0.0, 20.0],        # 推力 (牛顿)
    'roll_cmd': [-π/6, π/6],      # 滚转指令 (±30度)
    'pitch_cmd': [-π/6, π/6],     # 俯仰指令 (±30度)
    'yaw_rate_cmd': [-1.0, 1.0]   # 偏航角速度 (±1 rad/s)
}
```

## 跟踪性能指标

### 1. 距离保持性能

```python
# 目标: 保持4米最优跟踪距离
optimal_distance = 4.0  # 米
distance_tolerance = ±0.5  # 允许误差

# 性能评估
distance_error = abs(actual_distance - optimal_distance)
tracking_accuracy = 1.0 if distance_error < 0.5 else exp(-distance_error)
```

### 2. 视角保持性能

```python
# 目标: 摄像头始终指向人员
max_angle_error = 15°  # 最大允许角度偏差

# 性能评估
angle_error = angle_between(drone_facing, to_person_direction)
camera_tracking_accuracy = 1.0 if angle_error < 15° else cos(angle_error)
```

### 3. 轨迹平滑性

```python
# 评估控制输入变化率
control_smoothness = 1.0 / (1.0 + norm(control_derivative))

# 评估状态轨迹平滑性
trajectory_smoothness = 1.0 / (1.0 + norm(acceleration_jerk))
```

## 实际运行效果

当运行选项5 (Full Integration Test) 时，系统实现以下行为：

1. **启动阶段**: 无人机起飞到2米高度
2. **搜索阶段**: 如果未检测到人员，无人机原地旋转搜索
3. **跟踪阶段**: 检测到人员后，无人机自动切换到NMPC跟踪模式
4. **环绕运动**: 无人机以4米半径绕人员做圆周运动，相位角以0.5-0.8 rad/s速度演进
5. **自适应跟踪**: 根据人员移动速度动态调整轨道角速度
6. **姿态控制**: 摄像头始终朝向被跟踪人员
7. **平滑过渡**: 所有运动都经过NMPC优化，确保平滑无突变

通过这套NMPC系统，无人机能够实现稳定、平滑且智能的人员跟踪，满足专业航拍和监控应用的需求。
