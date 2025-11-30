# Mission Execution Architecture 详解

## 📐 系统架构概览

```
┌─────────────────────────────────────────────────────────────────────┐
│                          YOUR YAML MISSION                          │
│  mission:                                                           │
│    stages:                                                          │
│      - id: "takeoff"                                                │
│        type: "TAKEOFF"                                              │
│        transitions: {success: "fly_square", failure: "abort"}       │
└─────────────────────────────┬───────────────────────────────────────┘
                              │ publish to /mission_executor/plan
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│                      MISSION EXECUTOR NODE                          │
│  • 读取YAML，解析stages                                              │
│  • 管理状态机（当前在哪个stage）                                      │
│  • 决定何时转移到下一个stage                                          │
│  • 调用Action Manager的服务                                          │
└─────────────────────────────┬───────────────────────────────────────┘
                              │ call service /mission_actions/takeoff
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│                      ACTION MANAGER NODE                            │
│  • 拥有11个Action Module实例                                         │
│  • 全局互斥：同一时间只能1个module运行                                │
│  • 启动/停止/切换modules                                             │
│  • 发布events回Mission Executor                                     │
└─────────────────────────────┬───────────────────────────────────────┘
                              │ start TakeoffModule
                              ↓
┌─────────────────────────────────────────────────────────────────────┐
│                        TAKEOFF MODULE                               │
│  • 创建定时器，每0.2秒检查高度                                        │
│  • 发布waypoint到/drone/control/waypoint_command                     │
│  • 到达目标高度后调用self.succeed()                                   │
└─────────────────────────────┬───────────────────────────────────────┘
                              │ publish event "succeeded"
                              ↓
                    回到Mission Executor，进入下一个stage
```

---

## 🎯 Part 1: Mission Executor (状态机管理器)

### 核心职责
**Mission Executor = "导演"**
- 读剧本（YAML）
- 按顺序安排演员（Action Modules）上场
- 根据演出结果决定下一步

### 内部数据结构

```python
class MissionExecutor:
    # 存储所有stage的字典
    _stage_map = {
        "takeoff": StageSpec(id="takeoff", type="TAKEOFF", transitions={...}),
        "fly_square": StageSpec(id="fly_square", type="FLY_TO", transitions={...}),
        "land": StageSpec(id="land", type="LAND_AT_POINT", transitions={...}),
        ...
    }

    # 当前正在执行的stage
    _current_stage = StageSpec(id="takeoff", ...)

    # 任务是否激活
    _mission_active = True
```

### 执行流程（一个完整循环）

```
时间线：执行 takeoff → fly_square 的过程
═══════════════════════════════════════════════════════════════

t0: 收到YAML mission plan
    ↓
    [Mission Executor] 解析YAML → 填充 _stage_map

t1: 开始执行
    ↓
    [Mission Executor] _start_stage("takeoff")
    ├─ _current_stage = _stage_map["takeoff"]
    ├─ 发布params到 /mission_executor/action_params
    │   {"stage_id": "takeoff", "params": {"target_altitude": 3.0}}
    └─ 调用服务 /mission_actions/takeoff

t2: Action Manager收到服务调用
    ↓
    [Action Manager] 启动TakeoffModule
    ├─ TakeoffModule创建定时器（每0.2秒运行）
    └─ TakeoffModule开始发布waypoint指令

t3-t10: TakeoffModule执行中（8秒）
    ↓
    [TakeoffModule] 定时器每0.2秒检查：
    ├─ 当前高度：0.5m → 继续
    ├─ 当前高度：1.2m → 继续
    ├─ 当前高度：2.1m → 继续
    ├─ 当前高度：2.8m → 继续
    ├─ 当前高度：2.95m → 误差0.05m，开始计时
    └─ 当前高度：2.97m → 稳定3秒 → self.succeed("Reached altitude")

t11: TakeoffModule完成
    ↓
    [Action Manager] 收到succeed()
    ├─ 发布event到 /mission_actions/events
    │   {"action": "takeoff", "outcome": "succeeded", "message": "..."}
    └─ 清理TakeoffModule（停止定时器）

t12: Mission Executor收到event
    ↓
    [Mission Executor] _event_callback()
    ├─ 解析outcome = "succeeded"
    ├─ 查找transitions["success"] = "fly_square"
    └─ _start_stage("fly_square")

t13: 开始下一个stage
    ↓
    [Mission Executor] _start_stage("fly_square")
    ├─ _current_stage = _stage_map["fly_square"]
    ├─ 发布params到 /mission_executor/action_params
    └─ 调用服务 /mission_actions/fly_to

... 循环继续 ...
```

### 关键代码位置

```python
# 1. 收到YAML后的解析
def _plan_callback(self, msg):
    plan = yaml.safe_load(msg.data)
    for stage_dict in plan["stages"]["stage_list"]:
        stage = StageSpec(**stage_dict)
        self._stage_map[stage.id] = stage  # 存入字典

    initial_stage = plan["stages"]["initial"]
    self._start_stage(initial_stage)  # 开始第一个stage

# 2. 开始某个stage
def _start_stage(self, stage_id):
    stage = self._stage_map[stage_id]
    self._current_stage = stage

    if stage.type == "TAKEOFF":
        # 调用 /mission_actions/takeoff 服务
        self._call_action_service("mission_actions/takeoff", stage.params)

# 3. 收到Action完成事件
def _event_callback(self, msg):
    event = json.loads(msg.data)
    outcome = event["outcome"]  # "succeeded" or "failed"

    if outcome == "succeeded":
        next_stage = self._current_stage.transitions["success"]
        self._start_stage(next_stage)  # 转移到下一个stage
```

---

## 🎬 Part 2: Action Manager (演员经纪人)

### 核心职责
**Action Manager = "经纪人"**
- 管理11个演员（Action Modules）
- 确保同一时间只有1个演员在台上（全局互斥）
- 接收导演指令，安排演员上场
- 演员演完后通知导演

### 内部数据结构

```python
class ActionManager:
    # 11个Action Module实例（演员名册）
    modules = {
        "takeoff": TakeoffModule(context),
        "hover": HoverModule(context),
        "fly_to": FlyToTargetModule(context),
        "track_target": TrackTargetModule(context),
        "search": SearchModule(context),
        "lost_hold": LostHoldModule(context),
        "inspect": InspectModule(context),
        "land": LandModule(context),
        "delivery": DeliveryModule(context),
        "search_area": SearchAreaModule(context),
        "avoidance": AvoidanceModule(context),
    }

    # 全局互斥锁：当前正在运行的module名称
    _current_active_module = None  # 例如："takeoff"

    # 活动的handle字典
    _active_handles = {
        "takeoff": ActionHandle(...)  # 只有当前运行的module有handle
    }
```

### 全局互斥机制（最关键！）

```
场景：从SEARCH切换到TRACK_TARGET
═════════════════════════════════════════════════════════

初始状态：
    _current_active_module = "search"
    SearchModule正在运行（定时器每0.1秒旋转yaw）

t0: Mission Executor检测到人，调用 /mission_actions/track_target
    ↓
    [Action Manager] _start_action("track_target", goal)

t1: 检查全局互斥
    ↓
    if _current_active_module is not None:  # "search" 存在！
        if _current_active_module != "track_target":  # 不是同一个
            # 🔒 冲突！必须先停止旧的
            old_module = "search"
            old_handle = _active_handles["search"]

            old_handle.cancel()  # ← 调用SearchModule.cancel()

t2: SearchModule收到cancel()
    ↓
    [SearchModule] cancel()方法执行：
    ├─ self.stop_timers()  # 停止定时器！不再旋转
    ├─ self.on_cancel()    # 清理资源
    └─ self._set_result(CANCELED)  # 设置结果

t3: 旧module已停止，启动新module
    ↓
    [Action Manager]
    ├─ module = self.modules["track_target"]  # 找到TrackTargetModule
    ├─ handle = module.start(goal)            # 启动它
    ├─ _active_handles["track_target"] = handle
    └─ _current_active_module = "track_target"  # 更新互斥锁

t4: TrackTargetModule开始运行
    ↓
    [TrackTargetModule] on_start()
    ├─ 启用控制器
    ├─ 启用NMPC（publish Bool(True) to /nmpc/enable）
    ├─ 订阅NMPC输出
    └─ 创建监控定时器

现在状态：
    _current_active_module = "track_target"
    TrackTargetModule正在运行（转发NMPC指令）
    SearchModule已完全停止（定时器已取消）
```

### 为什么需要全局互斥？

**没有互斥会怎样？**
```
❌ 灾难场景：两个module同时发布指令

[SearchModule定时器]    每0.1秒发布：
  → /drone/control/waypoint_command: [0, 0, 3.0]（原地悬停）
  → /drone/control/attitude_command: [0, 0, 0.5]（yaw=0.5rad）

[TrackTargetModule]     同时发布：
  → /drone/control/waypoint_command: [5, 2, 3.0]（跟随目标）
  → /drone/control/attitude_command: [0, 0, 1.2]（yaw=1.2rad）

[Waypoint Controller]   收到：
  t=0.00s: [0, 0, 3.0]   ← SearchModule
  t=0.01s: [5, 2, 3.0]   ← TrackTarget
  t=0.10s: [0, 0, 3.0]   ← SearchModule又来了！
  t=0.11s: [5, 2, 3.0]   ← TrackTarget又来了！

结果：控制器疯狂来回切换目标 → 无人机乱飞！
```

**有互斥后：**
```
✅ 安全场景：只有一个module发布指令

[SearchModule]          已停止（定时器取消）
  → 不再发布任何指令

[TrackTargetModule]     独占发布：
  → /drone/control/waypoint_command: [5, 2, 3.0]
  → /drone/control/attitude_command: [0, 0, 1.2]

[Waypoint Controller]   收到：
  t=0.00s: [5, 2, 3.0]   ← 只有TrackTarget
  t=0.10s: [5, 3, 3.0]   ← 只有TrackTarget
  t=0.20s: [5, 4, 3.0]   ← 只有TrackTarget

结果：控制器稳定跟踪目标 → 无人机平稳飞行！
```

### 关键代码位置

```python
# 1. 服务回调（Mission Executor调用服务触发）
def _srv_takeoff(self, request, response):
    goal = TakeoffGoal(target_altitude=3.0)
    started = self._start_action("takeoff", goal)
    response.success = started
    return response

# 2. 启动action的核心逻辑（全局互斥实现）
def _start_action(self, name, goal):
    # 🔒 全局互斥检查
    if self._current_active_module is not None:
        if self._current_active_module != name:
            # 停止旧的module
            old_name = self._current_active_module
            old_handle = self._active_handles[old_name]
            old_handle.cancel()  # 立即取消
            old_handle.result(timeout=0.5)  # 等待清理

    # 启动新的module
    module = self.modules[name]
    handle = module.start(goal)

    self._active_handles[name] = handle
    self._current_active_module = name  # 更新互斥锁

    # 注册完成回调
    handle.future.add_done_callback(
        lambda fut: self._on_action_done(name, fut)
    )
    return True

# 3. module完成时的回调
def _on_action_done(self, name, future):
    result = future.result()

    # 发布event给Mission Executor
    event = {
        "action": name,
        "outcome": result.outcome,  # "succeeded" or "failed"
        "message": result.message
    }
    self.event_pub.publish(json.dumps(event))

    # 清理
    self._active_handles.pop(name)
    self._current_active_module = None  # 释放互斥锁
```

---

## 🎭 Part 3: Action Module (演员本身)

### 核心职责
**Action Module = "演员"**
- 专注做一件事（起飞、降落、跟踪等）
- 使用ActionContext发布控制指令
- 完成后报告结果（成功/失败）

### TakeoffModule完整示例

```python
class TakeoffModule(ActionModule):
    def __init__(self, context):
        super().__init__(context, "TakeoffModule")
        self._target = None
        self._timeout = 60.0
        self._start_time = 0.0

    # ────────────────────────────────────────────────
    # 1. Action Manager调用 module.start(goal) 时触发
    # ────────────────────────────────────────────────
    def on_start(self, goal):
        # 获取当前位置
        position = self.context.get_position()  # [0, 0, 0.5]

        # 计算目标位置（保持XY，改变Z）
        target_altitude = goal.target_altitude or 3.0
        self._target = np.array([position[0], position[1], target_altitude])
        # self._target = [0, 0, 3.0]

        # 启用控制器
        self.context.enable_waypoint_control(True)
        self.context.enable_yaw_control(True)

        # 发送初始指令
        self.context.send_waypoint(self._target)
        self.context.send_yaw(0, 0, 0)

        # 创建定时器（每0.2秒检查一次）
        self.create_timer(0.2, self._monitor_altitude)
        #               └─ 创建ROS2定时器，回调函数是_monitor_altitude

    # ────────────────────────────────────────────────
    # 2. 定时器每0.2秒调用这个函数
    # ────────────────────────────────────────────────
    def _monitor_altitude(self):
        position = self.context.get_position()
        altitude_error = abs(position[2] - self._target[2])

        # 检查是否到达目标
        if altitude_error <= 0.1:  # 误差小于10cm
            if self._stable_start_time is None:
                self._stable_start_time = self.context.now()

            stable_duration = self.context.now() - self._stable_start_time
            if stable_duration >= 3.0:  # 稳定3秒
                # ✅ 成功！
                self.succeed("Reached altitude 3.0m")
                # 这会触发：
                # 1. 停止定时器
                # 2. 设置ActionResult(SUCCEEDED)
                # 3. 触发ActionHandle的future回调
                # 4. Action Manager发布event给Mission Executor
                return
        else:
            self._stable_start_time = None

        # 定期刷新waypoint（避免控制器超时）
        self.context.send_waypoint(self._target)
```

### ActionContext（共享资源）

```python
class ActionContext:
    """所有Action Module共享的资源"""

    def __init__(self, node):
        self.node = node
        self.state = VehicleState()  # 当前位置、速度、yaw

        # ROS2 Publishers（所有module共用）
        self._waypoint_pub = node.create_publisher(
            PoseStamped, "/drone/control/waypoint_command", 10
        )
        self._yaw_pub = node.create_publisher(
            Vector3Stamped, "/drone/control/attitude_command", 10
        )

    def get_position(self):
        return self.state.position  # [x, y, z]

    def send_waypoint(self, position):
        msg = PoseStamped()
        msg.pose.position.x = position[0]
        msg.pose.position.y = position[1]
        msg.pose.position.z = position[2]
        self._waypoint_pub.publish(msg)

    def send_yaw(self, roll, pitch, yaw):
        msg = Vector3Stamped()
        msg.vector.x = roll
        msg.vector.y = pitch
        msg.vector.z = yaw
        self._yaw_pub.publish(msg)
```

---

## 📊 完整消息流（从YAML到无人机动作）

```
用户编写YAML → Mission Executor → Action Manager → Action Module → Controller → Drone
═══════════════════════════════════════════════════════════════════════════════════════

Step 1: 用户准备YAML
┌────────────────────────────────────────┐
│ mission:                               │
│   stages:                              │
│     - id: "takeoff"                    │
│       type: "TAKEOFF"                  │
│       params: {target_altitude: 3.0}   │
│       transitions: {success: "land"}   │
└────────────────────────────────────────┘
         │ publish to /mission_executor/plan
         ↓

Step 2: Mission Executor解析
┌────────────────────────────────────────┐
│ Mission Executor                       │
│ ├─ 解析YAML                            │
│ ├─ 创建_stage_map                      │
│ └─ _start_stage("takeoff")            │
│     ├─ publish params                  │
│     └─ call /mission_actions/takeoff   │
└────────────────────────────────────────┘
         │ ROS2 service call
         ↓

Step 3: Action Manager接收服务调用
┌────────────────────────────────────────┐
│ Action Manager                         │
│ ├─ _srv_takeoff() 被调用               │
│ ├─ 解析params创建goal                  │
│ └─ _start_action("takeoff", goal)     │
│     ├─ 检查全局互斥                     │
│     ├─ modules["takeoff"].start(goal)  │
│     └─ _current_active_module="takeoff"│
└────────────────────────────────────────┘
         │ module.start()
         ↓

Step 4: TakeoffModule执行
┌────────────────────────────────────────┐
│ TakeoffModule                          │
│ ├─ on_start(goal)                      │
│ │   ├─ 计算target=[0,0,3.0]            │
│ │   ├─ context.enable_waypoint()       │
│ │   ├─ context.send_waypoint(target)   │
│ │   └─ create_timer(0.2, _monitor)     │
│ │                                       │
│ └─ _monitor_altitude() [每0.2秒]       │
│     ├─ 检查高度误差                     │
│     ├─ 如果到达且稳定3秒                │
│     └─ self.succeed("Reached...")      │
└────────────────────────────────────────┘
         │ publish waypoint commands
         ↓

Step 5: Controller处理指令
┌────────────────────────────────────────┐
│ Waypoint Controller                    │
│ ├─ 收到waypoint: [0, 0, 3.0]          │
│ ├─ PID计算速度: [0, 0, 0.8]           │
│ └─ 发布到 /cmd_vel                     │
└────────────────────────────────────────┘
         │ velocity command
         ↓

Step 6: Gazebo执行
┌────────────────────────────────────────┐
│ Gazebo MulticopterVelocityControl      │
│ ├─ 收到cmd_vel: [0, 0, 0.8]           │
│ ├─ 计算推力                            │
│ └─ 更新无人机物理状态                   │
└────────────────────────────────────────┘
         │ physics update
         ↓
      无人机上升

Step 7: 任务完成后
┌────────────────────────────────────────┐
│ TakeoffModule                          │
│ └─ self.succeed()                      │
└────────────────────────────────────────┘
         │ ActionResult set
         ↓
┌────────────────────────────────────────┐
│ Action Manager                         │
│ ├─ _on_action_done() 回调触发          │
│ ├─ publish event to                    │
│ │   /mission_actions/events            │
│ │   {action:"takeoff",outcome:"succeeded"}
│ └─ _current_active_module = None      │
└────────────────────────────────────────┘
         │ event message
         ↓
┌────────────────────────────────────────┐
│ Mission Executor                       │
│ ├─ _event_callback() 收到event         │
│ ├─ outcome = "succeeded"               │
│ ├─ next = transitions["success"]="land"│
│ └─ _start_stage("land")                │
└────────────────────────────────────────┘
         │ 循环继续...
         ↓
```

---

## 🔑 关键概念总结

### 1. Mission Executor = 状态机
- **输入**：YAML mission plan
- **状态**：当前stage ID（takeoff, fly_square, land...）
- **转移条件**：action outcome（succeeded, failed, timeout）
- **输出**：调用Action Manager服务

### 2. Action Manager = 资源管理器
- **管理对象**：11个Action Module实例
- **核心机制**：全局互斥（`_current_active_module`）
- **输入**：ROS2服务调用（/mission_actions/xxx）
- **输出**：events（action结果）

### 3. Action Module = 执行单元
- **职责**：完成一个原子任务（起飞、降落、跟踪...）
- **工具**：ActionContext（发布控制指令）
- **生命周期**：start() → 运行定时器 → succeed()/fail()

### 4. 数据流向

```
YAML
  ↓
Mission Executor（状态机，决策）
  ↓ ROS2 service call
Action Manager（资源管理，互斥）
  ↓ start()
Action Module（执行任务）
  ↓ publish
Controller（PID控制）
  ↓ cmd_vel
Drone（物理运动）
  ↓ feedback odometry
Action Module（检查完成）
  ↓ succeed()
Action Manager（发送event）
  ↓ event
Mission Executor（状态转移）
  ↓ 循环
```

---

## ❓ 常见疑问

### Q1: 为什么需要两层（Executor + Manager）？
**A**: 职责分离
- **Executor**：只管逻辑（"现在该做什么"）
- **Manager**：只管资源（"怎么安全地切换演员"）

### Q2: 全局互斥会不会太严格？
**A**: 这是设计选择
- **优点**：绝对安全，不会有指令冲突
- **缺点**：不能同时做两件事（但对无人机来说，这是好事！）

### Q3: 如果我想同时hover和rotate怎么办？
**A**: 创建一个新的HoverAndRotateModule
- 不要试图同时运行HoverModule和SearchModule
- 全局互斥保证了控制指令的一致性

### Q4: timeout后为什么LandModule不失败了？
**A**: 现在改成了"持续尝试"模式
- 超时只是警告，继续下降
- 只有真正着陆（高度≤0.15m）才succeed
- 如果真的需要中断，用stage-level timeout

---

希望这个图解能帮你理清思路！🎉
