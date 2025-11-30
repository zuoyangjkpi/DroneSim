# Action Module: Timer机制和Hooks详解

## 🕐 Part 1: create_timer() 详解

### 什么是Timer？

**Timer = 定时器 = 周期性执行的函数**

在ROS2中，Timer是一个会**定期自动调用回调函数**的对象。

---

### create_timer()的工作原理

#### 基础概念

```python
# 创建一个定时器：每0.2秒执行一次_monitor_altitude函数
self.create_timer(0.2, self._monitor_altitude)
#                 │    └─ 回调函数（callback）
#                 └─ 周期（period，单位：秒）
```

#### ROS2 Timer的本质

```
创建Timer后：

t=0.0s   → ROS2内部启动定时器
t=0.2s   → 自动调用 _monitor_altitude()
t=0.4s   → 自动调用 _monitor_altitude()
t=0.6s   → 自动调用 _monitor_altitude()
t=0.8s   → 自动调用 _monitor_altitude()
...
持续执行，直到timer被cancel()
```

---

### ActionModule中的create_timer()实现

#### 代码位置：action_base.py

```python
class ActionModule:
    def __init__(self, context, name):
        self.context = context
        self.name = name
        self._timers = []  # 存储所有创建的timer

    # ────────────────────────────────────────────────────────
    # create_timer() - 创建并记录timer
    # ────────────────────────────────────────────────────────
    def create_timer(self, period: float, callback: Callable) -> Any:
        """创建ROS2定时器，并记录到_timers列表"""

        # Step 1: 调用ROS2 Node的create_timer()创建真正的timer
        timer = self.context.node.create_timer(period, callback)
        #                          └─ ROS2的Timer对象

        # Step 2: 记录到_timers列表（重要！用于后续取消）
        self._timers.append(timer)

        # Step 3: 返回timer对象
        return timer

    # ────────────────────────────────────────────────────────
    # stop_timers() - 停止所有timer
    # ────────────────────────────────────────────────────────
    def stop_timers(self) -> None:
        """取消所有已创建的timer"""

        for timer in self._timers:
            timer.cancel()  # 取消定时器（停止调用回调函数）

        self._timers.clear()  # 清空列表
```

#### 为什么要包装ROS2的create_timer？

**目的：统一管理和自动清理**

```
❌ 直接使用ROS2的create_timer：
    timer1 = self.context.node.create_timer(0.1, callback1)
    timer2 = self.context.node.create_timer(0.2, callback2)
    timer3 = self.context.node.create_timer(0.5, callback3)

    问题：module被cancel时，需要手动记住并取消所有timer
          容易遗漏 → timer继续运行 → 资源泄漏！

✅ 使用ActionModule的create_timer：
    self.create_timer(0.1, callback1)  # 自动记录到_timers
    self.create_timer(0.2, callback2)  # 自动记录到_timers
    self.create_timer(0.5, callback3)  # 自动记录到_timers

    优点：module被cancel时，调用self.stop_timers()
          → 自动取消所有timer → 不会泄漏！
```

---

### 实际使用示例：TakeoffModule

```python
class TakeoffModule(ActionModule):
    def on_start(self, goal):
        # ... 初始化代码 ...

        # 创建定时器：每0.2秒检查一次高度
        self.create_timer(0.2, self._monitor_altitude)
        #                 │    └─ 回调函数
        #                 └─ 每0.2秒 = 5 Hz

    def _monitor_altitude(self):
        """这个函数会被定时器每0.2秒自动调用一次"""

        position = self.context.get_position()
        altitude_error = abs(position[2] - self._target[2])

        if altitude_error <= 0.1:
            # 成功！
            self.succeed("Reached altitude")
            # 注意：succeed()内部会调用stop_timers()
            # 所以定时器会被自动停止
```

#### 执行时间线

```
t=0.0s  → TakeoffModule.on_start() 被调用
          ├─ 设置目标高度：self._target = [0, 0, 3.0]
          ├─ 发送waypoint指令
          └─ create_timer(0.2, self._monitor_altitude)
                └─ ROS2开始定时器

t=0.2s  → _monitor_altitude() 第1次被自动调用
          ├─ 当前高度：0.5m
          ├─ 误差：2.5m > 0.1m
          └─ 继续等待

t=0.4s  → _monitor_altitude() 第2次被自动调用
          ├─ 当前高度：1.0m
          ├─ 误差：2.0m > 0.1m
          └─ 继续等待

t=0.6s  → _monitor_altitude() 第3次被自动调用
          ├─ 当前高度：1.5m
          ├─ 误差：1.5m > 0.1m
          └─ 继续等待

... （每0.2秒重复）...

t=8.0s  → _monitor_altitude() 第40次被自动调用
          ├─ 当前高度：2.98m
          ├─ 误差：0.02m <= 0.1m（稳定3秒后）
          ├─ 调用：self.succeed("Reached altitude")
          │        └─ 内部调用：self.stop_timers()
          │                    └─ timer.cancel()
          └─ 定时器停止！

t=8.2s  → _monitor_altitude() 不再被调用（timer已取消）
```

---

### 多个Timer的例子

```python
class ComplexModule(ActionModule):
    def on_start(self, goal):
        # 创建多个定时器，用于不同的监控任务

        # Timer 1: 每0.1秒检查一次位置
        self.create_timer(0.1, self._check_position)

        # Timer 2: 每0.5秒检查一次传感器
        self.create_timer(0.5, self._check_sensors)

        # Timer 3: 每1.0秒发送心跳
        self.create_timer(1.0, self._send_heartbeat)

        # 所有timer都被记录到self._timers列表：
        # self._timers = [timer1, timer2, timer3]

    def _check_position(self):
        # 每0.1秒执行
        pass

    def _check_sensors(self):
        # 每0.5秒执行
        pass

    def _send_heartbeat(self):
        # 每1.0秒执行
        pass

    def cancel(self):
        # 当module被cancel时，自动停止所有timer
        self.stop_timers()
        # → timer1.cancel()
        # → timer2.cancel()
        # → timer3.cancel()
```

---

## 🪝 Part 2: Subclass Hooks 详解

### 什么是Hook？

**Hook = 钩子 = 预留的扩展点**

在面向对象编程中，Hook是**父类定义的、供子类重写的方法**。

---

### ActionModule中的Hooks

#### 代码位置：action_base.py

```python
class ActionModule:
    # ════════════════════════════════════════════════════════
    # PUBLIC API (由Action Manager调用)
    # ════════════════════════════════════════════════════════

    def start(self, goal) -> ActionHandle:
        """启动module（由Action Manager调用）"""
        self._handle = ActionHandle(self)
        self._goal = goal
        self._active = True

        # 🪝 调用子类的hook
        self.on_start(goal)  # ← 子类必须实现这个方法

        return self._handle

    def cancel(self) -> None:
        """取消module（由Action Manager调用）"""
        self.stop_timers()  # 停止所有定时器

        # 🪝 调用子类的hook
        self.on_cancel()  # ← 子类可以选择实现这个方法

        self._set_result(ActionOutcome.CANCELED, "canceled by request")

    # ════════════════════════════════════════════════════════
    # HOOKS FOR SUBCLASSES (子类必须/可以实现的方法)
    # ════════════════════════════════════════════════════════

    def on_start(self, goal: Any) -> None:
        """
        🪝 HOOK: 子类必须实现这个方法
        当module被启动时调用
        """
        raise NotImplementedError
        # ↑ 抛出异常，强制子类实现

    def on_cancel(self) -> None:
        """
        🪝 HOOK: 子类可以选择实现这个方法
        当module被取消时调用
        默认实现：停止定时器
        """
        self.stop_timers()  # 默认行为
        # 子类可以重写这个方法来添加额外的清理逻辑
```

---

### Hook的设计模式

#### 模板方法模式（Template Method Pattern）

```
父类定义流程框架：

class ActionModule:
    def start(self, goal):
        # 1. 通用的初始化（所有module都需要）
        self._handle = ActionHandle(self)
        self._active = True

        # 2. 🪝 调用子类的特定逻辑
        self.on_start(goal)  # ← Hook

        # 3. 通用的收尾（所有module都需要）
        return self._handle

子类只需要实现特定逻辑：

class TakeoffModule(ActionModule):
    def on_start(self, goal):
        # 只需要关心"起飞"这个特定任务的逻辑
        self._target = calculate_target(goal)
        self.create_timer(0.2, self._monitor_altitude)
```

#### 为什么要用Hook而不是直接重写start()？

**好处：封装通用逻辑，避免重复代码**

```
❌ 不使用Hook，每个子类都要重复：

class TakeoffModule(ActionModule):
    def start(self, goal):
        # 😫 每个子类都要写这些通用代码
        self._handle = ActionHandle(self)
        self._goal = goal
        self._active = True

        # ✅ 这才是特定逻辑
        self._target = ...
        self.create_timer(...)

        # 😫 每个子类都要写这些通用代码
        return self._handle

class LandModule(ActionModule):
    def start(self, goal):
        # 😫 又要重复一遍
        self._handle = ActionHandle(self)
        self._goal = goal
        self._active = True

        # ✅ 这才是特定逻辑
        self._target = ...
        self.create_timer(...)

        # 😫 又要重复一遍
        return self._handle


✅ 使用Hook，通用代码只在父类写一次：

class ActionModule:
    def start(self, goal):
        # ✅ 通用代码只在这里
        self._handle = ActionHandle(self)
        self._goal = goal
        self._active = True

        # 🪝 调用子类的hook
        self.on_start(goal)

        # ✅ 通用代码只在这里
        return self._handle

class TakeoffModule(ActionModule):
    def on_start(self, goal):
        # 😊 只写特定逻辑
        self._target = ...
        self.create_timer(...)

class LandModule(ActionModule):
    def on_start(self, goal):
        # 😊 只写特定逻辑
        self._target = ...
        self.create_timer(...)
```

---

### 实际使用示例

#### 示例1: on_start() Hook

```python
class TakeoffModule(ActionModule):
    def __init__(self, context):
        super().__init__(context, "TakeoffModule")
        # 初始化实例变量

    # ═══════════════════════════════════════════════════════
    # 🪝 实现on_start() hook
    # ═══════════════════════════════════════════════════════
    def on_start(self, goal: TakeoffGoal) -> None:
        """
        当Action Manager调用module.start(goal)时，
        父类的start()方法会调用这个hook
        """

        # 1. 获取当前位置
        position = self.context.get_position()

        # 2. 计算目标位置
        altitude = goal.target_altitude or 3.0
        self._target = np.array([position[0], position[1], altitude])

        # 3. 启用控制器
        self.context.enable_waypoint_control(True)

        # 4. 发送初始指令
        self.context.send_waypoint(self._target)

        # 5. 创建定时器
        self.create_timer(0.2, self._monitor_altitude)
        # 这个函数执行完毕后，返回到父类的start()方法
```

#### 示例2: on_cancel() Hook（可选重写）

```python
class TrackTargetModule(ActionModule):
    # ═══════════════════════════════════════════════════════
    # 🪝 重写on_cancel() hook（添加额外清理逻辑）
    # ═══════════════════════════════════════════════════════
    def on_cancel(self) -> None:
        """
        当module被取消时，需要额外清理NMPC
        """
        self.context.node.get_logger().info(
            "[TrackTargetModule] Canceling - disabling NMPC"
        )

        # 1. 禁用NMPC（特定于TrackTarget的清理）
        self._set_nmpc_enabled(False)

        # 2. 调用父类的默认实现（停止定时器）
        super().on_cancel()
        # 这会调用ActionModule.on_cancel() → stop_timers()
```

---

### Hook调用链

#### 正常启动流程

```
[Action Manager]
     ↓
module.start(goal)
     ↓
[ActionModule.start()] ← 父类方法
     ├─ 1. self._handle = ActionHandle(self)
     ├─ 2. self._active = True
     ├─ 3. self.on_start(goal) ← 🪝 调用子类的hook
     │        ↓
     │   [TakeoffModule.on_start()] ← 子类实现
     │        ├─ 计算目标
     │        ├─ 启用控制器
     │        └─ 创建定时器
     │        ↓ (返回)
     └─ 4. return self._handle
          ↓
[Action Manager] 获得handle
```

#### 取消流程

```
[Action Manager]
     ↓
module.cancel()
     ↓
[ActionModule.cancel()] ← 父类方法
     ├─ 1. self.stop_timers()
     ├─ 2. self.on_cancel() ← 🪝 调用子类的hook
     │        ↓
     │   [TrackTargetModule.on_cancel()] ← 子类重写
     │        ├─ 禁用NMPC
     │        └─ super().on_cancel() → stop_timers()
     │        ↓ (返回)
     └─ 3. self._set_result(CANCELED)
          ↓
[Action Manager] 收到canceled结果
```

---

## 🎯 完整生命周期示例

### TakeoffModule完整流程

```
═══════════════════════════════════════════════════════════
时间线：TakeoffModule从启动到完成
═══════════════════════════════════════════════════════════

[t0] Action Manager调用
     ├─ module = self.modules["takeoff"]
     └─ handle = module.start(goal)
          ↓

[t0.1] ActionModule.start() 执行（父类方法）
       ├─ self._handle = ActionHandle(self)
       ├─ self._goal = goal
       ├─ self._active = True
       ├─ self.on_start(goal) ← 🪝 调用子类hook
       │    ↓
       │  [t0.2] TakeoffModule.on_start() 执行
       │         ├─ position = self.context.get_position()  # [0, 0, 0.5]
       │         ├─ self._target = [0, 0, 3.0]
       │         ├─ self.context.enable_waypoint_control(True)
       │         ├─ self.context.send_waypoint(self._target)
       │         └─ self.create_timer(0.2, self._monitor_altitude)
       │              ├─ timer = context.node.create_timer(0.2, callback)
       │              ├─ self._timers.append(timer)  # 记录timer
       │              └─ return timer
       │         ↓ (on_start执行完毕)
       └─ return self._handle
            ↓

[t0.3] Action Manager获得handle
       ├─ self._active_handles["takeoff"] = handle
       └─ self._current_active_module = "takeoff"

─────────────────────────────────────────────────────────

[t0.5] ROS2 Timer开始工作
       每0.2秒自动调用_monitor_altitude()

[t0.7] _monitor_altitude() 第1次调用
       ├─ 高度：0.8m，误差2.2m
       └─ 继续等待

[t0.9] _monitor_altitude() 第2次调用
       ├─ 高度：1.1m，误差1.9m
       └─ 继续等待

... (每0.2秒重复) ...

[t8.0] _monitor_altitude() 第40次调用
       ├─ 高度：2.98m，误差0.02m
       ├─ 稳定3秒
       └─ self.succeed("Reached altitude")
            ↓
          [ActionModule.succeed()] 执行（父类方法）
            ├─ self._set_result(SUCCEEDED, "Reached altitude")
            │    ├─ self._handle.future.set_result(result)
            │    └─ self.stop_timers() ← 停止所有timer
            │         ├─ for timer in self._timers:
            │         │      timer.cancel() ← 取消定时器！
            │         └─ self._timers.clear()
            ├─ self._active = False
            └─ self._goal = None

─────────────────────────────────────────────────────────

[t8.1] Action Manager的future回调触发
       ├─ _on_action_done("takeoff", future)
       ├─ result = future.result()  # ActionResult(SUCCEEDED)
       ├─ 发布event到 /mission_actions/events
       └─ _current_active_module = None

─────────────────────────────────────────────────────────

[t8.2] Timer不再触发
       _monitor_altitude() 不会再被调用（已被cancel）

═══════════════════════════════════════════════════════════
```

---

## 🔧 对比：有Timer vs 无Timer

### 无Timer的方案（轮询，效率低）

```python
# ❌ 不好的设计
class TakeoffModuleWithoutTimer(ActionModule):
    def on_start(self, goal):
        self._target = calculate_target(goal)
        self.context.send_waypoint(self._target)

        # 阻塞等待（不好！）
        while True:
            position = self.context.get_position()
            if reached_target(position):
                self.succeed()
                break
            time.sleep(0.2)  # 阻塞整个程序！
```

**问题**：
- 阻塞ROS2事件循环
- 无法响应其他事件
- 无法被cancel

### 有Timer的方案（事件驱动，高效）

```python
# ✅ 好的设计
class TakeoffModule(ActionModule):
    def on_start(self, goal):
        self._target = calculate_target(goal)
        self.context.send_waypoint(self._target)

        # 创建定时器（非阻塞！）
        self.create_timer(0.2, self._monitor_altitude)

    def _monitor_altitude(self):
        # 每0.2秒被自动调用
        position = self.context.get_position()
        if reached_target(position):
            self.succeed()  # 自动停止timer
```

**优点**：
- 非阻塞，不影响ROS2事件循环
- 可以随时被cancel
- 可以同时处理多个事件
- 符合ROS2的异步编程模型

---

## 📝 总结

### create_timer()
- **本质**：创建ROS2定时器的包装方法
- **作用**：周期性执行回调函数
- **好处**：自动记录timer，方便统一清理
- **用法**：`self.create_timer(period, callback)`

### Subclass Hooks
- **本质**：父类预留的扩展点
- **作用**：让子类实现特定逻辑，父类控制整体流程
- **好处**：避免重复代码，统一管理生命周期
- **主要hooks**：
  - `on_start(goal)` - 必须实现，启动逻辑
  - `on_cancel()` - 可选实现，清理逻辑

### 设计模式
- **模板方法模式**：父类定义框架，子类填充细节
- **观察者模式**：Timer定期观察状态变化
- **策略模式**：不同module实现不同的on_start策略

希望这个详解能帮你完全理解Timer和Hooks的机制！🎉
