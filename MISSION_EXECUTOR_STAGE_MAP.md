# Mission Executor Stage Map 详解

## 🗺️ Stage Map 核心概念

**Stage Map = 任务图谱**
- 存储所有stage的字典（Dict）
- Key = stage ID（字符串）
- Value = StageSpec对象（包含type、params、transitions）

---

## 📊 Part 1: 数据结构详解

### 从YAML到Stage Map

**你的YAML文件**（`~/.ros/manual_mission_plan.yaml`）：
```yaml
mission:
  name: "square_flight"
stages:
  initial: "takeoff"
  stage_list:
    - id: "takeoff"
      type: "TAKEOFF"
      params: {target_altitude: null}
      transitions: {success: "fly_to_start", failure: "abort"}
      timeout: null

    - id: "fly_to_start"
      type: "FLY_TO"
      params: {waypoints: [[0.0, 0.0, 3.0]]}
      transitions: {success: "fly_square", failure: "abort"}

    - id: "fly_square"
      type: "FLY_TO"
      params: {waypoints: [[6,0,3], [6,6,3], [0,6,3]]}
      transitions: {success: "fly_back", failure: "abort"}

    - id: "fly_back"
      type: "FLY_TO"
      params: {waypoints: [[6,0,3], [0,0,3]]}
      transitions: {success: "land", failure: "abort"}

    - id: "land"
      type: "LAND_AT_POINT"
      transitions: {success: "complete", failure: "abort"}

    - id: "abort"
      type: "ABORT_MISSION"
      params: {reason: "mission_failed"}

    - id: "complete"
      type: "TERMINAL"
```

### 解析后的Stage Map（Python数据结构）

```python
_stage_map = {
    # ────────────────────────────────────────────────────────
    "takeoff": StageSpec(
        id="takeoff",
        stage_type="TAKEOFF",
        params={"target_altitude": None},
        transitions={
            "success": "fly_to_start",
            "failure": "abort"
        },
        timeout=None
    ),

    # ────────────────────────────────────────────────────────
    "fly_to_start": StageSpec(
        id="fly_to_start",
        stage_type="FLY_TO",
        params={"waypoints": [[0.0, 0.0, 3.0]]},
        transitions={
            "success": "fly_square",
            "failure": "abort"
        },
        timeout=None
    ),

    # ────────────────────────────────────────────────────────
    "fly_square": StageSpec(
        id="fly_square",
        stage_type="FLY_TO",
        params={"waypoints": [[6,0,3], [6,6,3], [0,6,3]]},
        transitions={
            "success": "fly_back",
            "failure": "abort"
        },
        timeout=None
    ),

    # ────────────────────────────────────────────────────────
    "fly_back": StageSpec(
        id="fly_back",
        stage_type="FLY_TO",
        params={"waypoints": [[6,0,3], [0,0,3]]},
        transitions={
            "success": "land",
            "failure": "abort"
        },
        timeout=None
    ),

    # ────────────────────────────────────────────────────────
    "land": StageSpec(
        id="land",
        stage_type="LAND_AT_POINT",
        params={},
        transitions={
            "success": "complete",
            "failure": "abort"
        },
        timeout=None
    ),

    # ────────────────────────────────────────────────────────
    "abort": StageSpec(
        id="abort",
        stage_type="ABORT_MISSION",
        params={"reason": "mission_failed"},
        transitions={},
        timeout=None
    ),

    # ────────────────────────────────────────────────────────
    "complete": StageSpec(
        id="complete",
        stage_type="TERMINAL",
        params={},
        transitions={},
        timeout=None
    ),
}
```

### StageSpec类定义

```python
@dataclass
class StageSpec:
    id: str                           # stage唯一标识
    stage_type: str                   # 类型：TAKEOFF, FLY_TO, LAND等
    params: Dict[str, Any]            # 参数字典
    transitions: Dict[str, str]       # 转换字典：outcome_key → next_stage_id
    timeout: Optional[float] = None   # 超时（秒）
```

---

## 🎯 Part 2: Stage Map构建过程

### 代码流程（从接收YAML到构建Map）

```python
# mission_executor_node.py

def _plan_callback(self, msg: String):
    """收到YAML mission plan"""

    # Step 1: 解析YAML字符串
    plan = yaml.safe_load(msg.data)
    # plan = {
    #     "mission": {"name": "square_flight", ...},
    #     "stages": {
    #         "initial": "takeoff",
    #         "stage_list": [...]
    #     }
    # }

    # Step 2: 清空旧的stage map
    self._stage_map.clear()

    # Step 3: 遍历stage_list，构建stage_map
    for stage_dict in plan["stages"]["stage_list"]:
        # stage_dict = {
        #     "id": "takeoff",
        #     "type": "TAKEOFF",
        #     "params": {...},
        #     "transitions": {...}
        # }

        # 创建StageSpec对象
        stage = StageSpec(
            id=stage_dict["id"],
            stage_type=stage_dict["type"],
            params=stage_dict.get("params", {}),
            transitions=stage_dict.get("transitions", {}),
            timeout=stage_dict.get("timeout")
        )

        # 存入字典（Key = stage ID）
        self._stage_map[stage.id] = stage

    # Step 4: 获取初始stage ID
    initial_stage_id = plan["stages"]["initial"]  # "takeoff"

    # Step 5: 开始执行第一个stage
    self._start_stage(initial_stage_id)
```

### 可视化构建过程

```
YAML文本
  ↓ yaml.safe_load()
Python字典
  ↓ 遍历stage_list
Stage对象列表
  ↓ 存入_stage_map[id]
Stage Map字典

最终结果：
_stage_map = {
    "takeoff": StageSpec(...),
    "fly_to_start": StageSpec(...),
    "fly_square": StageSpec(...),
    "fly_back": StageSpec(...),
    "land": StageSpec(...),
    "abort": StageSpec(...),
    "complete": StageSpec(...),
}
```

---

## 🔄 Part 3: 状态转换逻辑（核心！）

### 转换机制概览

```
当前状态：_current_stage = _stage_map["takeoff"]
         StageSpec(id="takeoff", transitions={"success": "fly_to_start", ...})

动作完成：收到event = {"outcome": "succeeded"}

转换逻辑：
1. outcome_key = map_outcome_to_key("succeeded")  → "success"
2. next_stage_id = _current_stage.transitions["success"]  → "fly_to_start"
3. _start_stage("fly_to_start")

新状态：_current_stage = _stage_map["fly_to_start"]
```

### 完整转换流程（以takeoff→fly_to_start为例）

```
时间线：执行takeoff stage，成功后转到fly_to_start
═══════════════════════════════════════════════════════════

t0: Mission Executor启动takeoff stage
    ↓
    _current_stage = _stage_map["takeoff"]
    _current_stage = StageSpec(
        id="takeoff",
        stage_type="TAKEOFF",
        transitions={"success": "fly_to_start", "failure": "abort"}
    )

t1: 调用Action Manager服务
    ↓
    call /mission_actions/takeoff

t2-t10: TakeoffModule执行中（8秒）
    ↓
    无人机从0m爬升到3m

t10: TakeoffModule完成
    ↓
    Action Manager发布event：
    {
        "action": "takeoff",
        "outcome": "succeeded",
        "message": "Reached altitude 3.0m"
    }

t11: Mission Executor收到event
    ↓
    def _event_callback(self, msg):
        event = json.loads(msg.data)
        outcome = event["outcome"]  # "succeeded"

        # 🔑 关键：将outcome映射到transition key
        if outcome == "succeeded":
            outcome_key = "success"
        elif outcome == "failed":
            outcome_key = "failure"
        elif outcome == "canceled":
            outcome_key = "canceled"

        # 执行转换
        self._transition_from_stage(outcome_key)

t12: 执行转换逻辑
    ↓
    def _transition_from_stage(self, outcome_key):
        # outcome_key = "success"

        # 从当前stage的transitions中查找下一个stage
        transitions = self._current_stage.transitions
        # transitions = {"success": "fly_to_start", "failure": "abort"}

        next_stage_id = transitions.get(outcome_key)
        # next_stage_id = "fly_to_start"

        if next_stage_id is None:
            # 没有定义这个outcome的转换，任务结束
            self.get_logger().warn(f"No transition for '{outcome_key}'")
            self._mission_active = False
            return

        # 记录转换
        self.get_logger().info(
            f"Stage {self._current_stage.id} → {next_stage_id} "
            f"via '{outcome_key}'"
        )

        # 启动下一个stage
        self._start_stage(next_stage_id)

t13: 启动下一个stage
    ↓
    _start_stage("fly_to_start")
    _current_stage = _stage_map["fly_to_start"]
    call /mission_actions/fly_to

... 循环继续 ...
```

---

## 🗺️ Part 4: 状态转换图（State Transition Diagram）

### 你的square_flight任务的完整状态图

```
                    开始
                     ↓
              ┌────────────┐
              │  takeoff   │
              │ (TAKEOFF)  │
              └─────┬──────┘
                    │
            success │     failure
                    ↓         ↘
          ┌──────────────┐     ┌───────┐
          │fly_to_start  │     │ abort │
          │  (FLY_TO)    │     └───────┘
          └──────┬───────┘         ↑
                 │                 │
         success │     failure ────┘
                 ↓
          ┌────────────┐
          │ fly_square │
          │  (FLY_TO)  │
          └─────┬──────┘
                │
        success │     failure
                ↓         ↘
          ┌───────────┐    │
          │ fly_back  │    │
          │ (FLY_TO)  │    │
          └─────┬─────┘    │
                │          │
        success │  failure │
                ↓          ↓
          ┌──────────┐  ┌───────┐
          │   land   │  │ abort │
          │ (LAND)   │  └───────┘
          └─────┬────┘
                │
        success │     failure
                ↓         ↘
          ┌──────────┐  ┌───────┐
          │ complete │  │ abort │
          │(TERMINAL)│  └───────┘
          └──────────┘
```

### 用表格表示所有转换

| 当前Stage | Outcome | 转换Key | 下一Stage | 说明 |
|-----------|---------|---------|-----------|------|
| takeoff | succeeded | success | fly_to_start | 起飞成功→飞到起点 |
| takeoff | failed | failure | abort | 起飞失败→中止任务 |
| fly_to_start | succeeded | success | fly_square | 到达起点→开始飞方形 |
| fly_to_start | failed | failure | abort | 飞行失败→中止 |
| fly_square | succeeded | success | fly_back | 方形飞完→飞回原点 |
| fly_square | failed | failure | abort | 飞行失败→中止 |
| fly_back | succeeded | success | land | 回到原点→降落 |
| fly_back | failed | failure | abort | 飞行失败→中止 |
| land | succeeded | success | complete | 降落成功→任务完成 |
| land | failed | failure | abort | 降落失败→中止 |
| abort | - | - | - | 终止节点 |
| complete | - | - | - | 终止节点 |

---

## 🔍 Part 5: 查找和转换的代码实现

### 核心函数：_start_stage()

```python
def _start_stage(self, stage_id: str) -> None:
    """启动指定的stage"""

    # Step 1: 从stage_map中查找stage
    stage = self._stage_map.get(stage_id)

    if stage is None:
        self.get_logger().error(f"Stage '{stage_id}' not found in map!")
        self._mission_active = False
        return

    # Step 2: 更新当前stage
    self._current_stage = stage
    self._current_stage_start = self.get_clock().now().nanoseconds / 1e9

    self.get_logger().info(
        f"➡️  Entering stage [{stage.id}] ({stage.stage_type})"
    )

    # Step 3: 检查特殊stage类型
    if stage.stage_type == "TERMINAL":
        self.get_logger().info(f"Mission reached terminal stage (success)")
        self._mission_active = False
        return

    if stage.stage_type == "ABORT_MISSION":
        reason = stage.params.get("reason", "unspecified")
        self.get_logger().error(f"Mission aborted: {reason}")
        self._mission_active = False
        return

    # Step 4: 查找对应的action service
    action_service = ACTION_SERVICES.get(stage.stage_type)
    # ACTION_SERVICES = {
    #     "TAKEOFF": "mission_actions/takeoff",
    #     "FLY_TO": "mission_actions/fly_to",
    #     "LAND_AT_POINT": "mission_actions/land",
    #     ...
    # }

    if action_service is None:
        self.get_logger().warn(f"No action for {stage.stage_type}")
        self._transition_from_stage("success")  # 跳过
        return

    # Step 5: 发布params
    self._publish_action_params(stage)

    # Step 6: 调用action service
    self._call_action_service(action_service, stage.params)
```

### 核心函数：_transition_from_stage()

```python
def _transition_from_stage(self, outcome_key: str) -> None:
    """根据outcome_key转换到下一个stage"""

    if self._current_stage is None:
        return

    # Step 1: 查找transitions字典
    transitions = self._current_stage.transitions
    # 例如：{"success": "fly_to_start", "failure": "abort"}

    # Step 2: 根据outcome_key查找下一个stage_id
    next_stage_id = transitions.get(outcome_key)

    if next_stage_id is None:
        # 没有定义这个outcome的转换
        self.get_logger().warn(
            f"Stage {self._current_stage.id} has no transition for '{outcome_key}'"
        )
        self._mission_active = False
        return

    # Step 3: 记录转换
    self.get_logger().info(
        f"Stage {self._current_stage.id} → {next_stage_id} via '{outcome_key}'"
    )

    # Step 4: 启动下一个stage
    self._start_stage(next_stage_id)
```

### Outcome映射逻辑

```python
def _event_callback(self, msg: String) -> None:
    """收到Action Manager的event"""

    event = json.loads(msg.data)
    # event = {
    #     "action": "takeoff",
    #     "outcome": "succeeded",
    #     "message": "Reached altitude 3.0m"
    # }

    outcome = event.get("outcome")

    # 🔑 关键：将Action Manager的outcome映射到YAML的transition key
    outcome_key_map = {
        "succeeded": "success",
        "failed": "failure",
        "canceled": "canceled",
        "exception": "failure",
        "failed_to_start": "failure",
    }

    outcome_key = outcome_key_map.get(outcome, "failure")

    # 执行转换
    self._transition_from_stage(outcome_key)
```

---

## 📝 Part 6: 实际执行示例（完整trace）

### 场景：执行square_flight任务

```
═══════════════════════════════════════════════════════════
任务开始：Mission Executor收到YAML
═══════════════════════════════════════════════════════════

[t0] _plan_callback() 被调用
     ├─ 解析YAML
     ├─ 构建_stage_map（7个stage）
     └─ _start_stage("takeoff")  ← initial stage

─────────────────────────────────────────────────────────

[t1] _start_stage("takeoff")
     ├─ _current_stage = _stage_map["takeoff"]
     │   StageSpec(
     │       id="takeoff",
     │       stage_type="TAKEOFF",
     │       transitions={"success": "fly_to_start", "failure": "abort"}
     │   )
     ├─ action_service = "mission_actions/takeoff"
     └─ call service /mission_actions/takeoff

─────────────────────────────────────────────────────────

[t2-t10] TakeoffModule执行中（8秒）
         无人机从0m爬升到3.0m

─────────────────────────────────────────────────────────

[t10] Action Manager发布event
      {"action": "takeoff", "outcome": "succeeded"}

─────────────────────────────────────────────────────────

[t11] _event_callback() 收到event
      ├─ outcome = "succeeded"
      ├─ outcome_key = "success"  ← 映射
      └─ _transition_from_stage("success")

─────────────────────────────────────────────────────────

[t12] _transition_from_stage("success")
      ├─ transitions = {"success": "fly_to_start", "failure": "abort"}
      ├─ next_stage_id = transitions["success"] = "fly_to_start"
      ├─ log: "Stage takeoff → fly_to_start via 'success'"
      └─ _start_stage("fly_to_start")

─────────────────────────────────────────────────────────

[t13] _start_stage("fly_to_start")
      ├─ _current_stage = _stage_map["fly_to_start"]
      │   StageSpec(
      │       id="fly_to_start",
      │       stage_type="FLY_TO",
      │       params={"waypoints": [[0,0,3]]},
      │       transitions={"success": "fly_square", "failure": "abort"}
      │   )
      ├─ action_service = "mission_actions/fly_to"
      └─ call service /mission_actions/fly_to

─────────────────────────────────────────────────────────

[t14-t20] FlyToTargetModule执行中（6秒）
          无人机飞到[0, 0, 3]

─────────────────────────────────────────────────────────

[t20] Action Manager发布event
      {"action": "fly_to", "outcome": "succeeded"}

─────────────────────────────────────────────────────────

[t21] _event_callback() → _transition_from_stage("success")
      ├─ next_stage_id = "fly_square"
      └─ _start_stage("fly_square")

─────────────────────────────────────────────────────────

[t22] _start_stage("fly_square")
      ├─ _current_stage = _stage_map["fly_square"]
      │   params={"waypoints": [[6,0,3], [6,6,3], [0,6,3]]}
      └─ call /mission_actions/fly_to

... 继续执行 fly_square → fly_back → land → complete ...

─────────────────────────────────────────────────────────

[tend] _start_stage("complete")
       ├─ stage_type = "TERMINAL"
       ├─ log: "Mission reached terminal stage (success)"
       ├─ _mission_active = False
       └─ 任务结束！

═══════════════════════════════════════════════════════════
```

---

## 🎯 关键点总结

### 1. Stage Map的本质
- **字典结构**：`{stage_id: StageSpec}`
- **一次性构建**：收到YAML后立即构建完整的map
- **随机访问**：通过stage_id快速查找任何stage

### 2. 转换的本质
- **查表操作**：`next_id = current_stage.transitions[outcome_key]`
- **状态更新**：`_current_stage = _stage_map[next_id]`
- **递归调用**：`_start_stage()` → action完成 → `_transition_from_stage()` → `_start_stage()`

### 3. Outcome映射
```
Action Manager发送:        Mission Executor使用:
"succeeded"         →      "success"
"failed"            →      "failure"
"canceled"          →      "canceled"
"exception"         →      "failure"
```

### 4. 特殊stage处理
- **TERMINAL**：任务成功结束，设置`_mission_active = False`
- **ABORT_MISSION**：任务中止，设置`_mission_active = False`
- **NO-OP stages**：立即调用`_transition_from_stage("success")`

---

## 🔧 常见疑问

### Q1: 为什么用字典而不是列表？
**A**: 字典提供O(1)查找速度
- 转换时需要快速找到下一个stage
- 列表需要O(n)遍历查找

### Q2: transitions字典的key为什么是"success"而不是"succeeded"？
**A**: 这是设计选择
- YAML中用简短的key（"success", "failure"）更简洁
- Action Manager的outcome是详细的（"succeeded", "failed"）
- Mission Executor负责映射转换

### Q3: 如果transitions中没有定义某个outcome怎么办？
**A**: 任务会停止
```python
next_stage_id = transitions.get("success")
if next_stage_id is None:
    self._mission_active = False  # 任务结束
```

### Q4: 可以有循环转换吗（例如search失败后重试）？
**A**: 可以！
```yaml
- id: "search"
  type: "SEARCH_AREA"
  transitions:
    target_found: "track"
    timeout: "search"  # ← 循环！超时后重新search
```

### Q5: Stage Map在运行时会修改吗？
**A**: 不会
- 一旦构建完成，Stage Map是只读的
- 只有`_current_stage`指针在移动
- 这保证了任务的可预测性

---

## 📐 数据流总结

```
YAML文本
  ↓ [yaml.safe_load()]
Python字典
  ↓ [遍历构建]
Stage Map (_stage_map)
  ↑ [查找]
  │
  ├─ _start_stage(id)      ← 启动某个stage
  │    ↓
  │  执行action
  │    ↓
  │  收到outcome event
  │    ↓
  ├─ _transition_from_stage(outcome_key)
  │    ↓
  │  查找transitions[outcome_key]
  │    ↓
  │  获得next_stage_id
  │    ↓
  └─ _start_stage(next_stage_id)  ← 循环
```

希望这个详解能帮你完全理解Stage Map的实现！🎉
