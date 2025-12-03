
---

# SLAM + AVIANS_ROS2 深度集成分析

**基于分支**: `dev/sabrina/semantic-planner`
**分析日期**: 2025-12-02
**系统目标**: 无GPS自主智能飞行

## 系统背景

- **SLAM代码库** (上游同事 `~/SLAM`): 提供视觉惯性SLAM、语义感知(Scene Graph)、密集建图(TSDF/ESDF)、路径规划(A* + RRT*)和任务执行(BehaviorTree)
- **AVIANS_ROS2代码库** (本工作空间): 提供任务执行(Mission Executor)、行为模块(Action Modules)、制导控制(Guidance Controllers)和底层控制器(Low-level Controllers)

---

## 问题1: SLAM系统完整讲解 (每个模块 + 信息流动)

### 系统架构概览 (8个核心包)

```
semantic_slam_ws/src/
├── orbslam3_ros2         # Visual-Inertial SLAM
├── scene_graph           # 感知+语义理解
├── semantic_planner      # A*/RRT* 路径规划
├── mission_executor      # 任务执行(BT框架)
├── voxblox_mapping       # TSDF/ESDF建图
├── dense_tsdf_mapping    # Dense TSDF
├── drone_safety          # 安全服务
└── semantic_slam_msgs    # 消息定义
```

### 模块 1️⃣: ORB-SLAM3 ROS2 (orbslam3_ros2)

**功能**: Visual-Inertial Odometry + 回环检测

**输入**:
- `/cam0/image_raw` (sensor_msgs/Image, 20Hz)
- `/imu0` (sensor_msgs/Imu, 200Hz)

**核心算法**:
- ORB特征提取 (500-1000 features/frame)
- IMU预积分 (姿态预测)
- 视觉惯性Bundle Adjustment
- 回环检测 (DBoW2)
- 位姿图优化 (g2o)

**输出**:
```
/camera_pose (geometry_msgs/PoseStamped, 30Hz)
├─ position: [x, y, z] (world坐标系)
├─ orientation: quaternion
└─ frame_id: "world"

/landmarks (semantic_slam_msgs/LandmarkArray)
├─ 3D地标点 ([x,y,z])
├─ observations_count (置信度)
├─ descriptor (ORB 256-bit)
└─ last_observed_time
```

**修改点** (相比原版):
- ✅ 发布 landmarks (原版不发布)
- ✅ ROS2 Jazzy 适配
- ✅ 地图重置接口 (`/slam/reset`)

---

### 模块 2️⃣: Scene Graph (scene_graph)

**功能**: 感知管道 (检测 → 跟踪 → 关系推理)

#### 子模块 A: YOLOE检测器 (`yoloe_detector_node.py`)

**输入**: `/cam0/image_raw`

**算法**:
```python
# YOLOE-11S (YOLOv11 最小模型)
detections = model.predict(
    image,
    conf=0.20,      # 置信度阈值
    iou=0.45,       # NMS IOU阈值
    imgsz=640       # 输入尺寸
)

# ByteTrack 多目标跟踪
tracked_objects = tracker.update(detections)
for obj in tracked_objects:
    obj.tracking_id  # 持久ID (跨帧)
```

**输出**:
```
/detections/objects (semantic_slam_msgs/ObjectInstanceArray)
├─ instance_id: UUID (全局唯一)
├─ class_id: int (0-110)
├─ class_name: str ("person", "car", "tree"...)
├─ tracking_id: int (ByteTrack ID)
├─ bbox_2d: [xmin, ymin, xmax, ymax]
├─ confidence: float (0.0-1.0)
└─ color: RGB (简单颜色提取)

/detections/visualization (sensor_msgs/Image)
└─ 带标注的可视化图像
```

**支持的类别**: 111类
- 80 COCO 类 (person, car, bus, ...)
- 31 自定义类 (tree, building, power_line, ...)

#### 子模块 B: Scene Graph Manager (`scene_graph_manager_node.cpp`)

**功能**: 空间关系检测 + SQL查询引擎

**输入**:
- `/detections/objects`
- `/landmarks`
- `/global_cloud` (持久语义点云)

**关系检测算法**:
```cpp
for (object_A : tracked_objects) {
    for (object_B : tracked_objects) {
        if (A == B) continue;

        Vector3d delta = B.position - A.position;
        float distance = delta.norm();

        // 空间关系判断
        if (distance < 3.0)
            relationships.add({A, B, NEAR});

        if (delta.z() > 1.0)
            relationships.add({A, B, ABOVE});

        // 相对方向
        float angle = atan2(delta.y(), delta.x());
        if (is_in_front(angle, A.orientation))
            relationships.add({A, B, IN_FRONT_OF});
    }
}
```

**输出**:
```
/scene_relationships (semantic_slam_msgs/RelationshipArray)
├─ subject_id: UUID
├─ object_id: UUID
├─ relationship_type: enum
│   ├─ NEAR (< 3m)
│   ├─ LEFT_OF / RIGHT_OF
│   ├─ IN_FRONT_OF / BEHIND
│   └─ ABOVE / BELOW
├─ confidence: float
└─ metric_value: float (距离/角度)
```

**SQL查询接口** (服务):
```
服务: /scene_graph/query
输入: "FIND car WHERE color='blue' NEAR building"
输出: ObjectInstanceArray (匹配对象列表)
```

---

### 模块 3️⃣: Voxblox Mapping (voxblox_mapping)

**功能**: ESDF (Euclidean Signed Distance Field) 地图生成

**输入**: `/landmarks` (ORB-SLAM3点云)

**算法流程**:
```
点云 → TSDF Integration → 距离场生成 → ESDF Map
     (体素融合)        (truncated SDF)   (距离查询)
```

**TSDF (Truncated SDF)**:
```cpp
// 每个体素存储到最近表面的signed distance
voxel.distance = signed_distance_to_surface
voxel.weight = observation_count

// 融合新观测
voxel_new = (voxel_old * weight_old + obs * weight_obs)
            / (weight_old + weight_obs)
```

**ESDF (Euclidean SDF)**:
```cpp
// 每个体素存储到最近障碍物的欧式距离
esdf(p) = min_distance(p, occupied_voxels)

// 用于梯度下降优化
gradient = ∇esdf(p)  // 指向远离障碍物的方向
```

**输出**:
```
/esdf_map (voxblox_msgs/Layer)
├─ voxel_size: 0.2m
├─ esdf_grid: [distance_field]
└─ frame_id: "world"

/tsdf_mesh (visualization_msgs/Marker)
└─ 三角网格可视化
```

**优势对比**:

| 特性 | OctoMap | Voxblox ESDF |
|-----|---------|--------------|
| 碰撞检测 | 二进制查询 | 连续距离场 |
| 路径优化 | A* grid | 梯度下降 |
| 查询速度 | O(log n) | O(1) |
| 性能 | 中等 | 快40% |

---

### 模块 4️⃣: Semantic Planner (semantic_planner)

**功能**: 分层路径规划 (全局A* + 局部RRT*)

#### 子模块 A: Information Manager

**功能**: 探索前沿检测 + 信息增益计算

**算法**:
```cpp
// 前沿检测 (Frontier Detection)
for (voxel : esdf_map) {
    if (is_free(voxel)) {
        for (neighbor : voxel.neighbors_26()) {
            if (is_unknown(neighbor)) {
                frontiers.add(voxel);  // 自由-未知边界
                break;
            }
        }
    }
}

// 信息增益 (Information Gain)
IG(frontier) = frontier_size * semantic_value
             + incomplete_objects_bonus
             + dynamic_objects_bonus
             - distance_penalty
             - revisit_penalty
```

**输出**:
```
/planner/frontiers (visualization_msgs/MarkerArray)
└─ 候选探索点 (按IG排序)
```

#### 子模块 B: Semantic A*

**功能**: 全局粗路径规划 (带语义代价)

**核心算法**:
```cpp
// A* 代价函数
f(n) = g(n) + h(n) + semantic_cost(n)
where:
  g(n) = 从起点到n的实际代价
  h(n) = 从n到终点的启发式代价 (欧式距离)
  semantic_cost(n) = 语义区域代价

// 语义代价查询
float semantic_cost(position) {
    semantic_id = octo_tree->getColor(position).r;
    return cost_map[semantic_id];  // YAML配置
}
```

**语义代价表** (`config/planner_params.yaml`):
```yaml
semantic_costs:
  # 吸引区域 (负代价)
  open_field: -5.0
  landing_zone: -15.0

  # 警告区域
  tree: 12.0
  garden: 8.0

  # 避让区域
  forest: 40.0
  crowd: 50.0

  # 禁飞区域
  building: 150.0
  power_line: 200.0
  no_fly_zone: 1000.0
```

**输出**:
```python
global_path: [waypoint_0, waypoint_1, ...]
├─ waypoint.position: [x, y, z]
├─ waypoint.yaw: float
└─ waypoint.metadata: {semantic_label, cost}
```

#### 子模块 C: Semantic RRT*

**功能**: 局部平滑路径优化

**3层安全膨胀系统**:
```
┌────────────────────────────────────┐
│   Comfort Zone (3.75m)             │
│   ┌────────────────────────────┐   │
│   │  Caution Zone (2.75m)      │   │
│   │  ┌──────────────────────┐  │   │
│   │  │ Forbidden (1.25m)    │  │   │
│   │  │   ┌──────────┐       │  │   │
│   │  │   │ Obstacle │       │  │   │
│   │  │   └──────────┘       │  │   │
│   │  └──────────────────────┘  │   │
│   └────────────────────────────┘   │
└────────────────────────────────────┘
```

**算法**:
```cpp
// RRT* 迭代
for (i = 0; i < max_iterations; i++) {
    // 在走廊内采样
    random_point = sample_in_corridor(global_path, corridor_width);

    // 找最近节点
    nearest = find_nearest_node(random_point);

    // 扩展
    new_node = steer(nearest, random_point, step_size);

    // 碰撞检测 (3层膨胀)
    if (is_collision_free(new_node)) {
        // 重连优化
        neighbors = find_neighbors(new_node, search_radius);
        parent = choose_best_parent(new_node, neighbors);
        tree.add(new_node, parent);
        rewire(neighbors, new_node);
    }
}
```

**碰撞检测**:
```cpp
bool is_collision_free(position) {
    // 豁免1: 当前位置 (ground truth)
    if (distance(position, current_robot_pose) < 0.5)
        return true;

    // 豁免2: 降落区
    if (is_in_landing_zone(position))
        return true;

    // 3层代价计算
    if (in_forbidden_zone(position))
        return false;  // 硬碰撞

    cost += caution_cost(position);   // 中等代价
    cost += comfort_cost(position);   // 轻微代价

    return cost < threshold;
}
```

**输出**:
```
/planner/path (nav_msgs/Path)
├─ poses: [PoseStamped, ...]
└─ header.frame_id: "world"

/planner/viewpoint_direction (geometry_msgs/Vector3Stamped)
└─ vector: [dx, dy, dz]  # 朝向未探索区域
```

---

### 模块 5️⃣: Mission Executor (mission_executor)

**功能**: BehaviorTree 任务执行框架

**依赖**: BehaviorTree.CPP v4

**输入**: YAML 任务文件

**示例 YAML**:
```yaml
mission:
  name: "search_and_land"

stages:
  initial: "query_scene"
  stage_list:
    - id: "query_scene"
      type: "QUERY_OBJECT"
      params:
        target_class: "person"
        sql: "FIND person NEAR building"
      transitions:
        success: "plan_path"
        failure: "search_area"

    - id: "plan_path"
      type: "NAVIGATE_TO_TARGET"  # 调用 Semantic Planner
      params:
        target: "${query_result[0].position}"
        altitude: 3.0
      transitions:
        success: "validate_landing"
        failure: "fallback"

    - id: "validate_landing"
      type: "VALIDATE_SAFETY"  # 调用 Safety Service
      params:
        clearance: 5.0
      transitions:
        success: "land"
        failure: "search_alternate"

    - id: "land"
      type: "LAND_AT_POINT"
      transitions:
        success: "complete"
```

**已实现 Action Nodes**:
- `TakeOff` - 起飞到指定高度
- `Land` - 安全降落
- `FlyTo` - 跟踪航点序列
- `QuerySceneGraph` - SQL查询 → 目标位置
- `ValidateSafety` - 碰撞/降落安全检查

**BT 执行流程**:
```
YAML → BT Parser → BT Builder
                      ↓
                  BT Executor
                      ↓
        ┌─────────────┼─────────────┐
        ↓             ↓             ↓
   TakeOff Node  Query Node    FlyTo Node
        │             │             │
        └─────────────┴─────────────┘
                      ↓
              Success / Failure
                      ↓
              下一个Stage
```

---

### 模块 6️⃣: Drone Safety (drone_safety)

**功能**: 安全验证服务

**服务定义**:

#### CHECK_LANDING
```
Service: /safety/check_landing
Request:
  - position: Point
  - clearance_radius: float
Response:
  - is_safe: bool
  - failure_reason: str
  - blocking_objects: str[]
```

#### CHECK_PATH
```
Service: /safety/check_path
Request:
  - waypoints: Path
  - corridor_width: float
Response:
  - collision_points: Point[]
  - collision_objects: str[]
```

#### PREDICT_COLLISION
```
Service: /safety/predict
Request:
  - object_id: UUID
  - time_horizon: float
Response:
  - predicted_positions: Point[]
  - collision_probability: float
```

---

### 完整数据流 (端到端)

```
[相机+IMU]
    ↓
[ORB-SLAM3] → /camera_pose, /landmarks
    ↓                ↓
    ↓           [Voxblox TSDF] → /esdf_map
    ↓                ↓
[YOLOE Detector] → /detections/objects
    ↓                ↓
[Scene Graph Manager] ← /landmarks
    ↓
/scene_relationships, /semantic_cloud
    ↓                ↓
[Semantic Planner] ← /esdf_map, /camera_pose
    ↓
/planner/path
    ↓
[Mission Executor BT]
    ↓
[Action Nodes] → 调用AVIANS控制器栈
```

---

## 问题2: AVIANS_ROS2 Option 1 如何对接?

### AVIANS Option 1 启动流程详解 (12步)

| 步骤 | 节点 | 包 | 关键Topic | 作用 |
|-----|------|-----|-----------|------|
| 1 | Gazebo | drone_description | /X3/odometry, /cam0/image_raw | 仿真环境 |
| 2 | YOLO检测器 | neural_network_detector | /person_detections | 人员检测 |
| 3 | 检测可视化 | neural_network_detector | - | 调试可视化 |
| 4 | TF发布器 | drone_tf_publisher.py | TF树 | 坐标变换 |
| 5 | RViz | visualization_node.py | - | 3D可视化 |
| 6 | 状态发布器 | drone_state_publisher | /drone/state | 状态机 |
| 7 | TF from pose | tf_from_uav_pose | /machine_1/pose | 位姿→TF |
| 8 | 投影模型 | projection_model | /person_detections_3d | 2D→3D |
| 9 | Pose协方差 | pose_cov_ops_interface | /X3/pose_with_covariance | 协方差转换 |
| 10 | NMPC跟踪器 | drone_nmpc_tracker | /target → /X3/cmd_vel | 目标跟踪MPC |
| 11 | 控制器栈 | drone_guidance_controllers + drone_low_level_controllers | 航点/偏航/速度 | 三层控制 |
| 11b | Action Manager | mission_action_modules | /mission_actions/* | 动作管理 |
| 12 | Mission Executor | mission_executor | /mission_executor/plan | 任务编排 |

### AVIANS Mission Executor 分析

**架构**: 状态机风格 (非BT)

**关键特性**:
```python
# 支持的Action类型
ACTION_SERVICES = {
    "TAKEOFF": "mission_actions/takeoff",
    "FLY_TO": "mission_actions/fly_to",
    "SEARCH_AREA": "mission_actions/search",
    "TRACK_TARGET": "mission_actions/track_target",
    "LAND_AT_POINT": "mission_actions/land",
    "HOVER": "mission_actions/hover",
}

# 逻辑Stage (自动成功)
NOOP_STAGES = {
    "QUERY_OBJECT",        # 场景图查询
    "COMPUTE_OFFSET_TARGET",  # 计算偏移
    "VALIDATE_SAFETY",     # 安全检查
    "NAVIGATE_TO_TARGET",  # 导航规划
}
```

**执行流程**:
```
YAML Mission → parse() → load stages
    ↓
start_stage(initial_id)
    ↓
┌─ NOOP Stage? → 自动成功
├─ Action Stage? → call service
└─ Terminal? → 结束

服务调用 → Action Manager → Action Module
    ↓
Event: "succeeded" / "failed"
    ↓
transition_to_next_stage()
```

---

## 集成架构设计 (最终方案)

```
┌────────────────────────────────────────────────────────────┐
│          SLAM系统 (上游 - 感知+规划)                        │
├────────────────────────────────────────────────────────────┤
│  ORB-SLAM3 → Scene Graph → Semantic Planner               │
│     │            │              │                          │
│     ↓            ↓              ↓                          │
│  位姿估计     语义理解       A*/RRT*路径                    │
└──────┬───────────┬─────────────┬───────────────────────────┘
       │           │             │
       │      ═════════════════════════
       │       集成适配层 (Interface)
       │      ═════════════════════════
       │           │             │
┌──────┴───────────┴─────────────┴───────────────────────────┐
│        AVIANS_ROS2 (下游 - 执行+控制)                       │
├────────────────────────────────────────────────────────────┤
│  Mission Executor → Action Manager → Controllers          │
│                                                            │
│  [两个Executor层级划分]                                     │
│  • SLAM Executor (BT): 高层任务解析                        │
│  • AVIANS Executor (SM): 低层动作编排                      │
└────────────────────────────────────────────────────────────┘
```

---

## 5个核心集成接口

### 接口 1: 位姿融合

**替换**: Gazebo Odometry → ORB-SLAM3 Pose

**实现**:
```python
# slam_pose_adapter_node.py
class SLAMPoseAdapter(Node):
    def __init__(self):
        self.slam_sub = self.create_subscription(
            PoseStamped, '/camera_pose', self.slam_cb, 10)
        self.odom_pub = self.create_publisher(
            Odometry, '/X3/odometry', 10)

        # 相机到机体外参 (需标定)
        self.T_base_cam = np.array([
            [0, 0, 1, 0.1],    # cam_x → base_z
            [-1, 0, 0, 0],     # cam_y → -base_x
            [0, -1, 0, 0.05],  # cam_z → -base_y
            [0, 0, 0, 1]
        ])

        self.prev_pose = None
        self.prev_time = None

    def slam_cb(self, msg):
        # 1. 坐标转换
        T_world_cam = pose_to_matrix(msg.pose)
        T_world_base = T_world_cam @ self.T_base_cam

        # 2. 速度估计 (差分)
        if self.prev_pose:
            dt = (msg.header.stamp - self.prev_time).nanoseconds / 1e9
            vel = (T_world_base[:3,3] - self.prev_pose[:3,3]) / dt
        else:
            vel = np.zeros(3)

        # 3. 发布Odometry
        odom = Odometry()
        odom.header = msg.header
        odom.header.frame_id = 'world'
        odom.child_frame_id = 'base_link'
        odom.pose.pose = matrix_to_pose(T_world_base)
        odom.twist.twist.linear = Vector3(*vel)

        self.odom_pub.publish(odom)

        self.prev_pose = T_world_base
        self.prev_time = msg.header.stamp
```

**Topic映射**:
```
/camera_pose (SLAM) → [适配器] → /X3/odometry (AVIANS)
```

---

### 接口 2: 检测融合

**挑战**: 两套YOLO
- SLAM: YOLOE-11S (111类, ByteTrack)
- AVIANS: person检测器

**方案**: 统一使用SLAM检测器

```python
# detection_bridge_node.py
class DetectionBridge(Node):
    def __init__(self):
        self.slam_sub = self.create_subscription(
            ObjectInstanceArray, '/detections/objects', self.cb, 10)
        self.avians_pub = self.create_publisher(
            PersonDetectionArray, '/person_detections', 10)

    def cb(self, slam_dets):
        # 过滤: 只保留person类
        persons = [d for d in slam_dets.instances
                   if d.class_name == 'person']

        # 格式转换
        avians_msg = PersonDetectionArray()
        for p in persons:
            det = PersonDetection()
            det.bbox = p.bbox_2d
            det.confidence = p.confidence
            det.tracking_id = p.tracking_id  # 持久ID!
            avians_msg.detections.append(det)

        self.avians_pub.publish(avians_msg)
```

**好处**:
- ✅ 节省算力 (一套YOLO)
- ✅ 获得 tracking_id (对 TRACK_TARGET 重要)
- ✅ 更多类别 (未来扩展)

---

### 接口 3: 任务规划对接

**两个Executor分工**:

```
┌──────────────────────────────────────────────────────┐
│ SLAM Mission Executor (BT框架)                       │
│  • 高层任务解析                                       │
│  • 场景图查询 (QuerySceneGraph Node)                 │
│  • 路径规划调用 (调用 Semantic Planner)               │
│  • 安全检查 (调用 Safety Services)                    │
│                                                      │
│  输出: Waypoint序列 + 高层动作命令                    │
└────────────────┬─────────────────────────────────────┘
                 │
                 ↓ /mission_executor/plan (YAML)
┌────────────────┴─────────────────────────────────────┐
│ AVIANS Mission Executor (状态机)                     │
│  • 低层动作编排                                       │
│  • 调用 Action Manager                               │
│  • 监控执行状态                                       │
│  • 处理超时/失败                                      │
│                                                      │
│  执行: TakeOff, FlyTo, Hover, Land, Track           │
└──────────────────────────────────────────────────────┘
```

**数据流**:
```
用户: "搜索人员并降落在旁边3米"
  ↓
SLAM Executor (BT):
  - QuerySceneGraph("FIND person")
    → person_position = [10, 5, 0]
  - ComputeOffset(person_position, distance=3.0)
    → landing_position = [13, 5, 0]
  - SemanticPlanner(goal=landing_position)
    → waypoints = [[0,0,2], [5,2.5,2], [10,5,2], [13,5,2]]
  - ValidateSafety(landing_position)
    → is_safe = True
  - 生成YAML:
    stages:
      - TAKEOFF (altitude=2.0)
      - FLY_TO (waypoints=...)
      - LAND_AT_POINT (position=landing_position)
  ↓
AVIANS Executor (SM):
  - Stage 1: 调用 TakeoffModule(2.0)
  - Stage 2: 调用 FlyToModule(waypoints)
  - Stage 3: 调用 LandModule(landing_position)
```

**YAML 适配器**:
```python
# mission_translator_node.py
class MissionTranslator(Node):
    def __init__(self):
        # 订阅SLAM规划结果
        self.slam_plan_sub = self.create_subscription(
            String, '/slam/mission_plan', self.translate, 10)
        # 发布到AVIANS executor
        self.avians_plan_pub = self.create_publisher(
            String, '/mission_executor/plan', 10)

    def translate(self, slam_yaml):
        plan = yaml.safe_load(slam_yaml.data)

        # 转换Stage类型
        for stage in plan['stages']['stage_list']:
            # SLAM使用QuerySceneGraph → AVIANS用QUERY_OBJECT(noop)
            if stage['type'] == 'QuerySceneGraph':
                stage['type'] = 'QUERY_OBJECT'

            # SLAM的FlyTo已包含完整waypoints
            if stage['type'] == 'FlyTo':
                stage['type'] = 'FLY_TO'
                # waypoints已在params中

        avians_yaml = String()
        avians_yaml.data = yaml.dump(plan)
        self.avians_plan_pub.publish(avians_yaml)
```

---

### 接口 4: Waypoint传递

**Semantic Planner → Waypoint Controller**

```python
# waypoint_relay_node.py
class WaypointRelay(Node):
    def __init__(self):
        self.path_sub = self.create_subscription(
            Path, '/planner/path', self.relay, 10)
        self.wp_pub = self.create_publisher(
            PoseStamped, '/drone/control/waypoint_command', 10)

        self.current_wp_idx = 0
        self.waypoints = []

        # 监听到达事件
        self.status_sub = self.create_subscription(
            Float64MultiArray, '/drone/controller/status',
            self.check_arrival, 10)

    def relay(self, path):
        self.waypoints = path.poses
        self.current_wp_idx = 0
        self.send_next_waypoint()

    def send_next_waypoint(self):
        if self.current_wp_idx < len(self.waypoints):
            wp = self.waypoints[self.current_wp_idx]
            self.wp_pub.publish(wp)
            self.get_logger().info(
                f"Sent waypoint {self.current_wp_idx+1}/{len(self.waypoints)}")

    def check_arrival(self, status):
        # status.data[1] = waypoint_reached
        if status.data[1] > 0.5:
            self.current_wp_idx += 1
            self.send_next_waypoint()
```

---

### 接口 5: 安全服务调用

**在 Action Module 中集成**:

```python
# 修改 LandModule
class LandModule(ActionBase):
    def __init__(self, node):
        super().__init__(node, "land")
        self.safety_client = node.create_client(
            CheckLanding, '/safety/check_landing')

    async def execute(self, params):
        target = params.get('position', [0, 0, 0])
        clearance = params.get('clearance', 5.0)

        # 1. 调用SLAM安全检查
        req = CheckLanding.Request()
        req.position.x = target[0]
        req.position.y = target[1]
        req.position.z = target[2]
        req.clearance_radius = clearance

        response = await self.safety_client.call_async(req)

        if not response.is_safe:
            self.log_error(f"Landing unsafe: {response.failure_reason}")
            self.log_error(f"Blocking: {response.blocking_objects}")
            return ActionResult.FAILED

        # 2. 执行降落
        self.publish_landing_command(target)
        await self.wait_for_touchdown()

        return ActionResult.SUCCEEDED
```

---


## 实施路线图 (8周)

### Phase 1: 基础对接 (Week 1-2)

**任务**:
- [ ] 克隆SLAM仓库 (`dev/sabrina/semantic-planner`分支)
- [ ] 创建 `slam_integration` 包
- [ ] 实现 `slam_pose_adapter`
- [ ] 测试SLAM位姿驱动Gazebo
- [ ] 统一检测器 (`detection_bridge`)

**验收标准**:
- SLAM `/camera_pose` → AVIANS `/X3/odometry`
- YOLOE检测 → `/person_detections`
- TF树正确 (`world → base_link → cam0`)

---

### Phase 2: 规划器集成 (Week 3-4)

**任务**:
- [ ] 编译 `semantic_planner` 包
- [ ] 测试 A*/RRT* 规划
- [ ] 实现 `waypoint_relay_node`
- [ ] 扩展 `FlyToModule` 支持路径跟踪

**验收标准**:
- `/planner/path` → Waypoint Controller
- Gazebo中无人机跟踪规划路径
- 避障功能验证

---

### Phase 3: 任务执行融合 (Week 5-6)

**任务**:
- [ ] 编译 `mission_executor` (SLAM的BT版本)
- [ ] 安装 BehaviorTree.CPP v4
- [ ] 实现 `mission_translator`
- [ ] 定义统一消息接口 (`slam_interfaces` 包)
- [ ] 扩展AVIANS Action Modules

**验收标准**:
- SLAM Executor生成的YAML → AVIANS执行
- 完整"搜索与降落"场景

---

### Phase 4: 语义感知 (Week 7-8)

**任务**:
- [ ] 集成 Scene Graph 查询
- [ ] 集成 Safety Services
- [ ] NMPC 集成 ESDF避障
- [ ] 完整场景测试

**验收标准**:
- SQL查询驱动任务
- 动态避障生效
- 多场景鲁棒性测试

---

## 需要与上游同事确认的清单

### 1. 坐标系约定
- [ ] World frame定义 (ENU? 原点?)
- [ ] 相机外参 `T_base_cam`
- [ ] IMU外参

### 2. 消息定义
- [ ] `/planner/path` 完整定义
- [ ] BT Executor输出格式
- [ ] Safety Service `.srv` 文件

### 3. 更新频率
- [ ] `/camera_pose`: 30Hz?
- [ ] `/planner/path`: 重规划频率?
- [ ] `/scene_relationships`: 更新率?

### 4. 部署架构
- [ ] 两个系统运行在哪里? (同一台PC?)
- [ ] 网络配置 (`ROS_DOMAIN_ID`?)
- [ ] Bag 文件测试数据

### 5. 测试数据
- [ ] EuRoC MH_01 bag
- [ ] 自定义 Gazebo 场景
- [ ] 真实飞行数据

---

## 总结

### SLAM系统优势

✅ **完整VIO定位** (ORB-SLAM3)
✅ **丰富语义理解** (111类 + 关系推理)
✅ **智能路径规划** (语义A* + 3层安全RRT*)
✅ **BT任务框架** (可扩展)

### AVIANS系统优势

✅ **成熟控制栈** (NMPC + 3层控制器)
✅ **灵活动作系统** (Action Manager + Modules)
✅ **状态机Executor** (轻量高效)
✅ **完整仿真环境** (Gazebo + RViz)

### 集成后能力

**SLAM感知+规划 + AVIANS执行+控制 = 完整无GPS自主飞行系统**

---

## 关键技术决策

### 决策1: 使用两层Executor架构 ✅

**理由**:
- SLAM Executor (BT): 高层任务解析、语义理解、路径规划
- AVIANS Executor (SM): 低层动作编排、实时执行、故障恢复
- 清晰的职责分离,两个团队可并行开发

### 决策2: 保留所有AVIANS Action Modules ✅

**理由**:
- 已验证的控制逻辑和参数
- 只需修改数据来源接口(20-40%工作量)
- 避免重复开发和调试

### 决策3: 统一使用SLAM的感知系统 ✅

**理由**:
- 节省算力(一套YOLO)
- 更强的语义理解(111类 vs 1类)
- 持久跟踪ID(ByteTrack)

### 决策4: 创建独立桥接包 ✅

**理由**:
- 解耦SLAM和AVIANS代码
- 易于版本管理和测试
- 支持多种部署场景(单机/分布式)

---

## 风险评估与缓解

| 风险 | 影响 | 概率 | 缓解措施 |
|-----|------|------|---------|
| 坐标系标定误差 | 高 | 中 | 使用Kalibr工具,离线验证 |
| SLAM丢失tracking | 高 | 中 | 惯性导航降级,紧急悬停 |
| 服务调用延迟 | 中 | 低 | 异步调用,超时重试 |
| BT学习曲线 | 低 | 高 | 先用简单任务,增量学习 |
| 网络通信故障 | 高 | 低 | 单机部署优先,DDS QoS配置 |
| 依赖版本冲突 | 中 | 低 | 统一ROS2 Jazzy,Docker容器化 |

---

## 依赖处理与工作空间配置

### 1. 消息包依赖处理

#### 问题描述
SLAM系统使用自定义消息包 `semantic_slam_msgs`,而AVIANS使用 `uav_msgs`。桥接节点需要同时依赖这两个包。

#### 解决方案 A: 修改 package.xml

在新建的 `slam_integration` 包中声明依赖:

```xml
<!-- slam_integration/package.xml -->
<package format="3">
  <name>slam_integration</name>
  <version>1.0.0</version>
  <description>SLAM与AVIANS的集成桥接包</description>

  <maintainer email="your_email@example.com">AVIANS团队</maintainer>
  <license>MIT</license>

  <!-- ROS2依赖 -->
  <depend>rclpy</depend>
  <depend>rclcpp</depend>

  <!-- SLAM消息依赖 -->
  <depend>semantic_slam_msgs</depend>

  <!-- AVIANS消息依赖 -->
  <depend>uav_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>nav_msgs</depend>
  <depend>sensor_msgs</depend>

  <!-- TF依赖 -->
  <depend>tf2_ros</depend>
  <depend>tf2_geometry_msgs</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### 解决方案 B: colcon build 选项

```bash
# 方案1: 单独编译消息包,然后编译桥接包
cd ~/AVIANS_ROS2
source /opt/ros/jazzy/setup.bash

# 先编译SLAM消息包
colcon build --packages-select semantic_slam_msgs
source install/setup.bash

# 再编译桥接包
colcon build --packages-select slam_integration
source install/setup.bash

# 方案2: 指定依赖顺序一次性编译
colcon build --packages-up-to slam_integration
```

---

### 2. 工作空间合并策略

#### 策略 A: 软链接方案 (推荐)

**优势**: 不破坏原有代码库结构,易于版本管理

```bash
# 1. 在AVIANS工作空间创建src链接
cd ~/AVIANS_ROS2/src

# 2. 软链接SLAM包到AVIANS工作空间
ln -s ~/SLAM/semantic_slam_ws/src/semantic_slam_msgs ./semantic_slam_msgs
ln -s ~/SLAM/semantic_slam_ws/src/orbslam3_ros2 ./orbslam3_ros2
ln -s ~/SLAM/semantic_slam_ws/src/scene_graph ./scene_graph
ln -s ~/SLAM/semantic_slam_ws/src/semantic_planner ./semantic_planner
ln -s ~/SLAM/semantic_slam_ws/src/mission_executor ./mission_executor_bt
ln -s ~/SLAM/semantic_slam_ws/src/voxblox_mapping ./voxblox_mapping
ln -s ~/SLAM/semantic_slam_ws/src/drone_safety ./drone_safety

# 3. 验证链接
ls -la ~/AVIANS_ROS2/src | grep slam

# 4. 编译
cd ~/AVIANS_ROS2
colcon build --symlink-install
```

#### 策略 B: Workspace Overlay 方案

**优势**: 保持两个工作空间独立,通过ROS2机制叠加

```bash
# 1. 编译SLAM工作空间
cd ~/SLAM/semantic_slam_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash

# 2. 编译AVIANS工作空间 (extend SLAM)
cd ~/AVIANS_ROS2
source ~/SLAM/semantic_slam_ws/install/setup.bash  # 关键!
colcon build --symlink-install
source install/setup.bash

# 3. 运行时环境
source ~/SLAM/semantic_slam_ws/install/setup.bash
source ~/AVIANS_ROS2/install/setup.bash
```

**注意**: Overlay方案要求每次打开新终端都要按顺序source两个setup.bash

#### 策略 C: 统一编译脚本

创建 `setup_integrated_workspace.sh`:

```bash
#!/bin/bash

SLAM_WS=~/SLAM/semantic_slam_ws
AVIANS_WS=~/AVIANS_ROS2

echo "🔧 设置集成工作空间..."

# 1. 创建软链接
cd $AVIANS_WS/src
for pkg in semantic_slam_msgs orbslam3_ros2 scene_graph semantic_planner \
           mission_executor voxblox_mapping drone_safety; do
    if [ ! -L "$pkg" ]; then
        ln -s $SLAM_WS/src/$pkg ./$pkg
        echo "✅ 链接 $pkg"
    fi
done

# 2. 编译
cd $AVIANS_WS
source /opt/ros/jazzy/setup.bash
echo "🔨 编译工作空间..."
colcon build --symlink-install --packages-skip-build-finished

# 3. 生成环境脚本
cat > $AVIANS_WS/source_integrated.sh << 'EOF'
#!/bin/bash
source /opt/ros/jazzy/setup.bash
source $(dirname $0)/install/setup.bash
export ROS_DOMAIN_ID=42
echo "✅ 集成环境已激活 (ROS_DOMAIN_ID=42)"
EOF

chmod +x $AVIANS_WS/source_integrated.sh
echo "✅ 完成! 使用方法: source ~/AVIANS_ROS2/source_integrated.sh"
```

---

### 3. 话题重定向配置

#### 场景: SLAM与AVIANS话题命名冲突

**问题**: SLAM的 `/camera_pose` 需要映射到AVIANS的 `/X3/odometry`

**解决方案**: 使用ROS2 remapping

```bash
# 方法1: 启动时重映射
ros2 run slam_integration slam_pose_adapter \
  --ros-args \
  -r /camera_pose:=/slam/camera_pose \
  -r /X3/odometry:=/avians/odometry

# 方法2: Launch文件配置
# slam_integration_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_integration',
            executable='slam_pose_adapter',
            name='slam_pose_adapter',
            remappings=[
                ('/camera_pose', '/slam/camera_pose'),
                ('/X3/odometry', '/avians/odometry'),
            ],
            parameters=[{
                'publish_rate': 30.0,
                'use_ekf': False,
            }]
        ),
    ])
```

#### 全局重映射表

| SLAM原始话题 | AVIANS目标话题 | 桥接节点 |
|-------------|---------------|---------|
| `/camera_pose` | `/X3/odometry` | slam_pose_adapter |
| `/detections/objects` | `/person_detections` | detection_bridge |
| `/planner/path` | `/drone/control/waypoint_command` | waypoint_relay |
| `/esdf_map` | `/octomap_full` | (可选)格式转换 |

---

## 集成测试方案

### 1. 测试脚本设计

#### `slam_avians_integration_test.sh` (完整版)

```bash
#!/bin/bash
# SLAM + AVIANS 集成测试脚本
# 用途: 验证两个系统的接口对接

set -e  # 遇到错误立即退出

# ============ 配置区 ============
AVIANS_WS=~/AVIANS_ROS2
SLAM_WS=~/SLAM/semantic_slam_ws
TEST_DURATION=30  # 秒
LOG_DIR=/tmp/slam_avians_test_$(date +%Y%m%d_%H%M%S)

# ============ 颜色定义 ============
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# ============ 工具函数 ============
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

check_command() {
    if ! command -v $1 &> /dev/null; then
        log_error "命令 '$1' 未找到,请安装"
        exit 1
    fi
}

# ============ 检查函数 ============
check_workspaces() {
    log_info "检查工作空间..."

    if [ ! -d "$AVIANS_WS" ]; then
        log_error "AVIANS工作空间不存在: $AVIANS_WS"
        exit 1
    fi

    if [ ! -d "$SLAM_WS" ]; then
        log_error "SLAM工作空间不存在: $SLAM_WS"
        exit 1
    fi

    # 检查是否已编译
    if [ ! -f "$AVIANS_WS/install/setup.bash" ]; then
        log_error "AVIANS工作空间未编译"
        exit 1
    fi

    log_info "✅ 工作空间检查通过"
}

check_dependencies() {
    log_info "检查依赖..."

    check_command "ros2"
    check_command "gz"

    # 检查ROS2包
    source $AVIANS_WS/install/setup.bash

    local required_pkgs=("semantic_slam_msgs" "slam_integration" "mission_action_modules")
    for pkg in "${required_pkgs[@]}"; do
        if ! ros2 pkg list | grep -q "^${pkg}$"; then
            log_error "缺少ROS2包: $pkg"
            exit 1
        fi
    done

    log_info "✅ 依赖检查通过"
}

# ============ 启动函数 ============
launch_slam_modules() {
    log_info "启动SLAM模块..."

    mkdir -p $LOG_DIR/slam

    # 1. ORB-SLAM3
    ros2 launch orbslam3_ros2 orbslam3.launch.py \
        > $LOG_DIR/slam/orbslam3.log 2>&1 &
    local slam_pid=$!
    log_info "  ORB-SLAM3 启动 (PID: $slam_pid)"
    sleep 3

    # 2. Scene Graph
    ros2 launch scene_graph scene_graph.launch.py \
        > $LOG_DIR/slam/scene_graph.log 2>&1 &
    log_info "  Scene Graph 启动 (PID: $!)"
    sleep 2

    # 3. Semantic Planner
    ros2 launch semantic_planner planner.launch.py \
        > $LOG_DIR/slam/planner.log 2>&1 &
    log_info "  Semantic Planner 启动 (PID: $!)"
    sleep 2

    log_info "✅ SLAM模块启动完成"
}

launch_bridge() {
    log_info "启动桥接节点..."

    mkdir -p $LOG_DIR/bridge

    # 1. 位姿适配器
    ros2 run slam_integration slam_pose_adapter \
        > $LOG_DIR/bridge/pose_adapter.log 2>&1 &
    log_info "  位姿适配器启动 (PID: $!)"

    # 2. 检测桥接
    ros2 run slam_integration detection_bridge \
        > $LOG_DIR/bridge/detection.log 2>&1 &
    log_info "  检测桥接启动 (PID: $!)"

    # 3. 航点中继
    ros2 run slam_integration waypoint_relay \
        > $LOG_DIR/bridge/waypoint.log 2>&1 &
    log_info "  航点中继启动 (PID: $!)"

    sleep 2
    log_info "✅ 桥接节点启动完成"
}

launch_avians_stack() {
    log_info "启动AVIANS控制栈..."

    mkdir -p $LOG_DIR/avians

    # 1. Gazebo
    ros2 launch drone_description gz.launch.py \
        > $LOG_DIR/avians/gazebo.log 2>&1 &
    log_info "  Gazebo 启动 (PID: $!)"
    sleep 5

    # 2. 控制器
    ros2 launch drone_guidance_controllers comprehensive_option1_only_controllers.py \
        > $LOG_DIR/avians/controllers.log 2>&1 &
    log_info "  控制器栈启动 (PID: $!)"
    sleep 2

    # 3. Action Manager
    ros2 run mission_action_modules action_manager \
        > $LOG_DIR/avians/action_manager.log 2>&1 &
    log_info "  Action Manager 启动 (PID: $!)"

    # 4. Mission Executor
    ros2 run mission_executor mission_executor_node \
        > $LOG_DIR/avians/executor.log 2>&1 &
    log_info "  Mission Executor 启动 (PID: $!)"

    sleep 3
    log_info "✅ AVIANS控制栈启动完成"
}

# ============ 监控函数 ============
monitor_integration() {
    log_info "监控集成状态 (${TEST_DURATION}秒)..."

    local start_time=$(date +%s)
    local end_time=$((start_time + TEST_DURATION))

    while [ $(date +%s) -lt $end_time ]; do
        echo -n "."

        # 检查关键话题
        local topics=(
            "/camera_pose"
            "/X3/odometry"
            "/detections/objects"
            "/person_detections"
            "/planner/path"
        )

        local failed_topics=()
        for topic in "${topics[@]}"; do
            if ! ros2 topic info $topic &> /dev/null; then
                failed_topics+=($topic)
            fi
        done

        if [ ${#failed_topics[@]} -gt 0 ]; then
            echo ""
            log_warn "缺失话题: ${failed_topics[@]}"
        fi

        sleep 1
    done

    echo ""
    log_info "✅ 监控完成"
}

verify_integration() {
    log_info "验证集成结果..."

    local success=true

    # 1. 检查位姿转换
    log_info "1️⃣ 检查位姿转换..."
    if ros2 topic hz /X3/odometry --once --timeout 5 &> /dev/null; then
        local hz=$(ros2 topic hz /X3/odometry --window 50 2>&1 | grep "average rate" | awk '{print $3}')
        log_info "  /X3/odometry 频率: ${hz} Hz"
        if (( $(echo "$hz < 20" | bc -l) )); then
            log_warn "  位姿频率过低 (期望 30Hz)"
            success=false
        fi
    else
        log_error "  /X3/odometry 无数据"
        success=false
    fi

    # 2. 检查检测融合
    log_info "2️⃣ 检查检测融合..."
    if ros2 topic echo /person_detections --once --timeout 5 &> /dev/null; then
        log_info "  ✅ 检测数据正常"
    else
        log_error "  ❌ 无person检测数据"
        success=false
    fi

    # 3. 检查TF树
    log_info "3️⃣ 检查TF树..."
    if ros2 run tf2_ros tf2_echo world base_link &> /dev/null; then
        log_info "  ✅ TF链接 world → base_link 正常"
    else
        log_error "  ❌ TF变换失败"
        success=false
    fi

    # 4. 检查规划器
    log_info "4️⃣ 检查规划器..."
    if ros2 topic list | grep -q "/planner/path"; then
        log_info "  ✅ 规划器话题存在"
    else
        log_warn "  规划器话题不存在 (可能未触发规划)"
    fi

    # 生成报告
    echo ""
    if [ "$success" = true ]; then
        log_info "🎉 集成测试通过!"
    else
        log_error "❌ 集成测试失败,请检查日志: $LOG_DIR"
    fi

    log_info "详细日志目录: $LOG_DIR"
}

cleanup() {
    log_info "清理进程..."
    pkill -f "ros2 launch" || true
    pkill -f "gz sim" || true
    pkill -f "slam_integration" || true
    sleep 2
    log_info "✅ 清理完成"
}

# ============ 主流程 ============
main() {
    log_info "========================================="
    log_info "  SLAM + AVIANS 集成测试"
    log_info "========================================="

    # 捕获退出信号
    trap cleanup EXIT

    # 1. 前置检查
    check_workspaces
    check_dependencies

    # 2. 启动系统
    launch_slam_modules
    launch_bridge
    launch_avians_stack

    # 3. 监控与验证
    monitor_integration
    verify_integration

    log_info "测试完成! 按 Ctrl+C 退出"
    read -r -p "是否保存日志? (y/n): " response
    if [[ ! "$response" =~ ^[Yy]$ ]]; then
        rm -rf $LOG_DIR
        log_info "日志已删除"
    fi
}

# 执行主流程
main "$@"
```

---

### 2. 测试场景详解

#### 场景 1: 基础位姿对接测试

**目标**: 验证SLAM位姿能否驱动AVIANS控制器

**步骤**:
1. 启动SLAM的ORB-SLAM3节点
2. 播放EuRoC数据集bag文件
3. 启动位姿适配器 `slam_pose_adapter`
4. 启动AVIANS的Gazebo和控制器栈
5. 观察无人机是否根据SLAM位姿移动

**验收标准**:
- `/X3/odometry` 频率 ≥ 20Hz
- TF树 `world → base_link` 正确
- Gazebo中无人机位姿与SLAM估计一致 (误差 < 0.5m)

**命令**:
```bash
# Terminal 1: SLAM
ros2 launch orbslam3_ros2 orbslam3.launch.py

# Terminal 2: 播放数据
ros2 bag play euroc_mh01.bag

# Terminal 3: 桥接
ros2 run slam_integration slam_pose_adapter

# Terminal 4: AVIANS
ros2 launch drone_description gz.launch.py

# Terminal 5: 验证
ros2 topic hz /X3/odometry
ros2 run tf2_ros tf2_echo world base_link
```

---

#### 场景 2: 端到端路径跟踪测试

**目标**: 验证从规划到执行的完整流程

**步骤**:
1. 启动完整SLAM栈 (SLAM + Planner + Scene Graph)
2. 启动完整AVIANS栈 (Controllers + Action Manager)
3. 启动所有桥接节点
4. 发送任务: "飞到(10, 5, 2)并避开障碍物"
5. 观察无人机是否沿着A*/RRT*规划的路径飞行

**验收标准**:
- `/planner/path` 成功生成路径
- 路径航点正确传递到 `/drone/control/waypoint_command`
- 无人机在Gazebo中跟踪路径 (横向误差 < 1m)
- 避开已知障碍物

**命令**:
```bash
# Terminal 1: 启动集成系统
./slam_avians_integration_test.sh

# Terminal 2: 发送任务
ros2 service call /semantic_planner/plan_path \
  semantic_slam_msgs/srv/PlanPath \
  "{goal: {position: {x: 10.0, y: 5.0, z: 2.0}}}"

# Terminal 3: 监控路径
ros2 topic echo /planner/path
ros2 topic echo /drone/control/waypoint_command

# Terminal 4: RViz可视化
rviz2 -d slam_avians_integration.rviz
```

---

#### 场景 3: 语义任务执行测试

**目标**: 验证"搜索人员并降落"完整任务

**步骤**:
1. 启动完整集成系统
2. 在Gazebo场景中添加人员模型
3. 发送YAML任务:
   ```yaml
   mission:
     stages:
       - TAKEOFF (altitude=3.0)
       - QUERY_OBJECT (target="person")
       - NAVIGATE_TO_TARGET (offset=[3,0,0])
       - VALIDATE_SAFETY
       - LAND_AT_POINT
   ```
4. 观察执行流程

**验收标准**:
- Scene Graph成功检测到person
- SQL查询返回person位置
- 规划器生成到目标的路径
- 安全检查通过
- 无人机降落在person旁边3米处 (误差 < 0.5m)

**命令**:
```bash
# Terminal 1: 完整系统
./comprehensive_test_suite.sh  # 选择集成模式

# Terminal 2: 查询场景图
ros2 service call /scene_graph/query \
  semantic_slam_msgs/srv/QuerySceneGraph \
  "{class_filter: 'person', radius: 50.0}"

# Terminal 3: 执行任务
ros2 topic pub /mission_executor/plan std_msgs/String \
  "data: '$(cat missions/search_and_land.yaml)'"

# Terminal 4: 监控状态
ros2 topic echo /drone/state
ros2 topic echo /mission_executor/status
```

---

## 下一步行动

### 立即行动 (本周)
1. **与上游同事确认**:
   - 坐标系约定 (world frame定义, ENU/NED)
   - 相机外参标定方法
   - 消息定义 (特别是 `/planner/path` 格式)

2. **环境准备**:
   - 获取SLAM代码库访问权限
   - 克隆 `dev/sabrina/semantic-planner` 分支
   - 在AVIANS环境中验证SLAM包编译

3. **依赖检查**:
   - 运行 `rosdep check` 验证依赖
   - 安装 BehaviorTree.CPP v4
   - 准备EuRoC测试数据集

### 短期计划 (2周内)
1. **Phase 1实施**:
   - 创建 `slam_integration` 包
   - 实现 `slam_pose_adapter` 节点
   - 标定相机-机体外参 (使用Kalibr)
   - 运行场景1测试

2. **接口验证**:
   - 端到端位姿测试 (SLAM → 控制器)
   - TF树验证
   - 频率和延迟测量

### 中期目标 (1-2月)
1. **Phase 2-3实施**:
   - 完成所有5个集成接口
   - 实现 `mission_translator`
   - 扩展AVIANS Action Modules

2. **仿真验证**:
   - Gazebo全流程测试
   - 运行场景2和场景3
   - 性能调优

3. **真机准备**:
   - 硬件标定
   - 飞控参数配置
   - 安全预案制定

---

## 实施优先级与建议

### 阶段1: 最小可验证集成 (优先级: 🔴 最高)

**时间**: Week 1-2
**目标**: 验证SLAM位姿能驱动AVIANS控制器

**关键任务**:
1. 创建 `slam_integration` 包结构
2. 实现 `slam_pose_adapter` 节点 (200行代码)
3. 编写单元测试 (pytest)
4. 运行场景1测试

**成功标准**:
- SLAM bag播放时,Gazebo无人机同步移动
- 位姿延迟 < 50ms
- TF树完整无断裂

**风险控制**:
- 如果坐标系转换有问题,先硬编码已知外参测试
- 如果频率不够,使用插值或卡尔曼滤波
- 准备降级方案: 使用Gazebo odometry作为备份

---

### 阶段2: 感知管道对接 (优先级: 🟠 高)

**时间**: Week 3-4
**目标**: 统一检测系统,获得语义理解能力

**关键任务**:
1. 实现 `detection_bridge` 节点
2. 修改 `TrackTargetModule` 使用 `tracking_id`
3. 测试持久跟踪效果
4. (可选) 扩展支持更多类别

**成功标准**:
- 检测数据正确转换
- 跟踪ID在遮挡后恢复
- 检测延迟 < 100ms

**优化建议**:
- 保留AVIANS原检测器作为备份
- 使用ROS2 QoS配置保证可靠传输
- 添加检测丢失监控和告警

---

### 阶段3: 路径规划集成 (优先级: 🟡 中)

**时间**: Week 5-6
**目标**: 使用语义A*和RRT*生成避障路径

**关键任务**:
1. 编译并测试 `semantic_planner` 包
2. 实现 `waypoint_relay` 节点
3. 扩展 `FlyToModule` 支持多航点跟踪
4. 调整 NMPC 参数适配新路径

**成功标准**:
- 规划路径成功发布
- 无人机平滑跟踪路径
- 动态避障生效

**技术债务**:
- 暂不集成ESDF到NMPC (Phase 4)
- 先使用全局避障,后续添加局部避障

---

### 阶段4: 高层任务执行 (优先级: 🟢 中低)

**时间**: Week 7-8
**目标**: 实现BT驱动的语义任务

**关键任务**:
1. 安装 BehaviorTree.CPP v4
2. 编译 SLAM `mission_executor`
3. 实现 `mission_translator`
4. 运行场景3测试

**成功标准**:
- 完整"搜索与降落"任务成功
- SQL查询正确返回目标
- 两层Executor协同工作

**扩展方向**:
- 自定义BT节点 (巡检、充电)
- 多机协同任务
- 动态任务重规划

---

### 阶段5: 生产级优化 (优先级: 🔵 低)

**时间**: Week 9+
**目标**: 性能调优和鲁棒性增强

**优化清单**:
- [ ] 添加EKF融合SLAM和IMU
- [ ] 实现SLAM丢失检测和降级
- [ ] NMPC集成ESDF梯度避障
- [ ] 添加Prometheus监控和Grafana仪表板
- [ ] Docker容器化部署
- [ ] CI/CD自动化测试

---

## 常见问题与解决方案

### Q1: SLAM初始化失败怎么办?

**症状**: `/camera_pose` 无数据,日志显示 "Tracking lost"

**原因**:
- 纹理不足 (白墙场景)
- 运动过快 (特征丢失)
- IMU未标定

**解决方案**:
```bash
# 1. 检查特征点数量
ros2 topic echo /slam/features --once
# 期望: > 100 features

# 2. 降低运动速度
# 在 FlyToModule 中限制速度:
max_velocity: 1.0  # m/s (默认3.0)

# 3. 重新标定IMU
kalibr_calibrate_imu_camera \
  --bag mh01.bag \
  --cam camchain.yaml \
  --imu imu.yaml \
  --target april_6x6.yaml
```

---

### Q2: 桥接节点延迟过高?

**症状**: `/X3/odometry` 延迟 > 100ms

**原因**:
- Python节点性能瓶颈
- 矩阵运算未优化
- ROS2 DDS配置不当

**解决方案**:
```cpp
// 改用C++实现关键节点
// slam_pose_adapter_node.cpp
class SLAMPoseAdapter : public rclcpp::Node {
    void slam_callback(const PoseStamped::SharedPtr msg) {
        // 使用Eigen加速矩阵运算
        Eigen::Affine3d T_world_cam = tf2::transformToEigen(msg->pose);
        Eigen::Affine3d T_world_base = T_world_cam * T_base_cam_;

        // 零拷贝发布
        auto odom = std::make_unique<Odometry>();
        odom->header = msg->header;
        // ... 填充数据
        odom_pub_->publish(std::move(odom));
    }
};
```

---

### Q3: TF树出现断裂?

**症状**: `tf2_echo` 报错 "Lookup would require extrapolation"

**原因**:
- 时间戳不同步
- TF发布频率不够
- 时钟源不一致

**解决方案**:
```python
# 在 slam_pose_adapter 中添加
self.tf_broadcaster = TransformBroadcaster(self)

def slam_cb(self, msg):
    # 发布 world → base_link TF
    t = TransformStamped()
    t.header = msg.header
    t.child_frame_id = 'base_link'
    t.transform = pose_to_transform(msg.pose)

    self.tf_broadcaster.sendTransform(t)

    # 同时发布 base_link → camera_link (静态外参)
    static_t = TransformStamped()
    static_t.header.stamp = msg.header.stamp
    static_t.header.frame_id = 'base_link'
    static_t.child_frame_id = 'camera_link'
    static_t.transform = self.T_base_cam_inverse

    self.static_tf_broadcaster.sendTransform(static_t)
```

---

## 附录: 快速参考

### 关键话题列表

| 话题 | 类型 | 发布者 | 订阅者 | 频率 |
|-----|------|--------|--------|------|
| `/camera_pose` | PoseStamped | ORB-SLAM3 | bridge, planner | 30Hz |
| `/detections/objects` | ObjectInstanceArray | YOLOE | scene_graph, bridge | 2Hz |
| `/planner/path` | Path | semantic_planner | waypoint_relay | 事件 |
| `/scene_graph/objects` | ObjectInstanceArray | scene_graph | mission_executor | 10Hz |
| `/X3/odometry` | Odometry | bridge | controllers | 30Hz |
| `/drone/control/waypoint_command` | PoseStamped | action_modules | waypoint_controller | 变化时 |
| `/X3/cmd_vel` | Twist | velocity_adapter | Gazebo/PX4 | 160Hz |

### 关键服务列表

| 服务 | 类型 | 提供者 | 调用者 |
|-----|------|--------|--------|
| `/scene_graph/query` | QuerySceneGraph | scene_graph | mission_executor |
| `/safety/check_landing` | CheckLanding | drone_safety | land_module |
| `/semantic_planner/plan_path` | PlanPath | semantic_planner | fly_to_module |
| `/mission_actions/*` | Trigger | action_manager | mission_executor |

### 快速诊断命令

```bash
# 检查SLAM状态
ros2 topic hz /camera_pose
ros2 topic echo /slam/reset --once

# 检查感知
ros2 topic hz /detections/objects
ros2 service call /scene_graph/query semantic_slam_msgs/srv/QuerySceneGraph "{class_filter: 'person', radius: 50.0}"

# 检查规划
ros2 topic echo /planner/path --once
ros2 topic list | grep planner

# 检查控制
ros2 topic hz /X3/cmd_vel
ros2 topic echo /drone/controller/status

# TF树验证
ros2 run tf2_tools view_frames
evince frames.pdf

# 性能分析
ros2 run tf2_tools tf2_monitor world base_link
ros2 topic bw /X3/odometry
ros2 topic delay /camera_pose /X3/odometry
```

---

## 总结与展望

### 集成后的系统能力

通过SLAM和AVIANS的深度集成,最终系统将具备以下能力:

**感知层**:
- ✅ 视觉惯性SLAM定位 (无GPS)
- ✅ 111类语义目标检测
- ✅ 持久多目标跟踪 (ByteTrack)
- ✅ 空间关系推理 (Scene Graph)
- ✅ TSDF/ESDF密集建图

**规划层**:
- ✅ 语义代价地图生成
- ✅ A*全局路径规划
- ✅ RRT*局部平滑优化
- ✅ 3层安全膨胀避障
- ✅ 信息增益驱动探索

**执行层**:
- ✅ BehaviorTree高层任务解析
- ✅ 状态机低层动作编排
- ✅ 11个可复用Action模块
- ✅ SQL驱动的语义任务
- ✅ 实时安全监控服务

**控制层**:
- ✅ NMPC轨迹优化
- ✅ PID三层控制器
- ✅ 速度/加速度限制
- ✅ 故障检测与恢复

### 技术亮点

1. **两层Executor架构**: 清晰的职责分离,便于并行开发和调试
2. **独立桥接包设计**: 解耦两套代码,支持灵活部署
3. **软链接工作空间方案**: 不破坏原有结构,易于版本管理
4. **完整测试框架**: 3个场景覆盖从基础到高级的验证
5. **渐进式集成路线**: 5个阶段从简单到复杂,风险可控

### 后续演进方向

**短期 (3个月)**:
- 完成真机飞行测试
- 性能调优和参数整定
- 增加更多任务场景 (巡检、充电)

**中期 (6个月)**:
- 多机协同任务
- 动态环境下的在线重规划
- 学习型规划器 (基于历史数据优化代价)

**长期 (1年+)**:
- 端到端视觉语言导航 (VLN)
- 强化学习策略优化
- 云边协同计算架构

---

**文档版本**: v2.0
**最后更新**: 2025-12-02
**维护者**: AVIANS团队

🎯 **下一步**: 立即与上游同事确认坐标系约定和消息格式,然后开始Phase 1实施!

**联系方式**:
- SLAM团队: 同事 (dev/sabrina/semantic-planner)
- AVIANS团队: 本地工作空间 (~/AVIANS_ROS2)

**相关资源**:
- SLAM代码库: `~/SLAM/semantic_slam_ws`
- 测试数据: EuRoC MH_01 dataset
- 标定工具: Kalibr toolbox
- BT框架: BehaviorTree.CPP v4 documentation

## 实施路线图 (8周)

### Phase 1: 基础对接 (Week 1-2)

**任务**:
- [ ] 克隆SLAM仓库 (`dev/sabrina/semantic-planner`分支)
- [ ] 创建 `slam_integration` 包
- [ ] 实现 `slam_pose_adapter`
- [ ] 测试SLAM位姿驱动Gazebo
- [ ] 统一检测器 (`detection_bridge`)

**验收标准**:
- SLAM `/camera_pose` → AVIANS `/X3/odometry`
- YOLOE检测 → `/person_detections`
- TF树正确 (`world → base_link → cam0`)

---

### Phase 2: 规划器集成 (Week 3-4)

**任务**:
- [ ] 编译 `semantic_planner` 包
- [ ] 测试 A*/RRT* 规划
- [ ] 实现 `waypoint_relay_node`
- [ ] 扩展 `FlyToModule` 支持路径跟踪

**验收标准**:
- `/planner/path` → Waypoint Controller
- Gazebo中无人机跟踪规划路径
- 避障功能验证

---

### Phase 3: 任务执行融合 (Week 5-6)

**任务**:
- [ ] 编译 `mission_executor` (SLAM的BT版本)
- [ ] 安装 BehaviorTree.CPP v4
- [ ] 实现 `mission_translator`
- [ ] 定义统一消息接口 (`slam_interfaces` 包)
- [ ] 扩展AVIANS Action Modules

**验收标准**:
- SLAM Executor生成的YAML → AVIANS执行
- 完整"搜索与降落"场景

---

### Phase 4: 语义感知 (Week 7-8)

**任务**:
- [ ] 集成 Scene Graph 查询
- [ ] 集成 Safety Services
- [ ] NMPC 集成 ESDF避障
- [ ] 完整场景测试

**验收标准**:
- SQL查询驱动任务
- 动态避障生效
- 多场景鲁棒性测试

---

## 需要与上游同事确认的清单

### 1. 坐标系约定
- [ ] World frame定义 (ENU? 原点?)
- [ ] 相机外参 `T_base_cam`
- [ ] IMU外参

### 2. 消息定义
- [ ] `/planner/path` 完整定义
- [ ] BT Executor输出格式
- [ ] Safety Service `.srv` 文件

### 3. 更新频率
- [ ] `/camera_pose`: 30Hz?
- [ ] `/planner/path`: 重规划频率?
- [ ] `/scene_relationships`: 更新率?

### 4. 部署架构
- [ ] 两个系统运行在哪里? (同一台PC?)
- [ ] 网络配置 (`ROS_DOMAIN_ID`?)
- [ ] Bag 文件测试数据

### 5. 测试数据
- [ ] EuRoC MH_01 bag
- [ ] 自定义 Gazebo 场景
- [ ] 真实飞行数据

---

## 总结

### SLAM系统优势

✅ **完整VIO定位** (ORB-SLAM3)
✅ **丰富语义理解** (111类 + 关系推理)
✅ **智能路径规划** (语义A* + 3层安全RRT*)
✅ **BT任务框架** (可扩展)

### AVIANS系统优势

✅ **成熟控制栈** (NMPC + 3层控制器)
✅ **灵活动作系统** (Action Manager + Modules)
✅ **状态机Executor** (轻量高效)
✅ **完整仿真环境** (Gazebo + RViz)

### 集成后能力

**SLAM感知+规划 + AVIANS执行+控制 = 完整无GPS自主飞行系统**

---

## 关键技术决策

### 决策1: 使用两层Executor架构 ✅

**理由**:
- SLAM Executor (BT): 高层任务解析、语义理解、路径规划
- AVIANS Executor (SM): 低层动作编排、实时执行、故障恢复
- 清晰的职责分离,两个团队可并行开发

### 决策2: 保留所有AVIANS Action Modules ✅

**理由**:
- 已验证的控制逻辑和参数
- 只需修改数据来源接口(20-40%工作量)
- 避免重复开发和调试

### 决策3: 统一使用SLAM的感知系统 ✅

**理由**:
- 节省算力(一套YOLO)
- 更强的语义理解(111类 vs 1类)
- 持久跟踪ID(ByteTrack)

### 决策4: 创建独立桥接包 ✅

**理由**:
- 解耦SLAM和AVIANS代码
- 易于版本管理和测试
- 支持多种部署场景(单机/分布式)

---

## 风险评估与缓解

| 风险 | 影响 | 概率 | 缓解措施 |
|-----|------|------|---------|
| 坐标系标定误差 | 高 | 中 | 使用Kalibr工具,离线验证 |
| SLAM丢失tracking | 高 | 中 | 惯性导航降级,紧急悬停 |
| 服务调用延迟 | 中 | 低 | 异步调用,超时重试 |
| BT学习曲线 | 低 | 高 | 先用简单任务,增量学习 |
| 网络通信故障 | 高 | 低 | 单机部署优先,DDS QoS配置 |
| 依赖版本冲突 | 中 | 低 | 统一ROS2 Jazzy,Docker容器化 |

---

## 下一步行动

### 立即行动 (本周)
1. 与上游同事确认坐标系约定和消息定义
2. 获取SLAM代码库访问权限
3. 在AVIANS环境中编译SLAM包(验证依赖)

### 短期计划 (2周内)
1. 实现 `slam_pose_adapter` 桥接节点
2. 标定相机-机体外参
3. 端到端位姿测试(SLAM → AVIANS控制器)

### 中期目标 (1-2月)
1. 完成Phase 1-3集成
2. Gazebo仿真全流程验证
3. 准备真机测试环境

---

## 附录: 快速参考

### 关键话题列表

| 话题 | 类型 | 发布者 | 订阅者 | 频率 |
|-----|------|--------|--------|------|
| `/camera_pose` | PoseStamped | ORB-SLAM3 | bridge, planner | 30Hz |
| `/detections/objects` | ObjectInstanceArray | YOLOE | scene_graph, bridge | 2Hz |
| `/planner/path` | Path | semantic_planner | waypoint_relay | 事件 |
| `/scene_graph/objects` | ObjectInstanceArray | scene_graph | mission_executor | 10Hz |
| `/X3/odometry` | Odometry | bridge | controllers | 30Hz |
| `/drone/control/waypoint_command` | PoseStamped | action_modules | waypoint_controller | 变化时 |
| `/X3/cmd_vel` | Twist | velocity_adapter | Gazebo/PX4 | 160Hz |

### 关键服务列表

| 服务 | 类型 | 提供者 | 调用者 |
|-----|------|--------|--------|
| `/scene_graph/query` | QuerySceneGraph | scene_graph | mission_executor |
| `/safety/check_landing` | CheckLanding | drone_safety | land_module |
| `/semantic_planner/plan_path` | PlanPath | semantic_planner | fly_to_module |
| `/mission_actions/*` | Trigger | action_manager | mission_executor |

### 快速诊断命令

```bash
# 检查SLAM状态
ros2 topic hz /camera_pose
ros2 topic echo /slam/reset --once

# 检查感知
ros2 topic hz /detections/objects
ros2 service call /scene_graph/query semantic_slam_msgs/srv/QuerySceneGraph "{class_filter: 'person', radius: 50.0}"

# 检查规划
ros2 topic echo /planner/path --once
ros2 topic list | grep planner

# 检查控制
ros2 topic hz /X3/cmd_vel
ros2 topic echo /drone/controller/status

# TF树验证
ros2 run tf2_tools view_frames
evince frames.pdf
```

---

**文档版本**: v1.0
**最后更新**: 2025-12-02
**维护者**: AVIANS团队

您们的分工非常合理,集成的关键是清晰定义接口和分层架构。建议先从Phase 1开始,逐步验证每个接口!
---

# SLAM + AVIANS 集成进度报告

**日期**: 2025-12-03
**方案**: 方案B - BT直接调用AVIANS Action Modules
**状态**: 接口实现完成，待安装依赖和测试

---

## ✅ 已完成的工作

### 1. Git分支创建
- ✅ AVIANS_ROS2: 创建 `integration` 分支
- ✅ SLAM: 创建 `soja_integration` 分支

### 2. 统一Action接口定义
- ✅ 创建 `uav_msgs/action/ExecuteModule.action`
- ✅ 支持所有11个AVIANS Modules的统一接口
- ✅ 使用JSON格式传递灵活参数
- ✅ 更新CMakeLists.txt并编译成功

**文件位置**:
- `/home/soja/AVIANS_ROS2/src/custom_msgs/uav_msgs/action/ExecuteModule.action`

### 3. AVIANS侧: Action Server实现
- ✅ 创建 `action_server_node.py` 暴露所有Modules
- ✅ 注册11个Action Modules到统一Action Server
- ✅ 使用MultiThreadedExecutor支持并发
- ✅ 更新setup.py添加entry point
- ✅ 更新package.xml添加uav_msgs依赖
- ✅ 编译成功

**文件位置**:
- `/home/soja/AVIANS_ROS2/src/mission_action_modules/mission_action_modules/action_server_node.py`

**Action Server接口**:
- Action Name: `/avians/execute_module`
- Action Type: `uav_msgs/action/ExecuteModule`

**支持的Modules**:
1. TAKEOFF
2. HOVER
3. FLY_TO
4. TRACK
5. SEARCH
6. INSPECT
7. LOST_HOLD
8. LAND
9. DELIVERY
10. SEARCH_AREA
11. AVOIDANCE

### 4. SLAM侧: BT Client节点实现
- ✅ 创建 `avians_action_nodes.hpp` 定义7个核心BT客户端节点
- ✅ 创建 `avians_action_nodes.cpp` 实现节点逻辑
- ✅ 在 `mission_manager.cpp` 中注册所有AVIANS节点到BT Factory
- ✅ 创建CMakeLists.txt用于编译

**文件位置**:
- `/home/soja/SLAM/semantic_slam_ws/src/mission_executor/include/mission_executor/nodes/avians_action_nodes.hpp`
- `/home/soja/SLAM/semantic_slam_ws/src/mission_executor/src/nodes/avians_action_nodes.cpp`
- `/home/soja/SLAM/semantic_slam_ws/src/mission_executor/CMakeLists.txt`

**BT节点列表**:
1. `AviansTakeoff` - 起飞
2. `AviansLand` - 降落
3. `AviansFlyTo` - 飞往目标
4. `AviansHover` - 悬停
5. `AviansTrack` - 目标跟踪
6. `AviansSearch` - 搜索
7. `AviansInspect` - 检视

---

## ⏳ 待完成的工作

### 1. 安装依赖 (SLAM侧)
**问题**: mission_executor编译失败，缺少BehaviorTree.CPP库

**解决方案**:
```bash
# 安装BehaviorTree.CPP v4
sudo apt update
sudo apt install ros-jazzy-behaviortree-cpp

# 或者从源码编译
git clone --branch v4.0.0 https://github.com/BehaviorTree/BehaviorTree.CPP.git
cd BehaviorTree.CPP
mkdir build && cd build
cmake ..
make
sudo make install
```

### 2. 编译SLAM workspace
```bash
cd /home/soja/SLAM/semantic_slam_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-up-to mission_executor --symlink-install
```

### 3. 实现真正的Action Client调用
**当前状态**: `avians_action_nodes.cpp` 中使用的是模拟实现（TODO标记）

**需要实现**:
在 `AviansModuleClientNode::onStart()` 中添加真实的ROS2 Action客户端：
```cpp
// TODO: 创建 rclcpp_action::Client
// TODO: 发送goal到 /avians/execute_module
// TODO: 在 onRunning() 中检查feedback和result
```

### 4. 创建测试任务YAML
创建一个简单的测试任务来验证集成：

**示例任务**: `/home/soja/SLAM/semantic_slam_ws/src/mission_executor/trees/test_avians_integration.yaml`
```yaml
mission:
  name: "test_avians_integration"
  initial_state: "takeoff"
  states:
    - id: "takeoff"
      type: "avians_takeoff"
      params:
        altitude: 3.0
      transitions:
        success: "hover"
        failure: "complete"

    - id: "hover"
      type: "avians_hover"
      params:
        duration: 5.0
      transitions:
        success: "land"

    - id: "land"
      type: "avians_land"
      transitions:
        success: "complete"

    - id: "complete"
      type: "terminal"
```

### 5. 端到端测试
```bash
# Terminal 1: 启动AVIANS Action Server
cd /home/soja/AVIANS_ROS2
source install/setup.bash
ros2 run mission_action_modules action_server

# Terminal 2: 启动SLAM Mission Executor
cd /home/soja/SLAM/semantic_slam_ws
source install/setup.bash
ros2 run mission_executor mission_executor_node \
  --ros-args -p mission_file:=trees/test_avians_integration.yaml

# Terminal 3: 监控
ros2 topic echo /avians/execute_module/_action/feedback
ros2 topic echo /avians/execute_module/_action/status
```

---

## 🎯 架构总结

### 接口流程
```
SLAM BT Executor
    │
    ├─> AviansTakeoffNode (BT Node)
    │       │
    │       └─> /avians/execute_module (ROS2 Action)
    │               │
    │               └─> ActionModuleServer
    │                       │
    │                       └─> TakeoffModule.start(goal)
    │                               │
    │                               └─> /drone/control/waypoint_command
    │                                       │
    │                                       └─> AVIANS Controllers
```

### 关键消息流
1. **Goal**: BT构建JSON参数 → Action Goal
2. **Feedback**: Module进度 → Action Feedback → BT RUNNING
3. **Result**: Module结果 → Action Result → BT SUCCESS/FAILURE

---

## 📝 后续优化

### Phase 2: 完整Action Client实现
- 使用 `rclcpp_action::Client<uav_msgs::action::ExecuteModule>`
- 添加超时处理
- 添加取消操作支持

### Phase 3: 复杂任务测试
- 测试"搜索与降落"完整场景
- 测试FlyTo + Planner集成
- 测试Track + NMPC集成

### Phase 4: 错误处理与恢复
- BT Fallback节点
- 超时重试策略
- 紧急降落逻辑

---

## 📂 关键文件清单

### AVIANS_ROS2 (integration分支)
- `src/custom_msgs/uav_msgs/action/ExecuteModule.action`
- `src/custom_msgs/uav_msgs/CMakeLists.txt`
- `src/mission_action_modules/mission_action_modules/action_server_node.py`
- `src/mission_action_modules/setup.py`
- `src/mission_action_modules/package.xml`

### SLAM (soja_integration分支)
- `src/mission_executor/include/mission_executor/nodes/avians_action_nodes.hpp`
- `src/mission_executor/src/nodes/avians_action_nodes.cpp`
- `src/mission_executor/src/mission_manager.cpp`
- `src/mission_executor/CMakeLists.txt`

---

## 🚀 启动命令速查

### AVIANS Action Server
```bash
cd ~/AVIANS_ROS2
source install/setup.bash
ros2 run mission_action_modules action_server
```

### SLAM Mission Executor
```bash
cd ~/SLAM/semantic_slam_ws
source install/setup.bash
ros2 run mission_executor mission_executor_node
```

### 验证接口
```bash
# 查看Action Server
ros2 action list
# 应该看到: /avians/execute_module

# 测试单个Module
ros2 action send_goal /avians/execute_module uav_msgs/action/ExecuteModule \
  "{module_name: 'TAKEOFF', goal_json: '{\"target_altitude\": 3.0}'}"
```

---

**下一步**: 安装BehaviorTree.CPP库并完成SLAM侧编译
