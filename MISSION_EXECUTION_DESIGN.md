# Mission Execution & Scene Graph Integration Design

## 1. High-Level Architecture
- **Scene Graph Interface Adapter**
  - Consumes SQL query results, safety function flags, relational detections, and motion predictions exposed by the upstream team.
  - Normalises outputs into ROS2 messages (e.g., `mission_msgs::SceneContext`, `mission_msgs::TaskDirective`).
- **Mission Executor**
  - Behavior Tree (BT) built with BehaviorTree.CPP (or equivalent) managing mission progress.
  - Root node is a Priority Selector: branch 0 handles safety overrides, branch 1 orchestrates mission sequences (`Takeoff → Navigate → Search → Land`).
  - Maintains a blackboard containing current pose, mission goals, constraints, and action status.
- **Action Module Manager**
  - Dynamically loads and coordinates individual Action Modules via a standard `start/cancel/feedback` interface.
  - Enforces priority rules (e.g., avoidance pre-empts everything else).
- **Action Modules**
  - Implement atomic behaviours (takeoff, fly-to, search, track, land, avoid, etc.).
  - Interface with existing NMPC, low-level PID, perception, and safety nodes.
- **Safety Supervisor**
  - Subscribes directly to Scene Graph safety functions and robot health topics.
  - Issues interrupts to Mission Executor when violations or emergencies are detected.

```
Ground Task Text  ─► Scene Graph Interface ─► Adapter ─► Mission Executor ─► Action Module Manager ─► NMPC / Controllers
                                              ▲                 │
                                              │                 ▼
                              Safety Functions / Motion Predictions ─► Safety Supervisor
```

## 2. Scene Graph Interface Utilisation
- **SQL Query Engine**
  - Upstream provides templated SQL queries for environment semantics.
  - Mission Executor requests tasks via ROS2 service `mission/sql_query` (inputs: template name, parameters) and receives structured rows.
  - Example: `SELECT door FROM street_scene WHERE house_number = 5;` → returns coordinates + attributes.
- **Safety Functions**
  - Safety adapter subscribes to topics such as `scene_graph/check_collision_zone`.
  - When a flag triggers (e.g., `collision_warning = TRUE`), Safety Supervisor issues a `mission_msgs::Interrupt` with reason `COLLISION_PREDICTED`.
- **Relational Detection**
  - Provides contextual relationships (e.g., “yellow garbage bin left of door”).
  - Mission Executor uses this to chain actions: `FlyToTarget(door_pose) → SearchArea(bin_left_zone) → Land`.
- **Motion Prediction**
  - Supplies moving-object trajectories.
  - Action Modules (especially `TrackTarget` and `AvoidDynamicObstacle`) update setpoints based on predicted motion.

## 3. Mission Executor (Behavior Tree) Workflow
The executor runs a BehaviorTree.CPP tree ticked at ~10 Hz. The root Priority node first evaluates the `SafetyOverride` subtree; if clear, control flows to the mission `Sequence` subtree whose leaves encapsulate Action Modules (`TakeoffNode`, `FlyToTargetNode`, etc.). Blackboard entries (current goal, updated waypoints, detection flags) are shared via `BT::Blackboard`.

1. **Task Reception**
   - Receive `mission_msgs::TaskDirective` containing parsed DSL/NLP output (mission ID, goal schema, optional keywords).
   - Query Scene Graph for required locations/objects via SQL adapter.
2. **Planning**
   - Populate the blackboard with mission steps (ordered list or BT nodes) using templates (e.g., `VisitLocation`, `InspectObject`, `DeliverAction`).
   - Each step binds to an Action Module with parameters derived from scene graph results.
3. **Execution Loop**
   - Activate the next Action Module, monitor feedback via `mission_msgs::ActionFeedback`.
   - Handle statuses:
     - `RUNNING`: continue monitoring.
     - `SUCCESS`: commit state, move to next step.
     - `FAILED`: attempt recovery strategy (retry, fallback action, abort mission).
     - `PREEMPTED`: re-plan after safety event.
4. **Safety Handling**
   - On interrupt, push the current action onto a stack, trigger high-priority module (`Avoidance`, `EmergencyHover`), wait for clearance, then resume or re-plan.
5. **Completion & Reporting**
   - Publish `mission_msgs::MissionStatus` with timeline, success or failure reasons, and high-level logs for debriefing.

## 4. Action Module Specification
All modules implement an interface:
```text
configure(parameters)
start(goal) -> ActionHandle
cancel(ActionHandle)
on_feedback(ActionHandle, status, telemetry)
```

### Core Modules
| Module | Purpose | Inputs | Outputs / Topics |
|--------|---------|--------|------------------|
| `TakeoffModule` | Vertical ascent to mission altitude | `target_altitude`, `safety_zone` | Publishes `/drone/control/waypoint_command`, waits for altitude band |
| `FlyToTargetModule` | Navigate from A to B | `target_pose`, `tolerance`, `cruise_speed` | Feeds local waypoints to NMPC (`/drone/control/waypoint_command`) |
| `SearchAreaModule` | Execute area scan pattern | `region_polygon`, `pattern_type`, `dwell_time` | Generates sweep waypoints; monitors perception feedback |
| `TrackTargetModule` | Maintain visual lock | `target_class`, `handedness`, `stand_off_distance` | Sets NMPC `target_position`/`target_class`; monitors YOLO detections |
| `InspectObjectModule` | Align sensors toward object | `object_pose`, `camera_mode` | Controls attitude commands & gimbal (future) |
| `HoverModule` | Hold position | `target_pose`, `duration` | Keeps NMPC in HOLD state |
| `LandModule` | Controlled descent | `touchdown_pose`, `approach_vector` | Publishes descent profile to controllers |
| `AvoidanceModule` | Handle collision alerts | `avoidance_vector`, `hold_timeout` | Immediately overrides current commands, coordinates with Safety Supervisor |

### Optional / Future Modules
- `DeliverPayloadModule`, `WaypointSurveyModule`, `DynamicRelocateModule`.

Each module should:
- Adhere to ROS2 action semantics (use `rclcpp_action` or custom topics).
- Provide heartbeat telemetry (`mission_msgs::ActionTelemetry`) for mission logging.
- Validate prerequisites (e.g., ensure global pose available before FlyToTarget).

## 5. ROS2 Interfaces

### Topics
- `/mission/task_directive` (`mission_msgs::TaskDirective`)
- `/mission/action_command` / `/mission/action_feedback`
- `/mission/interrupts` (`mission_msgs::Interrupt`)
- `/mission/status` (`mission_msgs::MissionStatus`)
- `/scene_graph/sql_query` (service) and `/scene_graph/query_result` (topic)
- `/scene_graph/safety_flags`, `/scene_graph/motion_prediction`

### Parameters / Configuration
- YAML (`mission_executor_config.yaml`) describing:
  - Action module plugin list.
  - Safety thresholds (timeouts, collision margins).
  - Scene graph query templates (name, SQL string, expected fields).

### Logging
- ROS2 rosbag for mission timeline.
- `/tmp/mission_executor.log` for detailed trace and replay.

## 6. Example Mission Flow
Mission text: “Find house number 5 on XXXXX street and land to the left of the yellow garbage bin.”

1. **Directive**: Adapter emits `TaskDirective{id=42, goal=FindHouse, params={street:"XXXXX", house_number:5}}`.
2. **Scene Graph Queries**:
   - `house_pose = SQL("FIND house WHERE street='XXXXX' AND number=5")`
   - `door_pose, bin_pose = SQL("FIND door, garbage_bin ...")`
   - `left_zone = RELATION(bin_pose, relation='left_area')`
3. **Plan**:
   - `Takeoff(target_altitude=15m)`
   - `FlyToTarget(target_pose=house_pose.approach_point)`
   - `SearchArea(region=door_pose.foyer, pattern='lawnmower')`
   - `TrackTarget(target_class='yellow_garbage_bin')` (optional, to confirm)
   - `FlyToTarget(target_pose=left_zone.touchdown)`
   - `Land(touchdown_pose=left_zone.touchdown)`
4. **Safety**: If `collision_warning` triggers during descent, pre-empt with `AvoidanceModule`, hover, and re-plan landing.

## 7. Upstream Interface Requirements & Invocation
- **Required Artifacts from Scene Graph Team**
  - **SQL Template Catalogue**: list of template IDs, SQL strings, required parameters, and returned column schema (including coordinate frames and units).
  - **Mission Text Parser Output**: message definition for `mission_msgs::TaskDirective` (fields for mission ID, goal type, semantic parameters). If parsing remains upstream, provide a ROS publisher endpoint; otherwise share grammar/spec so we can reimplement locally.
  - **Safety Function Topics**: topic names, message types, severity levels, and timing guarantees for collision prediction, clearance checking, and moving-object detection.
  - **Relational/Motion Topics**: message schema for object relationships (`left_of`, `near`, etc.) and motion predictions (trajectory array, validity horizon).
  - **Coordinate Convention**: clarification of world frame, ENU/NED conversions, and how polygons/poses are encoded (e.g., GeoJSON, array of points).
- **Invocation Flow (from mission start)**
  1. Receive `TaskDirective` from upstream publisher or request it through a provided service.
  2. For each mission phase, call `scene_graph/sql_query` with `{template_id, parameters}` (ROS2 service). Upstream responds with structured rows (`SceneGraphQueryResult` message).
  3. Subscribe to safety and relational topics continuously; Safety Supervisor listens for any non-zero `severity` and triggers `mission_msgs::Interrupt`.
  4. When relational context changes (e.g., new bin location), upstream publishes an update message; Mission Executor updates the BT blackboard and re-ticks affected subtrees.
  5. Optional: request motion predictions by posting to `scene_graph/request_motion_prediction` and listening on `scene_graph/motion_prediction`.

## 8. Testing & Validation
- **Unit Tests**: Mock Scene Graph responses, ensure Mission Executor transitions correctly.
- **Integration Tests**: Use Gazebo scenarios with scripted scene graph outputs.
- **Stress Cases**: Rapid scene changes, missing detections, repeated interrupts.
- **KPIs**: Mission completion rate, mean time between interrupts, recovery success percentage.

## 9. Handover Notes for Upstream Team
- Provide final message definitions and parameter schemas for the adapter.
- Share SQL template catalogue (name, description, inputs, return schema).
- Agree on safety flag semantics (enumerated reasons, severity levels).
- Establish versioned interface contracts to decouple mission logic from scene graph changes.

This document should enable both teams to coordinate the mission layer implementation while fully leveraging the Scene Graph Interface capabilities.
