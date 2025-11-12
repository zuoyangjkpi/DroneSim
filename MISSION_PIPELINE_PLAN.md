# Mission Planning & Execution Pipeline

This document captures the proposed end-to-end architecture for integrating the upstream semantic SLAM stack (scene graph, YAML missions, Octomap, safety services) with our downstream mission planner, hierarchical (A* + RRT*) path planner, mission executor, and action modules.

## 1. Pipeline Overview
- **Mission Input**: Ground station text → upstream NLP → YAML mission specification.
- **Knowledge Base**: Scene Graph (semantic objects, relationships, motion predictions) + Octomap (3D occupancy).
- **Mission Planner**: Our node that parses YAML, queries the scene graph, generates ordered sub-goals, and invokes the hierarchical planner (global A* + local RRT*) for each navigation segment.
- **Hierarchical Planner (A* + RRT*)**: Produces feasible waypoint trajectories (including desired attitude) evaluated against safety services.
- **Mission Executor (Behavior Tree)**: Consumes the planner output, handles safety overrides, and dispatches action modules.
- **Action Modules**: Atomic behaviors interfacing with existing controllers (waypoint/attitude layers, NMPC tracker, search/track routines, etc.).

```
YAML Mission → Mission Planner ─┬─ Scene Graph Queries
                                ├─ Octomap (2D projection for A*, 3D corridor for RRT*)
                                └─ Safety Services (validation)
               ↓
        A* + RRT* Paths + Action Queue
               ↓
        Mission Executor (BT)
               ↓
        Action Modules → Existing control stack
```

## 2. Scene Graph Utilization
1. **Object Fetching**: Use SQL-like interface (`GetObjectsByClass`, `GetNearestObject`, or future `/scene_graph/query` service) to obtain `ObjectInstance` messages with centroid, orientation, bbox, velocities, CLIP attributes.
2. **Relationship Reasoning**: Subscribe to `/scene_relationships` or query RelationshipDetector outputs for context such as `LEFT_OF`, `IN_FRONT_OF`, `NEAR`.
3. **Dynamic Awareness**: Consume motion predictions/dynamic relations (approaching objects, ETA) for planning constraints and safety checks.
4. **Adapter Node**: We will implement a ROS2 adapter that exposes:
   - `mission/query_scene` service (SQL template name + parameters → structured rows).
   - `mission/get_relationships` service/topic for filtered relationship sets.
   - Cached object registry for quick lookups during planning.

### 2.1 semantic_slam_ws interface highlights
- `semantic_octomap_bridge` publishes `/semantic/objects`, `/semantic/attributes`, `/semantic/octomap`, `/tracked_objects`, etc., all using `semantic_slam_msgs` (e.g., `ObjectInstanceArray`). `ObjectInstance` exposes `instance_id`, `class_name`, `centroid`, `velocity`, `is_moving`, `color`, `material`, `size_category`, so our blackboard keys must mirror those fields.
- All SQL lookups are handled by `semantic_octomap/sql_query_engine.py`, callable via the Python API or the planned `semantic_octomap_msgs/srv/QueryScene` service (`/scene_graph/query`, demoed with `ros2 run semantic_octomap_bridge query_scene`). The allowed columns and `FIND` shortcuts are documented in `config/SQL_QUERY_REFERENCE.md`.
- Relationship reasoning and Octomap live in the same workspace, meaning `/semantic/octomap` and `/global_octomap` topics (backed by `octomap_server`) are ready for us to subscribe to or request via standard services.

### 2.2 drone_safety service contract
- The `drone_safety` package exposes three primary services:
  - `/safety/check_landing` (`drone_safety/srv/CheckLandingSafety`): request carries `landing_point` and `clearance_radius`; response returns `is_safe`, `reason`, `threats[]`, `recommended_wait_time`.
  - `/safety/check_path` (`drone_safety/srv/CheckPathSafety`): request includes `start_point`, `goal_point`, `drone_speed`; response contains `is_safe`, `reason`, `collision_points[]`, `collision_times[]`, `recommended_wait_time`.
  - `/safety/find_safe_zone` (`drone_safety/srv/FindSafeZone`): request takes `search_center`, `search_radius`, `num_candidates`; response is a list of `SafeZone` messages (`position`, `safety_score`, `clearance_radius`, `nearby_objects`, `num_approaching`).
- The Mission Planner / Executor must forward YAML parameters such as `clearance`, `moving_object_radius`, and `speed_hint` when calling these services, and log the returned `reason` / `threats` so the Behavior Tree can pick an appropriate fallback.

## 3. YAML Mission Specification

The upstream repository now ships a reference schema (`semantic_slam_ws/src/drone_safety/config/YAML_STRUCTURE_REFERENCE.md`) that mirrors the structure we proposed. We should adopt the exact field names and enumerations to stay compatible.

### 3.1 Baseline Schema (Updated)
```yaml
mission:
  name: "search_and_land"
  metadata:
    priority: 1
    time_budget: 300        # seconds
    required_capabilities: [takeoff, fly_to, inspect, track, land]
    version: "1.0.0"
    author: "avians"

parameters:
  target:
    class: "trashcan"
    color: "yellow"
    max_distance: 120.0
  safety:
    clearance: 5.0
    speed: 3.0
    moving_object_radius: 2.0

stages:
  initial: "query_target"
  stage_list:
    - id: "query_target"
      type: "QUERY_OBJECT"
      params:
        sql_template: "find_objects"
        arguments:
          class: "${parameters.target.class}"
          color: "${parameters.target.color}"
          max_distance: "${parameters.target.max_distance}"
      outputs:
        object_list: "candidates"
      transitions:
        success: "compute_offset"
        no_results: "expand_search"
        failure: "mission_failed"
      timeout: 15
```
Subsequent stages follow the same structure; the reference file also demonstrates `mission_sequence` wrappers for master plans. We will keep using the `stages.stage_list` container and prefer UPPERCASE `type` values.

### 3.2 Required Stage Types & Parameters (Aligned with Upstream)
1. **QUERY_OBJECT**
   - `params.sql_template`, `params.arguments`.
   - `outputs.object_list` to name the variables written to the blackboard.
   - Transitions should cover `success`, `no_results`, and `failure`.
2. **COMPUTE_OFFSET_TARGET**
   - `params`: `reference_id`, `relation` (`IN_FRONT`, `LEFT_OF`, etc.), `distance`, `height_bias`.
   - `result_variable`: Name for the generated approach pose.
3. **VALIDATE_SAFETY**
   - `params`: `check_type` (`landing`, `path`, `zone`), `target_pose`, `clearance_radius`, `moving_object_radius`.
   - `result_variable`: e.g., `safety_assessment`.
   - Additional transitions such as `unsafe` or `timeout` should map to fallback stages.
4. **NAVIGATE_TO_TARGET**
   - `params`: `target_source`, `target_pose`, `altitude_mode`, `speed_hint`.
   - Should support `path_blocked` or `replan_required` transitions.
5. **COMPUTE_OFFSET_TARGET + NAVIGATE_TO_TARGET** combo is recommended for approach corridors.
6. **SEARCH_AREA**
   - `params`: `area_polygon`, `pattern` (`lawnmower`, `spiral`, `orbital`), `dwell_time`, `focus_class`.
   - `transitions`: `target_found`, `timeout`, `failure`.
7. **TRACK_TARGET**
   - `params`: `target_class`, `target_id`, `stand_off_distance`, `lose_timeout`, `min_duration`, `max_duration`.
   - Transitions: `success`, `target_lost`, `failure`.
8. **INSPECT_OBJECT**
   - `params`: `object_id`, `approach_offset`, `camera_mode`, `yaw_alignment`, `timeout`.
9. **DELIVER_PAYLOAD**
   - `params`: `drop_pose`, `release_height`, `confirm_sensor`.
10. **WAIT_OR_HOLD**
    - `params`: `duration`, `reason`, `loiter_radius`.
11. **LAND_AT_POINT**
    - `params`: `touchdown_pose`, `approach_vector`, `descent_profile`, `descent_rate`.
12. **ABORT_MISSION / RETURN_HOME**
    - `params`: `home_pose`, `reason`, `notify_gcs` as needed.

The upstream YAML examples also distinguish `TERMINAL` and `EMERGENCY` states—ensure our definitions for `mission_complete`, `mission_failed`, and emergency fallbacks match those terminologies.

### 3.3 Transition & blackboard requirements (from YAML_STRUCTURE_REFERENCE.md)
- Missions are wrapped in `stages.initial` + `stage_list[]`; each stage must define `id`, `type`, `params`, `transitions`, plus explicit `outputs.*` or `result_variable` entries so the Mission Planner blackboard knows where every value originates.
- Stage `type` must use the upstream uppercase enums (`QUERY_OBJECT`, `NAVIGATE_TO_TARGET`, `COMPUTE_OFFSET_TARGET`, `VALIDATE_SAFETY`, `SEARCH_AREA`, `TRACK_TARGET`, `INSPECT_OBJECT`, `WAIT_OR_HOLD`, `LAND_AT_POINT`, `ABORT_MISSION`, `TERMINAL`, etc.).
- `transitions` should include semantic keys seen in the upstream examples: `success`, `failure`, `no_results`, `unsafe`, `path_blocked`, `target_lost`, `timeout`, etc., which the Behavior Tree will map directly.
- `TERMINAL` and `ABORT_MISSION` stages may carry `exit_code` or `reason`; the Mission Executor is expected to forward those details to the ground control station.

### 3.4 Edge Scenarios To Cover
- Multiple candidate objects (choose nearest, highest confidence, or fallback order).
- Missing attributes (color unknown, orientation invalid).
- Dynamic targets (target is moving; specify re-query cadence).
- Safety failure fallback (alternate landing site, hold, abort).
- Composite missions (visit multiple locations sequentially or conditionally).
- Time-critical tasks (timeouts drive transitions).
- Payload operations (pickup before drop-off).
- Obstacle temporarily blocking path (trigger re-plan stage).

We will provide upstream with a JSON Schema or YAML schema file formalizing these fields to prevent ambiguity.

## 4. Mission Planner Responsibilities
1. **Parse YAML** and populate internal mission graph (stages with transitions).
2. **Interact with Scene Graph** to fetch or update object references required by each stage.
3. **Derive Navigation Sub-goals**:
   - Compute offsets using object geometry and relationships (e.g., 3 m in front of door).
   - Determine altitude/heading rules.
4. **Invoke the Hierarchical Planner** for each navigation segment:
   - **Global A***: project environment to 2D, plan coarse path, annotate segments with semantic tags and altitude hints.
   - **Local RRT***: for each segment, refine in 3D corridor using Octomap slices and safety constraints.
   - Outputs combined into ordered waypoint list `{position, yaw}`, plus timing or hold suggestions where needed.
5. **Call Safety Services**:
   - Validate candidate landing zones via `/safety/check_landing`.
   - Validate planned path via `/safety/check_path`.
6. **Package Execution Plan**:
   - Sequence of navigation plans with metadata (goal ID, replan triggers).
   - Non-navigation actions mapped to action module descriptors.
   - Publish to Mission Executor via topic/service (`mission/plan`).

## 5. Hierarchical Planning (A* + RRT*)
### 5.1 Global A* Layer
- **Purpose**: Fast coarse routing in 2D using a costmap derived from the Octomap (e.g., height-projected occupancy grid).
- **Inputs**:
  - `start_2d`, `goal_2d`.
  - Projected costmap with semantic costs (no-fly zones, preferred corridors).
  - Optional semantic weights (keep distance from people/vehicles).
- **Outputs**:
  - Coarse waypoint chain (2D positions) with metadata (segment labels, altitude hints).
- **Notes**:
  - Resample or simplify to reduce waypoint count.
  - Attach semantic tags (e.g., “approach corridor”, “inspection zone”) for downstream refinement.

### 5.2 Local RRT* Refinement
- **Purpose**: Refine each global segment in relevant 3D volume, enforce height/attitude constraints, handle tight spaces.
- **Inputs**:
  - Segment start pose (3D) and end pose.
  - 3D Octomap slice around the segment.
  - Dynamic objects to avoid, safety parameters.
- **Processing**:
  - Run RRT* within a bounded corridor that follows the global path.
  - Validate edges via occupancy checks and `/safety/check_path`.
  - Incorporate mission-specific constraints (minimum altitude, “stay left of object”).
- **Outputs**:
  - Refined waypoint list with explicit yaw targets and altitude trajectory.
  - Validation status and triggers for replanning.
- **Replan Triggers**:
  - Coarse path invalidated (new obstacle) → rerun A*.
  - Local RRT* fails or safety violation → adjust corridor or request alternate sub-goal.

## 6. Mission Executor (Behavior Tree)
- **Structure**:
  - Root Priority Node: `SafetyOverride` (listens to interrupts) → `MissionSequence`.
  - `SafetyOverride` Branch: executes `Avoidance`/`EmergencyHover`, waits for clearance, resumes or aborts.
  - `MissionSequence`: subtree per stage (Takeoff, Navigate, Search, Inspect, Track, Land).
- **Inputs**: Mission plan (waypoints + action descriptors), live scene updates, safety interrupts.
- **Responsibilities**:
  1. Activate action modules sequentially.
  2. Monitor feedback (`RUNNING / SUCCESS / FAILED / PREEMPTED`).
  3. Handle retries/fallback transitions per YAML mission stage definitions.
  4. Emit mission status and execution logs.

## 7. Action Modules
Each module implements a standard interface (`configure`, `start`, `cancel`, `update`), publishes telemetry, and interacts with existing controllers.

| Module | Purpose | Interfaces |
|--------|---------|------------|
| `TakeoffModule` | Command initial climb | Publishes to waypoint controller (takeoff profile). |
| `FlyToTargetModule` | Execute A*/RRT* waypoint list | Streams waypoints to `/drone/control/waypoint_command`, sets desired yaw. |
| `HoverModule` | Hold position | Sends position hold waypoint, monitors drift. |
| `SearchAreaModule` | Pattern scan | Generates pattern waypoints, toggles camera modes. |
| `TrackTargetModule` | Engage NMPC tracking | Signals NMPC node with target class/ID, monitors detection. |
| `InspectModule` | Attitude alignment | Adjusts yaw/pitch to view object, may control gimbal. |
| `DeliverModule` | Payload release | Coordinates release mechanism, ensures safe altitude. |
| `LandModule` | Controlled descent | Issues landing trajectory and clearance checks. |
| `AvoidanceModule` | Emergency response | Works with safety supervisor to deviate/hover. |

### Module Design Notes
- Accept route segments with optional `attitude_profile`.
- Report reasons for failure (e.g., `PATH_BLOCKED`, `TARGET_LOST`) to trigger proper BT transitions.
- Support preemption (safety events, replan).

## 8. Safety Integration
- Subscribe to upstream safety topics or call services for every critical action.
- Mission Executor listens on `/mission/interrupts` (our safety supervisor publishes) to pre-empt current action.
- Planner enforces margins from YAML (`clearance_radius`, `approach_timeout`).
- Align with the `semantic_slam_ws` `drone_safety` services: `VALIDATE_SAFETY` should default to `/safety/check_landing` or `/safety/check_path`, while `WAIT_OR_HOLD` / `LAND_AT_POINT` may call `/safety/find_safe_zone` on failure, store the returned `SafeZone` in a `result_variable`, and propagate `reason/threats` to Mission Executor logs.

## 9. Required Deliverables for Upstream Coordination
1. **Message & Service Definitions**: Provide `.msg`/`.srv` or interface doc for mission plan exchange, scene queries, and safety interrupts.
   - Map everything to `semantic_slam_msgs/ObjectInstance`, `ObjectInstanceArray`, `RelationshipArray` so our DTO / blackboard fields are one-to-one with upstream messages.
2. **YAML Schema**: Complete enumeration of stage types, required / optional params, and default behaviors, kept in sync with `src/drone_safety/config/YAML_STRUCTURE_REFERENCE.md`.
3. **Coordinate Frames**: Confirm world frame conventions (likely ENU), altitude reference, and orientation semantics.
4. **Octomap Access**: APIs or topics for retrieving occupancy data (e.g., `/semantic/octomap`, `/global_octomap`) plus notes on how to call the `semantic_octomap_bridge` query utilities.
5. **Sample Missions**: Curated YAML examples covering every stage type and edge case, along with the SQL templates they reference (see `config/SQL_QUERY_REFERENCE.md`).

## 10. Open Points for Discussion
- Preferred transport for scene graph queries (service vs. action vs. direct Python API).
- Expected update rate for dynamic objects and motion predictions.
- How upstream envisions mission abort / replanning being signaled in YAML.
- Hard limits on computational budget for A*/RRT* planning (time per update).
- Synchronization of mission clock / timestamps between stacks.
- Testing strategy: use of `mock_scene_publisher` to simulate scenarios before field deployment.
