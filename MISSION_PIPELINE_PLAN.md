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

## 3. YAML Mission Specification
### 3.1 Baseline Schema
```yaml
mission:
  name: "string"
  metadata:
    priority: int
    time_budget: float   # seconds
    required_capabilities: [takeoff, fly_to, inspect, track, deliver]
  stages:
    - id: "stage_name"
      type: enum          # e.g., QUERY, NAVIGATE, SEARCH, TRACK, INTERACT, LAND, HOLD
      params: {}          # stage-specific options
      transitions:
        success: "next_stage_id"
        failure: "fallback_stage_id"
```
### 3.2 Required Stage Types & Parameters
1. **QUERY_OBJECT**
   - `sql_template`: string (e.g., `find_blue_building`)
   - `arguments`: dict (key/value for SQL placeholders)
   - Expected outputs: object list (IDs, poses, attributes) stored on mission blackboard.
2. **COMPUTE_OFFSET_TARGET**
   - Uses relationship info to derive a new waypoint.
   - Params: `reference_id`, `relation` (`in_front`, `left_of`, `right_of`, `top_of`), `distance`, `height_bias`.
3. **VALIDATE_SAFETY**
   - Calls upstream `SafetyChecker` (landing/path) with provided geometry.
   - Params: `check_type` (`landing`, `path`, `zone`), `clearance_radius`, `moving_object_radius`.
4. **NAVIGATE_TO_TARGET**
   - Declares target pose list (either direct waypoint or object-derived).
   - Params: `target_source` (`object_centroid`, `computed_offset`, `explicit_pose`), `altitude_mode` (`constant`, `terrain_follow`, `object_top_plus`), `speed_hint`.
5. **SEARCH_AREA**
   - Params: `area_polygon`, `pattern` (`lawnmower`, `spiral`, `orbital`), `dwell_time`, optional `focus_class`.
6. **TRACK_TARGET**
   - Params: `target_class`, `target_id`, `stand_off_distance`, `lose_timeout`.
7. **INSPECT_OBJECT**
   - Params: `object_id`, `approach_offset`, `camera_mode`, `yaw_alignment`.
8. **DELIVER_PAYLOAD**
   - Params: `drop_pose`, `release_height`, `confirm_sensor`.
9. **WAIT_OR_HOLD**
   - Params: `duration`, `reason`, `loiter_radius`.
10. **LAND_AT_POINT**
    - Params: `touchdown_pose`, `approach_vector`, `descent_profile`.
11. **ABORT_MISSION / RETURN_HOME**
    - Params: `home_pose`, `reason`.

### 3.3 Edge Scenarios To Cover
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

## 9. Required Deliverables for Upstream Coordination
1. **Message & Service Definitions**: Provide `.msg`/`.srv` or interface doc for mission plan exchange, scene queries, and safety interrupts.
2. **YAML Schema**: Complete enumeration of stage types, mandatory/optional parameters, default behaviors.
3. **Coordinate Frames**: Confirm world frame conventions (likely ENU), altitude reference, and orientation semantics.
4. **Octomap Access**: API or topic for retrieving current occupancy map (e.g., `/semantic/octomap` message type).
5. **Sample Missions**: Curated YAML files covering each stage type and edge case to validate integration.

## 10. Open Points for Discussion
- Preferred transport for scene graph queries (service vs. action vs. direct Python API).
- Expected update rate for dynamic objects and motion predictions.
- How upstream envisions mission abort / replanning being signaled in YAML.
- Hard limits on computational budget for A*/RRT* planning (time per update).
- Synchronization of mission clock / timestamps between stacks.
- Testing strategy: use of `mock_scene_publisher` to simulate scenarios before field deployment.
