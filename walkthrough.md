# Octomap Planning Mode Walkthrough

I have implemented the "Octomap Planning" mode which allows the drone to plan paths using A* on an occupancy grid, bypassing the perception stack.

## Changes Implemented
1.  **New Service**: `uav_msgs/srv/PlanPath` for requesting path plans.
2.  **Path Planner Node**: `path_planner_node.py` implementing A* search (currently using a mock grid/obstacles for demonstration).
3.  **Action Module Update**: `FlyToTargetModule` now supports a `use_planner` parameter. When set, it requests a full path from the planner instead of flying directly.
4.  **Launch Script**: `launch_octomap_planning.py` to launch the planning stack.
5.  **Test Suite Integration**: Added "Option 6" to `comprehensive_test_suite.sh`.

## How to Test

### 1. Build the Workspace
Since new messages and packages were added, you must rebuild the workspace:
```bash
colcon build --packages-select uav_msgs mission_action_modules mission_executor
source install/setup.bash
```

### 2. Launch the System
Run the test suite and select Option 6:
```bash
./comprehensive_test_suite.sh
# Select Option 6
```
This will launch:
- Gazebo (Drone + World)
- Path Planner
- Mission Executor
- Action Manager
- Low Level Controllers

### 2. Send a Planning Command
In a separate terminal, use the manual mission planner to send a command that triggers planning:

```bash
ros2 run manual_mission_planner prompt_runner --prompt "Plan a path to 10 meters forward and fly there"
```
*Note: The keyword "plan a path" triggers the `use_planner: true` parameter.*

### 3. Verify Results
- **Terminal 1 (Launcher)**: Look for logs from `path_planner_node` saying "Path found with N waypoints (A* + RRT*)".
- **Terminal 1 (Launcher)**: Look for logs from `FlyToTargetModule` saying "Received path...".
- **Gazebo/RViz**: Watch the drone follow the generated path. It should fly **over** the obstacles (Z > 5m) instead of just around them.

### 4. Standalone RRT* Test
You can also run the standalone RRT* test script to verify the algorithm logic without launching the full simulation:
```bash
python3 test_rrt_standalone.py
```
Output should show "SUCCESS: Path went over the obstacle!".

## Next Steps
- Replace the mock grid in `path_planner_node.py` with real Octomap data by subscribing to `/octomap_full` or `/projected_map`.
- Implement RRT* for 3D refinement.
