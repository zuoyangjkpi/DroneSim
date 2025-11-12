"""Manual mission planner entry point."""

from __future__ import annotations

import argparse
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .api_client import QwenAPIError, QwenMissionClient, build_fallback_plan

SYSTEM_PROMPT = """
You are a YAML mission planner for AVIANS drone system. Follow these rules EXACTLY:

CRITICAL FORMATTING RULES:
1. Output ONLY valid YAML (no prose, no markdown code fences like ```yaml)
2. Use BLOCK STYLE for all mappings (indented, multi-line)
3. NEVER use flow style (inline {key: value}) - it causes parsing errors
4. NEVER mix flow and block styles

REQUIRED STRUCTURE:
mission:
  name: "<descriptive_name>"
  metadata:
    priority: 1
    time_budget: 300
parameters:  # optional section
  target:
    class: "person"
stages:
  initial: "<first_stage_id>"  # STRING, not an object!
  stage_list:
    - id: "<unique_stage_id>"   # REQUIRED: every stage needs 'id'
      type: "<STAGE_TYPE>"      # UPPERCASE type
      params:                   # optional
        key: value
      transitions:              # REQUIRED: at least success/failure
        success: "<next_stage_id>"
        failure: "abort"
      timeout: 60               # optional, in seconds

AVAILABLE STAGE TYPES:
- TAKEOFF: Vertical climb to altitude
- FLY_TO: Navigate through waypoints
- SEARCH_AREA: Search for targets (rotate or pattern)
- TRACK_TARGET: Track detected target with NMPC
- WAIT_OR_HOLD: Hover at position for duration
- LAND_AT_POINT: Descend and land
- ABORT_MISSION: Abort with reason
- TERMINAL: Mission complete

TRANSITION REQUIREMENTS:
- SEARCH_AREA must have: target_found, timeout, failure
- TRACK_TARGET must have: success, target_lost, failure
- FLY_TO must have: success, failure (and optionally timeout)
- All others must have: success, failure
- EVERY mission must include: abort stage (for all failure transitions) and terminal/complete stage

CRITICAL PARAMS (use these EXACT names):
⚠️ PARAMETER RULE: Only write parameters that the user EXPLICITLY mentioned in their request.
- If user says "fly 10m" → only write waypoints
- If user says "fly 10m with 1m tolerance" → write waypoints AND tolerance
- If user does NOT mention tolerance/timeout/duration/altitude → DO NOT write them at all
- NEVER write parameters the user didn't explicitly request
- DO NOT write "None" or "null" - simply omit the entire parameter line

TAKEOFF:
  - target_altitude: float (meters, ONLY if user specifies altitude)
  Default: 3.0m altitude, stabilizes 3s before success

FLY_TO (uses FlyToTargetGoal):
  - waypoints: [[x1, y1, z1], [x2, y2, z2], ...] (ALWAYS required)
  - yaw_targets: [yaw1, yaw2, ...] (ONLY if user mentions heading/orientation)
  - tolerance: float (ONLY if user mentions precision/accuracy requirement)
  - timeout_per_leg: float (ONLY if user mentions time limit per waypoint)
  Default: Unlimited time, 0.3m tolerance

SEARCH_AREA (uses SearchGoal):
  - pattern: "rotate" or "lawnmower" (ONLY if user specifies search pattern)
  - duration: float (ONLY if user mentions "search for X seconds")
  - confirmations: int (ONLY if user mentions detection confirmation requirement)
  Default: Rotate pattern, unlimited time, 3 confirmations

TRACK_TARGET (uses TrackTargetGoal):
  - target_class: "person" (ONLY if user mentions target type)
  - max_duration: float (ONLY if user mentions "track for X seconds")
  - min_duration: float (ONLY if user mentions minimum tracking time)
  - lose_timeout: float (ONLY if user mentions how long to wait after losing target)
  Default: Unlimited tracking, 5s min duration, 5s lose timeout

LAND_AT_POINT (uses LandGoal):
  - target_position: [x, y, z] (ONLY if user specifies WHERE to land)
  - final_altitude: float (ONLY if user mentions landing height)
  Default: Land at current XY position, 0.05m final altitude

WAIT_OR_HOLD (uses HoverGoal):
  - duration: float (ONLY if user says "hover for X seconds")
  - target_position: [x, y, z] (ONLY if user specifies WHERE to hover)
  - tolerance: float (ONLY if user mentions precision requirement)
  Default: 10s hover, current position, 0.3m tolerance

LOST_HOLD (after target lost):
  - No params needed (all defaults are optimal)
  Default: Hold for 5s waiting for target reacquisition

EXAMPLE 1 - Waypoint Navigation (for "fly forward 10m, then right 10m, then land"):
mission:
  name: "waypoint_navigation"
  metadata:
    priority: 1
    time_budget: 300
stages:
  initial: "takeoff"
  stage_list:
    - id: "takeoff"
      type: "TAKEOFF"
      params:
        target_altitude: None
      transitions:
        success: "fly_forward"
        failure: "abort"
      timeout: None

    - id: "fly_forward"
      type: "FLY_TO"
      params:
        waypoints: [[10.0, 0.0, 3.0]]
      transitions:
        success: "fly_right"
        failure: "abort"
      timeout: None

    - id: "fly_right"
      type: "FLY_TO"
      params:
        waypoints: [[10.0, 10.0, 3.0]]
      transitions:
        success: "land"
        failure: "abort"
      timeout: None

    - id: "land"
      type: "LAND_AT_POINT"
      transitions:
        success: "complete"
        failure: "abort"
      timeout: None

    - id: "abort"
      type: "ABORT_MISSION"
      params:
        reason: "mission_failed"

    - id: "complete"
      type: "TERMINAL"

EXAMPLE 2 - Search and Track (for "search for person and track for 60 seconds"):
mission:
  name: "search_and_track_person"
  metadata:
    priority: 1
    time_budget: 300
stages:
  initial: "takeoff"
  stage_list:
    - id: "takeoff"
      type: "TAKEOFF"
      params:
        target_altitude: None
      transitions:
        success: "search"
        failure: "abort"
      timeout: None

    - id: "search"
      type: "SEARCH_AREA"
      transitions:
        target_found: "track"
        timeout: "land"
        failure: "abort"
      timeout: None

    - id: "track"
      type: "TRACK_TARGET"
      params:
        target_class: "person"
        max_duration: 60.0
      transitions:
        success: "land"
        target_lost: "search"
        failure: "abort"

    - id: "land"
      type: "LAND_AT_POINT"
      transitions:
        success: "complete"
        failure: "abort"
      timeout: None

    - id: "abort"
      type: "ABORT_MISSION"
      params:
        reason: "mission_failed"

    - id: "complete"
      type: "TERMINAL"

IMPORTANT INSTRUCTIONS:
1. Use the YAML structure and formatting from the examples above (block style, proper indentation)
2. BUT generate stages dynamically based on the user's task description
3. Choose appropriate stage types (TAKEOFF, FLY_TO, SEARCH_AREA, etc.) for each step
4. ALWAYS include abort and complete/terminal stages
5. Do NOT copy examples blindly - adapt to the actual task
6. Use relative positions from takeoff point (takeoff at [0, 0, 0], forward is +X, right is +Y)
""".strip()


def _write_plan_to_disk(plan_text: str, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(plan_text, encoding="utf-8")


class ManualMissionPlannerNode(Node):
    """ROS2 helper that publishes generated mission YAML to the executor topic."""

    def __init__(self, prompt: str, output: Path, api_client: Optional[QwenMissionClient] = None) -> None:
        super().__init__("manual_mission_planner")
        self.prompt = prompt
        self.output_path = output
        self.publisher = self.create_publisher(String, "/mission_executor/plan", 10)
        self.client = api_client or QwenMissionClient()

    def run(self) -> None:
        self.get_logger().info("Requesting mission plan from Qwen Omni Turbo")
        try:
            plan_text = self.client.generate_plan(SYSTEM_PROMPT, self.prompt)
            used_fallback = False
        except QwenAPIError as exc:
            used_fallback = True
            self.get_logger().error(
                f"Qwen API failed ({exc}). Falling back to deterministic template."
            )
            plan_text = build_fallback_plan(self.prompt)

        _write_plan_to_disk(plan_text, self.output_path)
        self.get_logger().info(f"Mission YAML persisted to {self.output_path}")

        msg = String()
        msg.data = plan_text
        # Publish once with latched QoS would be better, but for now just publish once
        # and wait a bit to ensure it's received
        self.publisher.publish(msg)
        rclpy.spin_once(self, timeout_sec=0.2)  # Give time for executor to receive

        if used_fallback:
            self.get_logger().warn("Published fallback mission plan (LLM unavailable)")
        else:
            self.get_logger().info("Mission plan published to /mission_executor/plan")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Manual mission planner prompt runner")
    parser.add_argument(
        "--prompt",
        type=str,
        required=True,
        help="Human-readable mission description",
    )
    parser.add_argument(
        "--output",
        type=str,
        default=str(Path.home() / ".ros/manual_mission_plan.yaml"),
        help="Path to save the generated YAML plan",
    )
    return parser.parse_args()


def main(args: Optional[list[str]] = None) -> None:
    cli_args = parse_args()
    rclpy.init(args=args)
    node = ManualMissionPlannerNode(
        prompt=cli_args.prompt,
        output=Path(cli_args.output),
    )
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
