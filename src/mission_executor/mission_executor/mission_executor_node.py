"""ROS2 mission executor that consumes YAML plans and drives mission action modules."""

from __future__ import annotations

import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger
import yaml


@dataclass
class StageSpec:
    """In-memory representation of a mission stage."""

    stage_id: str
    stage_type: str
    params: Dict[str, Any] = field(default_factory=dict)
    transitions: Dict[str, str] = field(default_factory=dict)
    timeout: Optional[float] = None


class MissionExecutorNode(Node):
    """Behavior-tree inspired executor that sequentially runs mission stages."""

    ACTION_SERVICES = {
        "TAKEOFF": "mission_actions/takeoff",
        "FLY_TO": "mission_actions/fly_to",
        "SEARCH_AREA": "mission_actions/search",
        "TRACK_TARGET": "mission_actions/track_target",
        "LAND_AT_POINT": "mission_actions/land",
        "WAIT_OR_HOLD": "mission_actions/hover",  # Maps to HoverModule
        "HOVER": "mission_actions/hover",
        "LOST_HOLD": "mission_actions/lost_hold",
    }

    NOOP_STAGE_TYPES = {
        "QUERY_OBJECT",
        "COMPUTE_OFFSET_TARGET",
        "VALIDATE_SAFETY",
        "NAVIGATE_TO_TARGET",
        "INSPECT_OBJECT",
        "DELIVER_PAYLOAD",
        "DELIVER",
    }

    def __init__(self) -> None:
        super().__init__("mission_executor")
        self.plan_sub = self.create_subscription(
            String, "/mission_executor/plan", self._plan_callback, 10
        )
        self.event_sub = self.create_subscription(
            String, "/mission_actions/events", self._event_callback, 10
        )
        self.status_sub = self.create_subscription(
            Float64MultiArray, "/drone/controller/status", self._status_callback, 10
        )
        # Publisher for action parameters
        self.params_pub = self.create_publisher(
            String, "/mission_executor/action_params", 10
        )
        self.timer = self.create_timer(0.5, self._timer_callback)

        self._stage_map: Dict[str, StageSpec] = {}
        self._current_stage: Optional[StageSpec] = None
        self._current_stage_start: Optional[float] = None
        self._mission_active = False
        self._plan_name = ""
        self._pending_detection_trigger = False
        self._service_clients: Dict[str, any] = {}
        self._action_stage_binding: Dict[str, str] = {}
        self._current_action: Optional[str] = None
        self._detection_confirmations = 0
        self._required_confirmations = int(
            self.declare_parameter("track_detection_confirmations", 3).value
        )
        self._last_detection_time = 0.0
        self._latest_status = None
        self._persist_path = Path(
            self.declare_parameter(
                "persist_plan_path",
                str(Path.home() / ".ros" / "mission_executor_last_plan.yaml"),
            ).value
        )
        self.get_logger().info("Mission executor ready - waiting for /mission_executor/plan")

    # ------------------------------------------------------------------
    # Plan ingestion
    # ------------------------------------------------------------------
    def _plan_callback(self, msg: String) -> None:
        try:
            plan_dict = yaml.safe_load(msg.data)
        except yaml.YAMLError as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Failed to parse mission YAML: {exc}")
            return

        stages_section = self._extract_stages_section(plan_dict)
        if not stages_section:
            self.get_logger().error("Mission plan missing 'stages' block")
            return

        stage_list = stages_section.get("stage_list") or stages_section.get("state_list")
        if not stage_list:
            self.get_logger().error("Mission plan missing stage_list/state_list")
            return

        initial_stage = stages_section.get("initial")
        if not initial_stage and stage_list:
            initial_stage = stage_list[0].get("id")

        if not initial_stage:
            self.get_logger().error("Mission plan missing initial stage id")
            return

        self._stage_map = {}
        for raw_stage in stage_list:
            stage_id = str(raw_stage.get("id"))
            stage_type = str(raw_stage.get("type", "")).upper()
            params = raw_stage.get("params") or {}
            transitions = raw_stage.get("transitions") or {}
            timeout = self._coerce_float(raw_stage.get("timeout"))
            if timeout is None:
                timeout = 120.0
            if not stage_id or not stage_type:
                self.get_logger().warn(f"Skipping malformed stage entry: {raw_stage}")
                continue
            self._stage_map[stage_id] = StageSpec(
                stage_id=stage_id,
                stage_type=stage_type,
                params=params,
                transitions=transitions,
                timeout=timeout,
            )

        if initial_stage not in self._stage_map:
            self.get_logger().error(f"Initial stage '{initial_stage}' not found in stage_list")
            return

        mission_block = plan_dict.get("mission", {}) if isinstance(plan_dict, dict) else {}
        self._plan_name = mission_block.get("name", "generated_mission")
        self._persist_plan(msg.data)

        self.get_logger().info(
            f"Loaded mission '{self._plan_name}' with {len(self._stage_map)} stages (initial={initial_stage})"
        )
        self._start_stage(initial_stage)

    def _extract_stages_section(self, plan_dict: Any) -> Optional[Dict[str, Any]]:
        if not isinstance(plan_dict, dict):
            return None
        if "stages" in plan_dict:
            return plan_dict.get("stages")
        mission_block = plan_dict.get("mission")
        if isinstance(mission_block, dict) and "stages" in mission_block:
            return mission_block.get("stages")
        return None

    def _persist_plan(self, raw_yaml: str) -> None:
        try:
            self._persist_path.parent.mkdir(parents=True, exist_ok=True)
            self._persist_path.write_text(raw_yaml, encoding="utf-8")
        except OSError as exc:
            self.get_logger().warn(f"Failed to persist mission plan: {exc}")

    # ------------------------------------------------------------------
    # Stage execution
    # ------------------------------------------------------------------
    def _start_stage(self, stage_id: str) -> None:
        stage = self._stage_map.get(stage_id)
        if not stage:
            self.get_logger().error(f"Stage '{stage_id}' not found; aborting mission")
            self._mission_active = False
            return

        self._mission_active = True
        self._current_stage = stage
        now = self.get_clock().now().nanoseconds / 1e9
        self._current_stage_start = now
        self._pending_detection_trigger = False
        self._current_action = None

        self.get_logger().info(f"➡️  Entering stage [{stage.stage_id}] ({stage.stage_type})")

        if stage.stage_type == "TRACK_TARGET":
            self._last_detection_time = now

        if stage.stage_type in self.NOOP_STAGE_TYPES:
            self.get_logger().info(
                f"Stage {stage.stage_id} is a logical/no-op stage ({stage.stage_type}); auto-progress"
            )
            self._transition_from_stage("success")
            return

        if stage.stage_type == "TERMINAL":
            exit_code = stage.params.get("exit_code", 0)
            self.get_logger().info(
                f"Mission reached terminal stage '{stage.stage_id}' (exit_code={exit_code})"
            )
            self._mission_active = False
            return

        if stage.stage_type == "ABORT_MISSION":
            reason = stage.params.get("reason", "unspecified")
            self.get_logger().error(f"Mission aborted: {reason}")
            self._mission_active = False
            return

        action_name = self.ACTION_SERVICES.get(stage.stage_type)
        if not action_name:
            self.get_logger().warn(
                f"Stage {stage.stage_id} uses unsupported type {stage.stage_type}; treating as success"
            )
            self._transition_from_stage("success")
            return

        self._invoke_action(stage, action_name)

    def _invoke_action(self, stage: StageSpec, action_service: str) -> None:
        client = self._service_clients.get(action_service)
        if client is None:
            client = self.create_client(Trigger, action_service)
            self._service_clients[action_service] = client

        if not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().error(
                f"Service {action_service} unavailable; cannot execute stage {stage.stage_id}"
            )
            self._transition_from_stage("failure")
            return

        # Publish stage params for action_manager to use
        # Filter out None values (including string 'None', 'null') to use module defaults
        def is_none_value(v):
            if v is None:
                return True
            if isinstance(v, str) and v.lower() in ('none', 'null', ''):
                return True
            return False
        
        filtered_params = {k: v for k, v in stage.params.items() if not is_none_value(v)}
        params_msg = String()
        params_msg.data = json.dumps({
            "stage_id": stage.stage_id,
            "stage_type": stage.stage_type,
            "params": filtered_params,
            "timeout": stage.timeout
        })
        self.params_pub.publish(params_msg)
        self.get_logger().debug(f"Published params for stage {stage.stage_id}: {stage.params}")

        # Small delay to ensure params are received before service call
        import time
        time.sleep(0.05)

        request = Trigger.Request()
        future = client.call_async(request)
        future.add_done_callback(lambda fut, svc=action_service: self._log_service_response(stage, svc, fut))
        self._current_action = action_service
        self._action_stage_binding[action_service.split("/")[-1]] = stage.stage_id
        self.get_logger().info(
            f"Started action '{action_service}' for stage {stage.stage_id} with params {stage.params}"
        )

    def _log_service_response(self, stage: StageSpec, service: str, future) -> None:  # pylint: disable=unused-argument
        try:
            response = future.result()
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Service call to {service} failed: {exc}")
            self._transition_from_stage("failure")
            return

        if not response.success:
            self.get_logger().error(
                f"Service {service} rejected stage {stage.stage_id}: {response.message}"
            )
            self._transition_from_stage("failure")

    # ------------------------------------------------------------------
    # Event & status handling
    # ------------------------------------------------------------------
    def _event_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Received malformed mission action event: {msg.data}")
            return

        action = payload.get("action")
        outcome = payload.get("outcome")
        if not action or not outcome:
            return

        stage_id = self._action_stage_binding.get(action)
        current_stage_id = self._current_stage.stage_id if self._current_stage else None
        if not stage_id or stage_id != current_stage_id:
            return

        if outcome == "started":
            return

        self.get_logger().info(
            f"Stage {stage_id} received action outcome '{outcome}' (message={payload.get('message')})"
        )

        if outcome == "succeeded":
            self._transition_from_stage("success")
        elif outcome in ("failed", "exception", "failed_to_start"):
            self._transition_from_stage("failure")
        elif outcome == "canceled":
            cancel_key = "canceled" if "canceled" in self._current_stage.transitions else "failure"
            self._transition_from_stage(cancel_key)

    def _status_callback(self, msg: Float64MultiArray) -> None:
        self._latest_status = msg
        detected = bool(msg.data and msg.data[0] >= 0.5)
        now = self.get_clock().now().nanoseconds / 1e9
        if detected:
            self._detection_confirmations = min(
                self._detection_confirmations + 1, self._required_confirmations
            )
            self._last_detection_time = now
        else:
            self._detection_confirmations = 0

        if not self._mission_active or not self._current_stage:
            return

        stage_type = self._current_stage.stage_type
        transitions = self._current_stage.transitions
        if (
            stage_type == "SEARCH_AREA"
            and detected
            and self._detection_confirmations >= self._required_confirmations
            and not self._pending_detection_trigger
            and "target_found" in transitions
        ):
            self.get_logger().info("Target detected during SEARCH_AREA; following target_found transition")
            self._pending_detection_trigger = True
            self._transition_from_stage("target_found")

    # ------------------------------------------------------------------
    # Timer & timeout management
    # ------------------------------------------------------------------
    def _timer_callback(self) -> None:
        if not self._mission_active or not self._current_stage or not self._current_stage_start:
            return

        now = self.get_clock().now().nanoseconds / 1e9
        elapsed = now - self._current_stage_start

        # Handle declared timeout
        timeout = self._current_stage.timeout
        if timeout and elapsed >= timeout:
            key = "timeout" if "timeout" in self._current_stage.transitions else "failure"
            self.get_logger().warn(
                f"Stage {self._current_stage.stage_id} timed out after {elapsed:.1f}s ({key})"
            )
            self._transition_from_stage(key)
            return

        # Track-specific handling
        if self._current_stage.stage_type == "TRACK_TARGET":
            params = self._current_stage.params or {}
            max_duration = self._coerce_float(
                params.get("track_duration")
                or params.get("max_duration")
                or params.get("duration")
            )
            lose_timeout = self._coerce_float(params.get("lose_timeout")) or 5.0

            if max_duration and elapsed >= max_duration and "success" in self._current_stage.transitions:
                self.get_logger().info(
                    f"TRACK_TARGET stage satisfied duration {max_duration:.1f}s; transitioning to success"
                )
                self._transition_from_stage("success")
                return

            if (
                self._current_stage.transitions.get("target_lost")
                and (now - self._last_detection_time) > lose_timeout
            ):
                self.get_logger().warn("Target lost for TRACK_TARGET; taking target_lost branch")
                self._transition_from_stage("target_lost")
                return

    # ------------------------------------------------------------------
    # Transition helper
    # ------------------------------------------------------------------
    def _transition_from_stage(self, outcome_key: str) -> None:
        if not self._current_stage:
            return

        next_stage = self._current_stage.transitions.get(outcome_key)
        if not next_stage:
            self.get_logger().error(
                f"Stage {self._current_stage.stage_id} missing transition for '{outcome_key}'"
            )
            self._mission_active = False
            return

        # Clean up bindings for the departing stage
        if self._current_action:
            action_key = self._current_action.split("/")[-1]
            self._action_stage_binding.pop(action_key, None)
        self._current_action = None

        self.get_logger().info(
            f"Stage {self._current_stage.stage_id} → {next_stage} via '{outcome_key}'"
        )
        self._start_stage(next_stage)

    # ------------------------------------------------------------------
    # Utility helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _coerce_float(value: Any) -> Optional[float]:
        if value is None:
            return None
        try:
            return float(value)
        except (TypeError, ValueError):
            return None


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = MissionExecutorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
