from __future__ import annotations

import json
from enum import Enum, auto

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class SequenceState(Enum):
    TAKEOFF = auto()
    TAKEOFF_HOLD = auto()
    SEARCH = auto()
    TRACK = auto()
    LOST_HOLD = auto()
    ERROR = auto()


class MissionSequenceController(Node):
    """High-level orchestration that calls action modules in the required order."""

    def __init__(self) -> None:
        super().__init__("mission_sequence_controller")
        self.takeoff_height = float(self.declare_parameter("sequence.takeoff_height", 3.0).value)
        self.takeoff_hold_time = float(self.declare_parameter("sequence.post_takeoff_wait", 1.0).value)
        self.takeoff_tolerance = float(self.declare_parameter("sequence.altitude_tolerance", 0.1).value)

        self.state = SequenceState.TAKEOFF
        self.state_enter_time = self._now()
        self._active_actions: set[str] = set()
        self._current_altitude = 0.0
        self._altitude_ready = False
        self._have_odom = False
        self._takeoff_wait_logged = False

        self._action_clients = {
            "takeoff": self.create_client(Trigger, "mission_actions/takeoff"),
            "search": self.create_client(Trigger, "mission_actions/search"),
            "track_target": self.create_client(Trigger, "mission_actions/track_target"),
            "lost_hold": self.create_client(Trigger, "mission_actions/lost_hold"),
        }

        self.event_sub = self.create_subscription(
            String, "/mission_actions/events", self._event_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/X3/odometry", self._odom_callback, 10
        )

        self.timer = self.create_timer(0.5, self._tick)
        self.get_logger().info("Mission sequence controller ready (TAKEOFF phase)")

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry) -> None:
        self._current_altitude = float(msg.pose.pose.position.z)
        self._altitude_ready = (
            self._current_altitude >= self.takeoff_height - self.takeoff_tolerance
        )
        if not self._have_odom:
            self.get_logger().info("Received first odometry sample, takeoff sequence can start")
        self._have_odom = True
        self._takeoff_wait_logged = False

    def _event_callback(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self.get_logger().warn(f"Received malformed mission action event: {msg.data}")
            return

        action = payload.get("action")
        outcome = payload.get("outcome")
        message = payload.get("message", "")
        data = payload.get("data", {})
        if action and outcome != "started":
            self._active_actions.discard(action)

        if action == "takeoff":
            if outcome == "succeeded":
                self._transition(SequenceState.TAKEOFF_HOLD)
            elif outcome not in ("started",):
                self._fail_sequence(f"Takeoff {outcome}: {message}")
        elif action == "search":
            if outcome == "succeeded":
                self._transition(SequenceState.TRACK)
            elif outcome in ("failed", "exception", "failed_to_start"):
                self.get_logger().warn(f"Search failed ({outcome}). Restarting search.")
                self._transition(SequenceState.SEARCH)
        elif action == "track_target":
            if outcome in ("failed", "exception", "failed_to_start"):
                self.get_logger().warn(f"Track lost ({outcome}). Entering lost-hold.")
                self._transition(SequenceState.LOST_HOLD)
        elif action == "lost_hold":
            if outcome == "succeeded":
                # Check if target was reacquired or just stabilized
                if data.get("reacquired", False):
                    self.get_logger().info("Lost-hold succeeded: target reacquired → TRACK")
                    self._transition(SequenceState.TRACK)
                else:
                    self.get_logger().info("Lost-hold succeeded: stabilized without target → SEARCH")
                    self._transition(SequenceState.SEARCH)
            elif outcome in ("failed", "exception", "failed_to_start"):
                self.get_logger().warn("Lost-hold failed. Returning to search.")
                self._transition(SequenceState.SEARCH)

    # ------------------------------------------------------------------
    # State machine
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self.state == SequenceState.ERROR:
            return

        if self.state == SequenceState.TAKEOFF:
            if not self._have_odom:
                if not self._takeoff_wait_logged:
                    self.get_logger().warn("Waiting for /X3/odometry before issuing takeoff")
                    self._takeoff_wait_logged = True
                return
            self._ensure_action_running("takeoff")
        elif self.state == SequenceState.TAKEOFF_HOLD:
            if self._altitude_ready and self._elapsed_in_state() >= self.takeoff_hold_time:
                self._transition(SequenceState.SEARCH)
        elif self.state == SequenceState.SEARCH:
            self._ensure_action_running("search")
        elif self.state == SequenceState.TRACK:
            self._ensure_action_running("track_target")
        elif self.state == SequenceState.LOST_HOLD:
            self._ensure_action_running("lost_hold")

    def _ensure_action_running(self, name: str) -> None:
        if name in self._active_actions:
            return
        client = self._action_clients.get(name)
        if client is None:
            self.get_logger().error(f"No client for action '{name}'")
            self._fail_sequence(f"Missing client for {name}")
            return

        while not client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn(f"Waiting for service mission_actions/{name}...")

        request = Trigger.Request()
        future = client.call_async(request)
        self._active_actions.add(name)

        def _on_response(fut, action_name=name):
            try:
                resp = fut.result()
                if not resp.success:
                    self.get_logger().error(
                        f"Service mission_actions/{action_name} responded with failure: {resp.message}"
                    )
            except Exception as exc:  # pylint: disable=broad-except
                self.get_logger().error(
                    f"Service call mission_actions/{action_name} failed: {exc}"
                )

        future.add_done_callback(_on_response)

    def _transition(self, new_state: SequenceState) -> None:
        if self.state == new_state:
            return
        self.get_logger().info(f"Mission sequence transition: {self.state.name} -> {new_state.name}")
        self.state = new_state
        self.state_enter_time = self._now()

    def _elapsed_in_state(self) -> float:
        return self._now() - self.state_enter_time

    def _fail_sequence(self, reason: str) -> None:
        self.get_logger().error(f"Mission sequence aborted: {reason}")
        self.state = SequenceState.ERROR

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MissionSequenceController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
