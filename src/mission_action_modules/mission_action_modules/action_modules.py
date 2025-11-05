"""Concrete action modules used by the mission executor."""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence

import numpy as np

from std_msgs.msg import Bool, Float64MultiArray

from .action_base import ActionModule, ActionContext, ActionOutcome


# ---------------------------------------------------------------------------
# Goal dataclasses
# ---------------------------------------------------------------------------


@dataclass
class TakeoffGoal:
    target_altitude: Optional[float] = None  # meters
    timeout: Optional[float] = None  # seconds


@dataclass
class HoverGoal:
    duration: Optional[float] = None  # seconds
    target_position: Optional[Sequence[float]] = None  # xyz
    tolerance: Optional[float] = None  # meters


@dataclass
class FlyToTargetGoal:
    waypoints: Sequence[Sequence[float]]  # List of [x, y, z]
    yaw_targets: Optional[Sequence[float]] = None  # radians, optional per waypoint
    tolerance: Optional[float] = None  # meters
    timeout_per_leg: Optional[float] = None  # seconds


@dataclass
class TrackTargetGoal:
    target_class: Optional[str] = None
    target_id: Optional[int] = None
    stand_off_distance: Optional[float] = None
    min_duration: float = 5.0
    max_duration: Optional[float] = None
    acquire_timeout: float = 20.0
    lose_timeout: float = 5.0


@dataclass
class SearchGoal:
    pattern: str = "rotate"
    yaw_rate: Optional[float] = None  # rad/s
    altitude: Optional[float] = None  # meters
    duration: Optional[float] = None  # seconds
    center: Optional[Sequence[float]] = None  # xyz


@dataclass
class InspectGoal:
    target_point: Optional[Sequence[float]] = None  # xyz to look at
    target_yaw: Optional[float] = None  # radians
    hold_position: Optional[Sequence[float]] = None  # xyz
    timeout: Optional[float] = None  # seconds


@dataclass
class LandGoal:
    target_position: Optional[Sequence[float]] = None  # xyz
    final_altitude: float = 0.05  # meters above ground
    timeout: Optional[float] = None  # seconds


@dataclass
class DeliveryGoal:
    drop_pose: Optional[Sequence[float]] = None
    release_height: Optional[float] = None
    confirm_sensor: Optional[str] = None


@dataclass
class SearchAreaGoal:
    pattern: str = "lawnmower"
    area_vertices: Optional[Sequence[Sequence[float]]] = None
    altitude: Optional[float] = None
    spacing: Optional[float] = None


@dataclass
class AvoidanceGoal:
    reason: str = "obstacle"
    hold_time: float = 3.0


# ---------------------------------------------------------------------------
# Action module implementations
# ---------------------------------------------------------------------------


class TakeoffModule(ActionModule):
    """Climb vertically to target altitude while keeping XY constant."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "TakeoffModule")
        self._target: Optional[np.ndarray] = None
        self._timeout = 0.0
        self._start_time = 0.0

    def on_start(self, goal: TakeoffGoal) -> None:
        position = self.context.get_position()
        if position is None:
            self.fail("No odometry available for takeoff")
            return

        target_altitude = (
            goal.target_altitude
            if goal.target_altitude is not None
            else self.context.defaults.takeoff_altitude
        )
        self._timeout = (
            goal.timeout
            if goal.timeout is not None
            else self.context.defaults.takeoff_timeout
        )
        self._start_time = self.context.now()
        self._target = np.array([position[0], position[1], target_altitude], dtype=float)

        # Enable controllers
        self.context.enable_waypoint_control(True)
        self.context.enable_attitude_control(True)
        self.context.enable_velocity_control(True)

        self.context.send_waypoint(self._target)
        self.create_timer(0.2, self._monitor_altitude)

    def _monitor_altitude(self) -> None:
        if self._target is None:
            return

        position = self.context.get_position()
        if position is None:
            return

        altitude_error = abs(position[2] - self._target[2])
        elapsed = self.context.now() - self._start_time

        if altitude_error <= self.context.defaults.altitude_tolerance:
            self.succeed(f"Reached altitude {self._target[2]:.2f} m")
            return

        if elapsed > self._timeout:
            self.fail(
                f"Takeoff timeout ({elapsed:.1f}s) before reaching {self._target[2]:.2f} m"
            )
            return

        # Refresh waypoint periodically for robustness
        if elapsed % 1.0 < 0.2:
            self.context.send_waypoint(self._target)


class HoverModule(ActionModule):
    """Hold at a fixed position for a given duration."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "HoverModule")
        self._target: Optional[np.ndarray] = None
        self._tolerance = 0.0
        self._duration = 0.0
        self._start_time = 0.0

    def on_start(self, goal: HoverGoal) -> None:
        position = self.context.get_position()
        if position is None:
            self.fail("No odometry available for hover")
            return

        self._target = (
            np.array(goal.target_position, dtype=float)
            if goal.target_position is not None
            else position.copy()
        )
        self._duration = (
            goal.duration
            if goal.duration is not None
            else self.context.defaults.hover_duration
        )
        self._tolerance = (
            goal.tolerance
            if goal.tolerance is not None
            else self.context.defaults.position_tolerance
        )
        self._start_time = self.context.now()

        self.context.enable_waypoint_control(True)
        self.context.send_waypoint(self._target)

        self.create_timer(0.2, self._monitor_hover)

    def _monitor_hover(self) -> None:
        if self._target is None:
            return
        position = self.context.get_position()
        if position is None:
            return

        elapsed = self.context.now() - self._start_time
        distance = float(np.linalg.norm(position - self._target))

        if elapsed >= self._duration:
            self.succeed(f"Hover complete ({elapsed:.1f}s)")
            return

        if distance > self._tolerance * 2.0:
            self.context.send_waypoint(self._target)


class FlyToTargetModule(ActionModule):
    """Track a sequence of waypoints produced by the planner."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "FlyToTargetModule")
        self._waypoints: List[np.ndarray] = []
        self._yaw_targets: Optional[List[Optional[float]]] = None
        self._current_idx = 0
        self._start_time = 0.0
        self._timeout_per_leg = 15.0
        self._tolerance = 0.3

    def on_start(self, goal: FlyToTargetGoal) -> None:
        if not goal.waypoints:
            self.fail("No waypoints provided")
            return

        self._waypoints = [
            np.array(wp, dtype=float) for wp in goal.waypoints
        ]
        self._yaw_targets = (
            list(goal.yaw_targets) if goal.yaw_targets is not None else None
        )
        self._tolerance = (
            goal.tolerance
            if goal.tolerance is not None
            else self.context.defaults.position_tolerance
        )
        self._timeout_per_leg = (
            goal.timeout_per_leg if goal.timeout_per_leg is not None else 15.0
        )
        self._current_idx = 0
        self._start_time = self.context.now()

        self.context.enable_waypoint_control(True)
        self.context.enable_velocity_control(True)
        self.context.enable_attitude_control(True)

        self._command_current_waypoint()
        self.create_timer(0.2, self._monitor_progress)

    def _command_current_waypoint(self) -> None:
        target = self._waypoints[self._current_idx]
        self.context.send_waypoint(target)

        if self._yaw_targets:
            yaw = self._yaw_targets[self._current_idx]
            if yaw is not None:
                self.context.send_attitude(0.0, 0.0, float(yaw))

    def _monitor_progress(self) -> None:
        if not self._waypoints:
            return

        position = self.context.get_position()
        if position is None:
            return

        target = self._waypoints[self._current_idx]
        distance = float(np.linalg.norm(position - target))
        elapsed = self.context.now() - self._start_time

        if distance <= self._tolerance:
            self._current_idx += 1
            self._start_time = self.context.now()
            if self._current_idx >= len(self._waypoints):
                self.succeed("Reached final waypoint")
                return
            self._command_current_waypoint()
            return

        if elapsed > self._timeout_per_leg:
            self.fail(
                f"Timeout moving to waypoint {self._current_idx + 1}/{len(self._waypoints)}"
            )
            return

        # Refresh waypoint periodically for robustness
        if elapsed % 1.0 < 0.2:
            self.context.send_waypoint(target)


class TrackTargetModule(ActionModule):
    """Engage NMPC tracking loop and monitor target detection."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "TrackTargetModule")
        self._enable_pub = context.node.create_publisher(Bool, "/nmpc/enable", 10)
        self._status_sub = context.node.create_subscription(
            Float64MultiArray,
            "/drone/controller/status",
            self._status_callback,
            10,
        )
        self._goal: Optional[TrackTargetGoal] = None
        self._start_time = 0.0
        self._last_detection_time = 0.0
        self._currently_detected = False

    def on_start(self, goal: TrackTargetGoal) -> None:
        self._goal = goal
        self._start_time = self.context.now()
        self._last_detection_time = 0.0
        self._currently_detected = False

        enable_msg = Bool()
        enable_msg.data = True
        self._enable_pub.publish(enable_msg)

        if goal.target_class:
            self.context.node.get_logger().info(
                "[TrackTargetModule] Engage tracking for class '%s' (stand-off: %s)",
                goal.target_class,
                f"{goal.stand_off_distance:.2f} m" if goal.stand_off_distance else "default",
            )
        else:
            self.context.node.get_logger().info(
                "[TrackTargetModule] Engage tracking with default detector target"
            )

        self.create_timer(0.2, self._monitor_tracking)

    def _status_callback(self, msg: Float64MultiArray) -> None:
        now = self.context.now()
        if not msg.data:
            return
        detected = bool(int(msg.data[0]))
        self._currently_detected = detected
        if detected:
            self._last_detection_time = now

    def _monitor_tracking(self) -> None:
        if self._goal is None:
            return

        now = self.context.now()
        elapsed = now - self._start_time

        if self._last_detection_time == 0.0:
            if elapsed > self._goal.acquire_timeout:
                self.fail(
                    f"Unable to acquire target within {self._goal.acquire_timeout:.1f}s"
                )
            return

        time_since_detection = now - self._last_detection_time
        if time_since_detection > self._goal.lose_timeout:
            self.fail(
                f"Lost target for {time_since_detection:.1f}s (limit {self._goal.lose_timeout:.1f}s)"
            )
            return

        track_duration = self._last_detection_time - self._start_time
        if track_duration >= self._goal.min_duration:
            if self._goal.max_duration is None or elapsed >= self._goal.max_duration:
                self.succeed(f"Maintained tracking for {track_duration:.1f}s")


class SearchModule(ActionModule):
    """Simple search behaviour (default: rotate in place)."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "SearchModule")
        self._center: Optional[np.ndarray] = None
        self._yaw_rate = 0.0
        self._base_yaw = 0.0
        self._start_time = 0.0
        self._duration: Optional[float] = None

    def on_start(self, goal: SearchGoal) -> None:
        position = self.context.get_position()
        yaw = self.context.get_yaw() or 0.0
        if position is None:
            self.fail("No odometry available for search")
            return

        altitude = (
            goal.altitude
            if goal.altitude is not None
            else self.context.defaults.search_altitude
        )

        self._center = (
            np.array(goal.center, dtype=float)
            if goal.center is not None
            else position.copy()
        )
        self._center[2] = altitude
        self._yaw_rate = (
            goal.yaw_rate
            if goal.yaw_rate is not None
            else self.context.defaults.search_yaw_rate
        )
        self._duration = goal.duration
        self._base_yaw = yaw
        self._start_time = self.context.now()

        if goal.pattern.lower() not in ("rotate", "orbit"):
            self.fail(f"Search pattern '{goal.pattern}' not implemented yet")
            return

        self.context.enable_waypoint_control(True)
        self.context.enable_attitude_control(True)
        self.context.send_waypoint(self._center)

        self.create_timer(0.1, self._rotate_step)

    def _rotate_step(self) -> None:
        if self._center is None:
            return
        elapsed = self.context.now() - self._start_time
        yaw_cmd = self._base_yaw + self._yaw_rate * elapsed
        self.context.send_attitude(0.0, 0.0, yaw_cmd)

        if self._duration is not None and elapsed >= self._duration:
            self.succeed(f"Search completed after {elapsed:.1f}s")


class InspectModule(ActionModule):
    """Inspect a target by holding position and adjusting yaw."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "InspectModule")
        self._hold_position: Optional[np.ndarray] = None
        self._target_point: Optional[np.ndarray] = None
        self._target_yaw: Optional[float] = None
        self._timeout = 0.0
        self._start_time = 0.0

    def on_start(self, goal: InspectGoal) -> None:
        position = self.context.get_position()
        if position is None:
            self.fail("No odometry available for inspect")
            return

        self._hold_position = (
            np.array(goal.hold_position, dtype=float)
            if goal.hold_position is not None
            else position.copy()
        )

        self._target_point = (
            np.array(goal.target_point, dtype=float)
            if goal.target_point is not None
            else None
        )
        self._target_yaw = goal.target_yaw
        self._timeout = (
            goal.timeout
            if goal.timeout is not None
            else self.context.defaults.inspect_timeout
        )
        self._start_time = self.context.now()

        self.context.enable_waypoint_control(True)
        self.context.enable_attitude_control(True)
        self.context.send_waypoint(self._hold_position)

        self.create_timer(0.1, self._inspect_step)

    def _inspect_step(self) -> None:
        current_yaw = self.context.get_yaw() or 0.0
        position = self.context.get_position()
        if position is None or self._hold_position is None:
            return

        if self._target_point is not None:
            vector = self._target_point[:2] - position[:2]
            if np.linalg.norm(vector) > 1e-3:
                desired_yaw = float(np.arctan2(vector[1], vector[0]))
            else:
                desired_yaw = current_yaw
        elif self._target_yaw is not None:
            desired_yaw = float(self._target_yaw)
        else:
            desired_yaw = current_yaw

        self.context.send_attitude(0.0, 0.0, desired_yaw)

        elapsed = self.context.now() - self._start_time
        if elapsed >= self._timeout:
            self.succeed(f"Inspect completed after {elapsed:.1f}s")


class LandModule(ActionModule):
    """Descend to the specified landing point."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "LandModule")
        self._target: Optional[np.ndarray] = None
        self._final_altitude = 0.05
        self._timeout = 0.0
        self._start_time = 0.0

    def on_start(self, goal: LandGoal) -> None:
        position = self.context.get_position()
        if position is None:
            self.fail("No odometry available for landing")
            return

        target_xy = (
            np.array(goal.target_position[:2], dtype=float)
            if goal.target_position is not None
            else position[:2]
        )
        target_alt = (
            goal.target_position[2]
            if goal.target_position is not None and len(goal.target_position) >= 3
            else goal.final_altitude
        )
        self._final_altitude = max(0.0, goal.final_altitude)
        self._timeout = (
            goal.timeout
            if goal.timeout is not None
            else self.context.defaults.land_timeout
        )
        self._start_time = self.context.now()

        self._target = np.array([target_xy[0], target_xy[1], target_alt], dtype=float)

        self.context.enable_waypoint_control(True)
        self.context.enable_velocity_control(True)
        self.context.send_waypoint(self._target)

        self.create_timer(0.2, self._monitor_descent)

    def _monitor_descent(self) -> None:
        if self._target is None:
            return

        position = self.context.get_position()
        if position is None:
            return

        elapsed = self.context.now() - self._start_time
        altitude = position[2]
        if altitude <= self._final_altitude + self.context.defaults.altitude_tolerance:
            self.succeed("Landed successfully")
            return

        if elapsed > self._timeout:
            self.fail(f"Landing timeout after {elapsed:.1f}s (alt {altitude:.2f} m)")
            return

        # Refresh landing waypoint every second to keep controller engaged
        if elapsed % 1.0 < 0.2:
            self.context.send_waypoint(self._target)


class DeliveryModule(ActionModule):
    """Placeholder for payload delivery logic."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "DeliveryModule")

    def on_start(self, goal: DeliveryGoal) -> None:  # noqa: D401
        self.context.node.get_logger().warn(
            "[DeliveryModule] Not implemented yet; skipping action."
        )
        self.fail("Delivery action not implemented yet")


class SearchAreaModule(ActionModule):
    """Placeholder for structured area search patterns."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "SearchAreaModule")

    def on_start(self, goal: SearchAreaGoal) -> None:
        self.context.node.get_logger().warn(
            "[SearchAreaModule] Pattern '%s' not implemented yet", goal.pattern
        )
        self.fail(f"Search pattern '{goal.pattern}' not implemented yet")


class AvoidanceModule(ActionModule):
    """Placeholder for emergency avoidance manoeuvres."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "AvoidanceModule")
        self._start_time = 0.0
        self._hold_duration = 0.0

    def on_start(self, goal: AvoidanceGoal) -> None:
        position = self.context.get_position()
        if position is None:
            self.fail("No odometry available for avoidance")
            return

        self.context.enable_waypoint_control(True)
        self.context.send_waypoint(position)

        self._start_time = self.context.now()
        self._hold_duration = goal.hold_time
        self.create_timer(0.2, self._hold_step)

    def _hold_step(self) -> None:
        elapsed = self.context.now() - self._start_time
        if elapsed >= self._hold_duration:
            self.succeed(f"Avoidance hold completed ({elapsed:.1f}s)")
