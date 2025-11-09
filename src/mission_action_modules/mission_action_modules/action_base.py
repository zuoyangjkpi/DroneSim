from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, Optional, Any, List

import math
import numpy as np
from builtin_interfaces.msg import Time as TimeMsg
from geometry_msgs.msg import PoseStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class ActionOutcome(str, Enum):
    """Enumeration of possible action outcomes."""

    SUCCEEDED = "succeeded"
    FAILED = "failed"
    CANCELED = "canceled"
    RUNNING = "running"


@dataclass
class ActionResult:
    """Result returned by an action module."""

    outcome: ActionOutcome
    message: str = ""
    data: Dict[str, Any] = field(default_factory=dict)


class ActionHandle:
    """Handle returned when starting an action."""

    def __init__(self, module: "ActionModule") -> None:
        from concurrent.futures import Future

        self._module = module
        self.future: Future[ActionResult] = Future()

    def cancel(self) -> None:
        self._module.cancel()

    def done(self) -> bool:
        return self.future.done()

    def result(self, timeout: Optional[float] = None) -> ActionResult:
        return self.future.result(timeout=timeout)


@dataclass
class VehicleState:
    """Simple container for vehicle kinematic state."""

    position: Optional[np.ndarray] = None
    velocity: Optional[np.ndarray] = None
    yaw: Optional[float] = None
    stamp: float = 0.0


@dataclass
class ActionDefaults:
    """Default parameters derived from current system behaviour."""

    takeoff_altitude: float = 3.0  # meters
    takeoff_timeout: float = 15.0  # seconds
    hover_duration: float = 10.0  # seconds
    search_yaw_rate: float = 0.05  # rad/s
    search_altitude: float = 3.0  # meters
    inspect_timeout: float = 15.0  # seconds
    land_timeout: float = 20.0  # seconds
    altitude_tolerance: float = 0.1  # meters
    position_tolerance: float = 0.3  # meters


class ActionContext:
    """Provides shared ROS resources and state to action modules."""

    def __init__(self, node: Node) -> None:
        self.node = node
        self.state = VehicleState()
        self.defaults = ActionDefaults()

        # Publishers for low-level controllers
        self._waypoint_pub = node.create_publisher(
            PoseStamped, "/drone/control/waypoint_command", 10
        )
        self._waypoint_enable_pub = node.create_publisher(
            Bool, "/drone/control/waypoint_enable", 10
        )
        self._yaw_pub = node.create_publisher(
            Vector3Stamped, "/drone/control/attitude_command", 10
        )
        self._yaw_enable_pub = node.create_publisher(
            Bool, "/drone/control/attitude_enable", 10
        )
        self._velocity_enable_pub = node.create_publisher(
            Bool, "/drone/control/velocity_enable", 10
        )

        # Internal caches
        self._last_waypoint: Optional[np.ndarray] = None
        self._last_yaw_cmd: Optional[float] = None

    # ------------------------------------------------------------------
    # State update helpers
    # ------------------------------------------------------------------
    def update_odometry(self, msg: Odometry) -> None:
        position = np.array(
            [
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z,
            ],
            dtype=float,
        )
        velocity = np.array(
            [
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z,
            ],
            dtype=float,
        )

        # Compute yaw from quaternion
        q = msg.pose.pose.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = float(np.arctan2(siny_cosp, cosy_cosp))

        self.state.position = position
        self.state.velocity = velocity
        self.state.yaw = yaw
        self.state.stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9

    # ------------------------------------------------------------------
    # Controller helper utilities
    # ------------------------------------------------------------------
    def enable_waypoint_control(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._waypoint_enable_pub.publish(msg)

    def enable_yaw_control(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._yaw_enable_pub.publish(msg)

    def enable_velocity_control(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._velocity_enable_pub.publish(msg)

    def send_waypoint(
        self,
        position: np.ndarray,
        stamp: Optional[TimeMsg] = None,
    ) -> None:
        if stamp is None:
            stamp = self.node.get_clock().now().to_msg()
        pose = PoseStamped()
        pose.header.stamp = stamp
        pose.header.frame_id = "world"
        pose.pose.position.x = float(position[0])
        pose.pose.position.y = float(position[1])
        pose.pose.position.z = float(position[2])
        pose.pose.orientation.w = 1.0
        self._waypoint_pub.publish(pose)
        self._last_waypoint = position.copy()

    def send_yaw(
        self,
        roll: float,
        pitch: float,
        yaw: float,
    ) -> None:
        roll = self._wrap_angle(roll)
        pitch = self._wrap_angle(pitch)
        yaw = self._wrap_angle(yaw)
        msg = Vector3Stamped()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.vector.x = roll
        msg.vector.y = pitch
        msg.vector.z = yaw
        self._yaw_pub.publish(msg)
        self._last_yaw_cmd = yaw

    # ------------------------------------------------------------------
    # Convenience getters
    # ------------------------------------------------------------------
    def get_position(self) -> Optional[np.ndarray]:
        return None if self.state.position is None else self.state.position.copy()

    def get_velocity(self) -> Optional[np.ndarray]:
        return None if self.state.velocity is None else self.state.velocity.copy()

    def get_yaw(self) -> Optional[float]:
        return self.state.yaw

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds / 1e9

    def distance_to(self, target: np.ndarray) -> Optional[float]:
        position = self.state.position
        if position is None:
            return None
        return float(np.linalg.norm(position - target))

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


class ActionModule:
    """Base class for mission action modules."""

    def __init__(self, context: ActionContext, name: str) -> None:
        self.context = context
        self.name = name
        self._handle: Optional[ActionHandle] = None
        self._timers: List[Any] = []
        self._active = False
        self._goal: Optional[Any] = None

    # Public API --------------------------------------------------------
    def start(self, goal: Any) -> ActionHandle:
        if self._active:
            raise RuntimeError(f"{self.name} is already running")
        self._handle = ActionHandle(self)
        self._goal = goal
        self._active = True
        self.context.node.get_logger().info(f"[{self.name}] starting: {goal}")
        try:
            self.on_start(goal)
        except Exception as exc:  # pylint: disable=broad-except
            self.context.node.get_logger().error(
                f"[{self.name}] failed to start: {exc}"
            )
            self._set_result(ActionOutcome.FAILED, f"exception: {exc}")
        return self._handle

    def cancel(self) -> None:
        if not self._active:
            return
        self.context.node.get_logger().info(f"[{self.name}] canceled")
        self.on_cancel()
        self._set_result(ActionOutcome.CANCELED, "canceled by request")

    # Methods for subclasses -------------------------------------------
    def create_timer(self, period: float, callback: Callable[[], None]) -> Any:
        timer = self.context.node.create_timer(period, callback)
        self._timers.append(timer)
        return timer

    def stop_timers(self) -> None:
        for timer in self._timers:
            timer.cancel()
        self._timers.clear()

    def succeed(self, message: str = "", data: Optional[Dict[str, Any]] = None) -> None:
        self._set_result(ActionOutcome.SUCCEEDED, message, data)

    def fail(self, message: str) -> None:
        self._set_result(ActionOutcome.FAILED, message)

    # Hooks for subclasses ---------------------------------------------
    def on_start(self, goal: Any) -> None:  # pragma: no cover - abstract
        raise NotImplementedError

    def on_cancel(self) -> None:
        self.stop_timers()

    # Internal helpers -------------------------------------------------
    def _set_result(
        self,
        outcome: ActionOutcome,
        message: str = "",
        data: Optional[Dict[str, Any]] = None,
    ) -> None:
        if self._handle and not self._handle.future.done():
            result = ActionResult(outcome=outcome, message=message, data=data or {})
            self._handle.future.set_result(result)
        self.stop_timers()
        self._active = False
        self._goal = None
