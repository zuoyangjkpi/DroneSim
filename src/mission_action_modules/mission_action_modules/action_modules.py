"""Concrete action modules used by the mission executor."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Sequence

import math
import numpy as np

from std_msgs.msg import Bool, Float64MultiArray
import rcl_interfaces.srv
import rcl_interfaces.msg

from .action_base import ActionModule, ActionContext, ActionOutcome


# ---------------------------------------------------------------------------
# COCO class name to ID mapping for YOLO detector
# ---------------------------------------------------------------------------

COCO_CLASS_MAP = {
    "person": 0, "bicycle": 1, "car": 2, "motorbike": 3, "aeroplane": 4,
    "bus": 5, "train": 6, "truck": 7, "boat": 8, "traffic light": 9,
    "fire hydrant": 10, "stop sign": 11, "parking meter": 12, "bench": 13, "bird": 14,
    "cat": 15, "dog": 16, "horse": 17, "sheep": 18, "cow": 19,
    "elephant": 20, "bear": 21, "zebra": 22, "giraffe": 23, "backpack": 24,
    "umbrella": 25, "handbag": 26, "tie": 27, "suitcase": 28, "frisbee": 29,
    "skis": 30, "snowboard": 31, "sports ball": 32, "kite": 33, "baseball bat": 34,
    "baseball glove": 35, "skateboard": 36, "surfboard": 37, "tennis racket": 38, "bottle": 39,
    "wine glass": 40, "cup": 41, "fork": 42, "knife": 43, "spoon": 44,
    "bowl": 45, "banana": 46, "apple": 47, "sandwich": 48, "orange": 49,
    "broccoli": 50, "carrot": 51, "hot dog": 52, "pizza": 53, "donut": 54,
    "cake": 55, "chair": 56, "sofa": 57, "pottedplant": 58, "bed": 59,
    "diningtable": 60, "toilet": 61, "tvmonitor": 62, "laptop": 63, "mouse": 64,
    "remote": 65, "keyboard": 66, "cell phone": 67, "microwave": 68, "oven": 69,
    "toaster": 70, "sink": 71, "refrigerator": 72, "book": 73, "clock": 74,
    "vase": 75, "scissors": 76, "teddy bear": 77, "hair drier": 78, "toothbrush": 79,
}


def get_coco_class_id(class_name: Optional[str]) -> Optional[int]:
    """Convert COCO class name to class ID. Returns None if class_name is None or not found."""
    if class_name is None:
        return None
    return COCO_CLASS_MAP.get(class_name.lower())


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
    use_planner: bool = False  # If True, treat waypoints[0] as goal and plan path


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
    confirmations: int = 3
    target_class: Optional[str] = None  # COCO class name (e.g., "person", "car", "truck")


@dataclass
class LostHoldGoal:
    duration: float = 5.0  # seconds, wait for target reacquisition
    hold_position: Optional[Sequence[float]] = None  # xyz
    yaw: Optional[float] = None  # radians
    detection_confirmations: int = 3


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
        self._yaw_reference = 0.0
        self._timeout_warned = False
        self._stable_start_time: Optional[float] = None

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
        self._timeout_warned = False
        self._stable_start_time = None
        self._target = np.array([position[0], position[1], target_altitude], dtype=float)

        # Enable controllers
        self.context.enable_waypoint_control(True)
        self.context.enable_yaw_control(True)
        self.context.enable_velocity_control(True)
        self._yaw_reference = self.context.get_yaw() or 0.0

        self.context.send_waypoint(self._target)
        self.context.send_yaw(0.0, 0.0, self._yaw_reference)
        self.create_timer(0.2, self._monitor_altitude)

    def _monitor_altitude(self) -> None:
        if self._target is None:
            return

        position = self.context.get_position()
        if position is None:
            return

        altitude_error = abs(position[2] - self._target[2])
        elapsed = self.context.now() - self._start_time
        tolerance = self.context.defaults.altitude_tolerance
        stable_required = max(
            0.0, getattr(self.context.defaults, "takeoff_stable_duration", 3.0)
        )

        if altitude_error <= tolerance:
            if self._stable_start_time is None:
                self._stable_start_time = self.context.now()
            stable_elapsed = self.context.now() - self._stable_start_time
            if stable_elapsed >= stable_required:
                self.succeed(
                    f"Reached altitude {self._target[2]:.2f} m and held for "
                    f"{stable_required:.1f}s"
                )
                return
        else:
            self._stable_start_time = None

        # Debug logging every 1s
        if elapsed % 1.0 < 0.2:
            stable_t = 0.0
            if self._stable_start_time is not None:
                stable_t = self.context.now() - self._stable_start_time
            self.context.node.get_logger().info(
                f"[TakeoffDebug] Alt={position[2]:.2f}m, Target={self._target[2]:.2f}m, "
                f"Error={altitude_error:.3f}m, StableTime={stable_t:.1f}s"
            )

        if elapsed > self._timeout:
            # Do not abort control; keep publishing so the vehicle keeps climbing
            if not getattr(self, "_timeout_warned", False):
                self.context.node.get_logger().warn(
                    f"[TakeoffModule] Timeout after {elapsed:.1f}s before reaching "
                    f"{self._target[2]:.2f} m. Continuing to command climb."
                )
                self._timeout_warned = True
            # Reset timer to avoid spamming the log and continue commanding the same target
            self._start_time = self.context.now()
            self.context.send_waypoint(self._target)
            self.context.send_yaw(0.0, 0.0, self._yaw_reference)
            return

        # Refresh waypoint periodically for robustness
        if elapsed % 1.0 < 0.2:
            self.context.send_waypoint(self._target)
            self.context.send_yaw(0.0, 0.0, self._yaw_reference)


class HoverModule(ActionModule):
    """Hold at a fixed position for a given duration."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "HoverModule")
        self._target: Optional[np.ndarray] = None
        self._tolerance = 0.0
        self._duration: Optional[float] = 0.0
        self._start_time = 0.0
        self._hover_yaw = 0.0

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
        duration = (
            goal.duration
            if goal.duration is not None
            else getattr(self.context.defaults, "hover_duration", 10.0)
        )
        if duration is None or duration <= 0.0:
            self._duration = None  # hold indefinitely until canceled
        else:
            self._duration = duration
        self._tolerance = (
            goal.tolerance
            if goal.tolerance is not None
            else self.context.defaults.position_tolerance
        )
        self._start_time = self.context.now()

        self.context.enable_waypoint_control(True)
        self.context.enable_yaw_control(True)
        self.context.send_waypoint(self._target)
        self._hover_yaw = self.context.get_yaw() or 0.0
        self.context.send_yaw(0.0, 0.0, self._hover_yaw)

        self.create_timer(0.2, self._monitor_hover)

    def _monitor_hover(self) -> None:
        if self._target is None:
            return
        position = self.context.get_position()
        if position is None:
            return

        elapsed = self.context.now() - self._start_time
        distance = float(np.linalg.norm(position - self._target))

        if self._duration is not None and elapsed >= self._duration:
            self.succeed(f"Hover complete ({elapsed:.1f}s)")
            return

        if distance > self._tolerance * 2.0 or elapsed % 1.0 < 0.2:
            self.context.send_waypoint(self._target)
            self.context.send_yaw(0.0, 0.0, self._hover_yaw)


class FlyToTargetModule(ActionModule):
    """Track a sequence of waypoints produced by the planner."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "FlyToTargetModule")
        self._waypoints: List[np.ndarray] = []
        self._yaw_targets: Optional[List[Optional[float]]] = None
        self._current_idx = 0
        self._start_time = 0.0
        self._timeout_per_leg = -1.0  # Default to unlimited time
        self._tolerance = 0.3
        self._last_waypoint_refresh = 0.0
        
        # Path planning client
        from uav_msgs.srv import PlanPath
        self._plan_client = context.node.create_client(PlanPath, 'mission_planner/plan_path')

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
        # Default to no per-leg timeout and rely on mission_executor stage timeout
        self._timeout_per_leg = (
            goal.timeout_per_leg if goal.timeout_per_leg is not None else -1.0  # -1 means unlimited
        )
        self._last_waypoint_refresh = self.context.now()
        self._current_idx = 0
        self._start_time = self.context.now()

        self.context.enable_waypoint_control(True)
        self.context.enable_velocity_control(True)
        self.context.enable_yaw_control(True)

        if goal.use_planner and len(self._waypoints) > 0:
            self._request_path_plan(self._waypoints[-1])
        else:
            self._command_current_waypoint()
            self.create_timer(0.2, self._monitor_progress)

    def _request_path_plan(self, goal_pos: np.ndarray) -> None:
        from uav_msgs.srv import PlanPath
        from geometry_msgs.msg import Pose
        
        if not self._plan_client.wait_for_service(timeout_sec=1.0):
            self.fail("Path planner service unavailable")
            return
            
        current_pos = self.context.get_position()
        if current_pos is None:
            self.fail("No odometry for planning")
            return
            
        req = PlanPath.Request()
        req.start.position.x = float(current_pos[0])
        req.start.position.y = float(current_pos[1])
        req.start.position.z = float(current_pos[2])
        
        req.goal.position.x = float(goal_pos[0])
        req.goal.position.y = float(goal_pos[1])
        req.goal.position.z = float(goal_pos[2])
        
        future = self._plan_client.call_async(req)
        future.add_done_callback(self._on_plan_received)
        self.context.node.get_logger().info(f"Requested path to {goal_pos}")

    def _on_plan_received(self, future) -> None:
        try:
            resp = future.result()
        except Exception as e:
            self.fail(f"Planning failed: {e}")
            return
            
        if not resp.success:
            self.fail("Planner returned failure")
            return
            
        # Convert path to waypoints
        new_waypoints = []
        for pose in resp.path:
            new_waypoints.append(np.array([
                pose.position.x,
                pose.position.y,
                pose.position.z
            ]))
            
        if not new_waypoints:
            self.fail("Planner returned empty path")
            return
            
        self.context.node.get_logger().info(f"Received path with {len(new_waypoints)} waypoints")
        self._waypoints = new_waypoints
        self._current_idx = 0
        self._start_time = self.context.now()
        
        # Start execution
        self._command_current_waypoint()
        self.create_timer(0.2, self._monitor_progress)

    def _command_current_waypoint(self) -> None:
        target = self._waypoints[self._current_idx]
        self.context.send_waypoint(target)

        yaw_cmd = None
        if self._yaw_targets:
            yaw_cmd = self._yaw_targets[self._current_idx]
        if yaw_cmd is None:
            yaw_cmd = self.context.get_yaw() or 0.0
        self.context.send_yaw(0.0, 0.0, float(yaw_cmd))

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
                self.context.node.get_logger().info(
                    f"[FlyToTargetModule] Reached final waypoint {self._current_idx}/{len(self._waypoints)} | "
                    f"Distance: {distance:.3f}m <= Tolerance: {self._tolerance:.3f}m"
                )
                self.succeed("Reached final waypoint")
                return
            self.context.node.get_logger().info(
                f"[FlyToTargetModule] Reached waypoint {self._current_idx}/{len(self._waypoints)} | "
                f"Distance: {distance:.3f}m <= Tolerance: {self._tolerance:.3f}m | Moving to next"
            )
            self._command_current_waypoint()
            return

        # timeout_per_leg <= 0 means no timeout enforcement
        if self._timeout_per_leg > 0 and elapsed > self._timeout_per_leg:
            self.fail(
                f"Timeout moving to waypoint {self._current_idx + 1}/{len(self._waypoints)}"
            )
            return

        # Refresh the waypoint every 0.5 s so the controller never times out
        now = self.context.now()
        if now - self._last_waypoint_refresh >= 0.5:
            self.context.send_waypoint(target)
            self._last_waypoint_refresh = now


class TrackTargetModule(ActionModule):
    """Engage NMPC tracking loop and monitor target detection."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "TrackTargetModule")
        from geometry_msgs.msg import PoseStamped, Vector3Stamped

        # Publisher to enable/disable NMPC
        self._enable_pub = context.node.create_publisher(Bool, "/nmpc/enable", 10)

        # Subscribe to NMPC outputs and forward to ActionContext
        self._nmpc_waypoint_sub = context.node.create_subscription(
            PoseStamped,
            "/nmpc/waypoint_command",
            self._nmpc_waypoint_callback,
            10,
        )
        self._nmpc_attitude_sub = context.node.create_subscription(
            Vector3Stamped,
            "/nmpc/attitude_command",
            self._nmpc_attitude_callback,
            10,
        )

        # Subscribe to controller status
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

    def _reconfigure_yolo_detector(self, target_class: Optional[str]) -> None:
        """Reconfigure YOLO detector's desired_class parameter at runtime."""
        if target_class is None:
            return

        class_id = get_coco_class_id(target_class)
        if class_id is None:
            self.context.node.get_logger().warn(
                f"[TrackTargetModule] Unknown COCO class '{target_class}', cannot reconfigure YOLO detector"
            )
            return

        # Use ROS 2 parameter client to reconfigure YOLO detector node
        from rclpy.parameter import Parameter
        try:
            yolo_node_name = "/yolo12_detector_node"
            param_client = self.context.node.create_client(
                rcl_interfaces.srv.SetParameters,
                f"{yolo_node_name}/set_parameters"
            )

            if not param_client.wait_for_service(timeout_sec=1.0):
                self.context.node.get_logger().warn(
                    f"[TrackTargetModule] YOLO detector parameter service not available"
                )
                return

            # Create SetParameters request
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType

            request = SetParameters.Request()
            param = ParamMsg()
            param.name = "desired_class"
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = class_id
            request.parameters = [param]

            # Call service synchronously (with timeout)
            future = param_client.call_async(request)
            # Wait briefly for the parameter to be set
            import time
            time.sleep(0.1)

            self.context.node.get_logger().info(
                f"[TrackTargetModule] Reconfigured YOLO detector to detect class '{target_class}' (ID: {class_id})"
            )
        except Exception as exc:
            self.context.node.get_logger().warn(
                f"[TrackTargetModule] Failed to reconfigure YOLO detector: {exc}"
            )

    def on_start(self, goal: TrackTargetGoal) -> None:
        self._goal = goal
        self._start_time = self.context.now()
        self._last_detection_time = 0.0
        self._currently_detected = False

        # Reconfigure YOLO detector if target_class is specified
        if goal.target_class:
            self._reconfigure_yolo_detector(goal.target_class)

        # Enable low-level controllers via ActionContext
        self.context.enable_waypoint_control(True)
        self.context.enable_yaw_control(True)
        self.context.enable_velocity_control(True)

        # Enable NMPC
        self._set_nmpc_enabled(True)

        if goal.target_class:
            stand_off = f"{goal.stand_off_distance:.2f} m" if goal.stand_off_distance else "default"
            self.context.node.get_logger().info(
                f"[TrackTargetModule] Engage tracking for class '{goal.target_class}' (stand-off: {stand_off})"
            )
        else:
            self.context.node.get_logger().info(
                "[TrackTargetModule] Engage tracking with default detector target"
            )

        self.create_timer(0.2, self._monitor_tracking)

    def _nmpc_waypoint_callback(self, msg) -> None:
        """Forward NMPC waypoint commands to ActionContext (only when this module is active)."""
        if not self._active:
            return
        # Extract position from PoseStamped
        import numpy as np
        position = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])
        self.context.send_waypoint(position, stamp=msg.header.stamp)

    def _nmpc_attitude_callback(self, msg) -> None:
        """Forward NMPC attitude commands to ActionContext (only when this module is active)."""
        if not self._active:
            return
        # Extract roll, pitch, yaw from Vector3Stamped
        roll = float(msg.vector.x)
        pitch = float(msg.vector.y)
        yaw = float(msg.vector.z)
        self.context.send_yaw(roll, pitch, yaw)

    def _set_nmpc_enabled(self, enabled: bool) -> None:
        msg = Bool()
        msg.data = bool(enabled)
        self._enable_pub.publish(msg)

    def on_cancel(self) -> None:
        """Called when module is canceled - disable NMPC and controllers."""
        self.context.node.get_logger().info("[TrackTargetModule] Canceling - disabling NMPC and controllers")
        self._set_nmpc_enabled(False)
        # Controllers will be disabled by ActionBase._set_result()
        super().on_cancel()

    def succeed(self, message: str = "", data: Optional[Dict[str, Any]] = None) -> None:
        """Called when tracking succeeds - disable NMPC."""
        self.context.node.get_logger().info("[TrackTargetModule] Succeeded - disabling NMPC")
        self._set_nmpc_enabled(False)
        super().succeed(message, data)

    def fail(self, message: str) -> None:
        """Called when tracking fails - disable NMPC."""
        self.context.node.get_logger().info(f"[TrackTargetModule] Failed: {message} - disabling NMPC")
        self._set_nmpc_enabled(False)
        super().fail(message)

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
        if (
            self._goal.max_duration is not None
            and track_duration >= self._goal.min_duration
            and elapsed >= self._goal.max_duration
        ):
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
        self._last_waypoint_refresh = 0.0
        self._waypoint_refresh_period = 0.5  # seconds
        # SearchModule is a pure execution module - no detection monitoring
        # MissionSequenceController handles detection and module switching

    def _reconfigure_yolo_detector(self, target_class: Optional[str]) -> None:
        """Reconfigure YOLO detector's desired_class parameter at runtime."""
        if target_class is None:
            return

        class_id = get_coco_class_id(target_class)
        if class_id is None:
            self.context.node.get_logger().warn(
                f"[SearchModule] Unknown COCO class '{target_class}', cannot reconfigure YOLO detector"
            )
            return

        # Use ROS 2 parameter client to reconfigure YOLO detector node
        from rclpy.parameter import Parameter
        try:
            yolo_node_name = "/yolo12_detector_node"
            param_client = self.context.node.create_client(
                rcl_interfaces.srv.SetParameters,
                f"{yolo_node_name}/set_parameters"
            )

            if not param_client.wait_for_service(timeout_sec=1.0):
                self.context.node.get_logger().warn(
                    f"[SearchModule] YOLO detector parameter service not available"
                )
                return

            # Create SetParameters request
            from rcl_interfaces.srv import SetParameters
            from rcl_interfaces.msg import Parameter as ParamMsg, ParameterValue, ParameterType

            request = SetParameters.Request()
            param = ParamMsg()
            param.name = "desired_class"
            param.value.type = ParameterType.PARAMETER_INTEGER
            param.value.integer_value = class_id
            request.parameters = [param]

            # Call service synchronously (with timeout)
            future = param_client.call_async(request)
            # Wait briefly for the parameter to be set
            import time
            time.sleep(0.1)

            self.context.node.get_logger().info(
                f"[SearchModule] Reconfigured YOLO detector to detect class '{target_class}' (ID: {class_id})"
            )
        except Exception as exc:
            self.context.node.get_logger().warn(
                f"[SearchModule] Failed to reconfigure YOLO detector: {exc}"
            )

    def on_start(self, goal: SearchGoal) -> None:
        # Reconfigure YOLO detector if target_class is specified
        if goal.target_class:
            self._reconfigure_yolo_detector(goal.target_class)
            self.context.node.get_logger().info(
                f"[SearchModule] Searching for target class: {goal.target_class}"
            )

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
        self.context.enable_yaw_control(True)
        self.context.send_waypoint(self._center)
        # Publish initial yaw command immediately so the controller starts from current heading
        self.context.send_yaw(0.0, 0.0, yaw)
        self._last_waypoint_refresh = self.context.now()

        self.create_timer(0.1, self._rotate_step)

    def _rotate_step(self) -> None:
        if self._center is None:
            return
        now = self.context.now()
        elapsed = now - self._start_time
        # Accumulate yaw based on the start time so rotation is continuous
        yaw_cmd = self._base_yaw + self._yaw_rate * elapsed
        yaw_cmd = self.context._wrap_angle(yaw_cmd)
        self.context.send_yaw(0.0, 0.0, yaw_cmd)

        if now - self._last_waypoint_refresh >= self._waypoint_refresh_period:
            # Keep refreshing the waypoint so the PID layer never times out
            self.context.send_waypoint(self._center)
            self._last_waypoint_refresh = now

        # SearchModule only completes on timeout (if duration set) or external cancellation
        # MissionSequenceController monitors detections and switches modules
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

        if goal.hold_position is not None:
            self._hold_position = np.array(goal.hold_position, dtype=float)
        else:
            self._hold_position = position.copy()
            self._hold_position[2] = self.context.defaults.takeoff_altitude

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
        self.context.enable_yaw_control(True)
        self.context.send_waypoint(self._hold_position)

        self.create_timer(0.1, self._inspect_step)


class LostHoldModule(ActionModule):
    """Hold last known pose while waiting for detections to resume."""

    def __init__(self, context: ActionContext) -> None:
        super().__init__(context, "LostHoldModule")
        self._goal: Optional[LostHoldGoal] = None
        self._start_time = 0.0
        self._hold_position: Optional[np.ndarray] = None
        self._hold_yaw = 0.0
        self._confirmations_required = 3
        self._detection_streak = 0
        self._stable_start_time: Optional[float] = None
        self._status_sub = context.node.create_subscription(
            Float64MultiArray,
            "/drone/controller/status",
            self._status_callback,
            10,
        )

    def on_start(self, goal: LostHoldGoal) -> None:
        position = self.context.get_position()
        yaw = self.context.get_yaw() or 0.0
        if position is None:
            self.fail("No odometry available for lost-hold")
            return

        self._goal = goal
        self._start_time = self.context.now()
        self._confirmations_required = max(1, goal.detection_confirmations)
        self._detection_streak = 0
        if goal.hold_position is not None:
            self._hold_position = np.array(goal.hold_position, dtype=float)
        else:
            self._hold_position = position.copy()
            self._hold_position[2] = self.context.defaults.takeoff_altitude
        self._hold_yaw = goal.yaw if goal.yaw is not None else yaw
        self._stable_start_time = None

        self.context.enable_waypoint_control(True)
        self.context.enable_yaw_control(True)
        self.context.send_waypoint(self._hold_position)
        self.context.send_yaw(0.0, 0.0, self._hold_yaw)

        self.create_timer(0.1, self._hold_step)

    def _hold_step(self) -> None:
        if self._goal is None:
            return

        # Continuously refresh waypoint and yaw commands to keep the controller active
        if self._hold_position is not None:
            self.context.send_waypoint(self._hold_position)
        self.context.send_yaw(0.0, 0.0, self._hold_yaw)

        # ✅ Check if target reacquired
        if self._detection_streak >= self._confirmations_required:
            self.succeed("Target reacquired during hold", data={"reacquired": True})
            return

        # ✅ Check timeout - if target not reacquired within duration, transition to SEARCH
        elapsed = self.context.now() - self._start_time
        if elapsed >= self._goal.duration:
            self.succeed(f"Hold timeout ({elapsed:.1f}s), target not found", data={"reacquired": False})
            return

    def _status_callback(self, msg: Float64MultiArray) -> None:
        detected = bool(int(msg.data[0])) if msg.data else False
        if detected:
            self._detection_streak = min(self._detection_streak + 1, self._confirmations_required)
        else:
            self._detection_streak = 0

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

        self.context.send_yaw(0.0, 0.0, desired_yaw)

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
        self._landing_yaw = 0.0

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
        self._timeout_warned = False  # Initialize timeout warning flag

        self._target = np.array([target_xy[0], target_xy[1], target_alt], dtype=float)

        self.context.enable_waypoint_control(True)
        self.context.enable_velocity_control(True)
        self.context.enable_yaw_control(True)
        self.context.send_waypoint(self._target)
        self._landing_yaw = self.context.get_yaw() or 0.0
        self.context.send_yaw(0.0, 0.0, self._landing_yaw)

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

        # Keep trying instead of failing on timeout (mirrors TakeoffModule behavior)
        if elapsed > self._timeout:
            if not getattr(self, "_timeout_warned", False):
                self.context.node.get_logger().warn(
                    f"[LandModule] Timeout after {elapsed:.1f}s (alt {altitude:.2f} m). "
                    f"Continuing descent to {self._final_altitude:.2f} m."
                )
                self._timeout_warned = True
            # Reset the timer so the warning does not spam continuously
            self._start_time = self.context.now()

        # Refresh landing waypoint every second to keep controller engaged
        if elapsed % 1.0 < 0.2:
            self.context.send_waypoint(self._target)
            self.context.send_yaw(0.0, 0.0, self._landing_yaw)


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
        self.context.enable_yaw_control(True)
        self.context.send_waypoint(position)
        yaw = self.context.get_yaw() or 0.0
        self.context.send_yaw(0.0, 0.0, yaw)

        self._start_time = self.context.now()
        self._hold_duration = goal.hold_time
        self.create_timer(0.2, self._hold_step)

    def _hold_step(self) -> None:
        elapsed = self.context.now() - self._start_time
        if elapsed >= self._hold_duration:
            self.succeed(f"Avoidance hold completed ({elapsed:.1f}s)")
