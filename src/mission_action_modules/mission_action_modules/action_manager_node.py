from __future__ import annotations

from typing import Any, Dict, Optional
import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String, Float64MultiArray
from neural_network_msgs.msg import NeuralNetworkDetectionArray

from .action_base import ActionContext, ActionHandle, ActionResult
from .action_modules import (
    AvoidanceGoal,
    AvoidanceModule,
    DeliveryGoal,
    DeliveryModule,
    FlyToTargetGoal,
    FlyToTargetModule,
    HoverGoal,
    HoverModule,
    InspectGoal,
    InspectModule,
    LostHoldGoal,
    LostHoldModule,
    LandGoal,
    LandModule,
    SearchAreaGoal,
    SearchAreaModule,
    SearchGoal,
    SearchModule,
    TakeoffGoal,
    TakeoffModule,
    TrackTargetGoal,
    TrackTargetModule,
    get_coco_class_id,
)


class ActionManagerNode(Node):
    """Lightweight node that owns all action modules and exposes basic services."""

    def __init__(self) -> None:
        super().__init__("mission_action_manager")
        # Note: the Node base class keeps its ROS context in _context—do not overwrite it
        self._action_context = ActionContext(self)
        self.modules = {
            "takeoff": TakeoffModule(self._action_context),
            "hover": HoverModule(self._action_context),
            "fly_to": FlyToTargetModule(self._action_context),
            "track_target": TrackTargetModule(self._action_context),
            "search": SearchModule(self._action_context),
            "lost_hold": LostHoldModule(self._action_context),
            "inspect": InspectModule(self._action_context),
            "land": LandModule(self._action_context),
            "delivery": DeliveryModule(self._action_context),
            "search_area": SearchAreaModule(self._action_context),
            "avoidance": AvoidanceModule(self._action_context),
        }
        self._active_handles: Dict[str, ActionHandle] = {}
        # Global mutex: only one active module is allowed at a time
        self._current_active_module: Optional[str] = None

        # Cache for latest action params from mission_executor
        self._latest_params: Optional[Dict[str, Any]] = None
        self._target_class: Optional[str] = None
        self._target_class_id: Optional[int] = None

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, "/X3/odometry", self._odometry_callback, 10
        )
        self.detection_sub = self.create_subscription(
            NeuralNetworkDetectionArray,
            "/target_detections",
            self._detection_callback,
            10
        )

        # Subscribe to action params from mission_executor
        self.params_sub = self.create_subscription(
            String,
            "/mission_executor/action_params",
            self._params_callback,
            10
        )

        # Subscribe to NMPC internal status to get tracking details
        self.nmpc_status_sub = self.create_subscription(
            Float64MultiArray,
            "/nmpc/internal_status",
            self._nmpc_status_callback,
            10
        )

        # Publishers
        self.event_pub = self.create_publisher(String, "/mission_actions/events", 10)
        self.status_pub = self.create_publisher(
            Float64MultiArray,
            "/drone/controller/status",
            10
        )

        # Detection state
        self._target_detected = False
        self._last_detection_time = 0.0

        # NMPC tracking details (from /nmpc/internal_status)
        self._desired_tracking_distance = 0.0
        self._current_tracking_distance = 0.0
        self._tracking_altitude = 3.0
        self._optimization_time = 0.0
        self._iterations = 0.0
        self._cost = 0.0

        # Periodic status publishing
        self.create_timer(0.1, self._publish_status)  # 10 Hz

        # Basic services for manual testing / bring-up
        self.create_service(Trigger, "mission_actions/takeoff", self._srv_takeoff)
        self.create_service(Trigger, "mission_actions/fly_to", self._srv_fly_to)
        self.create_service(Trigger, "mission_actions/hover", self._srv_hover)
        self.create_service(Trigger, "mission_actions/search", self._srv_search)
        self.create_service(Trigger, "mission_actions/land", self._srv_land)
        self.create_service(Trigger, "mission_actions/track_target", self._srv_track_target)
        self.create_service(Trigger, "mission_actions/lost_hold", self._srv_lost_hold)

        self.get_logger().info(f"Mission action manager ready with {len(self.modules)} modules")

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------
    def _params_callback(self, msg: String) -> None:
        """Cache the latest action params from mission_executor."""
        try:
            self._latest_params = json.loads(msg.data)
            self.get_logger().debug(
                f"Received params for stage {self._latest_params.get('stage_id')}: "
                f"{self._latest_params.get('params')}"
            )
            self._update_target_class_from_params(self._latest_params)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Failed to parse action params: {exc}")
            self._latest_params = None
            self._target_class = None
            self._target_class_id = None

    def _create_goal_from_params(self, goal_class, default_goal):
        """Create a goal object from cached params, or use default."""
        if not self._latest_params or not self._latest_params.get("params"):
            return default_goal

        params = self._latest_params["params"]
        try:
            # Create goal with params from YAML
            goal = goal_class(**params)
            self.get_logger().info(f"Created goal with params: {params}")
            return goal
        except (TypeError, ValueError) as exc:
            self.get_logger().warn(
                f"Failed to create goal from params {params}: {exc}. Using defaults."
            )
            return default_goal

    def _srv_takeoff(self, _req, res):
        goal = self._create_goal_from_params(TakeoffGoal, TakeoffGoal())
        started = self._start_action("takeoff", goal)
        res.success = started
        res.message = "Takeoff command started" if started else "Takeoff already running"
        return res

    def _srv_fly_to(self, _req, res):
        goal = self._create_goal_from_params(FlyToTargetGoal, FlyToTargetGoal(waypoints=[]))
        started = self._start_action("fly_to", goal)
        res.success = started
        res.message = "FlyTo command started" if started else "FlyTo already running"
        return res

    def _srv_hover(self, _req, res):
        goal = self._create_goal_from_params(HoverGoal, HoverGoal())
        started = self._start_action("hover", goal)
        res.success = started
        res.message = "Hover command started" if started else "Hover already running"
        return res

    def _srv_search(self, _req, res):
        goal = self._create_goal_from_params(SearchGoal, SearchGoal())
        started = self._start_action("search", goal)
        res.success = started
        res.message = "Search command started" if started else "Search already running"
        return res

    def _srv_land(self, _req, res):
        goal = self._create_goal_from_params(LandGoal, LandGoal())
        started = self._start_action("land", goal)
        res.success = started
        res.message = "Landing command started" if started else "Landing already running"
        return res

    def _srv_track_target(self, _req, res):
        goal = self._create_goal_from_params(TrackTargetGoal, TrackTargetGoal())
        started = self._start_action("track_target", goal)
        res.success = started
        res.message = "TrackTarget command started" if started else "TrackTarget already running"
        return res

    def _srv_lost_hold(self, _req, res):
        goal = self._create_goal_from_params(LostHoldGoal, LostHoldGoal())
        started = self._start_action("lost_hold", goal)
        res.success = started
        res.message = "LostHold command started" if started else "LostHold already running"
        return res

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _odometry_callback(self, msg: Odometry) -> None:
        self._action_context.update_odometry(msg)

    def _detection_callback(self, msg: NeuralNetworkDetectionArray) -> None:
        """Process YOLO detections and update global detection state."""
        detected = False
        if self._target_class_id is None:
            detected = len(msg.detections) > 0
        else:
            detected = any(det.object_class == self._target_class_id for det in msg.detections)

        self._target_detected = detected
        if detected:
            self._last_detection_time = self.get_clock().now().nanoseconds / 1e9

    def _nmpc_status_callback(self, msg: Float64MultiArray) -> None:
        """Process NMPC internal status and extract tracking details."""
        if not msg.data or len(msg.data) < 6:
            return
        # NMPC publishes: [target_detected, desired_distance, altitude, opt_time, iterations, cost, current_distance]
        self._desired_tracking_distance = float(msg.data[1])
        self._tracking_altitude = float(msg.data[2])
        self._optimization_time = float(msg.data[3])
        self._iterations = float(msg.data[4])
        self._cost = float(msg.data[5])
        if len(msg.data) >= 7:
            self._current_tracking_distance = float(msg.data[6])
        else:
            self._current_tracking_distance = self._desired_tracking_distance

    def _publish_status(self) -> None:
        """Publish global controller status for all action modules."""
        msg = Float64MultiArray()
        # data[0]: target detected (from YOLO via ActionManager)
        # data[1-6]: tracking details (from NMPC via /nmpc/internal_status)
        msg.data = [
            float(self._target_detected),
            self._desired_tracking_distance,
            self._tracking_altitude,
            self._optimization_time,
            self._iterations,
            self._cost,
            self._current_tracking_distance,
        ]
        self.status_pub.publish(msg)

    def _update_target_class_from_params(self, params: Optional[Dict[str, Any]]) -> None:
        """Update target class filter based on latest action parameters."""
        target_class = None
        class_id = None
        if params:
            param_block = params.get("params") or {}
            maybe_class = param_block.get("target_class")
            if isinstance(maybe_class, str) and maybe_class:
                target_class = maybe_class
                class_id = get_coco_class_id(maybe_class)
                if class_id is None:
                    self.get_logger().warn(
                        f"Unknown target_class '{maybe_class}' provided; detections will not be filtered"
                    )
            elif maybe_class:
                self.get_logger().warn(f"target_class param has unexpected type: {type(maybe_class)}")

        self._target_class = target_class if class_id is not None else None
        self._target_class_id = class_id

    def _start_action(self, name: str, goal) -> bool:
        # Global mutex check: cancel any other running action first
        if self._current_active_module is not None and self._current_active_module != name:
            old_name = self._current_active_module
            old_handle = self._active_handles.get(old_name)
            if old_handle and not old_handle.done():
                self.get_logger().info(
                    f"🔄 Switching from '{old_name}' to '{name}' - canceling old action"
                )
                old_handle.cancel()  # Cancel the old module immediately
                # Wait for the old module to stop (max 0.5 s)
                try:
                    old_handle.result(timeout=0.5)
                except Exception:
                    pass  # Continue even if it times out

        module = self.modules[name]
        active_handle = self._active_handles.get(name)
        if active_handle and not active_handle.done():
            self.get_logger().warn(f"Action '{name}' already running")
            return False

        try:
            handle = module.start(goal)
        except RuntimeError as exc:
            self.get_logger().error(f"Failed to start {name}: {exc}")
            self._publish_event(name, "failed_to_start", str(exc))
            return False

        self._active_handles[name] = handle
        self._current_active_module = name  # Update the currently active module
        handle.future.add_done_callback(
            lambda fut, action_name=name: self._on_action_done(action_name, fut)
        )
        self._publish_event(name, "started", "action started")
        return True

    def _on_action_done(self, name: str, future) -> None:
        try:
            result: ActionResult = future.result()
            self.get_logger().info(
                f"Action '{name}' finished with {result.outcome.value}: {result.message}"
            )
            self._publish_event(name, result.outcome.value, result.message, result.data)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error(f"Action '{name}' raised exception: {exc}")
            self._publish_event(name, "exception", str(exc), {})
        finally:
            self._active_handles.pop(name, None)
            # Clear the active-module marker
            if self._current_active_module == name:
                self._current_active_module = None

    def _publish_event(self, action: str, outcome: str, message: str, data: dict = None) -> None:
        msg = String()
        msg.data = json.dumps({
            "action": action,
            "outcome": outcome,
            "message": message,
            "data": data or {},
        })
        self.event_pub.publish(msg)


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = ActionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
