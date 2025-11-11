from __future__ import annotations

from typing import Dict, Optional
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
)


class ActionManagerNode(Node):
    """Lightweight node that owns all action modules and exposes basic services."""

    def __init__(self) -> None:
        super().__init__("mission_action_manager")
        # 注意：Node 基类使用 _context 存放 ROS Context，不要覆盖
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
        # 全局互斥：同一时间只能有一个active module
        self._current_active_module: Optional[str] = None

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, "/X3/odometry", self._odometry_callback, 10
        )
        self.detection_sub = self.create_subscription(
            NeuralNetworkDetectionArray,
            "/person_detections",
            self._detection_callback,
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
        self._person_detected = False
        self._last_detection_time = 0.0

        # Periodic status publishing
        self.create_timer(0.1, self._publish_status)  # 10 Hz

        # Basic services for manual testing / bring-up
        self.create_service(Trigger, "mission_actions/takeoff", self._srv_takeoff)
        self.create_service(Trigger, "mission_actions/hover", self._srv_hover)
        self.create_service(Trigger, "mission_actions/search", self._srv_search)
        self.create_service(Trigger, "mission_actions/land", self._srv_land)
        self.create_service(Trigger, "mission_actions/track_target", self._srv_track_target)
        self.create_service(Trigger, "mission_actions/lost_hold", self._srv_lost_hold)

        self.get_logger().info("Mission action manager ready")

    # ------------------------------------------------------------------
    # Service callbacks
    # ------------------------------------------------------------------
    def _srv_takeoff(self, _req, res):
        started = self._start_action("takeoff", TakeoffGoal())
        res.success = started
        res.message = "Takeoff command started" if started else "Takeoff already running"
        return res

    def _srv_hover(self, _req, res):
        started = self._start_action("hover", HoverGoal())
        res.success = started
        res.message = "Hover command started" if started else "Hover already running"
        return res

    def _srv_search(self, _req, res):
        started = self._start_action("search", SearchGoal())
        res.success = started
        res.message = "Search command started" if started else "Search already running"
        return res

    def _srv_land(self, _req, res):
        started = self._start_action("land", LandGoal())
        res.success = started
        res.message = "Landing command started" if started else "Landing already running"
        return res
    
    def _srv_track_target(self, _req, res):
        started = self._start_action("track_target", TrackTargetGoal())
        res.success = started
        res.message = "TrackTarget command started" if started else "TrackTarget already running"
        return res

    def _srv_lost_hold(self, _req, res):
        started = self._start_action("lost_hold", LostHoldGoal())
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
        self._person_detected = len(msg.detections) > 0
        if self._person_detected:
            self._last_detection_time = self.get_clock().now().nanoseconds / 1e9

    def _publish_status(self) -> None:
        """Publish global controller status for all action modules."""
        msg = Float64MultiArray()
        # data[0]: person detected (bool as float)
        # data[1-5]: reserved for future use (tracking distance, altitude, etc.)
        msg.data = [
            float(self._person_detected),
            0.0,  # tracking_distance (placeholder)
            3.0,  # tracking_altitude (placeholder)
            0.0,  # optimization_time (placeholder)
            0.0,  # iterations (placeholder)
            0.0,  # cost (placeholder)
        ]
        self.status_pub.publish(msg)

    def _start_action(self, name: str, goal) -> bool:
        # 全局互斥检查：如果有其他action正在运行，先强制停止
        if self._current_active_module is not None and self._current_active_module != name:
            old_name = self._current_active_module
            old_handle = self._active_handles.get(old_name)
            if old_handle and not old_handle.done():
                self.get_logger().info(
                    f"🔄 Switching from '{old_name}' to '{name}' - canceling old action"
                )
                old_handle.cancel()  # 立即取消旧模块
                # 等待旧模块完全停止（最多等待0.5秒）
                try:
                    old_handle.result(timeout=0.5)
                except Exception:
                    pass  # 超时也继续

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
        self._current_active_module = name  # 更新当前活动模块
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
            # 清除当前活动模块标记
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
