from __future__ import annotations

from typing import Dict, Optional
import json

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import String

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
        self._context = ActionContext(self)
        self.modules = {
            "takeoff": TakeoffModule(self._context),
            "hover": HoverModule(self._context),
            "fly_to": FlyToTargetModule(self._context),
            "track_target": TrackTargetModule(self._context),
            "search": SearchModule(self._context),
            "lost_hold": LostHoldModule(self._context),
            "inspect": InspectModule(self._context),
            "land": LandModule(self._context),
            "delivery": DeliveryModule(self._context),
            "search_area": SearchAreaModule(self._context),
            "avoidance": AvoidanceModule(self._context),
        }
        self._active_handles: Dict[str, ActionHandle] = {}

        # Subscriptions
        self.odom_sub = self.create_subscription(
            Odometry, "/X3/odometry", self._odometry_callback, 10
        )

        # Basic services for manual testing / bring-up
        self.create_service(Trigger, "mission_actions/takeoff", self._srv_takeoff)
        self.create_service(Trigger, "mission_actions/hover", self._srv_hover)
        self.create_service(Trigger, "mission_actions/search", self._srv_search)
        self.create_service(Trigger, "mission_actions/land", self._srv_land)
        self.create_service(Trigger, "mission_actions/track_target", self._srv_track_target)
        self.create_service(Trigger, "mission_actions/lost_hold", self._srv_lost_hold)

        self.event_pub = self.create_publisher(String, "/mission_actions/events", 10)

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
        self._context.update_odometry(msg)

    def _start_action(self, name: str, goal) -> bool:
        module = self.modules[name]
        active_handle = self._active_handles.get(name)
        if active_handle and not active_handle.done():
            self.get_logger().warn("Action '%s' already running", name)
            return False

        try:
            handle = module.start(goal)
        except RuntimeError as exc:
            self.get_logger().error("Failed to start %s: %s", name, exc)
            self._publish_event(name, "failed_to_start", str(exc))
            return False

        self._active_handles[name] = handle
        handle.future.add_done_callback(
            lambda fut, action_name=name: self._on_action_done(action_name, fut)
        )
        self._publish_event(name, "started", "action started")
        return True

    def _on_action_done(self, name: str, future) -> None:
        try:
            result: ActionResult = future.result()
            self.get_logger().info(
                "Action '%s' finished with %s: %s",
                name,
                result.outcome.value,
                result.message,
            )
            self._publish_event(name, result.outcome.value, result.message)
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error("Action '%s' raised exception: %s", name, exc)
            self._publish_event(name, "exception", str(exc))
        finally:
            self._active_handles.pop(name, None)

    def _publish_event(self, action: str, outcome: str, message: str) -> None:
        msg = String()
        msg.data = json.dumps({
            "action": action,
            "outcome": outcome,
            "message": message,
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
