from __future__ import annotations

from typing import Dict, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_srvs.srv import Trigger

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
        self.context = ActionContext(self)
        self.modules = {
            "takeoff": TakeoffModule(self.context),
            "hover": HoverModule(self.context),
            "fly_to": FlyToTargetModule(self.context),
            "track_target": TrackTargetModule(self.context),
            "search": SearchModule(self.context),
            "inspect": InspectModule(self.context),
            "land": LandModule(self.context),
            "delivery": DeliveryModule(self.context),
            "search_area": SearchAreaModule(self.context),
            "avoidance": AvoidanceModule(self.context),
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
        started = self._start_action("search", SearchGoal(duration=15.0))
        res.success = started
        res.message = "Search command started" if started else "Search already running"
        return res

    def _srv_land(self, _req, res):
        started = self._start_action("land", LandGoal())
        res.success = started
        res.message = "Landing command started" if started else "Landing already running"
        return res

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _odometry_callback(self, msg: Odometry) -> None:
        self.context.update_odometry(msg)

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
            return False

        self._active_handles[name] = handle
        handle.future.add_done_callback(
            lambda fut, action_name=name: self._on_action_done(action_name, fut)
        )
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
        except Exception as exc:  # pylint: disable=broad-except
            self.get_logger().error("Action '%s' raised exception: %s", name, exc)
        finally:
            self._active_handles.pop(name, None)


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
