from __future__ import annotations

import math
from typing import Optional

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from .action_base import ActionContext, ActionOutcome
from .action_modules import (
    FlyToTargetGoal,
    FlyToTargetModule,
    HoverGoal,
    HoverModule,
    TakeoffGoal,
    TakeoffModule,
)


class WaypointTestOrchestrator(Node):
    """Standalone orchestrator to test waypoint controllers via action modules."""

    def __init__(self) -> None:
        super().__init__("waypoint_test_orchestrator")
        self.takeoff_altitude = float(self.declare_parameter("takeoff_altitude", 3.0).value)
        self.offset_distance = float(self.declare_parameter("offset_distance", 10.0).value)
        self.offset_heading_deg = float(self.declare_parameter("offset_heading_deg", 45.0).value)
        self.hover_duration = float(self.declare_parameter("hover_duration", 8.0).value)
        self.fly_tolerance = float(self.declare_parameter("fly_tolerance", 0.3).value)
        self.fly_timeout = float(self.declare_parameter("fly_timeout", 20.0).value)

        self.context = ActionContext(self)
        self.takeoff_module = TakeoffModule(self.context)
        self.fly_module = FlyToTargetModule(self.context)
        self.hover_module = HoverModule(self.context)

        self._odom_sub = self.create_subscription(
            Odometry, "/X3/odometry", self._odom_callback, 10
        )
        self._home_position: Optional[np.ndarray] = None
        self._target_point: Optional[np.ndarray] = None

        self._stage = "waiting_for_odom"
        self._takeoff_handle = None
        self._fly_handle = None
        self._hover_handle = None
        self._shutdown_timer = None
        self._shutdown_requested = False

        self.create_timer(0.5, self._tick)
        self.get_logger().info(
            "Waypoint test orchestrator ready: takeoff %.1fm, offset %.1fm @ %.1f°",
            self.takeoff_altitude,
            self.offset_distance,
            self.offset_heading_deg,
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry) -> None:
        self.context.update_odometry(msg)
        if self._home_position is None:
            self._home_position = np.array(
                [
                    msg.pose.pose.position.x,
                    msg.pose.pose.position.y,
                    msg.pose.pose.position.z,
                ],
                dtype=float,
            )
            self.get_logger().info(
                "Captured home position at (%.2f, %.2f, %.2f)",
                *self._home_position,
            )

    # ------------------------------------------------------------------
    # Sequencer logic
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self._stage == "waiting_for_odom":
            if self.context.get_position() is not None:
                self.get_logger().info("Odometry ready, initiating takeoff sequence")
                self._stage = "takeoff"
            return

        if self._stage == "takeoff":
            self._run_takeoff()
        elif self._stage == "fly_to":
            self._run_fly_to()
        elif self._stage == "hover":
            self._run_hover()
        elif self._stage in ("complete", "error"):
            self._schedule_shutdown()

    def _run_takeoff(self) -> None:
        if self._takeoff_handle is None:
            self.get_logger().info("Starting TakeoffModule toward %.2f m", self.takeoff_altitude)
            goal = TakeoffGoal(target_altitude=self.takeoff_altitude)
            self._takeoff_handle = self.takeoff_module.start(goal)
            return

        if not self._takeoff_handle.done():
            return

        result = self._takeoff_handle.result()
        if result.outcome == ActionOutcome.SUCCEEDED:
            self.get_logger().info("Takeoff complete: %s", result.message)
            self._takeoff_handle = None
            self._stage = "fly_to"
        else:
            self._fail_sequence(f"Takeoff {result.outcome}: {result.message}")

    def _run_fly_to(self) -> None:
        if self._fly_handle is None:
            target = self._compute_target_waypoint()
            if target is None:
                self.get_logger().warn("Waiting for home position before FlyToTarget")
                return
            heading = math.atan2(target[1] - self._home_position[1], target[0] - self._home_position[0])
            goal = FlyToTargetGoal(
                waypoints=[[float(target[0]), float(target[1]), float(target[2])]],
                yaw_targets=[heading],
                tolerance=self.fly_tolerance,
                timeout_per_leg=self.fly_timeout,
            )
            self._target_point = target
            self.get_logger().info(
                "FlyToTarget -> waypoint (%.2f, %.2f, %.2f) heading %.1f°",
                target[0],
                target[1],
                target[2],
                math.degrees(heading),
            )
            self._fly_handle = self.fly_module.start(goal)
            return

        if not self._fly_handle.done():
            return

        result = self._fly_handle.result()
        if result.outcome == ActionOutcome.SUCCEEDED:
            self.get_logger().info("FlyToTarget complete: %s", result.message)
            self._fly_handle = None
            self._stage = "hover"
        else:
            self._fail_sequence(f"FlyToTarget {result.outcome}: {result.message}")

    def _run_hover(self) -> None:
        if self._hover_handle is None:
            if self._target_point is None:
                self._fail_sequence("Target point unavailable for hover hold")
                return
            goal = HoverGoal(
                duration=self.hover_duration,
                target_position=self._target_point.tolist(),
                tolerance=self.fly_tolerance,
            )
            self.get_logger().info(
                "Holding position at waypoint for %.1f s", self.hover_duration
            )
            self._hover_handle = self.hover_module.start(goal)
            return

        if not self._hover_handle.done():
            return

        result = self._hover_handle.result()
        if result.outcome == ActionOutcome.SUCCEEDED:
            self.get_logger().info("Hover complete: %s", result.message)
            self._hover_handle = None
            self._stage = "complete"
        else:
            self._fail_sequence(f"Hover {result.outcome}: {result.message}")

    def _compute_target_waypoint(self) -> Optional[np.ndarray]:
        if self._home_position is None:
            return None
        heading_rad = math.radians(self.offset_heading_deg)
        dx = self.offset_distance * math.cos(heading_rad)
        dy = self.offset_distance * math.sin(heading_rad)
        return np.array(
            [
                self._home_position[0] + dx,
                self._home_position[1] + dy,
                self.takeoff_altitude,
            ],
            dtype=float,
        )

    def _fail_sequence(self, reason: str) -> None:
        if self._stage == "error":
            return
        self._stage = "error"
        self.get_logger().error("Waypoint test aborted: %s", reason)
        self.context.disable_all_controllers()

    def _schedule_shutdown(self) -> None:
        if self._shutdown_requested:
            return
        self._shutdown_requested = True
        if self._shutdown_timer is None:
            self._shutdown_timer = self.create_timer(2.0, self._request_shutdown)

    def _request_shutdown(self) -> None:
        if self._shutdown_timer:
            self._shutdown_timer.cancel()
            self._shutdown_timer = None
        if not rclpy.ok():
            return
        self.get_logger().info("Waypoint test orchestrator shutting down")
        rclpy.shutdown()


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaypointTestOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Waypoint test interrupted by user")
        node.context.disable_all_controllers()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
