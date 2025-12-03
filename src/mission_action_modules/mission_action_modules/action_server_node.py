#!/usr/bin/env python3
"""ROS2 Action Server exposing all AVIANS Action Modules.

This node provides a unified ROS2 Action interface for all Action Modules,
allowing external systems (like SLAM BT) to call them directly.
"""

from __future__ import annotations

import json
from typing import Any, Dict, Optional

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from uav_msgs.action import ExecuteModule
from nav_msgs.msg import Odometry

from .action_base import ActionContext, ActionOutcome
from .action_modules import (
    TakeoffModule,
    HoverModule,
    FlyToTargetModule,
    TrackTargetModule,
    SearchModule,
    InspectModule,
    LostHoldModule,
    LandModule,
    DeliveryModule,
    SearchAreaModule,
    AvoidanceModule,
    TakeoffGoal,
    HoverGoal,
    FlyToTargetGoal,
    TrackTargetGoal,
    SearchGoal,
    InspectGoal,
    LostHoldGoal,
    LandGoal,
    DeliveryGoal,
    SearchAreaGoal,
    AvoidanceGoal,
)


class ActionModuleServer(Node):
    """ROS2 Action Server exposing all AVIANS Action Modules."""

    def __init__(self):
        super().__init__("action_module_server")

        # Create action context
        self.context = ActionContext(self)

        # Subscribe to odometry
        self.create_subscription(
            Odometry,
            "/X3/odometry",
            self.context.update_odometry,
            10,
        )

        # Initialize all modules
        self.modules = {
            "TAKEOFF": TakeoffModule(self.context),
            "HOVER": HoverModule(self.context),
            "FLY_TO": FlyToTargetModule(self.context),
            "TRACK": TrackTargetModule(self.context),
            "SEARCH": SearchModule(self.context),
            "INSPECT": InspectModule(self.context),
            "LOST_HOLD": LostHoldModule(self.context),
            "LAND": LandModule(self.context),
            "DELIVERY": DeliveryModule(self.context),
            "SEARCH_AREA": SearchAreaModule(self.context),
            "AVOIDANCE": AvoidanceModule(self.context),
        }

        # Create Action Server with reentrant callback group
        self.callback_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self,
            ExecuteModule,
            "/avians/execute_module",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group,
        )

        self.get_logger().info("Action Module Server started")
        self.get_logger().info(f"Available modules: {list(self.modules.keys())}")

    def goal_callback(self, goal_request) -> GoalResponse:
        """Accept or reject incoming goal requests."""
        module_name = goal_request.module_name.upper()

        if module_name not in self.modules:
            self.get_logger().error(
                f"Unknown module: {module_name}. "
                f"Available: {list(self.modules.keys())}"
            )
            return GoalResponse.REJECT

        self.get_logger().info(f"Accepting goal for module: {module_name}")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle) -> CancelResponse:
        """Handle cancellation requests."""
        self.get_logger().info("Cancel request received")
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        """Execute the requested module."""
        request = goal_handle.request
        module_name = request.module_name.upper()
        goal_json = request.goal_json

        self.get_logger().info(
            f"Executing module: {module_name} with goal: {goal_json}"
        )

        # Get the module
        module = self.modules.get(module_name)
        if module is None:
            result = ExecuteModule.Result()
            result.success = False
            result.message = f"Unknown module: {module_name}"
            result.result_json = "{}"
            goal_handle.abort()
            return result

        # Parse goal JSON to module-specific goal
        try:
            goal_dict = json.loads(goal_json) if goal_json else {}
            goal_obj = self._create_goal_object(module_name, goal_dict)
        except (json.JSONDecodeError, ValueError) as e:
            result = ExecuteModule.Result()
            result.success = False
            result.message = f"Failed to parse goal: {e}"
            result.result_json = "{}"
            goal_handle.abort()
            return result

        # Start the module
        handle = module.start(goal_obj)

        # Monitor progress and publish feedback
        while not handle.done():
            # Check for cancellation
            if goal_handle.is_cancel_requested:
                handle.cancel()
                goal_handle.canceled()
                result = ExecuteModule.Result()
                result.success = False
                result.message = "Canceled by client"
                result.result_json = "{}"
                return result

            # Publish feedback
            feedback = ExecuteModule.Feedback()
            feedback.status = f"Running {module_name}"
            feedback.state = "RUNNING"
            feedback.progress = 0.5  # Could be more sophisticated
            goal_handle.publish_feedback(feedback)

            # Sleep briefly
            await rclpy.sleep_for(0.2)

        # Get result
        module_result = handle.result()
        result = ExecuteModule.Result()
        result.success = (module_result.outcome == ActionOutcome.SUCCEEDED)
        result.message = module_result.message
        result.result_json = json.dumps(module_result.data)

        if result.success:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        self.get_logger().info(
            f"Module {module_name} completed: {result.message}"
        )

        return result

    def _create_goal_object(self, module_name: str, goal_dict: Dict[str, Any]) -> Any:
        """Create module-specific goal object from dictionary."""
        goal_classes = {
            "TAKEOFF": TakeoffGoal,
            "HOVER": HoverGoal,
            "FLY_TO": FlyToTargetGoal,
            "TRACK": TrackTargetGoal,
            "SEARCH": SearchGoal,
            "INSPECT": InspectGoal,
            "LOST_HOLD": LostHoldGoal,
            "LAND": LandGoal,
            "DELIVERY": DeliveryGoal,
            "SEARCH_AREA": SearchAreaGoal,
            "AVOIDANCE": AvoidanceGoal,
        }

        goal_class = goal_classes.get(module_name)
        if goal_class is None:
            raise ValueError(f"No goal class for module: {module_name}")

        # Create goal from dictionary
        return goal_class(**goal_dict)


def main(args=None):
    rclpy.init(args=args)

    node = ActionModuleServer()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
