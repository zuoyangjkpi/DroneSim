from __future__ import annotations

import math
from typing import Optional, Tuple

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped, Vector3Stamped
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

        self.action_context = ActionContext(self)
        self.takeoff_module = TakeoffModule(self.action_context)
        self.fly_module = FlyToTargetModule(self.action_context)
        self.hover_module = HoverModule(self.action_context)

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
        self._latest_odom: Optional[Odometry] = None
        self._current_command_waypoint: Optional[np.ndarray] = None
        self._current_command_yaw: Optional[float] = None
        self._cmd_velocity: Optional[np.ndarray] = None
        self._cmd_ang_velocity: Optional[np.ndarray] = None

        self.create_timer(0.5, self._tick)
        self.create_timer(1.0, self._log_control_snapshot)

        self._velocity_cmd_sub = self.create_subscription(
            TwistStamped, "/drone/control/velocity_setpoint", self._velocity_cmd_callback, 10
        )
        self._angular_cmd_sub = self.create_subscription(
            Vector3Stamped, "/drone/control/angular_velocity_setpoint", self._angular_cmd_callback, 10
        )

        self.get_logger().info(
            f"Waypoint test orchestrator ready: takeoff {self.takeoff_altitude:.1f}m, "
            f"offset {self.offset_distance:.1f}m @ {self.offset_heading_deg:.1f}°"
        )

    # ------------------------------------------------------------------
    # ROS callbacks
    # ------------------------------------------------------------------
    def _odom_callback(self, msg: Odometry) -> None:
        self.action_context.update_odometry(msg)
        self._latest_odom = msg
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
                f"Captured home position at ({self._home_position[0]:.2f}, "
                f"{self._home_position[1]:.2f}, {self._home_position[2]:.2f})"
            )

    # ------------------------------------------------------------------
    # Sequencer logic
    # ------------------------------------------------------------------
    def _tick(self) -> None:
        if self._stage == "waiting_for_odom":
            if self.action_context.get_position() is not None:
                self.get_logger().info("Odometry ready, initiating takeoff sequence")
                self._stage = "takeoff"
            return

        if self._stage == "takeoff":
            self._run_takeoff()
        elif self._stage == "fly_to":
            self._run_fly_to()
        elif self._stage == "hover":
            self._run_hover()
        elif self._stage == "complete":
            self._run_continuous_tracking()
        elif self._stage == "error":
            self._schedule_shutdown()

    def _run_takeoff(self) -> None:
        if self._takeoff_handle is None:
            self.get_logger().info(
                f"Starting TakeoffModule toward {self.takeoff_altitude:.2f} m"
            )
            goal = TakeoffGoal(target_altitude=self.takeoff_altitude)
            self._takeoff_handle = self.takeoff_module.start(goal)
            position = self.action_context.get_position()
            if position is not None:
                self._current_command_waypoint = np.array(
                    [position[0], position[1], self.takeoff_altitude], dtype=float
                )
            self._current_command_yaw = self.action_context.get_yaw() or 0.0
            return

        if not self._takeoff_handle.done():
            return

        result = self._takeoff_handle.result()
        if result.outcome == ActionOutcome.SUCCEEDED:
            self.get_logger().info(f"Takeoff complete: {result.message}")
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
            self._current_command_waypoint = target.copy()
            self._current_command_yaw = float(heading)
            self.get_logger().info(
                f"FlyToTarget -> waypoint ({target[0]:.2f}, {target[1]:.2f}, {target[2]:.2f}) "
                f"heading {math.degrees(heading):.1f}°"
            )
            self._fly_handle = self.fly_module.start(goal)
            return

        if not self._fly_handle.done():
            return

        result = self._fly_handle.result()
        if result.outcome == ActionOutcome.SUCCEEDED:
            position = self.action_context.get_position()
            if position is not None and self._target_point is not None:
                final_dist = float(np.linalg.norm(position - self._target_point))
                self.get_logger().info(
                    f"FlyToTarget complete: {result.message} | "
                    f"Final distance to target: {final_dist:.2f}m | "
                    f"Entering continuous tracking mode for controller testing"
                )
            else:
                self.get_logger().info(f"FlyToTarget complete: {result.message} | Entering continuous tracking")
            self._fly_handle = None
            # 进入complete阶段，持续发布目标点
            self._stage = "complete"
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
            if self.hover_duration <= 0.0:
                self.get_logger().info("Holding position at waypoint indefinitely")
            else:
                self.get_logger().info(
                    f"Holding position at waypoint for {self.hover_duration:.1f} s"
                )
            self._current_command_waypoint = self._target_point.copy()
            self._hover_handle = self.hover_module.start(goal)
            return

        if not self._hover_handle.done():
            return

        result = self._hover_handle.result()
        if result.outcome == ActionOutcome.SUCCEEDED:
            # hover_duration=-1时不应该到这里，但如果到了就记录
            self.get_logger().info(
                f"Hover complete unexpectedly: {result.message} | hover_duration={self.hover_duration}"
            )
            self._hover_handle = None
            # 重新启动hover以继续测试
            self._stage = "hover"
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

    def _run_continuous_tracking(self) -> None:
        """持续发布目标路径点和yaw，用于长期控制器测试"""
        if self._target_point is None:
            return

        # 每秒发布一次目标点以保持控制器活跃
        current_time = self.get_clock().now().nanoseconds / 1e9
        if not hasattr(self, '_last_publish_time'):
            self._last_publish_time = 0.0

        if current_time - self._last_publish_time >= 1.0:
            self.action_context.send_waypoint(self._target_point)
            # 计算朝向目标的yaw（如果有home_position）
            if self._home_position is not None:
                heading = math.atan2(
                    self._target_point[1] - self._home_position[1],
                    self._target_point[0] - self._home_position[0]
                )
                self.action_context.send_yaw(0.0, 0.0, heading)
            self._last_publish_time = current_time

    def _fail_sequence(self, reason: str) -> None:
        if self._stage == "error":
            return
        prev_stage = self._stage
        self._stage = "error"
        self.get_logger().error(
            f"Waypoint test aborted: {reason} | Previous stage: {prev_stage}"
        )
        self.action_context.disable_all_controllers()

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
            self.get_logger().warn("Shutdown requested but rclpy not ok, already shutting down")
            return
        self.get_logger().info(
            f"Waypoint test orchestrator shutting down | Stage: {self._stage}"
        )
        rclpy.shutdown()

    # ------------------------------------------------------------------
    # Command/topic tracking
    # ------------------------------------------------------------------
    def _velocity_cmd_callback(self, msg: TwistStamped) -> None:
        self._cmd_velocity = np.array(
            [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            dtype=float,
        )

    def _angular_cmd_callback(self, msg: Vector3Stamped) -> None:
        self._cmd_ang_velocity = np.array(
            [msg.vector.x, msg.vector.y, msg.vector.z],
            dtype=float,
        )

    def _log_control_snapshot(self) -> None:
        if self._latest_odom is None:
            return

        # 优先使用 ActionContext 缓存的实际命令
        cmd_pos = self.action_context.get_last_waypoint()
        if cmd_pos is None:
            # 备用：使用预测值（仅在模块还未发送命令时）
            cmd_pos = self._current_command_waypoint
        if cmd_pos is None:
            return

        # 优先使用 ActionContext 缓存的实际姿态命令
        cached_att = self.action_context.get_last_attitude_command()
        if cached_att is not None:
            cmd_att = cached_att.copy()
        elif self._current_command_yaw is not None:
            # 备用：使用预测的偏航角
            cmd_att = np.array([0.0, 0.0, self._current_command_yaw], dtype=float)
        else:
            cmd_att = np.zeros(3, dtype=float)

        pos, vel, att, ang_vel = self._extract_state(self._latest_odom)
        cmd_vel = self._cmd_velocity if self._cmd_velocity is not None else np.zeros(3)
        cmd_ang = (
            self._cmd_ang_velocity if self._cmd_ang_velocity is not None else np.zeros(3)
        )

        err_pos = cmd_pos - pos  # 误差 = 目标 - 当前（更符合控制器视角）
        err_vel = cmd_vel - vel
        err_att = np.array([self._wrap_angle(cmd_att[i] - att[i]) for i in range(3)])
        err_ang = cmd_ang - ang_vel

        # 计算水平距离和偏航角（度）
        dist_xy = math.sqrt(err_pos[0]**2 + err_pos[1]**2)
        yaw_deg = math.degrees(att[2])
        cmd_yaw_deg = math.degrees(cmd_att[2])
        err_yaw_deg = math.degrees(err_att[2])

        # 分行打印，便于调参时查看
        self.get_logger().info("=" * 100)
        self.get_logger().info(
            f"[位置] 当前: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}) | "
            f"目标: ({cmd_pos[0]:.2f}, {cmd_pos[1]:.2f}, {cmd_pos[2]:.2f}) | "
            f"误差: ({err_pos[0]:.2f}, {err_pos[1]:.2f}, {err_pos[2]:.2f}) | "
            f"水平距离: {dist_xy:.2f}m"
        )
        self.get_logger().info(
            f"[速度] 当前: ({vel[0]:.2f}, {vel[1]:.2f}, {vel[2]:.2f}) | "
            f"目标: ({cmd_vel[0]:.2f}, {cmd_vel[1]:.2f}, {cmd_vel[2]:.2f}) | "
            f"误差: ({err_vel[0]:.2f}, {err_vel[1]:.2f}, {err_vel[2]:.2f})"
        )
        self.get_logger().info(
            f"[姿态] 当前Yaw: {yaw_deg:6.1f}° | "
            f"目标Yaw: {cmd_yaw_deg:6.1f}° | "
            f"误差Yaw: {err_yaw_deg:6.1f}° | "
            f"Roll/Pitch误差: ({math.degrees(err_att[0]):.1f}°, {math.degrees(err_att[1]):.1f}°)"
        )
        self.get_logger().info(
            f"[角速度] 当前: ({math.degrees(ang_vel[0]):.2f}, {math.degrees(ang_vel[1]):.2f}, {math.degrees(ang_vel[2]):.2f})°/s | "
            f"目标: ({math.degrees(cmd_ang[0]):.2f}, {math.degrees(cmd_ang[1]):.2f}, {math.degrees(cmd_ang[2]):.2f})°/s"
        )

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def _extract_state(msg: Odometry) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        pos = np.array(
            [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z],
            dtype=float,
        )

        # Odometry的线速度是机体坐标系，需要转换为世界坐标系
        vel_body = np.array(
            [msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z],
            dtype=float,
        )

        q = msg.pose.pose.orientation
        # 构建从机体到世界的旋转矩阵
        x, y, z, w = q.x, q.y, q.z, q.w
        norm = x * x + y * y + z * z + w * w
        if norm < 1e-8:
            vel = vel_body  # 四元数无效时，假设速度已经是世界坐标系
        else:
            s = 2.0 / norm
            xx, yy, zz = x * x * s, y * y * s, z * z * s
            xy, xz, yz = x * y * s, x * z * s, y * z * s
            wx, wy, wz = w * x * s, w * y * s, w * z * s
            R_world_from_body = np.array([
                [1.0 - (yy + zz), xy - wz, xz + wy],
                [xy + wz, 1.0 - (xx + zz), yz - wx],
                [xz - wy, yz + wx, 1.0 - (xx + yy)],
            ])
            vel = R_world_from_body @ vel_body  # 转换到世界坐标系

        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        sinp = 2.0 * (q.w * q.y - q.z * q.x)
        sinp = np.clip(sinp, -1.0, 1.0)
        pitch = math.asin(sinp)

        sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z)
        cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        att = np.array([roll, pitch, yaw], dtype=float)

        ang_vel = np.array(
            [msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z],
            dtype=float,
        )
        return pos, vel, att, ang_vel


def main(args: Optional[list[str]] = None) -> None:
    rclpy.init(args=args)
    node = WaypointTestOrchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Waypoint test interrupted by user")
        node.action_context.disable_all_controllers()
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
