#!/usr/bin/env python3
"""Gazebo MulticopterVelocityControl plugin adapter."""

import logging
from logging.handlers import RotatingFileHandler

import numpy as np
import rclpy
from geometry_msgs.msg import Twist, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool

LOG_PATH = '/tmp/drone_low_level_controllers.log'


def _init_file_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = RotatingFileHandler(LOG_PATH, maxBytes=5 * 1024 * 1024, backupCount=2)
        handler.setFormatter(logging.Formatter('%(asctime)s [%(name)s] %(message)s'))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False
    return logger


class MulticopterVelocityControlAdapter(Node):
    """Pass-through adapter that remaps ROS2 setpoints to Gazebo cmd_vel."""

    def __init__(self) -> None:
        super().__init__('multicopter_velocity_control_adapter')

        # Parameters
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('max_linear_velocity_xy', 5.0)
        self.declare_parameter('max_linear_velocity_z', 4.0)
        self.declare_parameter('max_angular_velocity_xy', 3.0)
        self.declare_parameter('max_angular_velocity_z', 3.0)
        self.declare_parameter('linear_cmd_filter_alpha', 0.3)
        self.declare_parameter('yaw_rate_filter_alpha', 0.4)
        self.declare_parameter('velocity_setpoint_timeout', 0.8)
        self.declare_parameter('angular_setpoint_timeout', 0.8)

        self.control_frequency = self.get_parameter('control_frequency').value
        self.max_linear_velocity_xy = self.get_parameter('max_linear_velocity_xy').value
        self.max_linear_velocity_z = self.get_parameter('max_linear_velocity_z').value
        self.max_angular_velocity_xy = self.get_parameter('max_angular_velocity_xy').value
        self.max_angular_velocity_z = self.get_parameter('max_angular_velocity_z').value
        self.linear_cmd_filter_alpha = float(np.clip(
            self.get_parameter('linear_cmd_filter_alpha').value, 0.0, 1.0))
        self.yaw_rate_filter_alpha = float(np.clip(
            self.get_parameter('yaw_rate_filter_alpha').value, 0.0, 1.0))
        self.velocity_setpoint_timeout = float(self.get_parameter('velocity_setpoint_timeout').value)
        self.angular_setpoint_timeout = float(self.get_parameter('angular_setpoint_timeout').value)

        # QoS profile for enable topic - transient local for latching behavior
        enable_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Publishers / subscribers
        self.cmd_vel_pub = self.create_publisher(Twist, '/X3/cmd_vel', 10)

        self.velocity_setpoint_sub = self.create_subscription(
            TwistStamped,
            '/drone/control/velocity_setpoint',
            self.velocity_setpoint_callback,
            10,
        )
        self.angular_velocity_setpoint_sub = self.create_subscription(
            Vector3Stamped,
            '/drone/control/angular_velocity_setpoint',
            self.angular_velocity_setpoint_callback,
            10,
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            '/X3/odometry',
            self.odometry_callback,
            10,
        )
        self.enable_sub = self.create_subscription(
            Bool,
            '/drone/control/velocity_enable',
            self.enable_callback,
            enable_qos,
        )

        # State
        self.desired_linear_world = np.zeros(3)
        self.desired_yaw_rate = 0.0
        self.filtered_linear_world = np.zeros(3)
        self.filtered_yaw_rate = 0.0
        self.rotation_world_from_body = None  # 3x3 matrix
        self.controller_active = False
        self._last_velocity_msg_time = None
        self._last_angular_msg_time = None
        self._velocity_stale = False
        self._angular_stale = False

        self.file_logger = _init_file_logger('multicopter_velocity_control_adapter')

        self.control_timer = self.create_timer(
            1.0 / self.control_frequency,
            self.control_loop,
        )

        self.get_logger().info('Gazebo MulticopterVelocityControl adapter initialised')

    # ------------------------------------------------------------------
    def velocity_setpoint_callback(self, msg: TwistStamped) -> None:
        self.desired_linear_world = np.array([
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z,
        ])
        self._last_velocity_msg_time = self.get_clock().now()
        self._velocity_stale = False

    def angular_velocity_setpoint_callback(self, msg: Vector3Stamped) -> None:
        self.desired_yaw_rate = float(msg.vector.z)
        self._last_angular_msg_time = self.get_clock().now()
        self._angular_stale = False

    def odometry_callback(self, msg: Odometry) -> None:
        quat = (
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        )
        matrix = self._quaternion_to_matrix(quat)
        if matrix is not None:
            self.rotation_world_from_body = matrix

    def enable_callback(self, msg: Bool) -> None:
        self.controller_active = bool(msg.data)
        if self.controller_active:
            self.get_logger().info('MulticopterVelocityControl adapter ENABLED')
            self.file_logger.info('mux_enabled')
            self.filtered_linear_world = self.desired_linear_world.copy()
            self.filtered_yaw_rate = float(self.desired_yaw_rate)
        else:
            self.get_logger().info('MulticopterVelocityControl adapter DISABLED -> zero cmd_vel')
            self.file_logger.info('mux_disabled_zero')
            self.desired_linear_world = np.zeros(3)
            self.desired_yaw_rate = 0.0
            self.filtered_linear_world = np.zeros(3)
            self.filtered_yaw_rate = 0.0
            self._publish_cmd()

    # ------------------------------------------------------------------
    def _publish_cmd(self) -> None:
        cmd = Twist()

        # Convert world linear velocity into body frame if attitude info is available
        if self.rotation_world_from_body is not None:
            v_body = self.rotation_world_from_body.T @ self.filtered_linear_world
        else:
            v_body = self.filtered_linear_world

        cmd.linear.x = float(v_body[0])
        cmd.linear.y = float(v_body[1])
        cmd.linear.z = float(v_body[2])

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = float(self.filtered_yaw_rate)

        # Apply limits
        xy_norm = np.linalg.norm([cmd.linear.x, cmd.linear.y])
        if xy_norm > self.max_linear_velocity_xy > 0.0:
            scale = self.max_linear_velocity_xy / xy_norm
            cmd.linear.x *= scale
            cmd.linear.y *= scale

        cmd.linear.z = float(np.clip(cmd.linear.z, -self.max_linear_velocity_z, self.max_linear_velocity_z))
        cmd.angular.z = float(np.clip(cmd.angular.z, -self.max_angular_velocity_z, self.max_angular_velocity_z))

        self.cmd_vel_pub.publish(cmd)

    def control_loop(self) -> None:
        if not self.controller_active:
            return
        now = self.get_clock().now()
        velocity_stale = False
        angular_stale = False
        if self.velocity_setpoint_timeout > 0.0:
            if self._last_velocity_msg_time is None:
                velocity_stale = True
            else:
                age = (now - self._last_velocity_msg_time).nanoseconds / 1e9
                velocity_stale = age > self.velocity_setpoint_timeout
        if self.angular_setpoint_timeout > 0.0:
            if self._last_angular_msg_time is None:
                angular_stale = True
            else:
                age = (now - self._last_angular_msg_time).nanoseconds / 1e9
                angular_stale = age > self.angular_setpoint_timeout

        if velocity_stale:
            if not self._velocity_stale and self._last_velocity_msg_time is not None:
                age = (now - self._last_velocity_msg_time).nanoseconds / 1e9
                self.get_logger().warn(
                    f'No velocity setpoint for {age:.2f}s -> zeroing linear command')
                self.file_logger.info('zero_linear_due_to_controller_stall')
            self.desired_linear_world = np.zeros(3)
            self.filtered_linear_world = np.zeros(3)
            self._velocity_stale = True
        else:
            self._velocity_stale = False

        if angular_stale:
            if not self._angular_stale and self._last_angular_msg_time is not None:
                age = (now - self._last_angular_msg_time).nanoseconds / 1e9
                self.get_logger().warn(
                    f'No angular setpoint for {age:.2f}s -> zeroing yaw rate')
                self.file_logger.info('zero_yaw_due_to_controller_stall')
            self.desired_yaw_rate = 0.0
            self.filtered_yaw_rate = 0.0
            self._angular_stale = True
        else:
            self._angular_stale = False

        if 0.0 < self.linear_cmd_filter_alpha < 1.0:
            self.filtered_linear_world = (
                (1.0 - self.linear_cmd_filter_alpha) * self.filtered_linear_world +
                self.linear_cmd_filter_alpha * self.desired_linear_world
            )
        else:
            self.filtered_linear_world = self.desired_linear_world.copy()

        if 0.0 < self.yaw_rate_filter_alpha < 1.0:
            self.filtered_yaw_rate = (
                (1.0 - self.yaw_rate_filter_alpha) * self.filtered_yaw_rate +
                self.yaw_rate_filter_alpha * self.desired_yaw_rate
            )
        else:
            self.filtered_yaw_rate = float(self.desired_yaw_rate)

        self._publish_cmd()

    @staticmethod
    def _quaternion_to_matrix(quat) -> np.ndarray | None:
        x, y, z, w = quat
        norm = x * x + y * y + z * z + w * w
        if norm < 1e-8:
            return None
        s = 2.0 / norm
        xx = x * x * s
        yy = y * y * s
        zz = z * z * s
        xy = x * y * s
        xz = x * z * s
        yz = y * z * s
        wx = w * x * s
        wy = w * y * s
        wz = w * z * s
        return np.array([
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ])


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MulticopterVelocityControlAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
