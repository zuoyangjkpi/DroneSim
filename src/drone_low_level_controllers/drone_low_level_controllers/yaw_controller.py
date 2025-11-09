#!/usr/bin/env python3
"""Yaw-only controller bridging NMPC outputs to Gazebo angular velocity commands."""

import logging
from logging.handlers import RotatingFileHandler
import math

import numpy as np
import rclpy
from geometry_msgs.msg import Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from scipy.spatial.transform import Rotation
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


class YawController(Node):
    """Single-axis yaw controller that outputs angular velocity commands."""

    def __init__(self) -> None:
        super().__init__('yaw_controller')

        # Parameters
        self.declare_parameter('control_frequency', 150.0)
        self.declare_parameter('yaw_tolerance', 0.035)
        self.declare_parameter('kp_yaw', 1.7)
        self.declare_parameter('ki_yaw', 0.03)
        self.declare_parameter('kd_yaw', 0.9)
        self.declare_parameter('max_angular_velocity_yaw', 2.3)
        self.declare_parameter('max_yaw_step', 0.2)
        self.declare_parameter('derivative_filter_alpha', 0.4)
        self.declare_parameter('yaw_integral_limit', 1.2)

        self.control_frequency = float(self.get_parameter('control_frequency').value)
        self.yaw_tolerance = float(self.get_parameter('yaw_tolerance').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.ki_yaw = float(self.get_parameter('ki_yaw').value)
        self.kd_yaw = float(self.get_parameter('kd_yaw').value)
        self.max_yaw_rate = float(self.get_parameter('max_angular_velocity_yaw').value)
        self.max_yaw_step = abs(float(self.get_parameter('max_yaw_step').value))
        self.derivative_filter_alpha = float(np.clip(
            self.get_parameter('derivative_filter_alpha').value, 0.0, 1.0))
        self.yaw_integral_limit = abs(float(self.get_parameter('yaw_integral_limit').value))

        # State
        self.current_yaw = None
        self.current_yaw_rate = 0.0
        self.target_yaw = None
        self.last_command_time = None
        self.controller_active = False
        self.yaw_integral = 0.0
        self.prev_error = 0.0
        self.last_control_time = None
        self.filtered_derivative = 0.0

        self.file_logger = _init_file_logger('yaw_controller')
        self._log_counter = 0

        # Publishers
        self.angular_velocity_pub = self.create_publisher(
            Vector3Stamped, '/drone/control/angular_velocity_setpoint', 10)
        self.yaw_reached_pub = self.create_publisher(
            Bool, '/drone/control/attitude_reached', 10)

        # Subscribers
        self.attitude_sub = self.create_subscription(
            Vector3Stamped,
            '/drone/control/attitude_command',
            self.attitude_callback,
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
            '/drone/control/attitude_enable',
            self.enable_callback,
            10,
        )

        self.control_timer = self.create_timer(1.0 / self.control_frequency, self.control_loop)
        self.get_logger().info('Yaw controller initialised')

    def enable_callback(self, msg: Bool) -> None:
        self.controller_active = bool(msg.data)
        if self.controller_active:
            self.file_logger.info('yaw_controller_enabled')
        else:
            self.file_logger.info('yaw_controller_disabled -> zero angular cmd')
            self._publish_zero_command()

    def attitude_callback(self, msg: Vector3Stamped) -> None:
        yaw_cmd = float(msg.vector.z)
        if not math.isfinite(yaw_cmd):
            return
        yaw_cmd = self._wrap_angle(yaw_cmd)
        if self.target_yaw is None:
            self.target_yaw = yaw_cmd
        else:
            delta = self._wrap_angle(yaw_cmd - self.target_yaw)
            if self.max_yaw_step > 0.0:
                delta = float(np.clip(delta, -self.max_yaw_step, self.max_yaw_step))
            self.target_yaw = self._wrap_angle(self.target_yaw + delta)
        self.last_command_time = self.get_clock().now()
        self.yaw_integral = 0.0
        self.prev_error = 0.0

    def odometry_callback(self, msg: Odometry) -> None:
        quat = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w,
        ]
        rotation = Rotation.from_quat(quat)
        euler = rotation.as_euler('xyz', degrees=False)
        self.current_yaw = float(euler[2])
        self.current_yaw_rate = float(msg.twist.twist.angular.z)

    def control_loop(self) -> None:
        if not self.controller_active or self.target_yaw is None or self.current_yaw is None:
            return

        now = self.get_clock().now()
        if self.last_control_time is None:
            dt = 1.0 / self.control_frequency
        else:
            dt = max(1e-3, (now - self.last_control_time).nanoseconds / 1e9)
        self.last_control_time = now

        delta = self.target_yaw - self.current_yaw
        pi = math.pi
        if abs(delta) <= pi:
            yaw_error = delta
        else:
            yaw_error = self._wrap_angle(self.target_yaw + self.current_yaw)

        # Integral with clamp
        limit = self.yaw_integral_limit if self.yaw_integral_limit > 0.0 else 1.5
        self.yaw_integral = float(np.clip(self.yaw_integral + yaw_error * dt, -limit, limit))

        # Derivative
        yaw_derivative_raw = (yaw_error - self.prev_error) / dt if dt > 0.0 else 0.0
        if 0.0 < self.derivative_filter_alpha < 1.0:
            self.filtered_derivative = (
                (1.0 - self.derivative_filter_alpha) * self.filtered_derivative +
                self.derivative_filter_alpha * yaw_derivative_raw
            )
            yaw_derivative = self.filtered_derivative
        else:
            self.filtered_derivative = yaw_derivative_raw
            yaw_derivative = yaw_derivative_raw
        self.prev_error = yaw_error

        yaw_rate_cmd = (
            self.kp_yaw * yaw_error +
            self.ki_yaw * self.yaw_integral +
            self.kd_yaw * yaw_derivative
        )
        yaw_rate_cmd = float(np.clip(yaw_rate_cmd, -self.max_yaw_rate, self.max_yaw_rate))

        cmd_msg = Vector3Stamped()
        cmd_msg.header.stamp = now.to_msg()
        cmd_msg.header.frame_id = 'X3/base_link'
        cmd_msg.vector.x = 0.0
        cmd_msg.vector.y = 0.0
        cmd_msg.vector.z = yaw_rate_cmd
        self.angular_velocity_pub.publish(cmd_msg)

        if abs(yaw_error) < self.yaw_tolerance:
            reached_msg = Bool()
            reached_msg.data = True
            self.yaw_reached_pub.publish(reached_msg)

        self._log_counter += 1
        if self._log_counter >= 10:
            self.file_logger.info(
                'yaw_ctrl cmd=%.3f err=%.3f integ=%.3f deriv=%.3f',
                yaw_rate_cmd, yaw_error, self.yaw_integral, yaw_derivative,
            )
            self._log_counter = 0

    def _publish_zero_command(self) -> None:
        cmd_msg = Vector3Stamped()
        cmd_msg.header.stamp = self.get_clock().now().to_msg()
        cmd_msg.header.frame_id = 'X3/base_link'
        cmd_msg.vector.x = 0.0
        cmd_msg.vector.y = 0.0
        cmd_msg.vector.z = 0.0
        self.angular_velocity_pub.publish(cmd_msg)

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = YawController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
