#!/usr/bin/env python3
"""Waypoint Controller Plugin for Drone Low-Level Control"""

import logging
from logging.handlers import RotatingFileHandler

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool


LOG_PATH = '/tmp/drone_guidance_controllers.log'


def _init_file_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = RotatingFileHandler(LOG_PATH, maxBytes=5 * 1024 * 1024, backupCount=2)
        handler.setFormatter(logging.Formatter('%(asctime)s [%(name)s] %(message)s'))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False
    return logger

class WaypointController(Node):
    def __init__(self):
        super().__init__('waypoint_controller')

        # Parameters
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('position_tolerance', 0.2)  # meters
        self.declare_parameter('velocity_tolerance', 0.1)  # m/s
        self.declare_parameter('waypoint_timeout', 2.0)

        # PID gains
        self.declare_parameter('kp_xy', 1.0)
        self.declare_parameter('ki_xy', 0.05)
        self.declare_parameter('kd_xy', 0.01)
        self.declare_parameter('kp_z', 1.0)
        self.declare_parameter('ki_z', 0.05)
        self.declare_parameter('kd_z', 0.01)

        # Max velocities
        self.declare_parameter('max_velocity_xy', 3.0)
        self.declare_parameter('max_velocity_z', 1.5)
        self.declare_parameter('max_vertical_error', 1.5)
        self.declare_parameter('integral_limit_xy', 4.0)
        self.declare_parameter('integral_limit_z', 1.5)

        # Get parameters
        self.control_frequency = self.get_parameter('control_frequency').value
        self.position_tolerance = self.get_parameter('position_tolerance').value
        self.velocity_tolerance = self.get_parameter('velocity_tolerance').value
        self.waypoint_timeout = float(self.get_parameter('waypoint_timeout').value)

        # PID gains
        self.kp_xy = self.get_parameter('kp_xy').value
        self.ki_xy = self.get_parameter('ki_xy').value
        self.kd_xy = self.get_parameter('kd_xy').value
        self.kp_z = self.get_parameter('kp_z').value
        self.ki_z = self.get_parameter('ki_z').value
        self.kd_z = self.get_parameter('kd_z').value

        self.max_velocity_xy = self.get_parameter('max_velocity_xy').value
        self.max_velocity_z = self.get_parameter('max_velocity_z').value
        self.max_vertical_error = abs(float(self.get_parameter('max_vertical_error').value))
        self.integral_limit_xy = abs(float(self.get_parameter('integral_limit_xy').value))
        self.integral_limit_z = abs(float(self.get_parameter('integral_limit_z').value))

        # State variables
        self.current_pose = None
        self.current_velocity = None
        self.target_waypoint = None
        self.controller_active = False
        self._last_waypoint_command = None
        self._waypoint_reached = False
        self._stale_waypoint_warned = False

        # PID state
        self.position_error_integral = np.zeros(3)
        self.position_error_previous = np.zeros(3)
        self.last_control_time = None
        self.last_waypoint_time = None

        # File logger for external monitoring
        self.file_logger = _init_file_logger('waypoint_controller')

        # QoS profile for enable topic - transient local for latching behavior
        enable_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Publishers
        self.velocity_setpoint_pub = self.create_publisher(
            TwistStamped, '/drone/control/velocity_setpoint', 10)
        self.waypoint_reached_pub = self.create_publisher(
            Bool, '/drone/control/waypoint_reached', 10)

        # Subscribers
        self.waypoint_sub = self.create_subscription(
            PoseStamped, '/drone/control/waypoint_command',
            self.waypoint_callback, 10)
        self.odometry_sub = self.create_subscription(
            Odometry, '/X3/odometry', self.odometry_callback, 10)
        self.enable_sub = self.create_subscription(
            Bool, '/drone/control/waypoint_enable',
            self.enable_callback, enable_qos)

        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_frequency, self.control_loop)

        self.get_logger().info(f'Waypoint controller initialized at {self.control_frequency} Hz')

    def waypoint_callback(self, msg: PoseStamped):
        """Receive waypoint command from high-level controller"""
        new_waypoint = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        is_new_target = True
        if self._last_waypoint_command is not None:
            # Treat very small numerical differences as the same waypoint
            if np.linalg.norm(new_waypoint - self._last_waypoint_command) < 1e-3:
                is_new_target = False

        self.target_waypoint = new_waypoint
        self.last_waypoint_time = self.get_clock().now()

        if is_new_target:
            # Reset PID state only when the target actually changes
            self.position_error_integral = np.zeros(3)
            self.position_error_previous = np.zeros(3)
            self.get_logger().info(f'New waypoint received: {self.target_waypoint}')
            self._waypoint_reached = False
            self._stale_waypoint_warned = False
        else:
            self.get_logger().debug('Waypoint refreshed (no change)')

        self._last_waypoint_command = new_waypoint.copy()

    def odometry_callback(self, msg: Odometry):
        """Update current drone state"""
        self.current_pose = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

        self.current_velocity = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z
        ])

    def enable_callback(self, msg: Bool):
        """Enable/disable waypoint controller"""
        self.controller_active = msg.data
        if self.controller_active:
            self.get_logger().info('Waypoint controller ENABLED')
            self.file_logger.info('controller_enabled')
        else:
            self.get_logger().info('Waypoint controller DISABLED')
            self.target_waypoint = None
            self.last_waypoint_time = None
            self.position_error_integral = np.zeros(3)
            self.position_error_previous = np.zeros(3)
            self.last_control_time = None
            self._waypoint_reached = False
            self.file_logger.info('controller_disabled -> hover command issued')

    def control_loop(self):
        """Main control loop for waypoint following"""
        current_time = self.get_clock().now()

        if not self.controller_active:
            return

        if self.current_pose is None:
            return

        if self.target_waypoint is None:
            return

        if (self.last_waypoint_time is not None and
            (current_time - self.last_waypoint_time).nanoseconds / 1e9 > self.waypoint_timeout):
            if not self._stale_waypoint_warned:
                stale_duration = (current_time - self.last_waypoint_time).nanoseconds / 1e9
                self.get_logger().warn(
                    f'Waypoint command stale for {stale_duration:.2f} s – holding last target'
                )
                self.file_logger.info('holding_last_waypoint_due_to_timeout')
                self._stale_waypoint_warned = True
        else:
            self._stale_waypoint_warned = False

        if self.current_velocity is None:
            self.current_velocity = np.zeros(3)

        # Calculate time step
        if self.last_control_time is not None:
            dt = (current_time - self.last_control_time).nanoseconds / 1e9
        else:
            dt = 1.0 / self.control_frequency
        self.last_control_time = current_time

        # Position error
        position_error = self.target_waypoint - self.current_pose
        if self.max_vertical_error > 0.0:
            position_error[2] = float(np.clip(
                position_error[2],
                -self.max_vertical_error,
                self.max_vertical_error,
            ))

        # Check if waypoint is reached
        position_magnitude = np.linalg.norm(position_error)
        velocity_magnitude = np.linalg.norm(self.current_velocity) if self.current_velocity is not None else 0.0

        reached_now = (
            position_magnitude < self.position_tolerance and
            velocity_magnitude < self.velocity_tolerance
        )

        if reached_now and not self._waypoint_reached:
            waypoint_reached_msg = Bool()
            waypoint_reached_msg.data = True
            self.waypoint_reached_pub.publish(waypoint_reached_msg)
        self._waypoint_reached = reached_now

        if reached_now:
            position_error = np.clip(
                position_error,
                -self.position_tolerance,
                self.position_tolerance
            )

        # PID control
        # Integral term (with windup prevention)
        previous_integral = self.position_error_integral.copy()
        self.position_error_integral += position_error * dt

        # Different integral management for XY vs Z
        integral_deadband = np.array([0.01, 0.01, 0.005])  # Small deadband for Z
        decay_factor_xy = 0.2
        decay_factor_z = 0.1  # More conservative for Z direction

        for i in range(3):
            if i < 2:  # XY directions - aggressive integral management
                if abs(position_error[i]) < integral_deadband[i]:
                    self.position_error_integral[i] *= decay_factor_xy
            else:  # Z direction - conservative integral management
                # Only apply deadband decay for very small errors (hovering precision)
                if abs(position_error[i]) < integral_deadband[i]:
                    self.position_error_integral[i] *= decay_factor_z

        # Anti-windup: reset integral if error flips sign
        for i in range(3):
            if i < 2:  # XY directions - aggressive reset
                if position_error[i] * self.position_error_previous[i] <= 0.0:
                    self.position_error_integral[i] *= decay_factor_xy
            else:  # Z direction - conservative reset to maintain altitude stability
                # Only reset for significant error changes (>5cm) to avoid hover instability
                if position_error[i] * self.position_error_previous[i] <= 0.0 and abs(position_error[i]) > 0.05:
                    self.position_error_integral[i] *= decay_factor_z
        max_integral = np.array([
            self.integral_limit_xy,
            self.integral_limit_xy,
            self.integral_limit_z
        ])  # Prevent windup
        self.position_error_integral = np.clip(
            self.position_error_integral, -max_integral, max_integral)

        # Derivative term
        error_derivative = np.zeros(3)
        if dt > 0:
            error_derivative = (position_error - self.position_error_previous) / dt

        # PID gains (different for xy and z)
        kp = np.array([self.kp_xy, self.kp_xy, self.kp_z])
        ki = np.array([self.ki_xy, self.ki_xy, self.ki_z])
        kd = np.array([self.kd_xy, self.kd_xy, self.kd_z])

        # Calculate velocity command
        velocity_command_raw = (kp * position_error +
                                ki * self.position_error_integral +
                                kd * error_derivative)

        # Apply velocity limits
        max_vel = np.array([self.max_velocity_xy, self.max_velocity_xy, self.max_velocity_z])
        velocity_command = np.clip(velocity_command_raw, -max_vel, max_vel)

        # Basic integral anti-windup tied to saturation
        saturation = np.greater(np.abs(velocity_command_raw), max_vel - 1e-6)
        integral_adjusted = False
        for i in range(3):
            if (saturation[i] and
                position_error[i] * velocity_command_raw[i] > 0.0):
                self.position_error_integral[i] = previous_integral[i]
                integral_adjusted = True
        if integral_adjusted:
            velocity_command_raw = (kp * position_error +
                                    ki * self.position_error_integral +
                                    kd * error_derivative)
            velocity_command = np.clip(velocity_command_raw, -max_vel, max_vel)

        # Publish velocity setpoint
        velocity_cmd = TwistStamped()
        velocity_cmd.header.stamp = current_time.to_msg()
        velocity_cmd.header.frame_id = 'world'
        velocity_cmd.twist.linear.x = velocity_command[0]
        velocity_cmd.twist.linear.y = velocity_command[1]
        velocity_cmd.twist.linear.z = velocity_command[2]

        self.velocity_setpoint_pub.publish(velocity_cmd)

        self.file_logger.info(
            'waypoint_ctrl vel_cmd=[%.3f, %.3f, %.3f] error=[%.3f, %.3f, %.3f] reached=%s',
            velocity_command[0], velocity_command[1], velocity_command[2],
            position_error[0], position_error[1], position_error[2],
            reached_now
        )

        # Update previous error
        self.position_error_previous = position_error.copy()

        # Debug logging
        self.get_logger().debug(
            f'Waypoint control: error={position_magnitude:.3f}m, '
            f'cmd_vel=[{velocity_command[0]:.2f}, {velocity_command[1]:.2f}, {velocity_command[2]:.2f}]'
        )

def main(args=None):
    rclpy.init(args=args)
    waypoint_controller = WaypointController()

    try:
        rclpy.spin(waypoint_controller)
    except KeyboardInterrupt:
        pass

    waypoint_controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
