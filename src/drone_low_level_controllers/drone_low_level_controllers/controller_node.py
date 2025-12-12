#!/usr/bin/env python3
"""
Cascaded Multicopter Controller Node

Implements a full control stack from velocity commands to motor speeds:
1. Velocity Controller: cmd_vel → desired attitude + thrust
2. Attitude Controller: desired attitude → desired angular rates
3. Angular Rate Controller: desired angular rates → torques
4. Mixer: thrust + torques → individual motor speeds

Coordinate frames:
- Body frame (FRD): Forward-Right-Down
- World frame (ENU): East-North-Up (ROS2 standard)
"""

import logging
from logging.handlers import RotatingFileHandler
from dataclasses import dataclass
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

# For Gazebo communication via ros_gz_bridge
from actuator_msgs.msg import Actuators

LOG_PATH = '/tmp/drone_low_level_controllers.log'


def _init_file_logger(name: str) -> logging.Logger:
    """Initialize file logger for external monitoring."""
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = RotatingFileHandler(LOG_PATH, maxBytes=5 * 1024 * 1024, backupCount=2)
        handler.setFormatter(logging.Formatter('%(asctime)s [%(name)s] %(message)s'))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False
    return logger


@dataclass
class ControllerGains:
    """Controller gain parameters."""
    # Velocity controller gains (P-D control)
    kp_vel_xy: float = 3.0  # From Gazebo velocityGain
    kp_vel_z: float = 4.0
    kd_vel_xy: float = 0.5
    kd_vel_z: float = 0.8

    # Attitude controller gains (P control)
    kp_att_roll: float = 4.0  # From Gazebo attitudeGain
    kp_att_pitch: float = 4.0
    kp_att_yaw: float = 5.0

    # Angular rate controller gains (P control)
    kp_rate_roll: float = 5.0  # From Gazebo angularRateGain
    kp_rate_pitch: float = 5.0
    kp_rate_yaw: float = 6.0

    # Limits
    max_tilt_angle: float = 0.5  # rad (~28.6 degrees)
    # X3 max thrust (~2.85e-05 * 800^2 * 8 ≈ 146 N)
    max_thrust: float = 150.0  # N (a bit above physical max to avoid clipping)
    min_thrust: float = 0.0  # N


@dataclass
class VehicleParameters:
    """Physical parameters of the vehicle."""
    mass: float = 4.6  # kg (from X3 base_link inertial)
    arm_length: float = 0.6  # m (rotor radius in model.sdf)
    gravity: float = 9.81  # m/s^2
    inertia_xx: float = 0.423  # kg·m^2 (from X3 base_link)
    inertia_yy: float = 0.423
    inertia_zz: float = 0.828

    # Motor parameters (from SDF)
    force_constant: float = 2.85e-05  # Force per motor speed squared
    moment_constant: float = 0.0533  # Moment arm coefficient

    # Motor speed limits
    max_motor_speed: float = 800.0  # rad/s (from SDF)
    min_motor_speed: float = 0.0  # rad/s

    # Number of motors
    num_motors: int = 8


class VelocityController:
    """
    Velocity controller: converts body-frame velocity commands to desired attitude and thrust.

    Input: desired velocity (body frame), current velocity (body frame)
    Output: desired roll, pitch angles and total thrust
    """

    def __init__(self, gains: ControllerGains, params: VehicleParameters):
        self.gains = gains
        self.params = params

    def compute(
        self,
        vel_cmd: np.ndarray,  # [vx, vy, vz] in body frame
        vel_current: np.ndarray,  # [vx, vy, vz] in body frame
        accel_current: np.ndarray,  # [ax, ay, az] in body frame (optional, can be zeros)
        yaw_current: float,  # Current yaw angle (rad)
    ) -> tuple[float, float, float, float]:  # (roll_des, pitch_des, yaw_des, thrust)
        """
        Compute desired attitude and thrust from velocity commands.

        Returns:
            roll_des: Desired roll angle (rad)
            pitch_des: Desired pitch angle (rad)
            yaw_des: Desired yaw angle (rad) - currently just maintains current yaw
            thrust: Total thrust command (N)
        """
        # Velocity error (body frame)
        vel_error = vel_cmd - vel_current

        # Desired acceleration (PD control)
        desired_accel = (
            self.gains.kp_vel_xy * vel_error[:2] -
            self.gains.kd_vel_xy * accel_current[:2]
        ) / self.params.mass
        desired_accel_z = (
            self.gains.kp_vel_z * vel_error[2] -
            self.gains.kd_vel_z * accel_current[2]
        ) / self.params.mass

        # Convert horizontal accelerations to desired roll/pitch
        # In body frame:
        # - positive roll (right) causes acceleration in +Y (right)
        # - positive pitch (nose up) causes acceleration in -X (backward)
        g = self.params.gravity

        # Desired roll from desired Y acceleration
        roll_des = np.arctan2(desired_accel[1], g)

        # Desired pitch from desired X acceleration
        # Note: pitch is negative of X acceleration direction
        pitch_des = np.arctan2(-desired_accel[0], g)

        # Limit tilt angles
        roll_des = np.clip(roll_des, -self.gains.max_tilt_angle, self.gains.max_tilt_angle)
        pitch_des = np.clip(pitch_des, -self.gains.max_tilt_angle, self.gains.max_tilt_angle)

        # Compute required thrust (mass * (g + az_des) / cos(roll) / cos(pitch))
        # This compensates for the tilt of the thrust vector
        cos_roll = np.cos(roll_des)
        cos_pitch = np.cos(pitch_des)
        thrust = self.params.mass * (g + desired_accel_z) / (cos_roll * cos_pitch)

        # Limit thrust
        thrust = np.clip(thrust, self.gains.min_thrust, self.gains.max_thrust)

        # For now, maintain current yaw (yaw rate control handled separately)
        yaw_des = yaw_current

        return roll_des, pitch_des, yaw_des, thrust


class AttitudeController:
    """
    Attitude controller: converts desired attitude to desired angular rates.

    Input: desired attitude (roll, pitch, yaw), current attitude
    Output: desired angular rates (p, q, r) in body frame
    """

    def __init__(self, gains: ControllerGains):
        self.gains = gains

    def compute(
        self,
        attitude_des: np.ndarray,  # [roll, pitch, yaw] desired
        attitude_current: np.ndarray,  # [roll, pitch, yaw] current
    ) -> np.ndarray:  # [p, q, r] desired angular rates
        """
        Compute desired angular rates from attitude error.

        Returns:
            angular_rate_des: [p, q, r] in body frame (rad/s)
        """
        # Attitude error
        attitude_error = attitude_des - attitude_current

        # Wrap yaw error to [-pi, pi]
        attitude_error[2] = self._wrap_angle(attitude_error[2])

        # P controller
        p_des = (self.gains.kp_att_roll / self.params.inertia_xx) * attitude_error[0]
        q_des = (self.gains.kp_att_pitch / self.params.inertia_yy) * attitude_error[1]
        r_des = (self.gains.kp_att_yaw / self.params.inertia_zz) * attitude_error[2]

        return np.array([p_des, q_des, r_des])

    @staticmethod
    def _wrap_angle(angle: float) -> float:
        """Wrap angle to [-pi, pi]."""
        return (angle + np.pi) % (2 * np.pi) - np.pi


class AngularRateController:
    """
    Angular rate controller: converts desired angular rates to torques.

    Input: desired angular rates (p, q, r), current angular rates
    Output: torques (tau_x, tau_y, tau_z)
    """

    def __init__(self, gains: ControllerGains):
        self.gains = gains

    def compute(
        self,
        rate_des: np.ndarray,  # [p, q, r] desired
        rate_current: np.ndarray,  # [p, q, r] current
    ) -> np.ndarray:  # [tau_x, tau_y, tau_z] torques
        """
        Compute torques from angular rate error.

        Returns:
            torques: [tau_roll, tau_pitch, tau_yaw] (N·m)
        """
        # Angular rate error
        rate_error = rate_des - rate_current

        # P controller
        tau_x = self.params.inertia_xx * self.gains.kp_rate_roll * rate_error[0]
        tau_y = self.params.inertia_yy * self.gains.kp_rate_pitch * rate_error[1]
        tau_z = self.params.inertia_zz * self.gains.kp_rate_yaw * rate_error[2]

        return np.array([tau_x, tau_y, tau_z])


class MotorMixer:
    """
    Motor mixer: converts thrust and torques to individual motor speeds.

    Layout matches the X3 model.sdf (8 rotors evenly spaced at 45° on a 0.6 m
    radius ring, nose pointing +X):

           2
        1     3
      0         4
        7     5
           6

    Motor directions (from SDF):
    0(CCW/+1), 1(CW/-1), 2(CCW/+1), 3(CW/-1),
    4(CW/-1), 5(CCW/+1), 6(CW/-1), 7(CCW/+1)
    """

    def __init__(self, params: VehicleParameters):
        self.params = params

        # Motor positions (x, y, z) with +X forward, +Y left, radius = arm_length
        angles = np.linspace(0, 2 * np.pi, params.num_motors, endpoint=False)
        self.motor_positions = np.stack(
            [params.arm_length * np.cos(angles),
             params.arm_length * np.sin(angles),
             np.zeros_like(angles)],
            axis=1,
        )

        # Motor directions: +1 for CCW, -1 for CW
        self.motor_directions = np.array([1, -1, 1, -1, -1, 1, -1, 1])

        # Build mixing matrix: [thrust; tau_x; tau_y; tau_z] -> [motor_speeds^2]
        self._build_mixing_matrix()

    def _build_mixing_matrix(self):
        """
        Build the mixing matrix that maps [thrust, tau_x, tau_y, tau_z] to motor speeds.

        For each motor i:
            F_i = k_f * omega_i^2  (thrust force)
            M_i = k_m * omega_i^2 * dir_i  (reaction torque, dir: +1=CCW, -1=CW)

        Total thrust: T = sum(F_i) = k_f * sum(omega_i^2)
        Roll torque: tau_x = sum(F_i * y_i) = k_f * sum(omega_i^2 * y_i)
        Pitch torque: tau_y = sum(F_i * x_i) = k_f * sum(omega_i^2 * x_i)
        Yaw torque: tau_z = sum(M_i) = k_m * sum(omega_i^2 * dir_i)

        Mixing matrix A such that: [T; tau_x; tau_y; tau_z] = A * [omega_0^2; ...; omega_7^2]
        """
        k_f = self.params.force_constant
        k_m = self.params.moment_constant

        num_motors = self.params.num_motors
        A = np.zeros((4, num_motors))

        for i in range(num_motors):
            x_i = self.motor_positions[i, 0]
            y_i = self.motor_positions[i, 1]
            dir_i = self.motor_directions[i]

            # Thrust row
            A[0, i] = k_f

            # Roll torque row (torque = force * moment_arm_y)
            A[1, i] = k_f * y_i

            # Pitch torque row (torque = force * moment_arm_x)
            # Note: positive pitch is nose up, which requires more thrust at rear
            A[2, i] = k_f * (-x_i)  # Negative because we want rear motors to increase for positive pitch

            # Yaw torque row (reaction torque)
            A[3, i] = k_f * k_m * dir_i

        # Compute pseudo-inverse for control allocation
        # omega_squared = pinv(A) * [T; tau_x; tau_y; tau_z]
        self.mixing_matrix_inv = np.linalg.pinv(A)

    def compute(
        self,
        thrust: float,  # Total thrust (N)
        torques: np.ndarray,  # [tau_x, tau_y, tau_z] (N·m)
    ) -> np.ndarray:  # motor speeds (rad/s) for 8 motors
        """
        Compute individual motor speeds from thrust and torques.

        Args:
            thrust: Total thrust command (N)
            torques: [tau_roll, tau_pitch, tau_yaw] (N·m)

        Returns:
            motor_speeds: Array of 8 motor speeds (rad/s)
        """
        # Command vector
        cmd = np.array([thrust, torques[0], torques[1], torques[2]])

        # Compute motor speeds squared using pseudo-inverse
        motor_speeds_squared = self.mixing_matrix_inv @ cmd

        # Clamp to non-negative and take square root
        motor_speeds_squared = np.maximum(motor_speeds_squared, 0.0)
        motor_speeds = np.sqrt(motor_speeds_squared)

        # Apply motor speed limits
        motor_speeds = np.clip(
            motor_speeds,
            self.params.min_motor_speed,
            self.params.max_motor_speed
        )

        return motor_speeds


class MulticopterControllerNode(Node):
    """
    Main controller node that implements the full cascaded control stack.
    """

    def __init__(self):
        super().__init__('multicopter_controller_node')

        # Declare parameters
        self._declare_parameters()

        # Load parameters
        gains = self._load_gains()
        params = self._load_vehicle_params()

        # Initialize controllers
        self.velocity_controller = VelocityController(gains, params)
        self.attitude_controller = AttitudeController(gains)
        self.rate_controller = AngularRateController(gains)
        self.mixer = MotorMixer(params)

        # State variables
        self.current_velocity_body = np.zeros(3)
        self.current_acceleration_body = np.zeros(3)
        self.current_attitude = np.zeros(3)  # [roll, pitch, yaw]
        self.current_angular_rate = np.zeros(3)  # [p, q, r]
        self.current_yaw_rate_cmd = 0.0  # From cmd_vel.angular.z

        self.cmd_vel = Twist()
        self.controller_enabled = bool(self.get_parameter('start_enabled').value)
        self.last_cmd_vel_time = None

        # Publishers
        self.motor_cmd_pub = self.create_publisher(
            Actuators,
            '/X3/command/motor_speed',
            10
        )

        # Subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/X3/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/X3/odometry',
            self.odometry_callback,
            10
        )

        self.enable_sub = self.create_subscription(
            Bool,
            '/X3/enable',
            self.enable_callback,
            10  # Match ros_gz_bridge default: QoSReliabilityPolicy.RELIABLE + DURABILITY_VOLATILE
        )

        # Control loop timer
        control_freq = self.get_parameter('control_frequency').value
        self.control_timer = self.create_timer(
            1.0 / control_freq,
            self.control_loop
        )

        # File logger
        self.file_logger = _init_file_logger('multicopter_controller')

        self.get_logger().info(f'Multicopter Controller Node initialized at {control_freq} Hz')
        self.get_logger().info(f'Publishing motor commands to /X3/command/motor_speed')
        if self.controller_enabled:
            self.get_logger().info('Controller START_ENABLED -> enabled on startup')
            self.file_logger.info('controller_enabled_startup')

    def _declare_parameters(self):
        """Declare ROS parameters."""
        self.declare_parameter('control_frequency', 160.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)
        self.declare_parameter('start_enabled', True)

        # Velocity controller gains
        self.declare_parameter('kp_vel_xy', 3.0)
        self.declare_parameter('kp_vel_z', 4.0)
        self.declare_parameter('kd_vel_xy', 0.5)
        self.declare_parameter('kd_vel_z', 0.8)

        # Attitude controller gains
        self.declare_parameter('kp_att_roll', 4.0)
        self.declare_parameter('kp_att_pitch', 4.0)
        self.declare_parameter('kp_att_yaw', 5.0)

        # Rate controller gains
        self.declare_parameter('kp_rate_roll', 5.0)
        self.declare_parameter('kp_rate_pitch', 5.0)
        self.declare_parameter('kp_rate_yaw', 6.0)

        # Limits
        self.declare_parameter('max_tilt_angle', 0.5)
        self.declare_parameter('max_thrust', 150.0)

        # Vehicle parameters
        self.declare_parameter('vehicle_mass', 4.6)
        self.declare_parameter('arm_length', 0.6)
        self.declare_parameter('inertia_xx', 0.423)
        self.declare_parameter('inertia_yy', 0.423)
        self.declare_parameter('inertia_zz', 0.828)

    def _load_gains(self) -> ControllerGains:
        """Load controller gains from parameters."""
        gains = ControllerGains()
        gains.kp_vel_xy = self.get_parameter('kp_vel_xy').value
        gains.kp_vel_z = self.get_parameter('kp_vel_z').value
        gains.kd_vel_xy = self.get_parameter('kd_vel_xy').value
        gains.kd_vel_z = self.get_parameter('kd_vel_z').value

        gains.kp_att_roll = self.get_parameter('kp_att_roll').value
        gains.kp_att_pitch = self.get_parameter('kp_att_pitch').value
        gains.kp_att_yaw = self.get_parameter('kp_att_yaw').value

        gains.kp_rate_roll = self.get_parameter('kp_rate_roll').value
        gains.kp_rate_pitch = self.get_parameter('kp_rate_pitch').value
        gains.kp_rate_yaw = self.get_parameter('kp_rate_yaw').value

        gains.max_tilt_angle = self.get_parameter('max_tilt_angle').value
        gains.max_thrust = self.get_parameter('max_thrust').value

        return gains

    def _load_vehicle_params(self) -> VehicleParameters:
        """Load vehicle parameters."""
        params = VehicleParameters()
        params.mass = self.get_parameter('vehicle_mass').value
        params.arm_length = self.get_parameter('arm_length').value
        params.inertia_xx = self.get_parameter('inertia_xx').value
        params.inertia_yy = self.get_parameter('inertia_yy').value
        params.inertia_zz = self.get_parameter('inertia_zz').value
        return params

    def cmd_vel_callback(self, msg: Twist):
        """Receive velocity commands."""
        self.cmd_vel = msg
        self.current_yaw_rate_cmd = msg.angular.z
        self.last_cmd_vel_time = self.get_clock().now()

    def odometry_callback(self, msg: Odometry):
        """Receive odometry feedback."""
        # Extract velocity (body frame)
        self.current_velocity_body = np.array([
            msg.twist.twist.linear.x,
            msg.twist.twist.linear.y,
            msg.twist.twist.linear.z,
        ])

        # Extract angular velocity (body frame)
        self.current_angular_rate = np.array([
            msg.twist.twist.angular.x,
            msg.twist.twist.angular.y,
            msg.twist.twist.angular.z,
        ])

        # Extract attitude from quaternion
        q = msg.pose.pose.orientation
        self.current_attitude = self._quaternion_to_euler(
            q.x, q.y, q.z, q.w
        )

        # Estimate acceleration (simple finite difference - could be improved)
        # For now, set to zero (PD controller will work with P only)
        # TODO: implement proper acceleration estimation

    def enable_callback(self, msg: Bool):
        """Enable/disable controller."""
        self.controller_enabled = msg.data
        if self.controller_enabled:
            self.get_logger().info('Controller ENABLED')
            self.file_logger.info('controller_enabled')
        else:
            self.get_logger().info('Controller DISABLED - sending zero motor commands')
            self.file_logger.info('controller_disabled')
            self._publish_zero_motors()

    def control_loop(self):
        """Main control loop - runs at control_frequency."""
        if not self.controller_enabled:
            return

        # Check cmd_vel timeout
        if self.last_cmd_vel_time is not None:
            age = (self.get_clock().now() - self.last_cmd_vel_time).nanoseconds / 1e9
            timeout = self.get_parameter('cmd_vel_timeout').value
            if age > timeout:
                self.get_logger().warn(f'cmd_vel timeout ({age:.2f}s) - zeroing commands', throttle_duration_sec=2.0)
                self.cmd_vel = Twist()

        # Extract velocity command (body frame)
        vel_cmd = np.array([
            self.cmd_vel.linear.x,
            self.cmd_vel.linear.y,
            self.cmd_vel.linear.z,
        ])

        # Step 1: Velocity Controller
        roll_des, pitch_des, yaw_des, thrust = self.velocity_controller.compute(
            vel_cmd,
            self.current_velocity_body,
            self.current_acceleration_body,
            self.current_attitude[2],  # current yaw
        )

        # Override yaw with yaw rate integration (if yaw rate command is provided)
        # For now, we'll use direct yaw rate control instead of yaw angle control
        # This is more common for velocity-based control

        # Step 2: Attitude Controller
        attitude_des = np.array([roll_des, pitch_des, yaw_des])
        rate_des = self.attitude_controller.compute(
            attitude_des,
            self.current_attitude
        )

        # Override yaw rate with direct command
        rate_des[2] = self.current_yaw_rate_cmd

        # Step 3: Angular Rate Controller
        torques = self.rate_controller.compute(
            rate_des,
            self.current_angular_rate
        )

        # Step 4: Motor Mixer
        motor_speeds = self.mixer.compute(thrust, torques)

        # Publish motor commands
        self._publish_motor_commands(motor_speeds)

    def _publish_motor_commands(self, motor_speeds: np.ndarray):
        """Publish motor speed commands."""
        msg = Actuators()
        msg.velocity = motor_speeds.tolist()
        self.motor_cmd_pub.publish(msg)

    def _publish_zero_motors(self):
        """Publish zero motor commands."""
        zero_speeds = np.zeros(8)
        self._publish_motor_commands(zero_speeds)

    @staticmethod
    def _quaternion_to_euler(x: float, y: float, z: float, w: float) -> np.ndarray:
        """
        Convert quaternion to Euler angles (roll, pitch, yaw).
        Convention: ZYX (yaw-pitch-roll)
        """
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw])


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = MulticopterControllerNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
