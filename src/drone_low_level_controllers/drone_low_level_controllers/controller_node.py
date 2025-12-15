#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Cascaded Multicopter Controller Node - World Frame Velocity Control (robust + matched params)

Pipeline (with decimation):
- Outer (WORLD): v_sp_world -> a_cmd_world (PID + D low-pass + accel limits) -> F_world (gravity compensated)
- F_world -> (tilt-limited b3, roll/pitch) + thrust via projection + thrust limits + optional slew
- Attitude P: (roll,pitch,yaw) -> body rate setpoint pqr_sp
- Rate PID (inertia-scaled): pqr_sp -> torques tau, with torque limits
- Mixer: [thrust, tau] -> motor omega

Frames:
- World: ENU (x East, y North, z Up)
- Body:  FLU (x Forward, y Left, z Up)

Important:
- odom_twist_in_world decides whether Odometry.twist.twist.linear is world or body.
"""

import logging
from logging.handlers import RotatingFileHandler
from dataclasses import dataclass
from typing import Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy

from geometry_msgs.msg import TwistStamped, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from actuator_msgs.msg import Actuators

LOG_PATH = "/tmp/drone_low_level_controllers.log"


def _init_file_logger(name: str) -> logging.Logger:
    logger = logging.getLogger(name)
    if not logger.handlers:
        handler = RotatingFileHandler(LOG_PATH, maxBytes=5 * 1024 * 1024, backupCount=2)
        handler.setFormatter(logging.Formatter("%(asctime)s [%(name)s] %(message)s"))
        logger.addHandler(handler)
        logger.setLevel(logging.INFO)
        logger.propagate = False
    return logger


@dataclass
class ControllerGains:
    # Outer loop: world velocity PID -> accel_cmd (m/s^2)
    kp_vel_xy: float = 3.0
    ki_vel_xy: float = 0.0
    kd_vel_xy: float = 0.4
    kp_vel_z: float = 4.0
    ki_vel_z: float = 0.0
    kd_vel_z: float = 0.6

    vel_integral_limit: float = 3.0            # (m/s) integrator state clamp
    vel_d_filter_tau: float = 0.05             # sec, derivative low-pass

    # Attitude P (euler error -> pqr_sp)
    kp_att_roll: float = 5.0
    kp_att_pitch: float = 5.0
    kp_att_yaw: float = 3.0

    # Rate PID (pqr error -> alpha_cmd), torque = I * alpha_cmd
    kp_rate_roll: float = 0.20
    ki_rate_roll: float = 0.02
    kd_rate_roll: float = 0.00
    kp_rate_pitch: float = 0.20
    ki_rate_pitch: float = 0.02
    kd_rate_pitch: float = 0.00
    kp_rate_yaw: float = 0.15
    ki_rate_yaw: float = 0.02
    kd_rate_yaw: float = 0.00

    rate_integral_limit: float = 3.0           # (rad/s) integrator state clamp

    # Limits
    max_tilt_angle: float = 0.5                # rad
    max_thrust: float = 150.0                  # N (total)
    min_thrust: float = 0.0                    # N

    max_acc_xy: float = 3.0                    # m/s^2
    max_acc_z_up: float = 5.0                  # m/s^2
    max_acc_z_down: float = 3.0                # m/s^2 (positive magnitude; applied to negative accel)

    max_torque_xy: float = 3.0                 # N*m
    max_torque_z: float = 2.0                  # N*m

    max_rate_xy: float = 4.0                   # rad/s (|p|,|q|)
    max_rate_yaw: float = 3.0                  # rad/s (|r|)

    thrust_slew_rate: float = 0.0              # N/s, 0 disables


@dataclass
class VehicleParameters:
    mass: float = 4.6
    gravity: float = 9.81
    inertia_xx: float = 0.423
    inertia_yy: float = 0.423
    inertia_zz: float = 0.828

    # thrust_i = k_f * omega_i^2
    force_constant: float = 2.85e-05

    # moment_constant treated as ratio (k_m / k_f). yaw torque: tau_z = (k_f * moment_constant) * dir * omega^2
    moment_constant: float = 0.0533

    max_motor_speed: float = 800.0
    min_motor_speed: float = 0.0
    num_motors: int = 8


class WorldVelocityToForce:
    """World ENU velocity PID -> desired world force F_w (includes gravity compensation)."""

    def __init__(self, gains: ControllerGains, params: VehicleParameters):
        self.g = gains
        self.p = params
        self.int_err = np.zeros(3)
        self.prev_err = np.zeros(3)
        self.prev_d = np.zeros(3)

    def reset(self):
        self.int_err[:] = 0.0
        self.prev_err[:] = 0.0
        self.prev_d[:] = 0.0

    def compute(self, v_sp_w: np.ndarray, v_w: np.ndarray, dt: float) -> np.ndarray:
        dt = max(1e-4, float(dt))
        err = v_sp_w - v_w

        kp = np.array([self.g.kp_vel_xy, self.g.kp_vel_xy, self.g.kp_vel_z], dtype=float)
        ki = np.array([self.g.ki_vel_xy, self.g.ki_vel_xy, self.g.ki_vel_z], dtype=float)
        kd = np.array([self.g.kd_vel_xy, self.g.kd_vel_xy, self.g.kd_vel_z], dtype=float)

        # integral
        self.int_err += err * dt
        self.int_err = np.clip(self.int_err, -self.g.vel_integral_limit, self.g.vel_integral_limit)

        # derivative with low-pass
        d_raw = (err - self.prev_err) / dt
        self.prev_err = err.copy()

        alpha = float(np.exp(-dt / max(1e-4, self.g.vel_d_filter_tau)))
        d_filt = alpha * self.prev_d + (1.0 - alpha) * d_raw
        self.prev_d = d_filt.copy()

        # accel command
        a_cmd = kp * err + ki * self.int_err + kd * d_filt

        # accel saturation
        a_xy = a_cmd[:2]
        axy_norm = float(np.linalg.norm(a_xy))
        if axy_norm > self.g.max_acc_xy and axy_norm > 1e-6:
            a_cmd[:2] = a_xy / axy_norm * self.g.max_acc_xy

        # z saturation (world z is UP)
        a_cmd[2] = float(np.clip(a_cmd[2], -self.g.max_acc_z_down, self.g.max_acc_z_up))

        # desired world force (gravity compensation in ENU)
        F_w = self.p.mass * a_cmd + np.array([0.0, 0.0, self.p.mass * self.p.gravity], dtype=float)

        # physical: avoid negative total vertical force demand
        F_w[2] = max(0.0, float(F_w[2]))
        return F_w


class ForceToAttitudeAndThrust:
    """
    Map desired world force F_w to:
    - desired roll/pitch (keeping yaw at current yaw)
    - thrust command (after tilt limiting), via projection: thrust = dot(F_w, b3_limited)
    """

    def __init__(self, gains: ControllerGains):
        self.g = gains

    def compute(self, F_w: np.ndarray, yaw_current: float) -> Tuple[float, float, float, float]:
        F_norm = float(np.linalg.norm(F_w))
        if F_norm < 1e-3:
            return 0.0, 0.0, float(yaw_current), 0.0

        # desired thrust direction b3 (world)
        b3 = F_w / F_norm
        if b3[2] < 1e-6:
            b3 = np.array([0.0, 0.0, 1.0], dtype=float)

        # tilt limit on b3
        max_tilt = float(self.g.max_tilt_angle)
        cos_max = float(np.cos(max_tilt))
        if b3[2] < cos_max:
            h = b3[:2]
            h_norm = float(np.linalg.norm(h))
            if h_norm < 1e-8:
                b3 = np.array([0.0, 0.0, 1.0], dtype=float)
            else:
                b3_xy = h / h_norm * float(np.sin(max_tilt))
                b3 = np.array([b3_xy[0], b3_xy[1], cos_max], dtype=float)

        # keep yaw
        b1_yaw = np.array([np.cos(yaw_current), np.sin(yaw_current), 0.0], dtype=float)
        b2 = np.cross(b3, b1_yaw)
        b2_norm = float(np.linalg.norm(b2))
        if b2_norm < 1e-6:
            b1_perp = np.array([-np.sin(yaw_current), np.cos(yaw_current), 0.0], dtype=float)
            b2 = np.cross(b3, b1_perp)
            b2_norm = max(1e-6, float(np.linalg.norm(b2)))
        b2 /= b2_norm
        b1 = np.cross(b2, b3)

        R_w_b = np.column_stack((b1, b2, b3))  # world_from_body
        roll, pitch, _ = self._rotation_matrix_to_euler_zyx(R_w_b)

        thrust_cmd = float(np.dot(F_w, b3))
        thrust_cmd = float(np.clip(thrust_cmd, self.g.min_thrust, self.g.max_thrust))

        return float(roll), float(pitch), float(yaw_current), thrust_cmd

    @staticmethod
    def _rotation_matrix_to_euler_zyx(R: np.ndarray) -> Tuple[float, float, float]:
        pitch = float(np.arcsin(np.clip(-R[2, 0], -1.0, 1.0)))
        roll = float(np.arctan2(R[2, 1], R[2, 2]))
        yaw = float(np.arctan2(R[1, 0], R[0, 0]))
        return roll, pitch, yaw


class AttitudeToRates:
    """Euler P controller -> desired body rates pqr (rad/s)."""

    def __init__(self, gains: ControllerGains):
        self.g = gains

    @staticmethod
    def _wrap(a: float) -> float:
        return (a + np.pi) % (2 * np.pi) - np.pi

    def compute(self, att_sp: np.ndarray, att: np.ndarray) -> np.ndarray:
        err = att_sp - att
        err[2] = self._wrap(float(err[2]))
        return np.array([
            self.g.kp_att_roll * err[0],
            self.g.kp_att_pitch * err[1],
            self.g.kp_att_yaw * err[2],
        ], dtype=float)


class RatesToTorques:
    """PID on body rates -> body torques, inertia-scaled + saturation."""

    def __init__(self, gains: ControllerGains, params: VehicleParameters):
        self.g = gains
        self.I = np.array([params.inertia_xx, params.inertia_yy, params.inertia_zz], dtype=float)
        self.int_err = np.zeros(3)
        self.prev_err = np.zeros(3)

    def reset(self):
        self.int_err[:] = 0.0
        self.prev_err[:] = 0.0

    def compute(self, pqr_sp: np.ndarray, pqr: np.ndarray, dt: float) -> np.ndarray:
        dt = max(1e-4, float(dt))
        err = pqr_sp - pqr

        self.int_err += err * dt
        self.int_err = np.clip(self.int_err, -self.g.rate_integral_limit, self.g.rate_integral_limit)

        derr = (err - self.prev_err) / dt
        self.prev_err = err.copy()

        kp = np.array([self.g.kp_rate_roll, self.g.kp_rate_pitch, self.g.kp_rate_yaw], dtype=float)
        ki = np.array([self.g.ki_rate_roll, self.g.ki_rate_pitch, self.g.ki_rate_yaw], dtype=float)
        kd = np.array([self.g.kd_rate_roll, self.g.kd_rate_pitch, self.g.kd_rate_yaw], dtype=float)

        alpha_cmd = kp * err + ki * self.int_err + kd * derr
        tau = self.I * alpha_cmd

        tau[0] = float(np.clip(tau[0], -self.g.max_torque_xy, self.g.max_torque_xy))
        tau[1] = float(np.clip(tau[1], -self.g.max_torque_xy, self.g.max_torque_xy))
        tau[2] = float(np.clip(tau[2], -self.g.max_torque_z, self.g.max_torque_z))
        return tau


class MotorMixer:
    """Octo mixer (FLU): [thrust, tau] -> omega (rad/s)."""

    def __init__(self, params: VehicleParameters):
        self.p = params

        self.motor_positions = np.array([
            [ 0.6,     0.0,    0.0],
            [ 0.4243,  0.4243, 0.0],
            [ 0.0,     0.6,    0.0],
            [-0.4243,  0.4243, 0.0],
            [-0.6,     0.0,    0.0],
            [-0.4243, -0.4243, 0.0],
            [ 0.0,    -0.6,    0.0],
            [ 0.4243, -0.4243, 0.0],
        ], dtype=float)

        # Make sure this matches your motor plugin directions (only affects yaw)!
        self.motor_directions = np.array([+1, -1, +1, -1, +1, -1, +1, -1], dtype=float)

        self._build()

    def _build(self):
        kf = float(self.p.force_constant)
        km = kf * float(self.p.moment_constant)

        n = int(self.p.num_motors)
        A = np.zeros((4, n), dtype=float)
        for i in range(n):
            x, y, _ = self.motor_positions[i]
            d = self.motor_directions[i]
            A[0, i] = kf
            A[1, i] = kf * y
            A[2, i] = kf * (-x)
            A[3, i] = km * d

        self.A_pinv = np.linalg.pinv(A)

    def compute(self, thrust: float, tau: np.ndarray) -> np.ndarray:
        cmd = np.array([thrust, tau[0], tau[1], tau[2]], dtype=float)
        omega2 = self.A_pinv @ cmd
        omega2 = np.maximum(omega2, 0.0)
        omega = np.sqrt(omega2)
        return np.clip(omega, self.p.min_motor_speed, self.p.max_motor_speed)


class MulticopterControllerNode(Node):
    def __init__(self):
        super().__init__("multicopter_controller_node")

        self._declare_parameters()
        self.g = self._load_gains()
        self.p = self._load_params()

        self.vel2F = WorldVelocityToForce(self.g, self.p)
        self.F2att = ForceToAttitudeAndThrust(self.g)
        self.att2rate = AttitudeToRates(self.g)
        self.rate2tau = RatesToTorques(self.g, self.p)
        self.mixer = MotorMixer(self.p)

        self.file_logger = _init_file_logger("multicopter_controller")

        # State
        self.pos_w = np.zeros(3)
        self.v_body = np.zeros(3)
        self.v_w = np.zeros(3)
        self.pqr = np.zeros(3)
        self.att = np.zeros(3)  # roll, pitch, yaw
        self.R_w_b = np.eye(3)

        # Setpoints
        self.v_sp_w = np.zeros(3)
        self.yaw_rate_sp = 0.0

        # Enable/timing
        self.enabled = bool(self.get_parameter("start_enabled").value)
        self.last_control_time = None
        self.last_vsp_time = None
        self.last_wsp_time = None

        # Decimation
        self.counter = 0
        self.main_hz = float(self.get_parameter("control_frequency").value)
        self.vel_hz = float(self.get_parameter("vel_control_frequency").value)
        self.att_hz = float(self.get_parameter("att_control_frequency").value)
        self.vel_decim = max(1, int(round(self.main_hz / max(1e-3, self.vel_hz))))
        self.att_decim = max(1, int(round(self.main_hz / max(1e-3, self.att_hz))))

        # Cached commands
        self.cached_roll_sp = 0.0
        self.cached_pitch_sp = 0.0
        self.cached_yaw_sp = 0.0
        self.cached_thrust = 0.0

        # Thrust slew
        self.thrust_prev = 0.0

        # Param: odom twist frame
        self.odom_twist_in_world = bool(self.get_parameter("odom_twist_in_world").value)

        # QoS for enable
        enable_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # pub/sub
        self.motor_pub = self.create_publisher(Actuators, "/X3/command/motor_speed", 10)
        self.create_subscription(TwistStamped, "/drone/control/velocity_setpoint", self._cb_vsp, 10)
        self.create_subscription(Vector3Stamped, "/drone/control/angular_velocity_setpoint", self._cb_wsp, 10)
        self.create_subscription(Bool, "/drone/control/velocity_enable", self._cb_enable, enable_qos)
        self.create_subscription(Odometry, "/X3/odometry", self._cb_odom, 10)

        self.timer = self.create_timer(1.0 / self.main_hz, self._loop)

        self.get_logger().info(
            f"[OK] Controller {self.main_hz:.1f}Hz (vel {self.vel_hz:.1f}Hz, att {self.att_hz:.1f}Hz), "
            f"odom_twist_in_world={self.odom_twist_in_world}"
        )

    def _declare_parameters(self):
        # Frequencies + timeouts + enable
        self.declare_parameter("control_frequency", 500.0)
        self.declare_parameter("vel_control_frequency", 100.0)
        self.declare_parameter("att_control_frequency", 250.0)
        self.declare_parameter("velocity_setpoint_timeout", 0.8)
        self.declare_parameter("angular_setpoint_timeout", 0.8)
        self.declare_parameter("start_enabled", True)

        # Odom twist frame selector
        self.declare_parameter("odom_twist_in_world", True)

        # Velocity PID
        self.declare_parameter("kp_vel_xy", 3.0)
        self.declare_parameter("ki_vel_xy", 0.0)
        self.declare_parameter("kd_vel_xy", 0.4)
        self.declare_parameter("kp_vel_z", 4.0)
        self.declare_parameter("ki_vel_z", 0.0)
        self.declare_parameter("kd_vel_z", 0.6)
        self.declare_parameter("vel_integral_limit", 3.0)
        self.declare_parameter("vel_d_filter_tau", 0.05)

        # Attitude P
        self.declare_parameter("kp_att_roll", 5.0)
        self.declare_parameter("kp_att_pitch", 5.0)
        self.declare_parameter("kp_att_yaw", 3.0)

        # Rate PID
        self.declare_parameter("kp_rate_roll", 0.20)
        self.declare_parameter("ki_rate_roll", 0.02)
        self.declare_parameter("kd_rate_roll", 0.00)
        self.declare_parameter("kp_rate_pitch", 0.20)
        self.declare_parameter("ki_rate_pitch", 0.02)
        self.declare_parameter("kd_rate_pitch", 0.00)
        self.declare_parameter("kp_rate_yaw", 0.15)
        self.declare_parameter("ki_rate_yaw", 0.02)
        self.declare_parameter("kd_rate_yaw", 0.00)
        self.declare_parameter("rate_integral_limit", 3.0)

        # Limits
        self.declare_parameter("max_tilt_angle", 0.5)
        self.declare_parameter("max_thrust", 150.0)
        self.declare_parameter("min_thrust", 0.0)

        self.declare_parameter("max_acc_xy", 3.0)
        self.declare_parameter("max_acc_z_up", 5.0)
        self.declare_parameter("max_acc_z_down", 3.0)

        self.declare_parameter("max_torque_xy", 3.0)
        self.declare_parameter("max_torque_z", 2.0)

        self.declare_parameter("max_rate_xy", 4.0)
        self.declare_parameter("max_rate_yaw", 3.0)

        self.declare_parameter("thrust_slew_rate", 0.0)

        # Vehicle params
        self.declare_parameter("vehicle_mass", 4.6)
        self.declare_parameter("gravity", 9.81)
        self.declare_parameter("inertia_xx", 0.423)
        self.declare_parameter("inertia_yy", 0.423)
        self.declare_parameter("inertia_zz", 0.828)
        self.declare_parameter("force_constant", 2.85e-05)
        self.declare_parameter("moment_constant", 0.0533)
        self.declare_parameter("max_motor_speed", 800.0)
        self.declare_parameter("min_motor_speed", 0.0)
        self.declare_parameter("num_motors", 8)

    def _load_gains(self) -> ControllerGains:
        g = ControllerGains()
        for k in g.__dataclass_fields__.keys():
            if self.has_parameter(k):
                setattr(g, k, float(self.get_parameter(k).value))
        return g

    def _load_params(self) -> VehicleParameters:
        p = VehicleParameters()
        p.mass = float(self.get_parameter("vehicle_mass").value)
        p.gravity = float(self.get_parameter("gravity").value)
        p.inertia_xx = float(self.get_parameter("inertia_xx").value)
        p.inertia_yy = float(self.get_parameter("inertia_yy").value)
        p.inertia_zz = float(self.get_parameter("inertia_zz").value)
        p.force_constant = float(self.get_parameter("force_constant").value)
        p.moment_constant = float(self.get_parameter("moment_constant").value)
        p.max_motor_speed = float(self.get_parameter("max_motor_speed").value)
        p.min_motor_speed = float(self.get_parameter("min_motor_speed").value)
        p.num_motors = int(self.get_parameter("num_motors").value)
        return p

    def _cb_vsp(self, msg: TwistStamped):
        self.v_sp_w = np.array([msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z], dtype=float)
        self.last_vsp_time = self.get_clock().now()

    def _cb_wsp(self, msg: Vector3Stamped):
        self.yaw_rate_sp = float(msg.vector.z)
        self.last_wsp_time = self.get_clock().now()

    def _cb_enable(self, msg: Bool):
        self.enabled = bool(msg.data)
        if not self.enabled:
            self.vel2F.reset()
            self.rate2tau.reset()
            self._publish_motors(np.zeros(self.p.num_motors, dtype=float))
            self.get_logger().warn("Controller disabled -> motors zeroed")
        else:
            self.get_logger().info("Controller enabled")

    def _cb_odom(self, msg: Odometry):
        self.pos_w = np.array([msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z], dtype=float)

        q = msg.pose.pose.orientation
        self.att = self._quat_to_euler(q.x, q.y, q.z, q.w)
        self.R_w_b = self._quat_to_R_w_b(q.x, q.y, q.z, q.w)

        v_lin = np.array([msg.twist.twist.linear.x, msg.twist.twist.linear.y, msg.twist.twist.linear.z], dtype=float)
        if self.odom_twist_in_world:
            self.v_w = v_lin
            self.v_body = self.R_w_b.T @ self.v_w
        else:
            self.v_body = v_lin
            self.v_w = self.R_w_b @ self.v_body

        self.pqr = np.array([msg.twist.twist.angular.x, msg.twist.twist.angular.y, msg.twist.twist.angular.z], dtype=float)

    def _loop(self):
        if not self.enabled:
            return

        now = self.get_clock().now()
        if self.last_control_time is None:
            dt = 1.0 / self.main_hz
        else:
            dt = (now - self.last_control_time).nanoseconds / 1e9
        self.last_control_time = now
        dt = max(1e-4, float(dt))

        # timeouts
        v_to = float(self.get_parameter("velocity_setpoint_timeout").value)
        w_to = float(self.get_parameter("angular_setpoint_timeout").value)

        if self.last_vsp_time is None or (now - self.last_vsp_time).nanoseconds / 1e9 > v_to:
            self.v_sp_w[:] = 0.0
        if self.last_wsp_time is None or (now - self.last_wsp_time).nanoseconds / 1e9 > w_to:
            self.yaw_rate_sp = 0.0

        self.counter += 1

        # Outer loop (decimated)
        if self.counter % self.vel_decim == 0:
            dt_vel = dt * self.vel_decim
            F_w = self.vel2F.compute(self.v_sp_w, self.v_w, dt_vel)
            roll_sp, pitch_sp, yaw_sp, thrust = self.F2att.compute(F_w, float(self.att[2]))

            # thrust slew limit
            slew = float(self.g.thrust_slew_rate)
            if slew > 0.0:
                delta = thrust - self.thrust_prev
                max_delta = slew * dt_vel
                delta = float(np.clip(delta, -max_delta, max_delta))
                thrust = self.thrust_prev + delta
            self.thrust_prev = thrust

            self.cached_roll_sp = roll_sp
            self.cached_pitch_sp = pitch_sp
            self.cached_yaw_sp = yaw_sp
            self.cached_thrust = thrust

        # Attitude -> rate setpoint (can be decimated too, but P so cheap)
        att_sp = np.array([self.cached_roll_sp, self.cached_pitch_sp, self.cached_yaw_sp], dtype=float)
        pqr_sp = self.att2rate.compute(att_sp, self.att)
        pqr_sp[2] = float(self.yaw_rate_sp)

        # rate setpoint saturation
        pqr_sp[0] = float(np.clip(pqr_sp[0], -self.g.max_rate_xy, self.g.max_rate_xy))
        pqr_sp[1] = float(np.clip(pqr_sp[1], -self.g.max_rate_xy, self.g.max_rate_xy))
        pqr_sp[2] = float(np.clip(pqr_sp[2], -self.g.max_rate_yaw, self.g.max_rate_yaw))

        # Rate -> torques
        tau = self.rate2tau.compute(pqr_sp, self.pqr, dt)

        # Mix
        omega = self.mixer.compute(self.cached_thrust, tau)
        self._publish_motors(omega)

    def _publish_motors(self, omega: np.ndarray):
        msg = Actuators()
        msg.velocity = omega.tolist()
        self.motor_pub.publish(msg)

    @staticmethod
    def _quat_to_euler(x: float, y: float, z: float, w: float) -> np.ndarray:
        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return np.array([roll, pitch, yaw], dtype=float)

    @staticmethod
    def _quat_to_R_w_b(x: float, y: float, z: float, w: float) -> np.ndarray:
        n = x * x + y * y + z * z + w * w
        if n < 1e-12:
            return np.eye(3)
        s = 2.0 / n

        xx, yy, zz = x * x * s, y * y * s, z * z * s
        xy, xz, yz = x * y * s, x * z * s, y * z * s
        wx, wy, wz = w * x * s, w * y * s, w * z * s

        return np.array([
            [1.0 - (yy + zz), xy - wz, xz + wy],
            [xy + wz, 1.0 - (xx + zz), yz - wx],
            [xz - wy, yz + wx, 1.0 - (xx + yy)],
        ], dtype=float)


def main(args=None):
    rclpy.init(args=args)
    node = MulticopterControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
