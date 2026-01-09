#!/usr/bin/env python3
import csv
import math
import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TwistStamped, Vector3Stamped, Vector3, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float64, Float64MultiArray, Int32
from actuator_msgs.msg import Actuators


def _quat_to_euler(x, y, z, w):
    # ZYX convention (roll, pitch, yaw)
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = 1.0 if t2 > 1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)

    return roll, pitch, yaw


class FlightDebugLogger(Node):
    def __init__(self):
        super().__init__("flight_debug_logger")

        self.declare_parameter("log_frequency", 50.0)
        self.declare_parameter("output_dir", "/home/soja/DroneSim/Test/Debug")
        self.declare_parameter("filename_prefix", "flight_debug")

        self.log_frequency = float(self.get_parameter("log_frequency").value)
        self.output_dir = str(self.get_parameter("output_dir").value)
        self.filename_prefix = str(self.get_parameter("filename_prefix").value)

        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = int(time.time())
        self.csv_path = os.path.join(
            self.output_dir, f"{self.filename_prefix}_{timestamp}.csv"
        )
        self.csv_file = open(self.csv_path, "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)

        self.columns = [
            "time_s",
            "odom_pos_x", "odom_pos_y", "odom_pos_z",
            "odom_vel_x", "odom_vel_y", "odom_vel_z",
            "odom_ang_vel_x", "odom_ang_vel_y", "odom_ang_vel_z",
            "odom_roll", "odom_pitch", "odom_yaw",
            "imu_ang_vel_x", "imu_ang_vel_y", "imu_ang_vel_z",
            "imu_lin_acc_x", "imu_lin_acc_y", "imu_lin_acc_z",
            "imu_roll", "imu_pitch", "imu_yaw",
            "vel_sp_x", "vel_sp_y", "vel_sp_z",
            "wp_cmd_x", "wp_cmd_y", "wp_cmd_z",
            "wp_enable",
            "wp_reached",
            "rate_sp_x", "rate_sp_y", "rate_sp_z",
            "att_cmd_roll", "att_cmd_pitch", "att_cmd_yaw",
            "att_sp_roll", "att_sp_pitch", "att_sp_yaw",
            "accel_sp_x", "accel_sp_y", "accel_sp_z",
            "cmd_att_roll", "cmd_att_pitch", "cmd_att_yaw",
            "cmd_rate_x", "cmd_rate_y", "cmd_rate_z",
            "cmd_thrust",
            "vel_enable", "att_enable", "x3_enable",
            "att_reached",
            "controller_status",
            "mixer_0", "mixer_1", "mixer_2", "mixer_3",
            "motor_0", "motor_1", "motor_2", "motor_3",
            "motor_4", "motor_5", "motor_6", "motor_7",
            "mode",
        ]
        self.csv_writer.writerow(self.columns)
        self.csv_file.flush()

        self.latest = {key: None for key in self.columns}
        self.start_time = self.get_clock().now()

        self.create_subscription(Odometry, "/X3/odometry", self._cb_odom, 10)
        self.create_subscription(Imu, "/imu/data", self._cb_imu, 10)
        self.create_subscription(
            TwistStamped, "/drone/control/velocity_setpoint", self._cb_vel_sp, 10
        )
        self.create_subscription(
            PoseStamped, "/drone/control/waypoint_command", self._cb_wp_cmd, 10
        )
        enable_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Bool, "/drone/control/waypoint_enable", self._cb_wp_enable, enable_qos
        )
        self.create_subscription(
            Bool, "/drone/control/waypoint_reached", self._cb_wp_reached, 10
        )
        self.create_subscription(
            Vector3Stamped,
            "/drone/control/angular_velocity_setpoint",
            self._cb_rate_sp,
            10,
        )
        self.create_subscription(
            Vector3Stamped, "/drone/control/attitude_command", self._cb_att_cmd, 10
        )
        self.create_subscription(
            Vector3Stamped, "/drone/debug/att_setpoint", self._cb_att_sp, 10
        )
        self.create_subscription(
            Vector3Stamped, "/drone/debug/accel_setpoint", self._cb_accel_sp, 10
        )
        self.create_subscription(
            Vector3, "/drone/debug/cmd_att", self._cb_cmd_att, 10
        )
        self.create_subscription(
            Vector3, "/drone/debug/cmd_rate", self._cb_cmd_rate, 10
        )
        self.create_subscription(
            Float64, "/drone/debug/cmd_thrust", self._cb_cmd_thrust, 10
        )
        self.create_subscription(
            Bool, "/drone/control/velocity_enable", self._cb_vel_enable, enable_qos
        )
        self.create_subscription(
            Bool, "/drone/control/attitude_enable", self._cb_att_enable, enable_qos
        )
        self.create_subscription(
            Bool, "/X3/enable", self._cb_x3_enable, enable_qos
        )
        self.create_subscription(
            Bool, "/drone/control/attitude_reached", self._cb_att_reached, 10
        )
        self.create_subscription(
            Float64MultiArray, "/drone/controller/status", self._cb_ctrl_status, 10
        )
        self.create_subscription(
            Float64MultiArray, "/drone/debug/mixer_input", self._cb_mixer, 10
        )
        self.create_subscription(
            Actuators, "/X3/command/motor_speed", self._cb_motor_speed, 10
        )
        self.create_subscription(Int32, "/drone/debug/mode", self._cb_mode, 10)

        period = 1.0 / max(self.log_frequency, 1.0)
        self.timer = self.create_timer(period, self._flush_row)

        self.get_logger().info(f"Logging to {self.csv_path}")

    def _set(self, key, value):
        if key in self.latest:
            self.latest[key] = value

    def _cb_odom(self, msg: Odometry):
        self._set("odom_pos_x", msg.pose.pose.position.x)
        self._set("odom_pos_y", msg.pose.pose.position.y)
        self._set("odom_pos_z", msg.pose.pose.position.z)
        self._set("odom_vel_x", msg.twist.twist.linear.x)
        self._set("odom_vel_y", msg.twist.twist.linear.y)
        self._set("odom_vel_z", msg.twist.twist.linear.z)
        self._set("odom_ang_vel_x", msg.twist.twist.angular.x)
        self._set("odom_ang_vel_y", msg.twist.twist.angular.y)
        self._set("odom_ang_vel_z", msg.twist.twist.angular.z)
        q = msg.pose.pose.orientation
        roll, pitch, yaw = _quat_to_euler(q.x, q.y, q.z, q.w)
        self._set("odom_roll", roll)
        self._set("odom_pitch", pitch)
        self._set("odom_yaw", yaw)

    def _cb_imu(self, msg: Imu):
        self._set("imu_ang_vel_x", msg.angular_velocity.x)
        self._set("imu_ang_vel_y", msg.angular_velocity.y)
        self._set("imu_ang_vel_z", msg.angular_velocity.z)
        self._set("imu_lin_acc_x", msg.linear_acceleration.x)
        self._set("imu_lin_acc_y", msg.linear_acceleration.y)
        self._set("imu_lin_acc_z", msg.linear_acceleration.z)
        q = msg.orientation
        roll, pitch, yaw = _quat_to_euler(q.x, q.y, q.z, q.w)
        self._set("imu_roll", roll)
        self._set("imu_pitch", pitch)
        self._set("imu_yaw", yaw)

    def _cb_vel_sp(self, msg: TwistStamped):
        self._set("vel_sp_x", msg.twist.linear.x)
        self._set("vel_sp_y", msg.twist.linear.y)
        self._set("vel_sp_z", msg.twist.linear.z)

    def _cb_wp_cmd(self, msg: PoseStamped):
        self._set("wp_cmd_x", msg.pose.position.x)
        self._set("wp_cmd_y", msg.pose.position.y)
        self._set("wp_cmd_z", msg.pose.position.z)

    def _cb_wp_enable(self, msg: Bool):
        self._set("wp_enable", 1.0 if msg.data else 0.0)

    def _cb_wp_reached(self, msg: Bool):
        self._set("wp_reached", 1.0 if msg.data else 0.0)

    def _cb_rate_sp(self, msg: Vector3Stamped):
        self._set("rate_sp_x", msg.vector.x)
        self._set("rate_sp_y", msg.vector.y)
        self._set("rate_sp_z", msg.vector.z)

    def _cb_att_cmd(self, msg: Vector3Stamped):
        self._set("att_cmd_roll", msg.vector.x)
        self._set("att_cmd_pitch", msg.vector.y)
        self._set("att_cmd_yaw", msg.vector.z)

    def _cb_att_sp(self, msg: Vector3Stamped):
        self._set("att_sp_roll", msg.vector.x)
        self._set("att_sp_pitch", msg.vector.y)
        self._set("att_sp_yaw", msg.vector.z)

    def _cb_accel_sp(self, msg: Vector3Stamped):
        self._set("accel_sp_x", msg.vector.x)
        self._set("accel_sp_y", msg.vector.y)
        self._set("accel_sp_z", msg.vector.z)

    def _cb_cmd_att(self, msg: Vector3):
        self._set("cmd_att_roll", msg.x)
        self._set("cmd_att_pitch", msg.y)
        self._set("cmd_att_yaw", msg.z)

    def _cb_cmd_rate(self, msg: Vector3):
        self._set("cmd_rate_x", msg.x)
        self._set("cmd_rate_y", msg.y)
        self._set("cmd_rate_z", msg.z)

    def _cb_cmd_thrust(self, msg: Float64):
        self._set("cmd_thrust", msg.data)

    def _cb_vel_enable(self, msg: Bool):
        self._set("vel_enable", 1.0 if msg.data else 0.0)

    def _cb_att_enable(self, msg: Bool):
        self._set("att_enable", 1.0 if msg.data else 0.0)

    def _cb_x3_enable(self, msg: Bool):
        self._set("x3_enable", 1.0 if msg.data else 0.0)

    def _cb_att_reached(self, msg: Bool):
        self._set("att_reached", 1.0 if msg.data else 0.0)

    def _cb_ctrl_status(self, msg: Float64MultiArray):
        if msg.data:
            self._set("controller_status", ",".join(f"{v:.6f}" for v in msg.data))
        else:
            self._set("controller_status", "")

    def _cb_mixer(self, msg: Float64MultiArray):
        data = list(msg.data)
        for i in range(4):
            self._set(f"mixer_{i}", data[i] if i < len(data) else None)

    def _cb_motor_speed(self, msg: Actuators):
        data = list(msg.velocity) if hasattr(msg, "velocity") else []
        for i in range(8):
            self._set(f"motor_{i}", data[i] if i < len(data) else None)

    def _cb_mode(self, msg: Int32):
        self._set("mode", msg.data)

    def _flush_row(self):
        now = self.get_clock().now()
        time_s = (now - self.start_time).nanoseconds / 1e9
        row = []
        for key in self.columns:
            if key == "time_s":
                row.append(f"{time_s:.6f}")
                continue
            value = self.latest.get(key)
            if value is None:
                row.append("")
            else:
                row.append(f"{value:.6f}" if isinstance(value, float) else str(value))
        self.csv_writer.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        try:
            self.csv_file.close()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = FlightDebugLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
