#!/usr/bin/env python3

import math
import threading
from typing import List, Tuple

import matplotlib.pyplot as plt
import numpy as np
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Bool


class CmdVelStepTester(Node):
    """Send step commands to /X3/cmd_vel and plot response from /X3/odometry."""

    def __init__(self) -> None:
        super().__init__("cmd_vel_step_tester")

        self.cmd_pub = self.create_publisher(Twist, "/X3/cmd_vel", 10)
        self.enable_pub = self.create_publisher(Bool, "/X3/enable", 10)
        self.odom_sub = self.create_subscription(
            Odometry, "/X3/odometry", self.odom_callback, 10
        )

        # Test sequence: (label, duration_sec, vx, vy, vz, yaw_rate)
        self.sequence: List[Tuple[str, float, float, float, float, float]] = [
            ("settle", 3.0, 0.0, 0.0, 0.0, 0.0),
            ("vx_step", 8.0, 2.0, 0.0, 0.0, 0.0),
            ("vx_zero", 5.0, 0.0, 0.0, 0.0, 0.0),
            ("vy_step", 8.0, 0.0, 2.0, 0.0, 0.0),
            ("vy_zero", 5.0, 0.0, 0.0, 0.0, 0.0),
            ("vz_step", 8.0, 0.0, 0.0, 1.5, 0.0),
            ("vz_zero", 5.0, 0.0, 0.0, 0.0, 0.0),
            ("yaw_step", 8.0, 0.0, 0.0, 0.0, 1.5),
            ("yaw_zero", 5.0, 0.0, 0.0, 0.0, 0.0),
        ]

        self.phase_index = 0
        self.phase_start_time: float | None = None
        self.enabled_sent = False

        # Logging buffers
        self.t: List[float] = []
        self.cmd_vx: List[float] = []
        self.cmd_vy: List[float] = []
        self.cmd_vz: List[float] = []
        self.cmd_yaw: List[float] = []
        self.meas_vx: List[float] = []
        self.meas_vy: List[float] = []
        self.meas_vz: List[float] = []
        self.meas_yaw: List[float] = []

        self._start_time: float | None = None
        self._last_odom_time: float | None = None

        # Control loop timer (50 Hz)
        self.timer = self.create_timer(0.02, self.control_loop)

        self.get_logger().info("CmdVel step tester initialised")

    def now_sec(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def odom_callback(self, msg: Odometry) -> None:
        t_now = self.now_sec()
        if self._start_time is None:
            return

        self._last_odom_time = t_now
        rel_t = t_now - self._start_time

        self.t.append(rel_t)
        self.meas_vx.append(msg.twist.twist.linear.x)
        self.meas_vy.append(msg.twist.twist.linear.y)
        self.meas_vz.append(msg.twist.twist.linear.z)
        self.meas_yaw.append(msg.twist.twist.angular.z)

    def current_command(self) -> Tuple[float, float, float, float]:
        """Compute current command based on elapsed time in the test sequence."""
        t_now = self.now_sec()
        if self._start_time is None:
            self._start_time = t_now
            self.phase_start_time = t_now

        if self.phase_index >= len(self.sequence):
            return 0.0, 0.0, 0.0, 0.0

        if self.phase_start_time is None:
            self.phase_start_time = t_now

        label, duration, vx, vy, vz, yaw = self.sequence[self.phase_index]
        phase_elapsed = t_now - self.phase_start_time

        if phase_elapsed >= duration:
            self.phase_index += 1
            if self.phase_index < len(self.sequence):
                next_label = self.sequence[self.phase_index][0]
                self.get_logger().info(f"Advancing to phase {self.phase_index}: {next_label}")
                self.phase_start_time = t_now
            return self.current_command()

        return vx, vy, vz, yaw

    def control_loop(self) -> None:
        # Send single enable pulse at start
        if not self.enabled_sent:
            msg = Bool()
            msg.data = True
            self.enable_pub.publish(msg)
            self.enabled_sent = True
            self.get_logger().info("Sent enable command on /X3/enable")

        vx, vy, vz, yaw = self.current_command()

        # Log commanded values aligned to "now"
        if self._start_time is not None:
            t_now = self.now_sec() - self._start_time
            self.cmd_vx.append(vx)
            self.cmd_vy.append(vy)
            self.cmd_vz.append(vz)
            self.cmd_yaw.append(yaw)
            # Keep t length consistent; only append if odom exists or commands started
            if len(self.t) < len(self.cmd_vx):
                self.t.append(t_now)

        cmd_msg = Twist()
        cmd_msg.linear.x = float(vx)
        cmd_msg.linear.y = float(vy)
        cmd_msg.linear.z = float(vz)
        cmd_msg.angular.z = float(yaw)
        self.cmd_pub.publish(cmd_msg)

    # Plotting helpers -------------------------------------------------
    def get_plot_data(self):
        t = np.array(self.t, dtype=float)
        if t.size == 0:
            return None

        def align(lst):
            arr = np.array(lst, dtype=float)
            if arr.size < t.size:
                pad = np.full(t.size - arr.size, np.nan)
                arr = np.concatenate([arr, pad])
            elif arr.size > t.size:
                arr = arr[: t.size]
            return arr

        return {
            "t": t,
            "cmd_vx": align(self.cmd_vx),
            "cmd_vy": align(self.cmd_vy),
            "cmd_vz": align(self.cmd_vz),
            "cmd_yaw": align(self.cmd_yaw),
            "meas_vx": align(self.meas_vx),
            "meas_vy": align(self.meas_vy),
            "meas_vz": align(self.meas_vz),
            "meas_yaw": align(self.meas_yaw),
        }


def main() -> None:
    rclpy.init()
    node = CmdVelStepTester()

    plt.ion()
    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(10, 8))
    (ax_vx, ax_vy, ax_vz, ax_yaw) = axes
    ax_vx.set_ylabel("vx [m/s]")
    ax_vy.set_ylabel("vy [m/s]")
    ax_vz.set_ylabel("vz [m/s]")
    ax_yaw.set_ylabel("yaw rate [rad/s]")
    ax_yaw.set_xlabel("time [s]")

    # Line handles
    lines = {
        "cmd_vx": ax_vx.plot([], [], "r--", label="cmd")[0],
        "meas_vx": ax_vx.plot([], [], "b-", label="meas")[0],
        "cmd_vy": ax_vy.plot([], [], "r--", label="cmd")[0],
        "meas_vy": ax_vy.plot([], [], "b-", label="meas")[0],
        "cmd_vz": ax_vz.plot([], [], "r--", label="cmd")[0],
        "meas_vz": ax_vz.plot([], [], "b-", label="meas")[0],
        "cmd_yaw": ax_yaw.plot([], [], "r--", label="cmd")[0],
        "meas_yaw": ax_yaw.plot([], [], "b-", label="meas")[0],
    }

    ax_vx.legend(loc="upper right")
    ax_vy.legend(loc="upper right")
    ax_vz.legend(loc="upper right")
    ax_yaw.legend(loc="upper right")

    # ROS + plotting loop
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            data = node.get_plot_data()
            if data is None:
                plt.pause(0.01)
                continue

            t = data["t"]
            lines["cmd_vx"].set_data(t, data["cmd_vx"])
            lines["meas_vx"].set_data(t, data["meas_vx"])
            lines["cmd_vy"].set_data(t, data["cmd_vy"])
            lines["meas_vy"].set_data(t, data["meas_vy"])
            lines["cmd_vz"].set_data(t, data["cmd_vz"])
            lines["meas_vz"].set_data(t, data["meas_vz"])
            lines["cmd_yaw"].set_data(t, data["cmd_yaw"])
            lines["meas_yaw"].set_data(t, data["meas_yaw"])

            for ax in axes:
                ax.relim()
                ax.autoscale_view()

            plt.tight_layout()
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info("Shutting down step tester")
        node.destroy_node()
        rclpy.shutdown()
        plt.ioff()
        plt.show()


if __name__ == "__main__":
    main()

