#!/usr/bin/env python3
import argparse
import csv
import os
import time

import matplotlib.pyplot as plt


def _load_csv(path):
    with open(path, "r", newline="") as f:
        reader = csv.DictReader(f)
        rows = [row for row in reader]
    return rows


def _series(rows, key):
    values = []
    for row in rows:
        raw = row.get(key, "")
        if raw is None or raw == "":
            values.append(None)
        else:
            try:
                values.append(float(raw))
            except ValueError:
                values.append(None)
    return values


def _plot_group(rows, t, keys, title, y_label, out_path):
    plt.figure(figsize=(10, 5))
    for key in keys:
        data = _series(rows, key)
        plt.plot(t, data, label=key)
    plt.grid(True, alpha=0.3)
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel(y_label)
    plt.legend()
    plt.tight_layout()
    plt.savefig(out_path, dpi=150)
    plt.close()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("csv_path", nargs="?", default="")
    args = parser.parse_args()

    debug_dir = "/home/soja/DroneSim/Test/Debug"
    csv_path = args.csv_path
    if not csv_path:
        candidates = [
            os.path.join(debug_dir, f)
            for f in os.listdir(debug_dir)
            if f.startswith("flight_debug_") and f.endswith(".csv")
        ]
        if not candidates:
            raise SystemExit("No flight_debug_*.csv found in Test/Debug.")
        csv_path = max(candidates, key=os.path.getmtime)

    rows = _load_csv(csv_path)
    if not rows:
        raise SystemExit("CSV is empty.")

    t = _series(rows, "time_s")
    stamp = time.strftime("%Y%m%d_%H%M%S")
    base = os.path.splitext(os.path.basename(csv_path))[0]
    out_dir = os.path.join(debug_dir, f"plots_{base}_{stamp}")
    os.makedirs(out_dir, exist_ok=True)

    _plot_group(
        rows, t,
        ["odom_pos_x", "odom_pos_y", "odom_pos_z"],
        "Odometry Position",
        "m",
        os.path.join(out_dir, "position.png"),
    )
    _plot_group(
        rows, t,
        ["odom_vel_x", "odom_vel_y", "odom_vel_z"],
        "Odometry Velocity",
        "m/s",
        os.path.join(out_dir, "velocity.png"),
    )
    _plot_group(
        rows, t,
        ["odom_roll", "odom_pitch", "odom_yaw"],
        "Odometry Attitude",
        "rad",
        os.path.join(out_dir, "attitude.png"),
    )
    _plot_group(
        rows, t,
        ["imu_roll", "imu_pitch", "imu_yaw"],
        "IMU Attitude",
        "rad",
        os.path.join(out_dir, "imu_attitude.png"),
    )
    _plot_group(
        rows, t,
        ["imu_lin_acc_x", "imu_lin_acc_y", "imu_lin_acc_z"],
        "IMU Linear Accel",
        "m/s^2",
        os.path.join(out_dir, "imu_linear_accel.png"),
    )
    _plot_group(
        rows, t,
        ["odom_ang_vel_x", "odom_ang_vel_y", "odom_ang_vel_z"],
        "Odometry Angular Velocity",
        "rad/s",
        os.path.join(out_dir, "odom_ang_vel.png"),
    )
    _plot_group(
        rows, t,
        ["vel_sp_x", "vel_sp_y", "vel_sp_z"],
        "Velocity Setpoint",
        "m/s",
        os.path.join(out_dir, "velocity_setpoint.png"),
    )
    _plot_group(
        rows, t,
        ["wp_cmd_x", "wp_cmd_y", "wp_cmd_z"],
        "Waypoint Command",
        "m",
        os.path.join(out_dir, "waypoint_command.png"),
    )
    _plot_group(
        rows, t,
        ["wp_enable", "wp_reached", "vel_enable", "att_enable", "x3_enable", "att_reached"],
        "Enable/Reached Flags",
        "bool",
        os.path.join(out_dir, "enable_flags.png"),
    )
    _plot_group(
        rows, t,
        ["rate_sp_x", "rate_sp_y", "rate_sp_z"],
        "Rate Setpoint",
        "rad/s",
        os.path.join(out_dir, "rate_setpoint.png"),
    )
    _plot_group(
        rows, t,
        ["att_cmd_roll", "att_cmd_pitch", "att_cmd_yaw"],
        "Attitude Command",
        "rad",
        os.path.join(out_dir, "att_cmd.png"),
    )
    _plot_group(
        rows, t,
        ["att_sp_roll", "att_sp_pitch", "att_sp_yaw"],
        "Attitude Setpoint (Inner Loop)",
        "rad",
        os.path.join(out_dir, "att_setpoint.png"),
    )
    _plot_group(
        rows, t,
        ["accel_sp_x", "accel_sp_y", "accel_sp_z"],
        "Accel Setpoint",
        "m/s^2",
        os.path.join(out_dir, "accel_setpoint.png"),
    )
    _plot_group(
        rows, t,
        ["cmd_att_roll", "cmd_att_pitch", "cmd_att_yaw"],
        "Tuning Cmd Attitude",
        "rad",
        os.path.join(out_dir, "cmd_att.png"),
    )
    _plot_group(
        rows, t,
        ["cmd_rate_x", "cmd_rate_y", "cmd_rate_z"],
        "Tuning Cmd Rate",
        "rad/s",
        os.path.join(out_dir, "cmd_rate.png"),
    )
    _plot_group(
        rows, t,
        ["mixer_0", "mixer_1", "mixer_2", "mixer_3"],
        "Mixer Input",
        "cmd",
        os.path.join(out_dir, "mixer_input.png"),
    )
    _plot_group(
        rows, t,
        ["motor_0", "motor_1", "motor_2", "motor_3",
         "motor_4", "motor_5", "motor_6", "motor_7"],
        "Motor Speed",
        "rad/s",
        os.path.join(out_dir, "motor_speed.png"),
    )

    print(f"Plots saved to: {out_dir}")


if __name__ == "__main__":
    main()
