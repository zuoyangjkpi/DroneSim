import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3, Vector3Stamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
import math
import time
import threading
import matplotlib
matplotlib.use('TkAgg')  # Force interactive backend
import matplotlib.pyplot as plt
import numpy as np

class TuningNode(Node):
    def __init__(self):
        super().__init__('controller_tuning_node')
        
        # QoS for enable topics (latched)
        enable_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        
        # Publishers
        self.enable_pub = self.create_publisher(Bool, '/drone/control/velocity_enable', enable_qos)
        self.enable_pub_x3 = self.create_publisher(Bool, '/X3/enable', 10)
        self.enable_pub_wp = self.create_publisher(Bool, '/drone/control/waypoint_enable', enable_qos)
        self.enable_pub_att = self.create_publisher(Bool, '/drone/control/attitude_enable', enable_qos)
        self.waypoint_pub = self.create_publisher(PoseStamped, '/drone/control/waypoint_command', 10)
        self.velocity_pub = self.create_publisher(TwistStamped, '/drone/control/velocity_setpoint', 10)
        self.attitude_pub = self.create_publisher(Vector3Stamped, '/drone/control/attitude_command', 10)
        self.mode_pub = self.create_publisher(Int32, '/drone/debug/mode', 10)
        self.cmd_att_pub = self.create_publisher(Vector3, '/drone/debug/cmd_att', 10)
        self.cmd_rate_pub = self.create_publisher(Vector3, '/drone/debug/cmd_rate', 10)
        self.cmd_thrust_pub = self.create_publisher(Float64, '/drone/debug/cmd_thrust', 10)
        
        # Subscribers (for data recording)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/X3/odometry', self.odom_callback, 10)
        self.mixer_sub = self.create_subscription(Float64MultiArray, '/drone/debug/mixer_input', self.mixer_callback, 10)
        
        # Data storage
        self.recording = False
        self.rate_times = []
        self.rate_data = []  # [roll_rate, pitch_rate, yaw_rate]
        self.att_times = []
        self.att_data = []   # [roll, pitch, yaw]
        self.vel_times = []
        self.vel_data = []   # [vx, vy, vz] in world frame
        self.pos_times = []
        self.pos_data = []   # [x, y, z]
        
        # State
        self.current_imu = None
        self.current_odom = None
        self.current_thrust_hover = 0.0
        self.target_waypoint = None
        self.odom_twist_in_world = False  # Keep in sync with controllers.yaml
        self.tuning_mode = 0
        self.default_yaw_cmd = self._load_initial_yaw()
        
        # Timer for continuous waypoint publishing
        self.waypoint_timer = self.create_timer(0.1, self.waypoint_timer_callback)  # 10Hz
        self.yaw_timer = self.create_timer(0.01, self.yaw_hold_timer_callback)  # 100Hz

    def mixer_callback(self, msg):
        if len(msg.data) >= 1:
            self.current_thrust_hover = msg.data[0]

    def imu_callback(self, msg):
        self.current_imu = msg
        if self.recording:
            t = self._now_sec()
            self.rate_times.append(t)
            self.rate_data.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            roll, pitch, yaw = self._quat_to_euler(msg.orientation)
            self.att_times.append(t)
            self.att_data.append([roll, pitch, yaw])

    def odom_callback(self, msg):
        self.current_odom = msg
        if self.recording:
            t = self._now_sec()
            v_body = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])
            if self.odom_twist_in_world:
                v_world = v_body
            else:
                v_world = self._rotate_body_to_world(v_body, msg.pose.pose.orientation)
            self.vel_times.append(t)
            self.vel_data.append([v_world[0], v_world[1], v_world[2]])
            pos = msg.pose.pose.position
            self.pos_times.append(t)
            self.pos_data.append([pos.x, pos.y, pos.z])

    def _now_sec(self):
        return self.get_clock().now().nanoseconds / 1e9

    def _quat_to_euler(self, q):
        # Roll, pitch, yaw (ZYX)
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        sinr_cosp = 2.0 * (w * x + y * z)
        cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)

        sinp = 2.0 * (w * y - z * x)
        if abs(sinp) >= 1.0:
            pitch = math.copysign(math.pi / 2.0, sinp)
        else:
            pitch = math.asin(sinp)

        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def _rotate_body_to_world(self, v_body, q):
        # Rotation matrix from body to world (ENU) using quaternion
        x = q.x
        y = q.y
        z = q.z
        w = q.w

        r00 = 1.0 - 2.0 * (y * y + z * z)
        r01 = 2.0 * (x * y - z * w)
        r02 = 2.0 * (x * z + y * w)
        r10 = 2.0 * (x * y + z * w)
        r11 = 1.0 - 2.0 * (x * x + z * z)
        r12 = 2.0 * (y * z - x * w)
        r20 = 2.0 * (x * z - y * w)
        r21 = 2.0 * (y * z + x * w)
        r22 = 1.0 - 2.0 * (x * x + y * y)

        rot = np.array([
            [r00, r01, r02],
            [r10, r11, r12],
            [r20, r21, r22],
        ])
        return rot @ v_body

    def _wrap_angle(self, angle):
        return (angle + math.pi) % (2.0 * math.pi) - math.pi

    def _load_initial_yaw(self):
        world_file = "/home/soja/DroneSim/src/drone_description/worlds/drone_world.sdf"
        try:
            with open(world_file, "r", encoding="utf-8", errors="ignore") as f:
                data = f.read()
        except OSError:
            return 0.0

        idx = data.find("model://x3")
        if idx == -1:
            return 0.0

        pose_start = data.find("<pose>", idx)
        pose_end = data.find("</pose>", pose_start)
        if pose_start == -1 or pose_end == -1:
            return 0.0

        pose_str = data[pose_start + len("<pose>"):pose_end].strip()
        parts = pose_str.split()
        if len(parts) < 6:
            return 0.0

        try:
            return float(parts[5])
        except ValueError:
            return 0.0

    def enable_drone(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)
        self.enable_pub_x3.publish(msg)
        self.enable_pub_wp.publish(msg)
        self.get_logger().info('Drone Enabled (Published to /drone/control/velocity_enable, /X3/enable, /drone/control/waypoint_enable)')

    def set_waypoint_enable(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.enable_pub_wp.publish(msg)
        state = "ENABLED" if enabled else "DISABLED"
        self.get_logger().info(f'Waypoint controller {state}')

    def set_attitude_enable(self, enabled):
        msg = Bool()
        msg.data = bool(enabled)
        self.enable_pub_att.publish(msg)
        state = "ENABLED" if enabled else "DISABLED"
        self.get_logger().info(f'Yaw controller {state}')

    def waypoint_timer_callback(self):
        """Continuously publish waypoint to prevent timeout"""
        if self.target_waypoint is not None:
            self.waypoint_pub.publish(self.target_waypoint)

    def yaw_hold_timer_callback(self):
        if self.tuning_mode != 0:
            return
        msg = Vector3Stamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.vector.x = 0.0
        msg.vector.y = 0.0
        msg.vector.z = float(self.default_yaw_cmd)
        self.attitude_pub.publish(msg)

    def set_waypoint(self, x, y, z):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.target_waypoint = msg

    def takeoff(self, height=2.0):
        self.get_logger().info(f'Taking off to {height}m...')
        self.set_waypoint(0.0, 0.0, height)

    def set_mode(self, mode):
        msg = Int32()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.tuning_mode = mode
        if mode == 0:
            self.set_attitude_enable(True)
        else:
            self.set_attitude_enable(False)
        self.get_logger().info(f'Set Tuning Mode: {mode}')

    def _start_recording(self):
        self.rate_times = []
        self.rate_data = []
        self.att_times = []
        self.att_data = []
        self.vel_times = []
        self.vel_data = []
        self.pos_times = []
        self.pos_data = []
        self.recording = True

    def _stop_recording(self):
        self.recording = False

    def _extract_axis(self, data_list, axis_index):
        return [row[axis_index] for row in data_list]

    def _get_hover_thrust(self):
        thrust_hover = self.current_thrust_hover
        if thrust_hover < 1.0:
            thrust_hover = 58.0
        self.get_logger().info(f"Using captured hover thrust: {thrust_hover:.2f}")
        return thrust_hover

    def _get_current_attitude(self):
        if self.current_imu is not None:
            return np.array(self._quat_to_euler(self.current_imu.orientation))
        if self.current_odom is not None:
            return np.array(self._quat_to_euler(self.current_odom.pose.pose.orientation))
        return np.zeros(3)

    def _get_current_velocity_world(self):
        if self.current_odom is None:
            return np.zeros(3)
        v_body = np.array([
            self.current_odom.twist.twist.linear.x,
            self.current_odom.twist.twist.linear.y,
            self.current_odom.twist.twist.linear.z
        ])
        if self.odom_twist_in_world:
            return v_body
        return self._rotate_body_to_world(v_body, self.current_odom.pose.pose.orientation)

    def _get_current_position(self):
        if self.current_odom is None:
            return np.array([0.0, 0.0, 2.0])
        pos = self.current_odom.pose.pose.position
        return np.array([pos.x, pos.y, pos.z])

    def run_rate_step(self, axis, magnitude, duration):
        thrust_hover = self._get_hover_thrust()
        axis_map = {"roll": 0, "pitch": 1, "yaw": 2}
        axis_index = axis_map[axis]
        rate_base = np.zeros(3)
        if self.current_imu is not None:
            rate_base = np.array([
                self.current_imu.angular_velocity.x,
                self.current_imu.angular_velocity.y,
                self.current_imu.angular_velocity.z
            ])
        vec = Vector3(x=float(rate_base[0]), y=float(rate_base[1]), z=float(rate_base[2]))
        if axis == "roll":
            vec.x = float(rate_base[0] + magnitude)
        elif axis == "pitch":
            vec.y = float(rate_base[1] + magnitude)
        elif axis == "yaw":
            vec.z = float(rate_base[2] + magnitude)

        self.set_waypoint_enable(False)
        self.set_mode(2)
        self._start_recording()

        setpoint = [vec.x, vec.y, vec.z][axis_index]
        self.get_logger().info(f'Applying Rate Step: {axis}={setpoint:.3f} for {duration}s')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_rate_pub.publish(vec)
            thrust_msg = Float64()
            thrust_msg.data = thrust_hover
            self.cmd_thrust_pub.publish(thrust_msg)
            time.sleep(0.01)  # 100Hz Pub

        self._stop_recording()
        self.set_mode(0)
        self.set_waypoint_enable(True)
        self.get_logger().info('Test Complete. Recovering to Position Control.')
        self.takeoff(2.0)

        return self.rate_times, self._extract_axis(self.rate_data, axis_index), setpoint

    def run_attitude_step(self, axis, magnitude, duration):
        thrust_hover = self._get_hover_thrust()
        axis_map = {"roll": 0, "pitch": 1, "yaw": 2}
        axis_index = axis_map[axis]

        att_base = self._get_current_attitude()
        att_cmd = att_base.copy()
        att_cmd[axis_index] += magnitude
        vec = Vector3(x=float(att_cmd[0]), y=float(att_cmd[1]), z=float(att_cmd[2]))

        self.set_waypoint_enable(False)
        self.set_mode(3)
        self._start_recording()

        self.get_logger().info(f'Applying Attitude Step: {axis}={att_cmd[axis_index]:.3f} for {duration}s')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_att_pub.publish(vec)
            thrust_msg = Float64()
            thrust_msg.data = thrust_hover
            self.cmd_thrust_pub.publish(thrust_msg)
            time.sleep(0.01)

        self._stop_recording()
        self.set_mode(0)
        self.set_waypoint_enable(True)
        self.get_logger().info('Test Complete. Recovering to Position Control.')
        self.takeoff(2.0)

        return self.att_times, self._extract_axis(self.att_data, axis_index), att_cmd[axis_index]

    def run_yaw_step_via_controller(self, magnitude, duration):
        thrust_hover = self._get_hover_thrust()
        att_base = self._get_current_attitude()
        yaw_target = self._wrap_angle(att_base[2] + magnitude)

        hold_att = Vector3(x=float(att_base[0]), y=float(att_base[1]), z=float(att_base[2]))

        self.set_waypoint_enable(False)
        self.set_mode(3)
        self.set_attitude_enable(True)
        self._start_recording()

        self.get_logger().info(f'Applying Yaw Step via yaw_controller: {yaw_target:.3f} rad for {duration}s')
        start_time = time.time()
        while time.time() - start_time < duration:
            self.cmd_att_pub.publish(hold_att)
            thrust_msg = Float64()
            thrust_msg.data = thrust_hover
            self.cmd_thrust_pub.publish(thrust_msg)

            cmd = Vector3Stamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.vector.x = 0.0
            cmd.vector.y = 0.0
            cmd.vector.z = float(yaw_target)
            self.attitude_pub.publish(cmd)

            time.sleep(0.01)

        self._stop_recording()
        self.set_mode(0)
        self.set_waypoint_enable(True)
        self.get_logger().info('Test Complete. Recovering to Position Control.')
        self.takeoff(2.0)

        return self.att_times, self._extract_axis(self.att_data, 2), yaw_target

    def run_velocity_step(self, axis, magnitude, duration):
        axis_map = {"x": 0, "y": 1, "z": 2}
        axis_index = axis_map[axis]

        vel_base = self._get_current_velocity_world()
        vel_cmd = vel_base.copy()
        vel_cmd[axis_index] += magnitude

        self.set_waypoint_enable(False)
        self.set_mode(4)
        self._start_recording()

        self.get_logger().info(f'Applying Velocity Step: {axis}={vel_cmd[axis_index]:.3f} for {duration}s')
        start_time = time.time()
        while time.time() - start_time < duration:
            msg = TwistStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.linear.x = float(vel_cmd[0])
            msg.twist.linear.y = float(vel_cmd[1])
            msg.twist.linear.z = float(vel_cmd[2])
            self.velocity_pub.publish(msg)
            time.sleep(0.01)

        self._stop_recording()
        self.set_mode(0)
        self.set_waypoint_enable(True)
        self.get_logger().info('Test Complete. Recovering to Position Control.')
        self.takeoff(2.0)

        return self.vel_times, self._extract_axis(self.vel_data, axis_index), vel_cmd[axis_index]

    def run_position_step(self, axis, magnitude, duration):
        axis_map = {"x": 0, "y": 1, "z": 2}
        axis_index = axis_map[axis]

        pos_base = self._get_current_position()
        pos_cmd = pos_base.copy()
        pos_cmd[axis_index] += magnitude

        self.set_waypoint_enable(True)
        self.set_mode(5)
        self.set_waypoint(pos_cmd[0], pos_cmd[1], pos_cmd[2])
        self._start_recording()

        self.get_logger().info(f'Applying Position Step: {axis}={pos_cmd[axis_index]:.3f} for {duration}s')
        time.sleep(duration)

        self._stop_recording()
        self.set_mode(0)
        self.get_logger().info('Test Complete. Recovering to Position Control.')
        self.takeoff(2.0)

        return self.pos_times, self._extract_axis(self.pos_data, axis_index), pos_cmd[axis_index]

    def plot_step_response(self, title, times, actual, setpoint, actual_label, y_label, filename_prefix):
        if len(times) == 0:
            print("No data recorded for this test.")
            return

        times = np.array(times) - times[0]
        actual = np.array(actual)

        fig = plt.figure(figsize=(10, 6))
        plt.plot(times, actual, 'b-', linewidth=2, label=actual_label)
        plt.axhline(y=setpoint, color='r', linestyle='--', linewidth=2,
                    label=f'Setpoint ({setpoint:.3f})')
        plt.grid(True, alpha=0.3)
        plt.legend(loc='best', fontsize=12)
        plt.title(title, fontsize=14, fontweight='bold')
        plt.xlabel('Time (s)', fontsize=12)
        plt.ylabel(y_label, fontsize=12)

        import os
        save_dir = "/home/soja/DroneSim/Test"
        filename = f"step_{filename_prefix}_{int(time.time())}.png"
        filepath = os.path.join(save_dir, filename)
        plt.savefig(filepath, dpi=150, bbox_inches='tight')
        print(f"\n✅ Plot saved to: {filepath}")
        print(f"   Open with: eog {filepath}")
        print(f"   Or: cd {save_dir} && ls -lh step_*.png")

        print("📊 Attempting to open plot window...")
        plt.ion()
        plt.show()
        plt.pause(0.1)

        input("\n⏸️  Press ENTER to close plot and continue...")
        plt.close(fig)

def main():
    rclpy.init()
    node = TuningNode()
    
    # Spin in a separate thread
    thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    thread.start()
    
    time.sleep(2)
    node.enable_drone()
    time.sleep(1)
    node.takeoff(2.0)
    
    print("Waiting 10s for stability...")
    time.sleep(10)
    
    rate_duration = 1.5
    att_duration = 2.0
    yaw_duration = 3.0
    vel_duration = 2.0
    pos_duration = 5.0

    while True:
        print("\n--- Controller Tuning Interface ---")
        print("1. Rate Step")
        print("2. Attitude Step")
        print("3. Velocity Step")
        print("4. Position Step")
        print("5. Yaw Step (via yaw_controller)")
        print("0. Exit")

        choice = input("Select Test: ")

        if choice == '0':
            break

        if choice == '1':
            print("Rate Axis: 1) Roll  2) Pitch  3) Yaw")
            axis_choice = input("Select Axis: ")
            axis_map = {'1': 'roll', '2': 'pitch', '3': 'yaw'}
            if axis_choice not in axis_map:
                continue
            axis = axis_map[axis_choice]
            mag = float(input("Magnitude (rad/s, e.g. 0.5): "))
            times, actual, setpoint = node.run_rate_step(axis, mag, rate_duration)
            title = f"Step Response: rate_{axis} {setpoint:.3f} rad/s"
            label = f"{axis.capitalize()} Rate (Actual)"
            node.plot_step_response(title, times, actual, setpoint, label, "Rate (rad/s)", f"rate_{axis}")
        elif choice == '2':
            print("Attitude Axis: 1) Roll  2) Pitch")
            axis_choice = input("Select Axis: ")
            axis_map = {'1': 'roll', '2': 'pitch'}
            if axis_choice not in axis_map:
                continue
            axis = axis_map[axis_choice]
            mag = float(input("Magnitude (rad, e.g. 0.2): "))
            times, actual, setpoint = node.run_attitude_step(axis, mag, att_duration)
            title = f"Step Response: att_{axis} {setpoint:.3f} rad"
            label = f"{axis.capitalize()} Angle (Actual)"
            node.plot_step_response(title, times, actual, setpoint, label, "Angle (rad)", f"att_{axis}")
        elif choice == '3':
            print("Velocity Axis: 1) X  2) Y  3) Z")
            axis_choice = input("Select Axis: ")
            axis_map = {'1': 'x', '2': 'y', '3': 'z'}
            if axis_choice not in axis_map:
                continue
            axis = axis_map[axis_choice]
            mag = float(input("Magnitude (m/s, e.g. 0.5): "))
            times, actual, setpoint = node.run_velocity_step(axis, mag, vel_duration)
            title = f"Step Response: vel_{axis} {setpoint:.3f} m/s"
            label = f"{axis.upper()} Velocity (Actual)"
            node.plot_step_response(title, times, actual, setpoint, label, "Velocity (m/s)", f"vel_{axis}")
        elif choice == '4':
            print("Position Axis: 1) X  2) Y  3) Z")
            axis_choice = input("Select Axis: ")
            axis_map = {'1': 'x', '2': 'y', '3': 'z'}
            if axis_choice not in axis_map:
                continue
            axis = axis_map[axis_choice]
            mag = float(input("Magnitude (m, e.g. 1.0): "))
            times, actual, setpoint = node.run_position_step(axis, mag, pos_duration)
            title = f"Step Response: pos_{axis} {setpoint:.3f} m"
            label = f"{axis.upper()} Position (Actual)"
            node.plot_step_response(title, times, actual, setpoint, label, "Position (m)", f"pos_{axis}")
        elif choice == '5':
            mag = float(input("Yaw Step Magnitude (rad, e.g. 0.5): "))
            times, actual, setpoint = node.run_yaw_step_via_controller(mag, yaw_duration)
            title = f"Step Response: yaw {setpoint:.3f} rad"
            label = "Yaw Angle (Actual)"
            node.plot_step_response(title, times, actual, setpoint, label, "Angle (rad)", "yaw")
        else:
            continue
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
