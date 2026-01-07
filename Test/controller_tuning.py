import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import Bool, Int32, Float64, Float64MultiArray
from geometry_msgs.msg import PoseStamped, Vector3
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
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
        self.waypoint_pub = self.create_publisher(PoseStamped, '/drone/control/waypoint_command', 10)
        self.mode_pub = self.create_publisher(Int32, '/drone/debug/mode', 10)
        self.cmd_att_pub = self.create_publisher(Vector3, '/drone/debug/cmd_att', 10)
        self.cmd_rate_pub = self.create_publisher(Vector3, '/drone/debug/cmd_rate', 10)
        self.cmd_thrust_pub = self.create_publisher(Float64, '/drone/debug/cmd_thrust', 10)
        
        # Subscribers (for data recording)
        self.imu_sub = self.create_subscription(Imu, '/imu/data', self.imu_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/X3/odometry', self.odom_callback, 10)
        self.mixer_sub = self.create_subscription(Float64MultiArray, '/drone/debug/mixer', self.mixer_callback, 10)
        
        # Data storage
        self.recording = False
        self.data_times = []
        self.data_rates = []  # [roll_rate, pitch_rate, yaw_rate]
        self.data_att = []    # [roll, pitch, yaw]
        self.data_thrust = [] # thrust command during step
        
        # State
        self.current_imu = None
        self.current_odom = None
        self.current_thrust_hover = 0.0
        self.target_waypoint = None
        
        # Timer for continuous waypoint publishing
        self.waypoint_timer = self.create_timer(0.1, self.waypoint_timer_callback)  # 10Hz

    def mixer_callback(self, msg):
        if len(msg.data) >= 1:
            self.current_thrust_hover = msg.data[0]

    def imu_callback(self, msg):
        self.current_imu = msg
        if self.recording:
            t = (self.get_clock().now().nanoseconds / 1e9)
            self.data_times.append(t)
            self.data_rates.append([msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z])
            # Conversion for attitude would be nice but raw rates are main interest for rate tuning

    def odom_callback(self, msg):
        self.current_odom = msg

    def enable_drone(self):
        msg = Bool()
        msg.data = True
        self.enable_pub.publish(msg)
        self.enable_pub_x3.publish(msg)
        self.enable_pub_wp.publish(msg)
        self.get_logger().info('Drone Enabled (Published to /drone/control/velocity_enable, /X3/enable, /drone/control/waypoint_enable)')

    def waypoint_timer_callback(self):
        """Continuously publish waypoint to prevent timeout"""
        if self.target_waypoint is not None:
            self.waypoint_pub.publish(self.target_waypoint)

    def takeoff(self, height=2.0):
        self.get_logger().info(f'Taking off to {height}m...')
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "world"
        msg.pose.position.x = 0.0
        msg.pose.position.y = 0.0
        msg.pose.position.z = height
        # Identity orientation
        msg.pose.orientation.w = 1.0
        
        # Store for continuous publishing
        self.target_waypoint = msg

    def set_mode(self, mode):
        msg = Int32()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f'Set Tuning Mode: {mode}')

    def run_step_test(self, axis, magnitude, duration, thrust_hover=None):
        """
        axis: 'rate_roll', 'rate_pitch', 'rate_yaw'
        magnitude: step size (rad/s)
        duration: seconds
        """
        if thrust_hover is None:
            thrust_hover = self.current_thrust_hover
            if thrust_hover < 1.0: # Sanity check, if 0 use default
                thrust_hover = 58.0
            self.get_logger().info(f"Using captured hover thrust: {thrust_hover:.2f}")

        self.data_times = []
        self.data_rates = []
        self.recording = True
        
        # Determine mode and command
        if 'rate' in axis:
            mode = 2
            vec = Vector3()
            if 'roll' in axis: vec.x = magnitude
            if 'pitch' in axis: vec.y = magnitude
            if 'yaw' in axis: vec.z = magnitude
            
            # Switch mode
            self.set_mode(mode)
            
            # Apply Step
            self.get_logger().info(f'Applying Step: {axis}={magnitude} for {duration}s')
            start_time = time.time()
            
            while time.time() - start_time < duration:
                self.cmd_rate_pub.publish(vec)
                
                # Keep thrust to hover approx
                thrust_msg = Float64()
                thrust_msg.data = thrust_hover
                self.cmd_thrust_pub.publish(thrust_msg)
                
                time.sleep(0.01) # 100Hz Pub
                
        # Recovery
        self.recording = False
        self.set_mode(0) # Back to Position Control
        self.get_logger().info('Test Complete. Recovering to Position Control.')
        
        # Re-issue hover waypoint just in case
        self.takeoff(2.0) 

        return self.data_times, self.data_rates

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
    
    print("Waiting 30s for stability...")
    time.sleep(30)
    
    while True:
        print("\n--- Controller Tuning Interface ---")
        print("1. Rate Roll Step (Impulse)")
        print("2. Rate Pitch Step (Impulse)")
        print("3. Rate Yaw Step (Impulse)")
        print("0. Exit")
        
        choice = input("Select Test: ")
        
        if choice == '0':
            break
            
        axis = ""
        mag = 0.0
        
        if choice == '1':
            axis = "rate_roll"
            mag = float(input("Magnitude (rad/s, e.g. 0.5): "))
        elif choice == '2':
            axis = "rate_pitch"
            mag = float(input("Magnitude (rad/s, e.g. 0.5): "))
        elif choice == '3':
            axis = "rate_yaw"
            mag = float(input("Magnitude (rad/s, e.g. 0.5): "))
        else:
            continue
            
        times, rates = node.run_step_test(axis, mag, 1.5) # Dynamic thrust capture used

        # Plotting
        if len(times) > 0:
            times = np.array(times) - times[0]
            rates = np.array(rates)

            # Create figure
            fig = plt.figure(figsize=(10, 6))
            if 'roll' in axis:
                plt.plot(times, rates[:, 0], 'b-', linewidth=2, label='Roll Rate (Actual)')
            if 'pitch' in axis:
                plt.plot(times, rates[:, 1], 'b-', linewidth=2, label='Pitch Rate (Actual)')
            if 'yaw' in axis:
                plt.plot(times, rates[:, 2], 'b-', linewidth=2, label='Yaw Rate (Actual)')

            plt.axhline(y=mag, color='r', linestyle='--', linewidth=2, label=f'Setpoint ({mag} rad/s)')
            plt.grid(True, alpha=0.3)
            plt.legend(loc='best', fontsize=12)
            plt.title(f'Step Response: {axis} {mag} rad/s', fontsize=14, fontweight='bold')
            plt.xlabel('Time (s)', fontsize=12)
            plt.ylabel('Rate (rad/s)', fontsize=12)

            # Save plot with absolute path
            import os
            save_dir = "/home/soja/DroneSim/Test"
            filename = f"step_{axis}_{int(time.time())}.png"
            filepath = os.path.join(save_dir, filename)
            plt.savefig(filepath, dpi=150, bbox_inches='tight')
            print(f"\n✅ Plot saved to: {filepath}")
            print(f"   Open with: eog {filepath}")
            print(f"   Or: cd {save_dir} && ls -lh step_*.png")

            # Try to display - use pause instead of show for better compatibility
            print("📊 Attempting to open plot window...")
            plt.ion()  # Turn on interactive mode
            plt.show()
            plt.pause(0.1)  # Small pause to render

            input("\n⏸️  Press ENTER to close plot and continue...")
            plt.close(fig)
            
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
