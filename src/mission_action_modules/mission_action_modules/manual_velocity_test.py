#!/usr/bin/env python3
"""
Manual velocity control test script
Bypasses PID controllers and directly publishes velocity commands to test Gazebo plugin
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import TwistStamped, Vector3Stamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import math
import time


class ManualVelocityTester(Node):
    def __init__(self):
        super().__init__('manual_velocity_tester')

        # Parameters
        self.declare_parameter('takeoff_height', 3.0)
        self.declare_parameter('target_distance', 10.0)
        self.declare_parameter('target_angle_deg', 135.0)
        self.declare_parameter('position_gain', 2.0)  # k for vx, vy
        self.declare_parameter('yaw_gain', 1.0)  # k2 for yaw
        self.declare_parameter('position_tolerance', 0.3)  # meters
        self.declare_parameter('yaw_tolerance', 0.1)  # radians

        self.takeoff_height = self.get_parameter('takeoff_height').value
        self.target_distance = self.get_parameter('target_distance').value
        self.target_angle_deg = self.get_parameter('target_angle_deg').value
        self.k_pos = self.get_parameter('position_gain').value
        self.k_yaw = self.get_parameter('yaw_gain').value
        self.pos_tol = self.get_parameter('position_tolerance').value
        self.yaw_tol = self.get_parameter('yaw_tolerance').value

        # QoS profile for enable topic - transient local for latching behavior
        enable_qos = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        # Publishers - directly to velocity adapter
        self.velocity_setpoint_pub = self.create_publisher(
            TwistStamped, '/drone/control/velocity_setpoint', 10
        )
        self.angular_setpoint_pub = self.create_publisher(
            Vector3Stamped, '/drone/control/angular_velocity_setpoint', 10
        )
        self.velocity_enable_pub = self.create_publisher(
            Bool, '/drone/control/velocity_enable', enable_qos
        )

        # Subscribers
        self.odom_sub = self.create_subscription(
            Odometry, '/X3/odometry', self.odometry_callback, 10
        )

        # State
        self.current_pose = None
        self.current_velocity = None
        self.takeoff_origin = None
        self.target_pose = None
        self.state = 'INIT'  # INIT, TAKEOFF, FLYING
        self.takeoff_complete = False
        self.target_reached = False

        # Timer for control loop
        self.control_timer = self.create_timer(0.05, self.control_loop)  # 20 Hz

        self.get_logger().info('✅ Manual Velocity Tester initialized')
        self.get_logger().info(f'   Target: {self.target_distance}m at {self.target_angle_deg}°')
        self.get_logger().info(f'   Position gain: {self.k_pos}, Yaw gain: {self.k_yaw}')

    def odometry_callback(self, msg):
        """Store current odometry"""
        self.current_pose = msg.pose.pose
        self.current_velocity = msg.twist.twist

    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        # yaw = atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y^2 + q.z^2))
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def wrap_angle(self, angle):
        """Wrap angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle


    def publish_velocity(self, vx, vy, vz, yaw_rate):
        """Publish velocity commands"""
        # Linear velocity
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.header.frame_id = 'world'
        vel_msg.twist.linear.x = float(vx)
        vel_msg.twist.linear.y = float(vy)
        vel_msg.twist.linear.z = float(vz)
        self.velocity_setpoint_pub.publish(vel_msg)

        # Angular velocity
        ang_msg = Vector3Stamped()
        ang_msg.header.stamp = self.get_clock().now().to_msg()
        ang_msg.header.frame_id = 'world'
        ang_msg.vector.z = float(yaw_rate)
        self.angular_setpoint_pub.publish(ang_msg)

    def enable_velocity_control(self, enable):
        """Enable/disable velocity control adapter"""
        msg = Bool()
        msg.data = enable
        self.velocity_enable_pub.publish(msg)

    def control_loop(self):
        """Main control loop"""
        if self.current_pose is None:
            return

        # Always ensure velocity control is enabled
        self.enable_velocity_control(True)

        if self.state == 'INIT':
            # Record initial position and set targets
            self.get_logger().info('📍 Initial position received, starting manual takeoff...')
            self.takeoff_origin = self.current_pose.position

            # Calculate target positions
            angle_rad = math.radians(self.target_angle_deg)
            self.target_pose = PoseStamped()
            self.target_pose.pose.position.x = self.takeoff_origin.x + self.target_distance * math.cos(angle_rad)
            self.target_pose.pose.position.y = self.takeoff_origin.y + self.target_distance * math.sin(angle_rad)
            self.target_pose.pose.position.z = self.takeoff_height

            target_yaw = angle_rad
            self.target_yaw = self.wrap_angle(target_yaw)

            self.get_logger().info(f'🎯 Takeoff target: height={self.takeoff_height:.2f}m')
            self.get_logger().info(f'🎯 Final target: ({self.target_pose.pose.position.x:.2f}, '
                                 f'{self.target_pose.pose.position.y:.2f}, '
                                 f'{self.target_pose.pose.position.z:.2f}), '
                                 f'yaw={math.degrees(self.target_yaw):.1f}°')

            # Enable velocity control
            self.enable_velocity_control(True)
            self.state = 'TAKEOFF'

        elif self.state == 'TAKEOFF':
            # Manual takeoff using simple proportional control
            dz = self.takeoff_height - self.current_pose.position.z

            # Simple proportional control for Z only
            vz_cmd = dz * self.k_pos

            # Limit Z velocity
            max_vel_z = 3.0
            vz_cmd = max(min(vz_cmd, max_vel_z), -max_vel_z)

            # Keep XY and yaw at zero during takeoff
            self.publish_velocity(0.0, 0.0, vz_cmd, 0.0)

            # Log every 20 cycles (1 second)
            if not hasattr(self, '_takeoff_log_counter'):
                self._takeoff_log_counter = 0
            self._takeoff_log_counter += 1

            if self._takeoff_log_counter >= 20:
                self.get_logger().info(
                    f'⬆️  TAKEOFF: Current Z={self.current_pose.position.z:.2f}m, '
                    f'Target Z={self.takeoff_height:.2f}m, dZ={dz:.2f}m, vz={vz_cmd:.2f}m/s'
                )
                self._takeoff_log_counter = 0

            # Check if reached takeoff height
            if abs(dz) < self.pos_tol:
                self.get_logger().info('✅ Takeoff height reached! Starting horizontal flight...')
                self.state = 'FLYING'

        elif self.state == 'FLYING':
            # Calculate position error
            dx = self.target_pose.pose.position.x - self.current_pose.position.x
            dy = self.target_pose.pose.position.y - self.current_pose.position.y
            dz = self.target_pose.pose.position.z - self.current_pose.position.z

            # Calculate yaw error
            current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
            dyaw = self.wrap_angle(self.target_yaw - current_yaw)

            # Calculate distance
            distance_xy = math.sqrt(dx**2 + dy**2)

            # Simple proportional control: v = k * error
            vx_cmd = dx * self.k_pos
            vy_cmd = dy * self.k_pos
            vz_cmd = dz * self.k_pos
            yaw_rate_cmd = dyaw * self.k_yaw

            # Limit velocities
            max_vel_xy = 5.0
            max_vel_z = 4.0
            max_yaw_rate = 3

            vel_xy_norm = math.sqrt(vx_cmd**2 + vy_cmd**2)
            if vel_xy_norm > max_vel_xy:
                scale = max_vel_xy / vel_xy_norm
                vx_cmd *= scale
                vy_cmd *= scale

            vz_cmd = max(min(vz_cmd, max_vel_z), -max_vel_z)
            yaw_rate_cmd = max(min(yaw_rate_cmd, max_yaw_rate), -max_yaw_rate)

            # Publish velocity commands
            self.publish_velocity(vx_cmd, vy_cmd, vz_cmd, yaw_rate_cmd)

            # Log every 20 cycles (1 second)
            if not hasattr(self, '_log_counter'):
                self._log_counter = 0
            self._log_counter += 1

            if self._log_counter >= 20:
                current_yaw = self.quaternion_to_yaw(self.current_pose.orientation)
                self.get_logger().info(
                    f'📊 Pos: ({self.current_pose.position.x:.2f}, {self.current_pose.position.y:.2f}, '
                    f'{self.current_pose.position.z:.2f}) | '
                    f'Err: ({dx:.2f}, {dy:.2f}, {dz:.2f}) | '
                    f'Dist: {distance_xy:.2f}m | '
                    f'Yaw: {math.degrees(current_yaw):.1f}° (target: {math.degrees(self.target_yaw):.1f}°, err: {math.degrees(dyaw):.1f}°) | '
                    f'Vel: ({vx_cmd:.2f}, {vy_cmd:.2f}, {vz_cmd:.2f}, yaw_rate: {yaw_rate_cmd:.2f})'
                )
                self._log_counter = 0

            # Check if reached target (only log once)
            if distance_xy < self.pos_tol and abs(dz) < self.pos_tol and abs(dyaw) < self.yaw_tol:
                if not self.target_reached:
                    self.get_logger().info('✅ Target reached! Continuing P-control to maintain position...')
                    self.target_reached = True


def main(args=None):
    rclpy.init(args=args)
    tester = ManualVelocityTester()

    try:
        rclpy.spin(tester)
    except KeyboardInterrupt:
        tester.get_logger().info('🛑 Interrupted by user')
    finally:
        # Disable velocity control before shutting down
        tester.enable_velocity_control(False)
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
