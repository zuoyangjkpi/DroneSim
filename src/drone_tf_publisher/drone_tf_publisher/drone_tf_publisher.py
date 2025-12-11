#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(roll: float, pitch: float, yaw: float):
    """Convert Euler angles to quaternion."""
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class DroneTfPublisher(Node):
    def __init__(self):
        super().__init__('drone_tf_publisher')

        # Default to ROS wall-clock time unless overridden externally
        if not self.has_parameter('use_sim_time'):
            self.declare_parameter('use_sim_time', False)

        # Create TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribe to drone odometry
        self.odom_sub = self.create_subscription(
            Odometry, '/X3/odometry', self.odom_callback, 10)

        self.get_logger().info('Drone TF Publisher started - bridging /X3/odometry to TF tree')

        # Timer to log TF publishing status
        self.status_timer = self.create_timer(2.0, self.log_status)

    def odom_callback(self, msg):
        """Convert odometry to TF transform."""
        try:
            # Prefer the incoming odometry timestamp (simulation time)
            stamp = msg.header.stamp
            if stamp.sec == 0 and stamp.nanosec == 0:
                stamp = self.get_clock().now().to_msg()

            parent_frame = msg.header.frame_id or 'world'
            child_frame = 'X3/base_link'

            # parent -> base_link
            base_transform = TransformStamped()
            base_transform.header.stamp = stamp
            base_transform.header.frame_id = parent_frame
            base_transform.child_frame_id = child_frame
            base_transform.transform.translation.x = msg.pose.pose.position.x
            base_transform.transform.translation.y = msg.pose.pose.position.y
            base_transform.transform.translation.z = msg.pose.pose.position.z
            base_transform.transform.rotation = msg.pose.pose.orientation

            # X3/base_link -> X3/camera_link (match SDF pose: 0.2 forward, 30° pitch down)
            cam_link = TransformStamped()
            cam_link.header.stamp = stamp
            cam_link.header.frame_id = child_frame
            cam_link.child_frame_id = 'X3/camera_link'
            cam_link.transform.translation.x = 0.2
            cam_link.transform.translation.y = 0.0
            cam_link.transform.translation.z = 0.0
            # 采用正的俯仰角（30°）保持与SDF一致，配合光学坐标系旋转后指向地面
            qx, qy, qz, qw = quaternion_from_euler(0.0, math.pi / 6, 0.0)
            cam_link.transform.rotation.x = qx
            cam_link.transform.rotation.y = qy
            cam_link.transform.rotation.z = qz
            cam_link.transform.rotation.w = qw

            # X3/camera_link -> X3/camera_rgb_optical_frame (standard optical transform)
            cam_optical = TransformStamped()
            cam_optical.header.stamp = stamp
            cam_optical.header.frame_id = 'X3/camera_link'
            cam_optical.child_frame_id = 'X3/camera_optical_frame'
            cam_optical.transform.translation.x = 0.0
            cam_optical.transform.translation.y = 0.0
            cam_optical.transform.translation.z = 0.0
            qx_opt, qy_opt, qz_opt, qw_opt = quaternion_from_euler(-math.pi/2, 0.0, -math.pi/2)
            cam_optical.transform.rotation.x = qx_opt
            cam_optical.transform.rotation.y = qy_opt
            cam_optical.transform.rotation.z = qz_opt
            cam_optical.transform.rotation.w = qw_opt

            # X3/camera_link -> X3/camera_link/camera_front (sensor frame used by detections)
            cam_sensor = TransformStamped()
            cam_sensor.header.stamp = stamp
            cam_sensor.header.frame_id = 'X3/camera_link'
            cam_sensor.child_frame_id = 'X3/camera_link/camera_front'
            cam_sensor.transform.translation.x = 0.0
            cam_sensor.transform.translation.y = 0.0
            cam_sensor.transform.translation.z = 0.0
            cam_sensor.transform.rotation.x = 0.0
            cam_sensor.transform.rotation.y = 0.0
            cam_sensor.transform.rotation.z = 0.0
            cam_sensor.transform.rotation.w = 1.0

            # Provide an explicit optical frame for the front camera so detectors can consume REP103 frames
            cam_sensor_optical = TransformStamped()
            cam_sensor_optical.header.stamp = stamp
            cam_sensor_optical.header.frame_id = 'X3/camera_link/camera_front'
            cam_sensor_optical.child_frame_id = 'X3/camera_link/camera_front_optical_frame'
            cam_sensor_optical.transform.translation.x = 0.0
            cam_sensor_optical.transform.translation.y = 0.0
            cam_sensor_optical.transform.translation.z = 0.0
            cam_sensor_optical.transform.rotation.x = qx_opt
            cam_sensor_optical.transform.rotation.y = qy_opt
            cam_sensor_optical.transform.rotation.z = qz_opt
            cam_sensor_optical.transform.rotation.w = qw_opt

            self.tf_broadcaster.sendTransform(
                [base_transform, cam_link, cam_optical, cam_sensor, cam_sensor_optical]
            )

        except Exception as e:
            self.get_logger().error(f'Failed to publish TF: {e}')

    def log_status(self):
        """Log current status for debugging."""
        try:
            self.get_logger().info('TF Publisher running - listening for /X3/odometry')
        except Exception as e:
            self.get_logger().error(f'Status error: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = DroneTfPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
