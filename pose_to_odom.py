#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time

class PoseToOdomConverter(Node):
    def __init__(self):
        super().__init__('pose_to_odom_converter')
        
        # Subscribe to X3 pose
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/X3/pose',
            self.pose_callback,
            10
        )
        
        # Publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, '/X3/odometry', 10)
        
        self.get_logger().info('Pose to Odometry converter started')
        
        # For velocity calculation
        self.last_pose = None
        self.last_time = None
        
    def pose_callback(self, msg):
        # Create odometry message
        odom = Odometry()
        
        # Header
        odom.header = msg.header
        odom.header.frame_id = 'X3/odometry'
        odom.child_frame_id = 'X3/base_link'
        
        # Position and orientation
        odom.pose.pose = msg.pose
        
        # Calculate velocity if we have previous data
        current_time = time.time()
        if self.last_pose is not None and self.last_time is not None:
            dt = current_time - self.last_time
            if dt > 0:
                dx = msg.pose.position.x - self.last_pose.position.x
                dy = msg.pose.position.y - self.last_pose.position.y
                dz = msg.pose.position.z - self.last_pose.position.z
                
                odom.twist.twist.linear.x = dx / dt
                odom.twist.twist.linear.y = dy / dt
                odom.twist.twist.linear.z = dz / dt
        
        # Store current pose for next iteration
        self.last_pose = msg.pose
        self.last_time = current_time
        
        # Publish
        self.odom_pub.publish(odom)
        
        self.get_logger().info(f'Converted pose to odom: x={msg.pose.position.x:.2f}, y={msg.pose.position.y:.2f}, z={msg.pose.position.z:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = PoseToOdomConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
