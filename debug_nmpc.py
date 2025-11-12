#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import numpy as np

class NMPCDebugNode(Node):
    def __init__(self):
        super().__init__('nmpc_debug_node')

        self.waypoint_sub = self.create_subscription(
            PoseStamped,
            '/drone/control/waypoint_command',
            self.waypoint_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/X3/odometry',
            self.odom_callback,
            10
        )

        self.current_pos = None
        self.last_waypoint = None

    def odom_callback(self, msg):
        self.current_pos = np.array([
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ])

    def waypoint_callback(self, msg):
        waypoint = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ])

        if self.current_pos is not None:
            delta = waypoint - self.current_pos
            distance = np.linalg.norm(delta)

            print(f"\n[DEBUG] Waypoint Command:")
            print(f"  Current pos: [{self.current_pos[0]:.3f}, {self.current_pos[1]:.3f}, {self.current_pos[2]:.3f}]")
            print(f"  Target pos:  [{waypoint[0]:.3f}, {waypoint[1]:.3f}, {waypoint[2]:.3f}]")
            print(f"  Delta:       [{delta[0]:.3f}, {delta[1]:.3f}, {delta[2]:.3f}]")
            print(f"  Distance:    {distance:.3f} m")

            if self.last_waypoint is not None:
                waypoint_change = np.linalg.norm(waypoint - self.last_waypoint)
                print(f"  Waypoint change: {waypoint_change:.3f} m")

        self.last_waypoint = waypoint

def main():
    rclpy.init()
    node = NMPCDebugNode()
    print("NMPC Debug Node Started - Monitoring waypoint commands...")
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
