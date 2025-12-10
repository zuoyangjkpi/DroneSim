#!/usr/bin/env python3
"""
Test node for NMPC Drone Person Tracker
Provides simulated person detections and drone state for testing
"""

import rclpy
from rclpy.node import Node
import numpy as np
import math
import time

from geometry_msgs.msg import Twist, PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, Header
from neural_network_msgs.msg import NeuralNetworkDetectionArray, NeuralNetworkDetection
from sensor_msgs.msg import Image
import tf2_ros

class NMPCTestNode(Node):
    """Test node for NMPC tracker"""
    
    def __init__(self):
        super().__init__('nmpc_test_node')
        
        # Test parameters
        self.test_mode = 'circular'  # 'circular', 'linear', 'stationary'
        self.person_speed = 0.5  # m/s
        self.test_duration = 60.0  # seconds
        
        # Simulation state
        self.start_time = time.time()
        self.person_position = np.array([0.0, 0.0, 0.0])
        self.drone_position = np.array([3.0, 3.0, 2.0])
        self.drone_velocity = np.array([0.0, 0.0, 0.0])
        
        # TF broadcaster
        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        
        # Publishers
        self.detection_pub = self.create_publisher(
            NeuralNetworkDetectionArray,
            '/person_detections',
            10
        )
        
        # Removed: self.odom_pub - odometry should come from Gazebo simulation only
        
        self.enable_pub = self.create_publisher(
            Bool,
            '/nmpc/enable',
            10
        )
                
        # Subscriber for control commands (to simulate drone response)
        self.cmd_sub = self.create_subscription(
            Twist,
            '/X3/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Timers
        self.detection_timer = self.create_timer(0.1, self.publish_detection)  # 10 Hz
        # Removed: self.odom_timer - odometry comes from Gazebo simulation
        self.enable_timer = self.create_timer(1.0, self.publish_enable)  # 1 Hz
        self.tf_timer = self.create_timer(0.05, self.publish_transforms)  # 20 Hz
        
        self.get_logger().info(f"NMPC Test Node started - Mode: {self.test_mode}")
        self.get_logger().info("Publishing simulated person detections and drone odometry")
    
    def cmd_vel_callback(self, msg: Twist):
        """Simulate drone response to control commands"""
        # Simple integration of velocity commands
        dt = 0.05  # 20 Hz update rate
        
        # Update drone velocity (simplified dynamics)
        self.drone_velocity[0] = msg.linear.x
        self.drone_velocity[1] = msg.linear.y
        self.drone_velocity[2] = msg.linear.z
        
        # Update drone position
        self.drone_position += self.drone_velocity * dt
        
        # Keep drone within reasonable bounds
        self.drone_position[0] = np.clip(self.drone_position[0], -20.0, 20.0)
        self.drone_position[1] = np.clip(self.drone_position[1], -20.0, 20.0)
        self.drone_position[2] = np.clip(self.drone_position[2], 0.5, 10.0)
    
    def update_person_position(self):
        """Update simulated person position based on test mode"""
        current_time = time.time() - self.start_time
        
        if self.test_mode == 'circular':
            # Person walks in a circle
            radius = 3.0
            angular_freq = self.person_speed / radius
            self.person_position[0] = radius * math.cos(angular_freq * current_time)
            self.person_position[1] = radius * math.sin(angular_freq * current_time)
            self.person_position[2] = 0.0
            
        elif self.test_mode == 'linear':
            # Person walks back and forth
            period = 10.0  # seconds for one cycle
            amplitude = 4.0  # meters
            self.person_position[0] = amplitude * math.sin(2 * math.pi * current_time / period)
            self.person_position[1] = 0.0
            self.person_position[2] = 0.0
            
        elif self.test_mode == 'stationary':
            # Person stays in one place
            self.person_position = np.array([2.0, 1.0, 0.0])
        
        # Add minimal noise for more stable tracking
        noise = np.random.normal(0, 0.01, 3)  # 1cm standard deviation for more stability
        self.person_position += noise
    
    def publish_detection(self):
        """Publish simulated person detection"""
        # Update person position based on test mode
        self.update_person_position()
        
        # Create detection message
        detection_array = NeuralNetworkDetectionArray()
        detection_array.header.stamp = self.get_clock().now().to_msg()
        detection_array.header.frame_id = "camera_link"
        
        # Create person detection
        detection = NeuralNetworkDetection()
        detection.object_class = 1  # Assuming 1 represents "person" class
        detection.detection_score = 0.85 + 0.1 * np.random.random()  # 0.85-0.95 confidence
        
        # Simulate bounding box based on distance
        distance = np.linalg.norm(self.person_position - self.drone_position)
        
        # Simple projection model
        if distance > 0.5:
            bbox_height = 0.3 / distance  # Inverse relationship
            bbox_width = bbox_height * 0.6  # Aspect ratio
            
            # Center position (with minimal stable noise)
            center_x = 0.5 + 0.02 * np.random.normal()  # Centered with minimal noise
            center_y = 0.6 + 0.01 * np.random.normal()  # Slightly below center with minimal noise
            
            # Ensure bounding box is within image bounds
            bbox_height = np.clip(bbox_height, 0.05, 0.8)
            bbox_width = np.clip(bbox_width, 0.03, 0.6)
            center_x = np.clip(center_x, bbox_width/2, 1.0 - bbox_width/2)
            center_y = np.clip(center_y, bbox_height/2, 1.0 - bbox_height/2)
            
            # Convert center/size to min/max coordinates (assuming normalized coordinates 0-1)
            # If using pixel coordinates, multiply by image width/height
            image_width = 640  # Assumed image width
            image_height = 480  # Assumed image height
            
            # Convert normalized coordinates to pixel coordinates
            xmin = int((center_x - bbox_width/2) * image_width)
            xmax = int((center_x + bbox_width/2) * image_width)
            ymin = int((center_y - bbox_height/2) * image_height)
            ymax = int((center_y + bbox_height/2) * image_height)
            
            # Set bounding box coordinates
            detection.xmin = xmin
            detection.xmax = xmax
            detection.ymin = ymin
            detection.ymax = ymax
            
            # Set variance values (optional, for uncertainty estimation)
            detection.variance_xmin = 1.0
            detection.variance_xmax = 1.0
            detection.variance_ymin = 1.0
            detection.variance_ymax = 1.0
            
            detection_array.detections.append(detection)
        
        # Publish detection
        self.detection_pub.publish(detection_array)
    
    # Removed publish_odometry method - odometry should come from Gazebo simulation only
    
    def publish_enable(self):
        """Publish enable signal"""
        enable_msg = Bool()
        enable_msg.data = True
        self.enable_pub.publish(enable_msg)
    
    def publish_transforms(self):
        """Publish TF transforms"""
        # Implementation for TF publishing
        pass
    
    # Commented out to avoid clashing with Gazebo's camera topics
    # def publish_image(self):
    #     """Publish simulated camera image"""
    #     # Create a simple test image
    #     image = Image()
    #     image.header.stamp = self.get_clock().now().to_msg()
    #     image.header.frame_id = "camera_link"
    #     image.height = 480
    #     image.width = 640
    #     image.encoding = "rgb8"
    #     image.is_bigendian = False
    #     image.step = 640 * 3
    #     image.data = [100] * (640 * 480 * 3)  # Simple gray image
    #     
    #     self.image_pub.publish(image)

    def set_test_mode(self, mode: str):
        """Change test mode"""
        if mode in ['circular', 'linear', 'stationary']:
            self.test_mode = mode
            self.get_logger().info(f"Test mode changed to: {mode}")
        else:
            self.get_logger().warn(f"Unknown test mode: {mode}")

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = NMPCTestNode()
        
        # Print instructions
        print("\n" + "="*50)
        print("NMPC Tracker Test Node")
        print("="*50)
        print("This node provides simulated data for testing the NMPC tracker:")
        print("- Person detections at /person_detections")
        print("- Drone odometry at /X3/odometry")
        print("- Enable signal at /nmpc/enable")
        print("\nThe simulated person moves in a circular pattern.")
        print("Launch the NMPC tracker in another terminal to see it in action!")
        print("="*50 + "\n")
        
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nTest node stopped by user")
    except Exception as e:
        print(f"Error in test node: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
