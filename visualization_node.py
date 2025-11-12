#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from neural_network_msgs.msg import NeuralNetworkDetectionArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray
from std_msgs.msg import Float64MultiArray
import math

class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        # Publishers
        self.person_marker_pub = self.create_publisher(MarkerArray, '/person_position_markers', 10)
        self.trajectory_marker_pub = self.create_publisher(MarkerArray, '/drone_trajectory_markers', 10)
        self.drone_path_pub = self.create_publisher(Path, '/drone_path', 10)
        self.drone_marker_pub = self.create_publisher(MarkerArray, '/drone_position_markers', 10)

        # Subscribers
        self.detection_sub = self.create_subscription(
            NeuralNetworkDetectionArray, '/person_detections', self.detection_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/X3/odometry', self.odometry_callback, 10)
        self.person_estimate_sub = self.create_subscription(
            PoseStamped, '/nmpc/person_estimate', self.person_estimate_callback, 10)
        self.actor_pose_sub = self.create_subscription(
            PoseStamped, '/actor/walking_person/pose', self.actor_pose_callback, 10)
        self.gazebo_poses_sub = self.create_subscription(
            PoseArray, '/gazebo/all_poses', self.gazebo_poses_callback, 10)
        self.nmpc_status_sub = self.create_subscription(
            Float64MultiArray, '/drone/controller/status', self.nmpc_status_callback, 10)

        # Timer for publishing trajectory
        self.timer = self.create_timer(1.0, self.publish_trajectory)

        # Note: Person position now comes directly from Gazebo actor via /actor/walking_person/pose

        # Data storage
        self.latest_person_detections = []
        self.drone_path = Path()
        self.drone_path.header.frame_id = "world"
        self.current_drone_position = [0.0, 0.0, 2.0]  # Default drone position

        # Trajectory parameters (circular orbit around person) - will be updated from NMPC
        self.desired_tracking_distance = 3.5  # Default preferred horizontal spacing
        self.current_tracking_distance = 0.0
        self.orbit_radius = self.desired_tracking_distance
        self.orbit_height = 3.0  # Default: TRACKING_FIXED_ALTITUDE from NMPC config
        self.tracking_altitude = 3.0  # Will be updated from NMPC status
        self.current_person_position = None  # Predicted position provided by NMPC
        self.actual_person_position = None   # True position from Gazebo actor
        self.person_estimate_available = False

        # Camera parameters (approximate)
        self.image_width = 640
        self.image_height = 480
        self.camera_fov_horizontal = 1.3962634  # ~80 degrees in radians

        self.get_logger().info('Visualization node started')

    def detection_callback(self, msg):
        """Process person detections and create position markers"""
        self.latest_person_detections = msg.detections
        self._publish_person_markers()

    def person_estimate_callback(self, msg: PoseStamped):
        """Receive NMPC-estimated person position in world frame."""
        self.current_person_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.person_estimate_available = True
        self._publish_person_markers()

    def odometry_callback(self, msg):
        """Update drone path with current position"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        
        # Update current drone position
        self.current_drone_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]
        
        self.drone_path.poses.append(pose)
        
        # Keep only last 100 poses to avoid memory issues
        if len(self.drone_path.poses) > 100:
            self.drone_path.poses.pop(0)
        
        self.drone_path.header.stamp = self.get_clock().now().to_msg()
        self.drone_path_pub.publish(self.drone_path)

        # Publish drone position marker
        self.publish_drone_marker()
    
    def actor_pose_callback(self, msg):
        """Update actual person position from Gazebo actor"""
        # msg is now PoseStamped, not PoseArray
        pose = msg.pose  # Extract Pose from PoseStamped
        self.actual_person_position = [
            pose.position.x,
            pose.position.y,
            pose.position.z
        ]
        self.get_logger().info(f'Updated actor position from Gazebo: x={pose.position.x:.2f}, y={pose.position.y:.2f}, z={pose.position.z:.2f}')
        self._publish_person_markers()
    
    def gazebo_poses_callback(self, msg):
        """Update actual person position from Gazebo all poses topic"""
        if msg.poses:
            # Find the walking person by looking for poses that move (not static models)
            # Usually models in Gazebo follow a pattern: ground_plane, building, then actors
            # Look for a pose that's at human height and not at origin
            found_person = False
            for i, pose in enumerate(msg.poses):
                # Skip static models (ground plane, buildings) - they're usually at z=0 or fixed positions
                # Look for entities at human height (0.8-1.2m) and not at origin
                if (0.8 <= pose.position.z <= 1.2 and
                    (abs(pose.position.x) > 0.1 or abs(pose.position.y) > 0.1)):

                    self.actual_person_position = [
                        pose.position.x,
                        pose.position.y,
                        pose.position.z
                    ]
                    self.get_logger().debug(f'Updated person position from all poses (entity {i}): x={pose.position.x:.2f}, y={pose.position.y:.2f}, z={pose.position.z:.2f}')
                    found_person = True
                    self._publish_person_markers()
                    break

            if not found_person:
                self.get_logger().debug('No person found in Gazebo poses')

    def nmpc_status_callback(self, msg):
        """Update tracking parameters from NMPC status"""
        # msg.data format: [person_detected, desired_distance, tracking_altitude, optimization_time, iterations_used, cost_value, current_distance]
        if len(msg.data) >= 3:
            self.desired_tracking_distance = msg.data[1]
            self.orbit_radius = self.desired_tracking_distance
            self.tracking_altitude = msg.data[2]
            # Update orbit_height to match NMPC's tracking altitude
            if self.current_person_position:
                self.orbit_height = self.tracking_altitude - self.current_person_position[2]
        if len(msg.data) >= 7:
            self.current_tracking_distance = msg.data[6]
        else:
            self.current_tracking_distance = self.desired_tracking_distance

    def _publish_person_markers(self):
        """Publish markers for predicted (NMPC) and actual person positions."""
        marker_array = MarkerArray()

        predicted_marker = self._create_predicted_person_marker()
        if predicted_marker is not None:
            marker_array.markers.append(predicted_marker)

        actual_marker = self._create_actual_person_marker()
        if actual_marker is not None:
            marker_array.markers.append(actual_marker)

        if marker_array.markers:
            self.person_marker_pub.publish(marker_array)

    def publish_trajectory(self):
        """Publish desired circular trajectory around person"""
        marker_array = MarkerArray()

        # Create circular trajectory markers
        num_points = 32
        if self.current_person_position is None:
            return
        person_x, person_y, person_z = self.current_person_position

        # Create trajectory circle
        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0

        # Set marker properties
        marker.scale.x = 0.1  # Line width
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 0.8

        # Generate circular points
        # Use tracking_altitude from NMPC instead of person_z + orbit_height
        for i in range(num_points + 1):  # +1 to close the circle
            angle = 2.0 * math.pi * i / num_points
            point = Point()
            point.x = person_x + self.orbit_radius * math.cos(angle)
            point.y = person_y + self.orbit_radius * math.sin(angle)
            point.z = self.tracking_altitude  # Use NMPC's fixed altitude
            marker.points.append(point)

        marker_array.markers.append(marker)

        # Add text marker showing desired vs current horizontal distance
        distance_marker = Marker()
        distance_marker.header.frame_id = "world"
        distance_marker.header.stamp = self.get_clock().now().to_msg()
        distance_marker.ns = "tracking_distance_text"
        distance_marker.id = 1
        distance_marker.type = Marker.TEXT_VIEW_FACING
        distance_marker.action = Marker.ADD
        distance_marker.pose.position.x = person_x
        distance_marker.pose.position.y = person_y
        distance_marker.pose.position.z = self.tracking_altitude + 0.5
        distance_marker.scale.z = 0.4
        distance_marker.color.r = 1.0
        distance_marker.color.g = 1.0
        distance_marker.color.b = 0.0
        distance_marker.color.a = 0.9
        distance_marker.text = (
            f"Desired Distance: {self.desired_tracking_distance:.2f} m\n"
            f"Current Distance: {self.current_tracking_distance:.2f} m"
        )
        marker_array.markers.append(distance_marker)

        self.trajectory_marker_pub.publish(marker_array)

    def publish_drone_marker(self):
        """Publish current drone position marker"""
        marker_array = MarkerArray()

        # Create drone position marker
        drone_marker = Marker()
        drone_marker.header.frame_id = "world"
        drone_marker.header.stamp = self.get_clock().now().to_msg()
        drone_marker.ns = "drone_position"
        drone_marker.id = 0
        drone_marker.type = Marker.MESH_RESOURCE
        drone_marker.action = Marker.ADD

        # Set drone position
        drone_marker.pose.position.x = self.current_drone_position[0]
        drone_marker.pose.position.y = self.current_drone_position[1]
        drone_marker.pose.position.z = self.current_drone_position[2]
        drone_marker.pose.orientation.w = 1.0

        # Use drone mesh if available, otherwise use a simple shape
        drone_marker.mesh_resource = "package://drone_description/meshes/drone.dae"
        drone_marker.mesh_use_embedded_materials = True

        # Set marker properties - GREEN for drone
        drone_marker.scale.x = 1.0
        drone_marker.scale.y = 1.0
        drone_marker.scale.z = 1.0
        drone_marker.color.r = 0.0
        drone_marker.color.g = 1.0
        drone_marker.color.b = 0.0
        drone_marker.color.a = 0.8

        marker_array.markers.append(drone_marker)

        # Also add a simpler arrow marker as fallback
        arrow_marker = Marker()
        arrow_marker.header.frame_id = "world"
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = "drone_arrow"
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        # Set arrow position (slightly above drone for visibility)
        arrow_marker.pose.position.x = self.current_drone_position[0]
        arrow_marker.pose.position.y = self.current_drone_position[1]
        arrow_marker.pose.position.z = self.current_drone_position[2] + 0.3
        arrow_marker.pose.orientation.w = 1.0

        # Set arrow properties - GREEN pointing up
        arrow_marker.scale.x = 0.8  # Length
        arrow_marker.scale.y = 0.1  # Width
        arrow_marker.scale.z = 0.1  # Height
        arrow_marker.color.r = 0.0
        arrow_marker.color.g = 1.0
        arrow_marker.color.b = 0.0
        arrow_marker.color.a = 1.0

        marker_array.markers.append(arrow_marker)

        self.drone_marker_pub.publish(marker_array)

    def _create_predicted_person_marker(self):
        if not self.person_estimate_available or self.current_person_position is None:
            return None

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "person_predicted"
        marker.id = 0
        marker.type = Marker.CYLINDER  # Changed from SPHERE to CYLINDER
        marker.action = Marker.ADD

        # Cylinder position: center should be at half height above ground
        marker.pose.position.x = float(self.current_person_position[0])
        marker.pose.position.y = float(self.current_person_position[1])
        marker.pose.position.z = 0.85  # Half of cylinder height (1.7/2) to place bottom at ground
        marker.pose.orientation.w = 1.0

        # Cylinder dimensions (similar to human shape)
        marker.scale.x = 0.5  # diameter
        marker.scale.y = 0.5  # diameter
        marker.scale.z = 1.7  # height (typical human height)

        # Yellow color instead of red
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7

        return marker

    def _create_actual_person_marker(self) -> Marker:
        if self.actual_person_position is None:
            return None

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "person_actual"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        # Cylinder position: center should be at half height above ground
        marker.pose.position.x = float(self.actual_person_position[0])
        marker.pose.position.y = float(self.actual_person_position[1])
        marker.pose.position.z = 0.9  # Half of cylinder height (1.8/2) to place bottom at ground
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 1.8
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.45

        return marker


def main(args=None):
    rclpy.init(args=args)
    node = VisualizationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
