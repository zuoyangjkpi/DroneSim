#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, PoseArray
from neural_network_msgs.msg import NeuralNetworkDetectionArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64MultiArray


class VisualizationNode(Node):
    def __init__(self):
        super().__init__('visualization_node')

        self.person_marker_pub = self.create_publisher(MarkerArray, '/person_position_markers', 10)
        self.trajectory_marker_pub = self.create_publisher(MarkerArray, '/drone_trajectory_markers', 10)
        self.drone_path_pub = self.create_publisher(Path, '/drone_path', 10)
        self.drone_marker_pub = self.create_publisher(MarkerArray, '/drone_position_markers', 10)

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

        self.timer = self.create_timer(1.0, self.publish_trajectory)

        self.latest_person_detections = []
        self.drone_path = Path()
        self.drone_path.header.frame_id = "world"
        self.current_drone_position = [0.0, 0.0, 2.0]

        self.desired_tracking_distance = 3.5
        self.current_tracking_distance = 0.0
        self.orbit_radius = self.desired_tracking_distance
        self.orbit_height = 3.0
        self.tracking_altitude = 3.0
        self.current_person_position = None
        self.actual_person_position = None
        self.person_estimate_available = False

        self.image_width = 640
        self.image_height = 480
        self.camera_fov_horizontal = 1.3962634

        self.get_logger().info('Visualization node started')

    def detection_callback(self, msg: NeuralNetworkDetectionArray):
        self.latest_person_detections = msg.detections
        self._publish_person_markers()

    def person_estimate_callback(self, msg: PoseStamped):
        self.current_person_position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        self.person_estimate_available = True
        self._publish_person_markers()

    def odometry_callback(self, msg: Odometry):
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose

        self.current_drone_position = [
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z
        ]

        self.drone_path.poses.append(pose)
        if len(self.drone_path.poses) > 100:
            self.drone_path.poses.pop(0)

        self.drone_path.header.stamp = self.get_clock().now().to_msg()
        self.drone_path_pub.publish(self.drone_path)

        self.publish_drone_marker()

    def actor_pose_callback(self, msg: PoseStamped):
        pose = msg.pose
        self.actual_person_position = [
            pose.position.x,
            pose.position.y,
            pose.position.z
        ]
        self._publish_person_markers()

    def gazebo_poses_callback(self, msg: PoseArray):
        # Kept for compatibility; no-op in current pipeline
        pass

    def nmpc_status_callback(self, msg: Float64MultiArray):
        data = msg.data
        if len(data) >= 3:
            self.tracking_altitude = data[0]
            self.current_tracking_distance = data[1]
            self.orbit_radius = data[2]

    def _publish_person_markers(self):
        marker_array = MarkerArray()

        predicted_marker = self._create_predicted_person_marker()
        actual_marker = self._create_actual_person_marker()

        if predicted_marker is not None:
            marker_array.markers.append(predicted_marker)
        if actual_marker is not None:
            marker_array.markers.append(actual_marker)

        for m in marker_array.markers:
            m.lifetime.sec = 1

        if marker_array.markers:
            self.person_marker_pub.publish(marker_array)

    def publish_trajectory(self):
        if self.current_person_position is None:
            return

        marker_array = MarkerArray()

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "drone_trajectory"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.05
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 0.9

        marker_array.markers.append(marker)
        self.trajectory_marker_pub.publish(marker_array)

    def publish_drone_marker(self):
        marker_array = MarkerArray()

        drone_marker = Marker()
        drone_marker.header.frame_id = "world"
        drone_marker.header.stamp = self.get_clock().now().to_msg()
        drone_marker.ns = "drone_position"
        drone_marker.id = 0
        drone_marker.type = Marker.MESH_RESOURCE
        drone_marker.action = Marker.ADD

        drone_marker.pose.position.x = self.current_drone_position[0]
        drone_marker.pose.position.y = self.current_drone_position[1]
        drone_marker.pose.position.z = self.current_drone_position[2]
        drone_marker.pose.orientation.w = 1.0

        drone_marker.mesh_resource = "package://drone_description/meshes/drone.dae"
        drone_marker.mesh_use_embedded_materials = True

        drone_marker.scale.x = 1.0
        drone_marker.scale.y = 1.0
        drone_marker.scale.z = 1.0
        drone_marker.color.r = 0.0
        drone_marker.color.g = 1.0
        drone_marker.color.b = 0.0
        drone_marker.color.a = 0.8

        marker_array.markers.append(drone_marker)

        arrow_marker = Marker()
        arrow_marker.header.frame_id = "world"
        arrow_marker.header.stamp = self.get_clock().now().to_msg()
        arrow_marker.ns = "drone_arrow"
        arrow_marker.id = 1
        arrow_marker.type = Marker.ARROW
        arrow_marker.action = Marker.ADD

        arrow_marker.pose.position.x = self.current_drone_position[0]
        arrow_marker.pose.position.y = self.current_drone_position[1]
        arrow_marker.pose.position.z = self.current_drone_position[2] + 0.3
        arrow_marker.pose.orientation.w = 1.0

        arrow_marker.scale.x = 0.8
        arrow_marker.scale.y = 0.1
        arrow_marker.scale.z = 0.1
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
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.current_person_position[0])
        marker.pose.position.y = float(self.current_person_position[1])
        marker.pose.position.z = 0.85
        marker.pose.orientation.w = 1.0

        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 1.7
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.7

        return marker

    def _create_actual_person_marker(self):
        if self.actual_person_position is None:
            return None

        marker = Marker()
        marker.header.frame_id = "world"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "person_actual"
        marker.id = 1
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position.x = float(self.actual_person_position[0])
        marker.pose.position.y = float(self.actual_person_position[1])
        marker.pose.position.z = 0.9
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
