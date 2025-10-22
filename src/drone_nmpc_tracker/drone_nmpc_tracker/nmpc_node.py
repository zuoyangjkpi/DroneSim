#!/usr/bin/env python3
"""
ROS2 Node for NMPC Drone Person Tracking
Adapted for ROS2 Jazzy and Python 3.12
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, qos_profile_sensor_data
from rclpy.duration import Duration
from rclpy.time import Time
import numpy as np
import math
from typing import Optional
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException

# ROS2 message types
from geometry_msgs.msg import Twist, PoseStamped, Vector3Stamped, TwistStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float64MultiArray, Bool
from visualization_msgs.msg import Marker, MarkerArray
from neural_network_msgs.msg import NeuralNetworkDetectionArray

from .nmpc_controller import DroneNMPCController
from .config import nmpc_config

class NMPCTrackerNode(Node):
    """ROS2 Node for NMPC-based drone person tracking"""
    
    def __init__(self):
        super().__init__('nmpc_tracker_node')
        
        # Initialize NMPC controller
        self.controller = DroneNMPCController()
        
        # Node state
        self.drone_state_received = False
        self.person_detected = False
        self.last_person_detection_time = 0.0
        self.control_enabled = True

        # Lost target handling - adaptive strategy based on loss history
        self.lost_count = 0  # Track how many times target was lost
        self.successful_track_duration = 0.0  # Track how long we successfully tracked
        self.last_successful_track_start = 0.0  # When did successful tracking start

        # Vehicle references
        self.home_position: Optional[np.ndarray] = None
        self.home_yaw: float = 0.0
        self.last_tracking_yaw: float = 0.0
        self.last_known_position: Optional[np.ndarray] = None
        self.takeoff_target_altitude: Optional[float] = None

        # State machine management
        self.state: Optional[str] = None
        self.state_enter_time: float = 0.0
        self.takeoff_alt_reached_time: Optional[float] = None
        self.search_reference_position: Optional[np.ndarray] = None
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize ROS2 interfaces
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()
        self._switch_state('TAKEOFF')
        
        self.get_logger().info("NMPC Tracker Node initialized")
    
    def _init_parameters(self):
        """Initialize ROS2 parameters"""
        # Declare parameters with default values
        self.declare_parameter('control_frequency', 10.0)
        self.declare_parameter('person_timeout', 10.0)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('drone_frame', 'X3/base_link')
        self.declare_parameter('world_frame', 'world')
        self.declare_parameter('camera_frame', 'X3/camera_link')
        self.declare_parameter('camera_optical_frame', 'X3/camera_rgb_optical_frame')
        # Align with projection_model output topic started in the integration script
        self.declare_parameter('projected_detection_topic', '/person_detections/world_frame')
        self.declare_parameter('detection_topic', '/person_detections')
        self.declare_parameter('detection_score_threshold', 0.5)
        self.declare_parameter('camera_image_width', 640)
        self.declare_parameter('camera_image_height', 480)
        self.declare_parameter('camera_fov_horizontal', math.radians(80.0))
        self.declare_parameter('camera_fov_vertical', math.radians(60.0))
        self.declare_parameter('person_anchor_height', 1.7)
        self.declare_parameter('takeoff_altitude', 3.0)
        self.declare_parameter('takeoff_ascent_rate', 4.0)
        self.declare_parameter('takeoff_hold_duration', 10.0)
        self.declare_parameter('takeoff_min_stable_time', 1.2)
        self.declare_parameter('lost_target_hold_duration', 10.0)
        self.declare_parameter('altitude_tolerance', 0.05)
        self.declare_parameter('search_yaw_rate', 0.05)
        self.declare_parameter('tracking_phase_offset', 0.0)
        self.declare_parameter('tracking_height_offset', nmpc_config.TRACKING_HEIGHT_OFFSET)
        self.declare_parameter('person_position_filter_alpha', nmpc_config.PERSON_POSITION_FILTER_ALPHA)
        self.declare_parameter('track_detection_confirmations', 3)
        # Get parameter values
        self.control_frequency = self.get_parameter('control_frequency').value
        self.person_timeout = self.get_parameter('person_timeout').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.drone_frame = self.get_parameter('drone_frame').value
        self.world_frame = self.get_parameter('world_frame').value
        self.camera_frame = self.get_parameter('camera_frame').value
        self.camera_optical_frame = self.get_parameter('camera_optical_frame').value
        self.projected_detection_topic = self.get_parameter('projected_detection_topic').value
        self.detection_topic = self.get_parameter('detection_topic').value
        self.detection_score_threshold = float(self.get_parameter('detection_score_threshold').value)
        self.camera_image_width = int(self.get_parameter('camera_image_width').value)
        self.camera_image_height = int(self.get_parameter('camera_image_height').value)
        self.camera_fov_horizontal = float(self.get_parameter('camera_fov_horizontal').value)
        self.camera_fov_vertical = float(self.get_parameter('camera_fov_vertical').value)
        self.person_anchor_height = float(self.get_parameter('person_anchor_height').value)
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').value
        self.takeoff_ascent_rate = float(self.get_parameter('takeoff_ascent_rate').value)
        self.takeoff_hold_duration = self.get_parameter('takeoff_hold_duration').value
        self.takeoff_min_stable_time = float(self.get_parameter('takeoff_min_stable_time').value)
        self.lost_target_hold_duration = self.get_parameter('lost_target_hold_duration').value
        self.altitude_tolerance = max(0.01, self.get_parameter('altitude_tolerance').value)
        self.search_yaw_rate = self.get_parameter('search_yaw_rate').value
        self.tracking_phase_offset = self.get_parameter('tracking_phase_offset').value
        self.tracking_height_offset = self.get_parameter('tracking_height_offset').value
        self.person_position_filter_alpha = float(self.get_parameter('person_position_filter_alpha').value)
        self.controller.set_tracking_height_offset(self.tracking_height_offset)
        self.fixed_tracking_altitude = nmpc_config.TRACKING_FIXED_ALTITUDE
        self.controller.set_fixed_altitude(self.fixed_tracking_altitude)
        self.required_detection_confirmations = max(1, int(self.get_parameter('track_detection_confirmations').value))

        self._filtered_person_position = None
        self._detection_streak = 0
        self._pending_phase_init = None

        # 光学坐标系（REP 103）到相机机体系（x 前、y 左、z 上）的转换矩阵及逆矩阵
        self._rot_optical_to_camera = np.array([
            [0.0, 0.0, 1.0],
            [-1.0, 0.0, 0.0],
            [0.0, -1.0, 0.0]
        ], dtype=np.float64)

        self.get_logger().info(f"Control frequency: {self.control_frequency} Hz")
        self.get_logger().info(f"Person timeout: {self.person_timeout} seconds")
    
    def _init_publishers(self):
        """Initialize ROS2 publishers"""
        # QoS profile for control commands (reliable, low latency)
        control_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS profile for visualization (best effort, higher depth)
        viz_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Control command publishers for low-level controllers
        self.waypoint_pub = self.create_publisher(
            PoseStamped, 
            '/drone/control/waypoint_command', 
            control_qos
        )
        
        self.attitude_pub = self.create_publisher(
            Vector3Stamped, 
            '/drone/control/attitude_command', 
            control_qos
        )
        
        
        # Control enable/disable publishers
        self.waypoint_enable_pub = self.create_publisher(
            Bool,
            '/drone/control/waypoint_enable',
            control_qos
        )
        
        self.attitude_enable_pub = self.create_publisher(
            Bool,
            '/drone/control/attitude_enable',
            control_qos
        )

        self.velocity_enable_pub = self.create_publisher(
            Bool,
            '/drone/control/velocity_enable',
            control_qos
        )


        # Status publisher
        self.status_pub = self.create_publisher(
            Float64MultiArray,
            nmpc_config.TOPIC_STATUS,
            control_qos
        )

        self.person_estimate_pub = self.create_publisher(
            PoseStamped,
            '/nmpc/person_estimate',
            control_qos
        )
        self.get_logger().info("Person estimate publisher initialized on /nmpc/person_estimate")

        # Visualization publishers
        if self.enable_visualization:
            self.trajectory_pub = self.create_publisher(
                MarkerArray,
                nmpc_config.TOPIC_TRAJECTORY_VIS,
                viz_qos
            )
            
            self.target_pub = self.create_publisher(
                Marker,
                nmpc_config.TOPIC_TARGET_VIS,
                viz_qos
            )
        
        self.get_logger().info("Publishers initialized")
    
    def _init_subscribers(self):
        """Initialize ROS2 subscribers"""
        # Use ROS2 standard sensor QoS preset
        sensor_qos = qos_profile_sensor_data
        
        # Drone state subscriber (odometry)
        self.odom_sub = self.create_subscription(
            Odometry,
            nmpc_config.TOPIC_DRONE_STATE,
            self.drone_state_callback,
            sensor_qos
        )
        
        # Person detection subscriber (using projected detections from projection_model)
        detection_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.detection_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            self.projected_detection_topic,
            self.projected_detection_callback,
            detection_qos
        )

        self.detection_array_sub = self.create_subscription(
            NeuralNetworkDetectionArray,
            self.detection_topic,
            self.detection_array_callback,
            detection_qos
        )

        # Control enable/disable subscriber
        self.enable_sub = self.create_subscription(
            Bool,
            '/nmpc/enable',
            self.enable_callback,
            sensor_qos  # Use sensor_qos for consistency
        )

        self.get_logger().info("Subscribers initialized")
    
    def _init_timers(self):
        """Initialize ROS2 timers"""
        # Main control loop timer
        control_period = 1.0 / self.control_frequency
        self.control_timer = self.create_timer(
            control_period,
            self.control_loop_callback
        )
        
        # Status publishing timer (lower frequency)
        self.status_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publish_status
        )
        
        self.get_logger().info(f"Control loop running at {self.control_frequency} Hz")
    
    def drone_state_callback(self, msg: Odometry):
        """Process drone state from odometry"""
        try:
            # Extract position
            position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])
            
            # Extract velocity
            velocity = np.array([
                msg.twist.twist.linear.x,
                msg.twist.twist.linear.y,
                msg.twist.twist.linear.z
            ])
            
            # Extract orientation (quaternion to Euler)
            q = msg.pose.pose.orientation
            orientation = self._quaternion_to_euler(q.x, q.y, q.z, q.w)
            
            # Extract angular velocity
            angular_velocity = np.array([
                msg.twist.twist.angular.x,
                msg.twist.twist.angular.y,
                msg.twist.twist.angular.z
            ])
            
            # Update controller state
            self.controller.set_drone_state(position, velocity, orientation, angular_velocity)
            self.drone_state_received = True
            if self.home_position is None:
                self.home_position = position.copy()
                self.home_yaw = orientation[2]
                # Only climb if current altitude is below the desired takeoff altitude
                self.takeoff_target_altitude = max(self.takeoff_altitude, self.home_position[2])
                self.last_tracking_yaw = self.home_yaw
            
            # ✅ 添加调试信息 - 降低频率
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 1

            if self._debug_counter % 50 == 0:  # 每50次更新才打印一次
                self.get_logger().info(f"Drone state: pos=[{position[0]:.2f}, {position[1]:.2f}, {position[2]:.2f}], vel=[{velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f}]")
            
        except Exception as e:
            self.get_logger().error(f"Error processing drone state: {e}")
    
    def projected_detection_callback(self, msg: PoseWithCovarianceStamped):
        """Process projected person detection messages from projection_model."""
        try:
            current_time = self.get_clock().now().nanoseconds / 1e9

            # Extract person position from projected detection
            person_position = np.array([
                msg.pose.pose.position.x,
                msg.pose.pose.position.y,
                msg.pose.pose.position.z
            ])

            # Estimate velocity (simple finite difference)
            filtered_position = self._filter_person_position(person_position)
            person_velocity = self._estimate_person_velocity(filtered_position, current_time)

            # Update controller
            self.controller.set_person_detection(
                filtered_position,
                person_velocity,
                detection_time=current_time,
                allow_phase_change=(self.state == 'TRACK'),
            )
            self._detection_streak += 1
            self._handle_person_detection(filtered_position, person_velocity)

        except Exception as e:
            self.get_logger().error(f"Error processing projected person detection: {e}")

    def detection_array_callback(self, msg: NeuralNetworkDetectionArray):
        """Fallback projection when world-frame detections are not available."""
        detection_count = len(msg.detections)
        if detection_count == 0:
            return
        self.get_logger().debug(f"Received {detection_count} raw detections")

        best_detection = max(msg.detections, key=lambda det: det.detection_score)
        if best_detection.detection_score < self.detection_score_threshold:
            self.get_logger().debug(
                f"Detection score {best_detection.detection_score:.2f} below threshold {self.detection_score_threshold:.2f}"
            )
            return

        person_position = self._project_detection_to_world(best_detection, msg.header)
        if person_position is None:
            self.get_logger().debug("Unable to project detection to world frame")
            return

        timestamp = self._stamp_to_float(msg.header.stamp)
        filtered_position = self._filter_person_position(person_position)
        person_velocity = self._estimate_person_velocity(filtered_position, timestamp)

        self.controller.set_person_detection(
            filtered_position,
            person_velocity,
            detection_time=timestamp,
            allow_phase_change=(self.state == 'TRACK'),
        )
        self._detection_streak += 1
        self._handle_person_detection(filtered_position, person_velocity)
    
    def _filter_person_position(self, position: np.ndarray) -> np.ndarray:
        if self._filtered_person_position is None:
            self._filtered_person_position = position.copy()
        else:
            alpha = float(np.clip(self.person_position_filter_alpha, 0.0, 1.0))
            self._filtered_person_position = (alpha * position +
                (1.0 - alpha) * self._filtered_person_position)
        return self._filtered_person_position.copy()

    def _estimate_person_velocity(self, position: np.ndarray, timestamp: float) -> np.ndarray:
        """Estimate person velocity using finite differences"""
        # Simple velocity estimation - in practice, you might use a Kalman filter
        if hasattr(self, '_last_person_position') and hasattr(self, '_last_person_time'):
            dt = timestamp - self._last_person_time
            if dt > 0.01:  # Avoid very small time differences
                velocity = (position - self._last_person_position) / dt
                # Smooth velocity estimate
                if hasattr(self, '_person_velocity_estimate'):
                    alpha = 0.3  # Smoothing factor
                    velocity = alpha * velocity + (1 - alpha) * self._person_velocity_estimate
                self._person_velocity_estimate = velocity
            else:
                velocity = getattr(self, '_person_velocity_estimate', np.zeros(3))
        else:
            velocity = np.zeros(3)
        
        # Store for next iteration
        self._last_person_position = position.copy()
        self._last_person_time = timestamp
        
        return velocity

    def _project_detection_to_world(self, detection, header: Header) -> Optional[np.ndarray]:
        """Transform a 2D detection into a 3D world point using TF and camera intrinsics."""
        if not self.drone_state_received:
            return None

        try:
            lookup_time = Time.from_msg(header.stamp)
        except Exception:
            lookup_time = self.get_clock().now()

        frame_candidates = []
        frame_id = header.frame_id or ""
        if frame_id:
            frame_candidates.append(frame_id)
            if not frame_id.endswith('_optical_frame'):
                frame_candidates.append(frame_id + '_optical_frame')

        frame_candidates.extend([
            self.camera_optical_frame,
            self.camera_frame
        ])

        transform = None
        used_frame = None
        last_error = None
        for candidate in frame_candidates:
            try:
                transform = self.tf_buffer.lookup_transform(
                    self.world_frame,
                    candidate,
                    lookup_time,
                    timeout=Duration(seconds=0.2)
                )
                if transform:
                    used_frame = candidate
                    break
            except TransformException as exc:
                last_error = exc
                # 若因为时间外推失败，尝试使用最近可用的TF
                if 'future' in str(exc).lower() or 'extrapolation' in str(exc).lower():
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            self.world_frame,
                            candidate,
                            Time(),  # 最新可用 TF
                            timeout=Duration(seconds=0.2)
                        )
                        if transform:
                            used_frame = candidate
                            self.get_logger().warn(
                                f"TF extrapolation for {candidate} (requested {lookup_time.nanoseconds * 1e-9:.3f}s);"
                                " 改用最新可用的 TF 数据"
                            )
                            break
                    except TransformException as exc_latest:
                        last_error = exc_latest
                continue

        if transform is None:
            if last_error is not None:
                self.get_logger().warn(
                    f"TF lookup failed for detection projection (frames={frame_candidates}): {last_error}"
                )
            else:
                self.get_logger().warn(
                    f"TF lookup failed for detection projection (frames={frame_candidates})"
                )
            return None

        width = max(1, self.camera_image_width)
        height = max(1, self.camera_image_height)
        fx = width / (2.0 * math.tan(self.camera_fov_horizontal / 2.0))
        fy = height / (2.0 * math.tan(self.camera_fov_vertical / 2.0))
        cx = width / 2.0
        cy = height / 2.0

        xmin = detection.xmin
        xmax = detection.xmax
        ymin = detection.ymin
        ymax = detection.ymax

        # Support detectors that publish normalized bounding boxes in [0, 1]
        if max(abs(xmin), abs(xmax), abs(ymin), abs(ymax)) <= 1.5:
            xmin *= width
            xmax *= width
            ymin *= height
            ymax *= height

        center_x = 0.5 * (xmin + xmax)
        center_y = 0.5 * (ymin + ymax)

        x_norm = (center_x - cx) / fx
        y_norm = (center_y - cy) / fy

        ray_cam = np.array([x_norm, y_norm, 1.0], dtype=np.float64)
        ray_cam_norm = np.linalg.norm(ray_cam)
        if ray_cam_norm < 1e-6:
            return None
        ray_cam /= ray_cam_norm

        rotation = self._quaternion_to_matrix(
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w
        )
        if used_frame and not used_frame.endswith('optical_frame'):
            rotation = rotation @ self._rot_optical_to_camera
        ray_world = rotation @ ray_cam
        ray_world_norm = np.linalg.norm(ray_world)
        if ray_world_norm < 1e-6:
            self.get_logger().warn("Projected ray has near-zero norm after rotation")
            return None
        ray_world /= ray_world_norm

        # Debug: Check if ray direction makes sense for downward-tilted camera
        if ray_world[2] > 0:
            self.get_logger().warn(
                f"⚠️  射线方向向上 ray_world[2]={ray_world[2]:.3f} > 0，这对于向下倾斜的相机是异常的\n"
                f"   📐 相机光学坐标系射线: [{ray_cam[0]:.3f}, {ray_cam[1]:.3f}, {ray_cam[2]:.3f}]\n"
                f"   🔄 旋转后世界坐标系射线: [{ray_world[0]:.3f}, {ray_world[1]:.3f}, {ray_world[2]:.3f}]"
            )

        if abs(ray_world[2]) < 1e-6:
            self.get_logger().warn('Detection ray parallel to ground plane')
            return None

        camera_position = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z
        ], dtype=np.float64)

        anchor_height = self.person_anchor_height
        height_diff = anchor_height - camera_position[2]
        t = height_diff / ray_world[2]

        # Handle invalid geometry cases
        if t <= 0.0:
            # For downward-tilted camera, if ray is pointing up (ray_world[2] > 0)
            # but target is below camera (height_diff < 0), this is a coordinate system issue
            if ray_world[2] > 0 and height_diff < 0:
                self.get_logger().warn(
                    f"🔧 检测到坐标系异常，尝试修正射线方向 (t={t:.3f})\n"
                    f"   📍 相机位置: [{camera_position[0]:.2f}, {camera_position[1]:.2f}, {camera_position[2]:.2f}]\n"
                    f"   📏 人员锚定高度: {anchor_height:.2f}m\n"
                    f"   ➡️  原射线方向: [{ray_world[0]:.3f}, {ray_world[1]:.3f}, {ray_world[2]:.3f}]"
                )
                # Force ray to point downward for tilted camera
                ray_world[2] = -abs(ray_world[2])
                t = height_diff / ray_world[2]
                self.get_logger().info(f"   ✅ 修正后射线方向: [{ray_world[0]:.3f}, {ray_world[1]:.3f}, {ray_world[2]:.3f}], t={t:.3f}")
            else:
                self.get_logger().warn(
                    f"❌ 几何计算无效: t={t:.3f} <= 0\n"
                    f"   📍 相机位置: [{camera_position[0]:.2f}, {camera_position[1]:.2f}, {camera_position[2]:.2f}]\n"
                    f"   📏 人员锚定高度: {anchor_height:.2f}m\n"
                    f"   ➡️  射线方向: [{ray_world[0]:.3f}, {ray_world[1]:.3f}, {ray_world[2]:.3f}]\n"
                    f"   📐 分子: {height_diff:.3f}, 分母: {ray_world[2]:.3f}"
                )
                return None

        # Final check after potential correction
        if t <= 0.0:
            return None

        world_point = camera_position + t * ray_world
        world_point[2] = anchor_height
        return world_point

    def _stamp_to_float(self, stamp) -> float:
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _quaternion_to_matrix(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        norm = math.sqrt(x * x + y * y + z * z + w * w)
        if norm == 0.0:
            return np.eye(3)
        x /= norm
        y /= norm
        z /= norm
        w /= norm

        xx = x * x
        yy = y * y
        zz = z * z
        xy = x * y
        xz = x * z
        yz = y * z
        wx = w * x
        wy = w * y
        wz = w * z

        return np.array([
            [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
            [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
            [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]
        ], dtype=np.float64)
    
    def _handle_person_detection(self, person_position: np.ndarray, person_velocity: np.ndarray):
        """Common logic when a person detection is received"""
        current_time = self._now()
        first_detection = not self.person_detected
        self.person_detected = True
        self.last_person_detection_time = current_time
        if first_detection:
            self.get_logger().info(f"✅ Person detected at {person_position}")
        # 计算并存储用于初始化的期望相位角
        if self.state != 'TRACK':
            current_pos = self._get_current_position()
            rel = person_position[:2] - current_pos[:2]
            if np.linalg.norm(rel) > 1e-3:
                self._pending_phase_init = math.atan2(rel[1], rel[0])
            else:
                self._pending_phase_init = self._get_current_yaw()
        # 检查是否已确认足够次数的检测
        if self._detection_streak < self.required_detection_confirmations:
            self._publish_person_estimate(person_position)
            self.person_detected = False
            return

        self.person_detected = True
        self._publish_person_estimate(person_position)

        # ⚠️ 关键安全检查：只有在完成起飞阶段后才能切换到TRACK状态
        # 避免起飞过程中因检测到人而导致的不稳定跟踪行为
        if self.state != 'TRACK' and self.state != 'TAKEOFF':
            # 只有当不在起飞状态时才允许立即切换到TRACK
            self.get_logger().info("🎯 非起飞状态下检测到人员，立即切换到TRACK模式")
            self._switch_state('TRACK')
        elif self.state == 'TAKEOFF':
            # 如果在TAKEOFF状态，检测到的人员信息会被保存，
            # 等待起飞完成后在_step_takeoff()中安全切换到TRACK
            self.get_logger().info(f"🚁 起飞阶段检测到人员 (连续检测: {self._detection_streak}次)，等待起飞完成后切换到跟踪模式")

    def _publish_person_estimate(self, position: np.ndarray) -> None:
        """Publish NMPC-estimated person position for downstream consumers."""
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.world_frame
        msg.pose.position.x = float(position[0])
        msg.pose.position.y = float(position[1])
        msg.pose.position.z = float(position[2])
        msg.pose.orientation.w = 1.0
        self.person_estimate_pub.publish(msg)
        self.get_logger().info(
            f"Published person estimate at {position.tolist()}"
        )

    def enable_callback(self, msg: Bool):
        """Handle control enable/disable commands"""
        self.control_enabled = msg.data
        current_mode = self.state if self.state is not None else "UNKNOWN"
        if self.control_enabled:
            self.get_logger().info(f"Control enabled, mode: {current_mode}")
        else:
            self.get_logger().info(f"Control disabled, mode: {current_mode}")
        
        # Enable/disable low-level controllers
        enable_msg = Bool()
        enable_msg.data = self.control_enabled
        
        self.waypoint_enable_pub.publish(enable_msg)
        self.attitude_enable_pub.publish(enable_msg)
        self.velocity_enable_pub.publish(enable_msg)
        if not self.control_enabled:
            return
    
    def control_loop_callback(self):
        """Main control loop"""
        current_time = self._now()

        if not self.control_enabled:
            self.get_logger().warn("Control not enabled")
            return

        if not self.drone_state_received:
            self.get_logger().warn("No drone state received - waiting for odometry data")
            return

        if self.person_detected and current_time - self.last_person_detection_time > self.person_timeout:
            self.person_detected = False
            self.controller.clear_detection()
            self._filtered_person_position = None
            self._detection_streak = 0
            self.get_logger().warn("Person detection timeout - entering hold state")
            if self.state == 'TRACK':
                self._enter_lost_hold()

        if self.state == 'TAKEOFF':
            self._step_takeoff(current_time)
            return

        if self.state == 'SEARCH':
            # In SEARCH mode, be more aggressive about switching to TRACK
            # Check for both confirmed detection OR recent detection streak
            if self.person_detected or (self._detection_streak >= 2 and
                                       current_time - self.last_person_detection_time < 1.0):
                self.get_logger().info(f"🎯 SEARCH模式检测到人员，切换到TRACK模式 (连续检测: {self._detection_streak}次)")
                self._switch_state('TRACK')
            else:
                self._send_search_commands(current_time)
                return

        if self.state == 'LOST_HOLD':
            if self.person_detected:
                # Reset loss count on successful re-acquisition
                self.get_logger().info(f"✅ 在HOLD期间重新找到目标，重置丢失计数")
                self.lost_count = max(0, self.lost_count - 1)  # Reward quick recovery
                self._switch_state('TRACK')
            else:
                self._send_lost_hold_command()
                # Use adaptive hold duration instead of fixed duration
                hold_duration = getattr(self, 'current_hold_duration', self.lost_target_hold_duration)
                if current_time - self.state_enter_time >= hold_duration:
                    self.get_logger().info(f"⏰ HOLD超时 ({hold_duration}秒)，切换到搜索模式")
                    self._switch_state('SEARCH')
                return

        if self.state == 'TRACK':
            if not self.person_detected:
                self._enter_lost_hold()
                return

            # Reset lost count after successful tracking for some time (reward stability)
            if self.last_successful_track_start > 0:
                successful_duration = current_time - self.last_successful_track_start
                if successful_duration > 30.0 and self.lost_count > 0:  # 30 seconds of successful tracking
                    self.get_logger().info(f"🎯 成功跟踪{successful_duration:.1f}秒，重置丢失计数 ({self.lost_count} → 0)")
                    self.lost_count = 0

            self._perform_tracking()
            return

        # Fallback: ensure we always have a valid control mode
        self._switch_state('SEARCH')

    def _perform_tracking(self):
        """Run NMPC optimization and push the resulting commands downstream."""
        if not self.person_detected:
            # 在TRACK模式下若丢失目标，立即切换由上层逻辑处理
            self.get_logger().warn('TRACK模式下未检测到目标，忽略跟踪命令')
            return

        # Advance orbit target only during active tracking
        self.controller.advance_tracking_target(
            self._now(),
            allow_phase_change=True,
        )

        try:
            control, info = self.controller.optimize()
        except Exception as exc:  # 捕捉优化失败，避免节点崩溃
            self.get_logger().error(f'NMPC优化失败: {exc}')
            self._enter_lost_hold()
            return

        self._send_tracking_commands(control)

        if self.enable_visualization and info:
            self._publish_visualization(info)

    def _send_lost_hold_command(self):
        """在目标丢失时保持当前位置并维持最后的航向。"""
        if self.last_known_position is None:
            hold_position = self._get_current_position().copy()
        else:
            hold_position = self.last_known_position.copy()

        if np.isnan(hold_position).any():
            hold_position = self._get_current_position().copy()

        # 确保高度至少保持在起飞高度附近，防止意外下降
        target_altitude = self.takeoff_target_altitude or self.takeoff_altitude
        if target_altitude is not None:
            hold_position[2] = float(target_altitude)

        yaw = self.last_tracking_yaw if self.last_tracking_yaw is not None else self._get_current_yaw()

        self._publish_waypoint(hold_position, source='LOST_HOLD', yaw=yaw)
        self._publish_attitude(yaw)

    def _send_tracking_commands(self, control: np.ndarray):
        """Send tracking commands to low-level controllers"""
        # Extract target position from controller
        target_position = self.controller.target_position
        
        # Create and publish waypoint command
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = self.world_frame
        waypoint_msg.pose.position.x = float(target_position[0])
        waypoint_msg.pose.position.y = float(target_position[1])
        waypoint_msg.pose.position.z = float(target_position[2])
        waypoint_msg.pose.orientation.w = 1.0
        
        self.waypoint_pub.publish(waypoint_msg)

        # Retrieve desired attitude from NMPC output
        desired_attitude = getattr(self.controller, "target_attitude", None)

        if desired_attitude is not None and len(desired_attitude) >= 2:
            roll_cmd = float(np.clip(desired_attitude[0], -math.pi / 4, math.pi / 4))
            pitch_cmd = float(np.clip(desired_attitude[1], -math.pi / 4, math.pi / 4))
        else:
            roll_cmd = 0.0
            pitch_cmd = 0.0

        current_pos = self.controller.current_state.data[nmpc_config.STATE_X:nmpc_config.STATE_Z+1]
        if self.controller.person_detected:
            person_pos = self.controller.person_position
            to_person = person_pos - current_pos
            yaw_cmd = math.atan2(to_person[1], to_person[0])
        else:
            yaw_cmd = self.last_tracking_yaw if self.last_tracking_yaw is not None else self._get_current_yaw()

        yaw_cmd = math.atan2(math.sin(yaw_cmd), math.cos(yaw_cmd))

        # Create and publish attitude command
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = self.drone_frame
        attitude_msg.vector.x = roll_cmd
        attitude_msg.vector.y = pitch_cmd
        attitude_msg.vector.z = yaw_cmd
        
        self.attitude_pub.publish(attitude_msg)

        self.last_tracking_yaw = yaw_cmd
        self._log_waypoint_command(
            source="NMPC",
            position=target_position,
            velocity=None,
            yaw=yaw_cmd,
            control=control
        )

    def _send_search_commands(self, current_time: float):
        if self.search_reference_position is None:
            self.search_reference_position = self._get_current_position().copy()

        if self.takeoff_target_altitude is not None:
            target_altitude = self.takeoff_target_altitude
        else:
            target_altitude = self.takeoff_altitude

        base_yaw = self.home_yaw if self.home_position is not None else 0.0
        elapsed = max(0.0, current_time - self.state_enter_time)
        yaw_command = base_yaw + self.search_yaw_rate * elapsed

        waypoint = np.array([
            self.search_reference_position[0],
            self.search_reference_position[1],
            target_altitude
        ])
        self._publish_waypoint(waypoint, source="SEARCH", yaw=yaw_command)

        self._publish_attitude(yaw_command)

    def _get_current_position(self) -> np.ndarray:
        """获取当前无人机位置"""
        return self.controller.current_state.data[nmpc_config.STATE_X:nmpc_config.STATE_Z+1]

    def _get_current_yaw(self) -> float:
        return self.controller.current_state.data[nmpc_config.STATE_YAW]

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _publish_waypoint(
        self,
        position: np.ndarray,
        *,
        source: Optional[str] = None,
        yaw: Optional[float] = None,
        velocity: Optional[np.ndarray] = None,
        control: Optional[np.ndarray] = None,
    ) -> None:
        waypoint_msg = PoseStamped()
        waypoint_msg.header.stamp = self.get_clock().now().to_msg()
        waypoint_msg.header.frame_id = self.world_frame
        waypoint_msg.pose.position.x = float(position[0])
        waypoint_msg.pose.position.y = float(position[1])
        waypoint_msg.pose.position.z = float(position[2])
        waypoint_msg.pose.orientation.w = 1.0
        self.waypoint_pub.publish(waypoint_msg)
        if source is not None:
            self._log_waypoint_command(
                source=source,
                position=position,
                velocity=velocity,
                yaw=yaw,
                control=control,
            )

    def _publish_attitude(self, yaw: float, roll: float = 0.0, pitch: float = 0.0):
        attitude_msg = Vector3Stamped()
        attitude_msg.header.stamp = self.get_clock().now().to_msg()
        attitude_msg.header.frame_id = self.drone_frame
        attitude_msg.vector.x = float(roll)
        attitude_msg.vector.y = float(pitch)
        attitude_msg.vector.z = float(yaw)
        self.attitude_pub.publish(attitude_msg)
        if self.state is not None:
            att_vector = [round(float(roll), 3), round(float(pitch), 3), round(float(yaw), 3)]
            self.get_logger().info(
                f"Attitude command [{self.state}]: rpy={att_vector}"
            )

    def _log_waypoint_command(
        self,
        *,
        source: str,
        position: np.ndarray,
        velocity: Optional[np.ndarray] = None,
        yaw: Optional[float] = None,
        control: Optional[np.ndarray] = None,
    ) -> None:
        """Emit a structured log for outbound waypoint commands."""
        pos_fmt = np.round(np.asarray(position), 3).tolist()
        details = [f"pos={pos_fmt}"]

        if velocity is not None:
            vel_fmt = np.round(np.asarray(velocity), 3).tolist()
            details.append(f"vel={vel_fmt}")
        if yaw is not None:
            details.append(f"yaw={round(float(yaw), 3)}")
        if control is not None:
            control_fmt = np.round(np.asarray(control), 3).tolist()
            details.append(f"control={control_fmt}")

        self.get_logger().info(f"Waypoint command [{source}]: " + "; ".join(details))

    def _step_takeoff(self, current_time: float):
        if self.home_position is None or self.takeoff_target_altitude is None:
            return

        elapsed = max(0.0, current_time - self.state_enter_time)
        ramp_altitude = self.home_position[2] + self.takeoff_ascent_rate * elapsed
        target_altitude = min(self.takeoff_target_altitude, ramp_altitude)
        target_altitude = max(target_altitude, self.home_position[2])

        waypoint = np.array([
            self.home_position[0],
            self.home_position[1],
            target_altitude
        ])
        self._publish_waypoint(waypoint, source="TAKEOFF")
        self._publish_attitude(self.home_yaw)

        current_altitude = self._get_current_position()[2]

        # 🚁 关键改进：确保起飞高度达到后才能切换到TRACK
        if abs(current_altitude - self.takeoff_target_altitude) < self.altitude_tolerance:
            if self.takeoff_alt_reached_time is None:
                self.takeoff_alt_reached_time = current_time
                self.get_logger().info(f"✅ 起飞高度已达到: {current_altitude:.2f}m (目标: {self.takeoff_target_altitude:.2f}m)")

            # 只有在高度稳定达到后才允许切换到跟踪模式
            altitude_stable_duration = current_time - self.takeoff_alt_reached_time
            min_stable_time = max(0.5, self.takeoff_min_stable_time)

            if altitude_stable_duration >= min_stable_time:
                if self.person_detected and self._detection_streak >= self.required_detection_confirmations:
                    self.get_logger().info(f"🎯 起飞完成且检测到人员，切换到TRACK模式 (稳定时间: {altitude_stable_duration:.1f}s)")
                    self._switch_state('TRACK')
                    return
                elif not self.person_detected and altitude_stable_duration >= self.takeoff_hold_duration:
                    self.get_logger().info("🔍 起飞完成但未检测到人员，切换到SEARCH模式")
                    self._switch_state('SEARCH')
            else:
                # 高度刚达到，还在稳定中
                if self._detection_streak >= self.required_detection_confirmations:
                    remaining_time = min_stable_time - altitude_stable_duration
                    self.get_logger().info(f"⏳ 高度稳定中，{remaining_time:.1f}秒后可切换到跟踪模式")
        else:
            # 高度未达到，重置稳定时间
            self.takeoff_alt_reached_time = None

    def _enter_lost_hold(self):
        current_time = self.get_clock().now().nanoseconds / 1e9

        # Update loss statistics
        self.lost_count += 1
        if self.last_successful_track_start > 0:
            self.successful_track_duration = current_time - self.last_successful_track_start

        self.last_known_position = self._get_current_position().copy()
        self.last_tracking_yaw = self._get_current_yaw()
        self._detection_streak = 0
        self.controller.clear_detection()
        self._filtered_person_position = None

        # Adaptive lost hold strategy based on loss history
        if self.lost_count == 1:
            self.get_logger().info(f"🔍 第1次丢失目标，HOLD {self.lost_target_hold_duration}秒")
        elif self.lost_count <= 3:
            # Reduce hold time for frequent losses - maybe tracking is unstable
            reduced_hold_time = max(5.0, self.lost_target_hold_duration * 0.5)
            self.get_logger().info(f"⚠️ 第{self.lost_count}次丢失目标，缩短HOLD时间到{reduced_hold_time}秒 (跟踪可能不稳定)")
            # Temporarily reduce hold duration for this instance
            self.current_hold_duration = reduced_hold_time
        else:
            # After many losses, use shorter hold time and suggest more aggressive search
            very_short_hold_time = 3.0
            self.get_logger().warn(f"🚨 第{self.lost_count}次丢失目标，极短HOLD {very_short_hold_time}秒后立即搜索 (频繁丢失)")
            self.current_hold_duration = very_short_hold_time

        self._switch_state('LOST_HOLD')

    def _switch_state(self, new_state: str):
        previous = self.state
        if previous == new_state:
            return
        self.state = new_state
        self.state_enter_time = self._now()

        if new_state == 'TAKEOFF':
            self.takeoff_alt_reached_time = None
        if new_state == 'SEARCH':
            current_position = self._get_current_position()
            self.search_reference_position = current_position.copy()
            self.person_detected = False
            self.controller.clear_detection()
            self._filtered_person_position = None
        else:
            self.search_reference_position = None
        if new_state == 'TRACK':
            # Start tracking successful tracking duration
            current_time = self.get_clock().now().nanoseconds / 1e9
            self.last_successful_track_start = current_time

            # Log loss statistics for debugging
            if self.lost_count > 0:
                self.get_logger().info(f"📊 开始跟踪 (历史丢失次数: {self.lost_count})")

            # 初始相位使用当前位置，避免不必要的移动
            person_pos = self.controller.person_position
            current_pos = self._get_current_position()
            rel = current_pos[:2] - person_pos[:2]  # 从人指向无人机
            if np.linalg.norm(rel) > 1e-3:
                # 设置为当前位置的相位角度，保持在原地不动
                phase = math.atan2(rel[1], rel[0])
            else:
                # 如果距离很近，则使用当前航向
                phase = self._get_current_yaw()
            # 重置控制器的相位
            self.controller.reset_phase(phase)
            self.get_logger().info(f"✅ 初始化跟踪相位: {phase:.2f} rad ({math.degrees(phase):.1f}°)")
            # 应用高度偏移和固定高度设置
            self.controller.set_tracking_height_offset(self.tracking_height_offset)
            self.fixed_tracking_altitude = nmpc_config.TRACKING_FIXED_ALTITUDE
            self.controller.set_fixed_altitude(self.fixed_tracking_altitude)
            # 更新最后的跟踪航向
            self.last_tracking_yaw = phase
        if new_state == 'LOST_HOLD' and self.last_known_position is None:
            self.last_known_position = self._get_current_position().copy()
        if new_state != 'LOST_HOLD':
            self.last_known_position = None

        self.get_logger().info(f"状态切换: {previous} -> {new_state}")
    
    def _publish_visualization(self, info: dict):
        """Publish visualization markers"""
        try:
            # Publish trajectory
            if 'trajectory' in info:
                self._publish_trajectory_markers(info['trajectory'])
            
            # Publish target marker
            self._publish_target_marker()
            
        except Exception as e:
            self.get_logger().error(f"Error publishing visualization: {e}")
    
    def _publish_trajectory_markers(self, trajectory):
        """Publish trajectory as marker array"""
        marker_array = MarkerArray()
        
        for i, state in enumerate(trajectory):
            marker = Marker()
            marker.header.frame_id = self.world_frame
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = "nmpc_trajectory"
            marker.id = i
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            
            # Position
            marker.pose.position.x = float(state[nmpc_config.STATE_X])
            marker.pose.position.y = float(state[nmpc_config.STATE_Y])
            marker.pose.position.z = float(state[nmpc_config.STATE_Z])
            marker.pose.orientation.w = 1.0
            
            # Size
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.scale.z = 0.1
            
            # Color (fade along trajectory)
            alpha = 1.0 - (i / len(trajectory))
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            marker.color.a = alpha
            
            marker.lifetime = rclpy.duration.Duration(seconds=0.5).to_msg()
            
            marker_array.markers.append(marker)
        
        self.trajectory_pub.publish(marker_array)
    
    def _publish_target_marker(self):
        """Publish target position marker"""
        marker = Marker()
        marker.header.frame_id = self.world_frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "nmpc_target"
        marker.id = 0
        marker.type = Marker.ARROW
        marker.action = Marker.ADD
        
        # Position
        marker.pose.position.x = float(self.controller.target_position[0])
        marker.pose.position.y = float(self.controller.target_position[1])
        marker.pose.position.z = float(self.controller.target_position[2])
        marker.pose.orientation.w = 1.0
        
        # Size
        marker.scale.x = 0.5
        marker.scale.y = 0.1
        marker.scale.z = 0.1
        
        # Color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 0.8
        
        marker.lifetime = rclpy.duration.Duration(seconds=1.0).to_msg()
        
        self.target_pub.publish(marker)
    
    def publish_status(self):
        """Publish controller status"""
        try:
            status = self.controller.get_status()

            msg = Float64MultiArray()
            msg.data = [
                float(status['person_detected']),
                status['tracking_distance'],
                self.fixed_tracking_altitude,  # Add tracking altitude
                status['optimization_time'],
                float(status['iterations_used']),
                status['cost_value']
            ]

            self.status_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing status: {e}")
    
    def _quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> np.ndarray:
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return np.array([roll, pitch, yaw])

def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = NMPCTrackerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
