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
from typing import Optional, List
import tf2_ros
from tf2_ros import TransformException

# ROS2 message types
from geometry_msgs.msg import PoseStamped, Vector3Stamped, PoseWithCovarianceStamped
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
        self.control_enabled = False
        self.last_tracking_yaw: float = 0.0
        self._planned_waypoint_sequence: List[np.ndarray] = []
        self._current_waypoint_index = 0
        self._last_plan_sequence_id = -1
        self._waypoint_reached_tolerance = 0.3
        self._plan_reset_threshold = 0.5
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Initialize ROS2 interfaces
        self._init_parameters()
        self._init_publishers()
        self._init_subscribers()
        self._init_timers()

        self.get_logger().info("NMPC Tracker Node initialized (control enable/disable now managed by TrackTargetModule)")
    
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
        self.declare_parameter('tracking_phase_offset', 0.0)
        self.declare_parameter('tracking_height_offset', nmpc_config.TRACKING_HEIGHT_OFFSET)
        self.declare_parameter('person_position_filter_alpha', nmpc_config.PERSON_POSITION_FILTER_ALPHA)
        self.declare_parameter('track_detection_confirmations', 1)
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
        self.tracking_phase_offset = self.get_parameter('tracking_phase_offset').value
        self.tracking_height_offset = self.get_parameter('tracking_height_offset').value
        self.person_position_filter_alpha = float(self.get_parameter('person_position_filter_alpha').value)
        self.controller.set_tracking_height_offset(self.tracking_height_offset)
        self.fixed_tracking_altitude = nmpc_config.TRACKING_FIXED_ALTITUDE
        self.controller.set_fixed_altitude(self.fixed_tracking_altitude)
        self.required_detection_confirmations = max(1, int(self.get_parameter('track_detection_confirmations').value))

        self._filtered_person_position = None
        self._detection_streak = 0

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
        
        # Control command publishers - 发布到NMPC专用话题，由TrackTarget模块转发
        # 这样确保只有TrackTarget模块激活时，NMPC的指令才会被转发到下游控制器
        self.waypoint_pub = self.create_publisher(
            PoseStamped,
            '/nmpc/waypoint_command',  # 修改为NMPC专用话题
            control_qos
        )

        self.attitude_pub = self.create_publisher(
            Vector3Stamped,
            '/nmpc/attitude_command',  # 修改为NMPC专用话题
            control_qos
        )


        # ❌ 移除NMPC直接控制低层控制器的enable publisher
        # ✅ 现在由TrackTargetModule通过ActionContext统一管理控制器enable/disable


        # Status publisher - publish to NMPC-specific topic to avoid conflict with ActionManager
        # ActionManager publishes global status to /drone/controller/status
        self.status_pub = self.create_publisher(
            Float64MultiArray,
            '/nmpc/internal_status',  # Use separate topic, not nmpc_config.TOPIC_STATUS
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
            self.last_tracking_yaw = orientation[2]
            
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
                allow_phase_change=True,
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
            allow_phase_change=True,
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

        if self._detection_streak < self.required_detection_confirmations:
            # Still waiting for confirmation streak
            self.person_detected = False
            self._publish_person_estimate(person_position)
            return

        self.person_detected = True
        self.last_person_detection_time = current_time
        if first_detection:
            self.get_logger().info(f"✅ Person detected at {person_position}")

        self._publish_person_estimate(person_position)

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
        """Handle control enable/disable commands from TrackTargetModule"""
        self.control_enabled = msg.data
        if self.control_enabled:
            self.get_logger().info("NMPC control enabled by TrackTargetModule")
            # Reset detection state when enabled to avoid stale timeout
            self.person_detected = False
            self._detection_streak = 0
            self._filtered_person_position = None
            self.last_person_detection_time = self._now()
            self.controller.clear_detection()
        else:
            self.get_logger().info("NMPC control disabled")

        # ❌ 不再由NMPC自己enable/disable低层控制器
        # ✅ TrackTargetModule会通过ActionContext统一管理
        if not self.control_enabled:
            return
    
    def control_loop_callback(self):
        """Main control loop: only run NMPC tracking when enabled and detections confirmed."""
        current_time = self._now()

        if not self.control_enabled:
            return

        if not self.drone_state_received:
            self.get_logger().warn("No drone state received - waiting for odometry data")
            return

        if self.person_detected and current_time - self.last_person_detection_time > self.person_timeout:
            self.person_detected = False
            self.controller.clear_detection()
            self._filtered_person_position = None
            self._detection_streak = 0
            self.get_logger().warn("Person detection timeout - tracking paused")

        if not self.person_detected:
            return

        self._perform_tracking()

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
            return

        # Standard MPC: only execute the first point from the optimized sequence
        self._send_tracking_commands_first_point_only(control, info)

        if self.enable_visualization and info:
            self._publish_visualization(info)

    def _send_tracking_commands_first_point_only(self, control: np.ndarray, info: dict):
        """Send only the first point from the optimized trajectory (standard MPC approach)."""

        # !!!!! BYPASS MODE FOR DEBUGGING !!!!!
        # Set to True to bypass NMPC and send target_position directly
        BYPASS_NMPC = True

        if BYPASS_NMPC:
            # DEBUG: Bypass NMPC completely, send target_position directly
            if hasattr(self.controller, 'target_position') and self.controller.target_position is not None:
                target_position = self.controller.target_position.copy()
                current_pos = self._get_current_position()
                delta = target_position - current_pos
                self.get_logger().info(
                    f'[BYPASS MODE] Sending target_position directly: '
                    f'delta=[{delta[0]:.3f}, {delta[1]:.3f}, {delta[2]:.3f}]m, '
                    f'dist={np.linalg.norm(delta):.3f}m',
                    throttle_duration_sec=0.5
                )
            else:
                self.get_logger().error('No target_position available in bypass mode')
                return
        else:
            # Extract the first waypoint from the optimized trajectory
            if 'trajectory' not in info or len(info['trajectory']) == 0:
                self.get_logger().warn('No trajectory in NMPC output - cannot send command')
                return

            # Standard MPC: execute only the first step from the optimized sequence
            # trajectory[0] = current state, trajectory[1] = next desired state
            if len(info['trajectory']) > 1:
                next_state = info['trajectory'][1]
                target_position = np.array(next_state.data[nmpc_config.STATE_X:nmpc_config.STATE_Z+1])
            else:
                # Fallback if trajectory has only one point
                self.get_logger().warn('Trajectory has only 1 point, using controller target_position')
                if hasattr(self.controller, 'target_position') and self.controller.target_position is not None:
                    target_position = self.controller.target_position.copy()
                else:
                    self.get_logger().error('No valid target position available')
                    return

        # Debug logging to diagnose why tracking distance is not achieved
        current_pos = self._get_current_position()
        delta = target_position - current_pos
        if hasattr(self.controller, 'target_position'):
            target_dist = np.linalg.norm(self.controller.target_position - current_pos)
            traj_dist = np.linalg.norm(delta)
            self.get_logger().info(
                f'MPC step: traj[1]_dist={traj_dist:.3f}m, target_dist={target_dist:.3f}m, '
                f'delta=[{delta[0]:.3f}, {delta[1]:.3f}, {delta[2]:.3f}]',
                throttle_duration_sec=1.0
            )

        # Override altitude to fixed tracking altitude
        target_position[2] = float(self.fixed_tracking_altitude)

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
            roll_cmd = float(desired_attitude[0])
            pitch_cmd = float(desired_attitude[1])
        else:
            roll_cmd = 0.0
            pitch_cmd = 0.0

        # Compute yaw to face the person
        person_pos = self._filtered_person_position
        if person_pos is not None:
            drone_pos = self._get_current_position()
            dx = person_pos[0] - drone_pos[0]
            dy = person_pos[1] - drone_pos[1]
            yaw_cmd = float(np.arctan2(dy, dx))
        else:
            yaw_cmd = self.last_tracking_yaw

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
            source="NMPC_FIRST_POINT",
            position=target_position,
            velocity=None,
            yaw=yaw_cmd
        )

    def _refresh_waypoint_queue(self):
        plan_id = self.controller.get_plan_sequence_id()
        if plan_id == self._last_plan_sequence_id:
            return

        sequence = self.controller.get_planned_waypoints()
        if not sequence:
            return

        reset_index = True
        if self._planned_waypoint_sequence:
            first_diff = np.linalg.norm(sequence[0] - self._planned_waypoint_sequence[0])
            last_diff = np.linalg.norm(sequence[-1] - self._planned_waypoint_sequence[-1])
            length_changed = len(sequence) != len(self._planned_waypoint_sequence)
            reset_index = (
                first_diff > self._plan_reset_threshold or
                last_diff > self._plan_reset_threshold or
                length_changed
            )

        self._planned_waypoint_sequence = [wp.copy() for wp in sequence]
        self._last_plan_sequence_id = plan_id
        if reset_index:
            self._current_waypoint_index = 0

    def _current_tracking_waypoint(self) -> np.ndarray:
        if self._planned_waypoint_sequence and self._current_waypoint_index < len(self._planned_waypoint_sequence):
            return self._planned_waypoint_sequence[self._current_waypoint_index]
        return self.controller.target_position.copy()

    def _advance_waypoint_progress(self):
        if not self._planned_waypoint_sequence:
            return

        current_position = self._get_current_position()
        waypoint = self._current_tracking_waypoint()
        distance = np.linalg.norm(current_position - waypoint)

        if distance <= self._waypoint_reached_tolerance:
            if self._current_waypoint_index < len(self._planned_waypoint_sequence) - 1:
                self._current_waypoint_index += 1
    def _send_tracking_commands(self, control: np.ndarray):
        """Send tracking commands to low-level controllers"""
        # Advance waypoint progress based on current position
        self._advance_waypoint_progress()

        # Current target waypoint
        target_position = self._current_tracking_waypoint().copy()
        target_position[2] = float(self.fixed_tracking_altitude)

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
            source="NMPC_WAYPOINT",
            position=target_position,
            velocity=None,
            yaw=yaw_cmd,
            control=control
        )

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
        att_vector = [round(float(roll), 3), round(float(pitch), 3), round(float(yaw), 3)]
        self.get_logger().info(
            f"Attitude command rpy={att_vector}"
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
            desired_distance = status.get(
                'desired_tracking_distance',
                nmpc_config.DESIRED_TRACKING_DISTANCE_XY,
            )
            current_distance = status.get(
                'current_tracking_distance',
                status.get('tracking_distance', 0.0),
            )

            msg.data = [
                float(status['person_detected']),
                desired_distance,
                self.fixed_tracking_altitude,  # tracking altitude for visualisation
                status['optimization_time'],
                float(status['iterations_used']),
                status['cost_value'],
                current_distance,
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
