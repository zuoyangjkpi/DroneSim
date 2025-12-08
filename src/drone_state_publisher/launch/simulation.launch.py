#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
    ExecuteProcess,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch import conditions
from launch_ros.actions import Node


def generate_launch_description():
    # Package directories
    drone_description = get_package_share_directory("drone_description")
    neural_network_detector_dir = get_package_share_directory("neural_network_detector")
    
    # File paths
    models_path = os.path.join(drone_description, "models")
    default_world_file = os.path.join(drone_description, "worlds", "drone_world.sdf")
    config_file = os.path.join(drone_description, "config", "bridge.yaml")
    rviz_config = os.path.join(drone_description, "config", "drone.rviz")
    urdf_file = os.path.join(drone_description, "urdf", "x3_drone.urdf")
    
    # YOLO model paths - use relative paths from neural_network_detector package
    neural_network_detector_path = get_package_share_directory("neural_network_detector")
    default_yolo_model_path = os.path.join(neural_network_detector_path, "third_party", "YOLOs-CPP", "models", "yolo12n.onnx")
    default_yolo_labels_path = os.path.join(neural_network_detector_path, "third_party", "YOLOs-CPP", "quantized_models", "coco.names")

    # Launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true',
        description='Use simulation time for all nodes'
    )
    
    declare_robot_id = DeclareLaunchArgument(
        'robot_id', default_value='1',
        description='Robot ID for this instance'
    )
    
    declare_num_robots = DeclareLaunchArgument(
        'num_robots', default_value='1',
        description='Total number of robots'
    )
    
    declare_world_file = DeclareLaunchArgument(
        'world_file', default_value='drone_world.sdf',
        description='World file to load (e.g., drone_world.sdf or city_drone_world.sdf)'
    )

    # Environment variables for Gazebo
    libgl_env = SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")
    gallium_env = SetEnvironmentVariable(name="GALLIUM_DRIVER", value="llvmpipe")
    
    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([
            str(Path(drone_description).parent.resolve()),
            models_path,
            os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        ])
    )

    # Base parameters for ALL nodes
    def get_base_params():
        return {
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }

    # =============================================================================
    # 1. ROBOT DESCRIPTION PUBLISHER
    # =============================================================================
    
    # Read URDF file
    with open(urdf_file, 'r') as infp:
        robot_description = infp.read()
    
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    # =============================================================================
    # 2. GAZEBO SIMULATION
    # =============================================================================
    
    # Build world file path from argument
    world_file_path = [drone_description, '/worlds/', LaunchConfiguration('world_file')]
    
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py")
        ),
        launch_arguments=[('gz_args', ['-v 4 -r ', drone_description, '/worlds/', LaunchConfiguration('world_file')])]
    )

    # ROS-Gazebo bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        parameters=[{
            'config_file': config_file,
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # =============================================================================
    # 2. CRITICAL: STATIC TRANSFORMS TO MAP GAZEBO FRAMES TO ROS FRAMES
    # =============================================================================
    
    # Map Gazebo base frame to ROS robot frame
    base_to_machine_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "1", "X3/base_link", "machine_1"],
        parameters=[get_base_params()],
        output="screen",
        name="base_to_machine_tf"
    )
    
    # Map Gazebo camera link to ROS camera frame
    # camera_link_tf = Node(
    #     package="tf2_ros", 
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0", "0", "0", "0", "1", "X3/camera_link", "machine_1_camera_link"],
    #     parameters=[get_base_params()],
    #     output="screen",
    #     name="camera_link_tf"
    # )
    
    # Map Gazebo optical frame to ROS optical frame (MOST CRITICAL)
    # optical_frame_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher", 
    #     arguments=["0", "0", "0", "0", "0", "0", "1", "X3/camera_optical_frame", "machine_1_camera_rgb_optical_link"],
    #     parameters=[get_base_params()],
    #     output="screen",
    #     name="optical_frame_tf"
    # )
    
    # World frame setup
    # world_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0", "0", "0", "0", "1", "world", "X3/base_link"],
    #     parameters=[get_base_params()],
    #     output="screen",
    #     name="world_tf"
    # )

    # =============================================================================
    # 3. DRONE STATE PUBLISHER
    # =============================================================================
    
    drone_state_publisher_node = Node(
        package='drone_state_publisher',
        executable='drone_state_publisher_node',
        name='drone_state_publisher',
        output='screen',
        parameters=[{
            **get_base_params(),
            'optimal_distance': 4.0,
            'optimal_height_offset': 7.0,
            'optimal_angle_offset': 0.0,
        }]
    )

    # =============================================================================
    # 4. TF TRANSFORM PUBLISHER (FIXED FRAME NAMES)
    # =============================================================================
    
#     tf_from_uav_pose_node = Node(
#     package='tf_from_uav_pose_ros2',
#     executable='tf_from_uav_pose_node',
#     name='tf_from_uav_pose',
#     output='screen',
#     parameters=[{
#         **get_base_params(),
        
#         # Topic mapping
#         'pose_topic_name': '/machine_1/pose',
#         'raw_pose_topic_name': '/machine_1/pose/raw', 
#         'std_pose_topic_name': '/machine_1/pose/corr/std',
#         'std_raw_pose_topic_name': '/machine_1/pose/raww/std',
        
#         # Frame IDs to match the SDF camera frames
#         'machine_frame_id': 'X3/base_link',
#         'world_frame_id': 'world',
#         'camera_frame_id': 'X3/camera_link',
#         'camera_rgb_optical_frame_id': 'X3/camera_optical_frame',
        
#         # Let Gazebo handle camera transforms through SDF
#         'camera_static_publish.publish': False,
#         'camera_static_publish.topic': '/machine_1/camera/pose',
#         'camera_static_publish.pose_optical_topic': '/machine_1/camera/pose_optical'
#     }]
# )

    # Static transforms to map Gazebo frames to expected ROS frames
    # gazebo_to_ros_base_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=["0", "0", "0", "0", "0", "0", "1", "X3/base_link", "machine_1"],
    #     parameters=[get_base_params()],
    #     output="screen",
    #     name="gazebo_to_ros_base_tf"
    # )

    # Map the tilted optical frame to expected ROS frame name
    # camera_optical_tf = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher", 
    #     arguments=["0", "0", "0", "0", "0", "0", "1", "X3/camera_optical_frame", "machine_1_camera_rgb_optical_link"],
    #     parameters=[get_base_params()],
    #     output="screen",
    #     name="gazebo_to_ros_camera_optical_tf"
    # )

    # Also create a manual camera pose publisher for the projection model
    # This publishes the camera pose in world coordinates for the projector to use
    # camera_pose_publisher = Node(
    #     package="tf2_ros",
    #     executable="static_transform_publisher",
    #     arguments=[
    #         "0.2", "0", "0",           # Camera position: 0.2m forward
    #         "0", "0.5236", "0", "1"    # Camera orientation: 30° pitch down (quaternion approximation)
    #     ],
    #     parameters=[get_base_params()],
    #     output="screen",
    #     name="camera_pose_static_tf"
    # )
    tf_from_uav_pose_node = Node(
        package='tf_from_uav_pose_ros2',
        executable='tf_from_uav_pose_node',
        name='tf_from_uav_pose',
        output='screen',
        parameters=[{
            **get_base_params(),
            
            # Topic mapping
            'pose_topic_name': '/machine_1/pose',
            'raw_pose_topic_name': '/machine_1/pose/raw', 
            'std_pose_topic_name': '/machine_1/pose/corr/std',
            'std_raw_pose_topic_name': '/machine_1/pose/raww/std',
            
            # FIXED: Frame IDs to match static transforms above
            'machine_frame_id': 'machine_1',                      # Matches base_to_machine_tf
            'world_frame_id': 'world',
            'camera_frame_id': 'machine_1_camera_link',           # Matches camera_link_tf  
            'camera_rgb_optical_frame_id': 'machine_1_camera_rgb_optical_link',  # Matches optical_frame_tf
            
            # Camera static transform (from machine_1 to camera)
            'dont_publish_tfs': False, 
            'camera_static_publish.publish': True,
            'camera_static_publish.tf_parameters': [
                0.2, 0.0, 0.0,                    # Position from SDF
                0.0, 0.2588, 0.0, 0.9659      # Rotation from SDF (20 degrees pitch)
            ],
            'camera_static_publish.topic': '/machine_1/camera/pose',
            'camera_static_publish.pose_optical_topic': '/machine_1/camera/pose_optical'
        }]
    )

    # =============================================================================
    # 5. NEURAL NETWORK DETECTION
    # =============================================================================
    
    yolo_detector_node = Node(
        package="neural_network_detector",
        executable="yolo12_detector_node", 
        name="yolo12_detector_node",
        output='screen',
        parameters=[{
            **get_base_params(),
            'model_path': default_yolo_model_path,
            'labels_path': default_yolo_labels_path,
            'use_gpu': False,
            'confidence_threshold': 0.3,  # LOWERED for more detections
            'iou_threshold': 0.3,
            'desired_class': 0,  # Person class
            'desired_width': 300,
            'desired_height': 300,
            'aspect_ratio': 1.333333333,
            'border_dropoff': 0.05,
            'publish_debug_image': True,
            'max_update_force': True,
            'max_update_rate_hz': 4.0,
            'feedback_timeout_sec': 5.0,
            'var_const_x_min': 0.00387,
            'var_const_x_max': 0.00347,
            'var_const_y_min': 0.00144,
            'var_const_y_max': 0.00452,
        }],
        remappings=[
            ('image_raw', '/camera/image_raw'),
            ('detections', '/person_detections'),
            ('detection_count', '/person_detection_count'),  
            ('feedback', '/neural_network_feedback'),
            ('debug_image', '/detection_debug_image'),
        ]
    )

    # =============================================================================
    # 6. 3D PROJECTION MODEL (FIXED FRAME TOPICS)
    # =============================================================================
    
    projector_node = Node(
        package='projection_model',
        executable='projection_model_node',
        name='model_distance_from_height_node',
        parameters=[{
            **get_base_params(),
            
            # Topics
            'projected_object_topic': "/machine_1/object_detections/projected_to_world",
            'camera_debug_topic': "/machine_1/object_detections/camera_debug",
            'detections_topic': "/person_detections",
            'tracker_topic': "/machine_1/target_tracker/pose",
            'offset_topic': "/machine_1/target_tracker/offset",
            'feedback_topic': "/neural_network_feedback",
            
            # FIXED: Frame topics to match tf_from_uav_pose output
            'topics.robot': "/machine_1/pose/raww/std",
            'topics.camera': "/machine_1/camera/pose",
            'topics.optical': "/machine_1/camera/pose_optical",
            
            
            # Model parameters
            'height_model_mean': 1.8,
            'height_model_var': 1.0,
            'uncertainty_scale_head': 1.0,
            'uncertainty_scale_feet': 1.0,
            
            # Camera info
            'camera.info_topic': "/camera/camera_info"
        }],
        # arguments=['--ros-args', '--log-level', 'DEBUG'],  # ← Add this line
        output='screen'
    )

    # =============================================================================
    # 7. DISTRIBUTED KALMAN FILTER TRACKER
    # =============================================================================
    
    distributed_kf_node = Node(
        package='target_tracker_distributed_kf',
        executable='distributed_kf_node',
        name='distributed_kf_3d',
        parameters=[{
            **get_base_params(),
            'robotID': LaunchConfiguration('robot_id'),
            'numRobots': LaunchConfiguration('num_robots'),
            
            # More lenient parameters for easier tracking
            'initialUncertaintyPosXY': 100.0,
            'initialUncertaintyPosZ': 10.0,
            'falsePositiveThresholdSigma': 3.0,  # LOWERED from 6.0
            'reset_time_threshold': 30.0,       # INCREASED from 10.0
            
            # Process noise parameters
            'noisePosXVar': 0.0,
            'noiseVelXVar': 0.5,
            'noiseOffXVar': 0.02,
            'noisePosYVar': 0.0,
            'noiseVelYVar': 0.5,
            'noiseOffYVar': 0.02,
            'noisePosZVar': 0.0,
            'noiseVelZVar': 0.5,
            'noiseOffZVar': 0.02,
            
            'velocityDecayTime': 3.0,
            'offsetDecayTime': 30.0,
            
            'pub_topic': '/machine_1/target_tracker/pose',
            'velPub_topic': '/machine_1/target_tracker/twist',
            'offset_topic': '/machine_1/target_tracker/offset',
            'pose_topic': '/machine_1/pose',
            'measurement_topic_suffix_self': '/machine_1/object_detections/projected_to_world',
            'measurement_topic_suffix': 'object_detections/projected_to_world',
            
            'cache_size': 40,
        }],
        output='screen'
    )

    # =============================================================================
    # 8. DRONE WAYPOINT CONTROLLER
    # =============================================================================
    
    # waypoint_controller = Node(
    #     package='drone_description',
    #     executable='waypoint_controller',
    #     name='waypoint_controller',
    #     output='screen',
    #     parameters=[{
    #         **get_base_params(),
    #         'max_horizontal_speed': 1.0,
    #         'max_vertical_speed': 0.8,
    #         'max_yaw_rate': 0.5,
    #         'waypoint_tolerance': 0.05,
    #         'yaw_p_gain': 0.3,
    #         'yaw_d_gain': 0.1,
    #         'prediction_time': 1.0,
    #         'enable_debug': True,
    #         'pure_tracking_mode': False,
    #         'cmd_vel_topic': '/X3/cmd_vel',
    #         'odom_topic': '/X3/odometry',
    #     }]
    # )

    improved_waypoint_controller = Node(
    package='drone_description',
    executable='waypoint_controller',  # New executable name
    name='improved_waypoint_controller',
    output='screen',
    parameters=[{
        **get_base_params(),
        
        # Basic control limits
        'max_horizontal_speed': 0.8,
        'max_vertical_speed': 0.5,
        'max_yaw_rate': 0.3,
        'waypoint_tolerance': 0.15,
        
        # Position PID gains
        'pos_kp': 1.2,
        'pos_ki': 0.1,
        'pos_kd': 0.3,
        
        # Yaw PID gains
        'yaw_kp': 0.8,
        'yaw_ki': 0.05,
        'yaw_kd': 0.15,
        
        # Advanced control parameters
        'prediction_time': 0.5,
        'command_filter_alpha': 0.7,
        'approach_distance': 1.0,
        'min_speed_factor': 0.1,
        
        # Modes and debug
        'pure_tracking_mode': False,
        'enable_debug': True,
        
        # Topics
        'cmd_vel_topic': '/X3/cmd_vel',
        'odom_topic': '/X3/odometry',
    }]
)

    # =============================================================================
    # 9. VISUALIZATION
    # =============================================================================
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        parameters=[get_base_params()],
        output="screen"
    )

    # =============================================================================
    # 10. LAUNCH SEQUENCE
    # =============================================================================
    
    return LaunchDescription([
        # Launch arguments
        declare_use_sim_time,
        declare_robot_id,
        declare_num_robots,
        declare_world_file,
        
        # Environment
        libgl_env,
        gallium_env,
        gazebo_resource_path,
        
        # Start Gazebo first
        gazebo,
        gz_ros2_bridge,
        # robot_state_publisher_node,  # DISABLED - gz.launch.py already provides this
        
        # Static transforms MUST come early
        # TimerAction(period=2.0, actions=[
            # base_to_machine_tf,
            # camera_link_tf, 
            # optical_frame_tf,
            # world_tf,
        # ]),
        
        # Core nodes
        TimerAction(period=5.0, actions=[
            tf_from_uav_pose_node,
            drone_state_publisher_node,
            yolo_detector_node,
            projector_node,
            # distributed_kf_node,
            # improved_waypoint_controller,  # DISABLED - using NMPC instead

        ]),
        
        # Detection and tracking
        # TimerAction(period=8.0, actions=[
        # ]),
        
        # TimerAction(period=12.0, actions=[
        # ]),
        
        # TimerAction(period=15.0, actions=[
        # ]),
        
        # TimerAction(period=18.0, actions=[
        # ]),
        
        # Visualization last
        # TimerAction(period=20.0, actions=[
            rviz_node
        # ])
    ])