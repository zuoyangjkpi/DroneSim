
import os
from ament_index_python.packages import get_package_share_directory
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    SetEnvironmentVariable,
    IncludeLaunchDescription,
    TimerAction,
    DeclareLaunchArgument,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, TextSubstitution


def generate_launch_description():
    # Paths
    drone_description = get_package_share_directory("drone_description")
    models_path = os.path.join(drone_description, "models")
    default_world_file = os.path.join(drone_description, "worlds", "drone_world.sdf")
    config_file = os.path.join(drone_description, "config", "bridge.yaml")
    default_gui_config = os.path.join(drone_description, "config", "gui_tugbot_teleop.config")
    rviz_config = os.path.join(drone_description, "config", "drone.rviz")
    urdf_file = os.path.join(drone_description, "urdf", "x3_drone.urdf")
    world_file = LaunchConfiguration("world_file")
    use_gui_config = LaunchConfiguration("use_gui_config")
    gui_config = LaunchConfiguration("gui_config")
    yolo_target_class = LaunchConfiguration("yolo_target_class")

    # Environment variables
    # Environment variables - DISABLED to fix crash with hardware drivers
    # libgl_env = SetEnvironmentVariable(name="LIBGL_ALWAYS_SOFTWARE", value="1")
    # gallium_env = SetEnvironmentVariable(name="GALLIUM_DRIVER", value="llvmpipe")

    declare_world_arg = DeclareLaunchArgument(
        "world_file",
        default_value=default_world_file,
        description="Absolute path to Gazebo world file"
    )

    declare_use_gui_config_arg = DeclareLaunchArgument(
        "use_gui_config",
        default_value="false",
        description="Whether to pass --gui-config to Gazebo Sim",
    )

    declare_gui_config_arg = DeclareLaunchArgument(
        "gui_config",
        default_value=default_gui_config,
        description="Absolute path to Gazebo GUI configuration file",
    )

    declare_yolo_class_arg = DeclareLaunchArgument(
        "yolo_target_class",
        default_value="0",
        description="YOLO target class ID (0=person, 2=car, 7=truck, etc. See coco.names)",
    )

    gazebo_resource_path = SetEnvironmentVariable(
        name="GZ_SIM_RESOURCE_PATH",
        value=":".join([
            str(Path(drone_description).parent.resolve()),
            models_path,
            os.environ.get("GZ_SIM_RESOURCE_PATH", "")
        ])
    )

    # Set custom GUI resource path for modified Teleop plugin
    gz_gui_resource_path = SetEnvironmentVariable(
        name="GZ_GUI_RESOURCE_PATH",
        value=":".join([
            os.path.join(drone_description, "config"),
            os.environ.get("GZ_GUI_RESOURCE_PATH", "")
        ])
    )

    set_gazebo_model_path = SetEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=models_path + ':' + os.environ.get('GAZEBO_MODEL_PATH', '')
    )

    gz_sim_launch = os.path.join(
        get_package_share_directory("ros_gz_sim"),
        "launch",
        "gz_sim.launch.py",
    )

    # Launch Gazebo (default GUI config)
    gazebo_default = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        condition=UnlessCondition(use_gui_config),
        launch_arguments=[
            ("gz_args", [TextSubstitution(text="-v 4 -r "), world_file])
        ],
    )

    # Launch Gazebo with a custom GUI config (e.g. include Teleop panel for tugbot)
    gazebo_with_gui_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gz_sim_launch),
        condition=IfCondition(use_gui_config),
        launch_arguments=[
            (
                "gz_args",
                [
                    TextSubstitution(text="-v 4 -r --gui-config "),
                    gui_config,
                    TextSubstitution(text=" "),
                    world_file,
                ],
            )
        ],
    )

    # ROS-Gazebo bridge
    gz_ros2_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        output='screen',
        parameters=[{'config_file': config_file}]
    )

    # Robot state publisher
    robot_description = Command(['cat ', urdf_file])
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{'robot_description': robot_description}]
    )

    # Static transform
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
        output="screen"
    )

    # Drone enabler - Use Python script instead of ros2 topic pub
    # This creates a simple publisher that continuously enables the drone
    
    # Waypoint controller - DISABLED because NMPC controller is used instead
    # controller = Node(
    #     package='drone_description',
    #     executable='waypoint_controller',
    #     name='waypoint_controller',
    #     output='screen'
    # )

    # RViz2
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_config],
        output="screen"
    )

    # Get YOLO model paths using environment variable or relative to workspace
    workspace_path = os.environ.get('COLCON_PREFIX_PATH', '').split(':')[0] if os.environ.get('COLCON_PREFIX_PATH') else ''
    if workspace_path:
        workspace_root = str(Path(workspace_path).parent)
        default_model_path = os.path.join(workspace_root, "src", "neural_network_detector", "third_party", "YOLOs-CPP", "models", "yolo12n.onnx")
        default_labels_path = os.path.join(workspace_root, "src", "neural_network_detector", "third_party", "YOLOs-CPP", "models", "coco.names")
    else:
        # Fallback to install directory paths
        neural_network_detector_share = get_package_share_directory("neural_network_detector")
        default_model_path = os.path.join(neural_network_detector_share, "models", "yolo12n.onnx")
        default_labels_path = os.path.join(neural_network_detector_share, "models", "coco.names")

    yolo_node = Node(
        package="neural_network_detector",
        executable="yolo12_detector_node",
        name="yolo12_detector_node",
        output="screen",
        parameters=[
            {'model_path': default_model_path},
            {'labels_path': default_labels_path},
            {'use_gpu': False},
            {'confidence_threshold': 0.5},
            {'desired_class': yolo_target_class},  # Configurable COCO class (0=person, 2=car, 7=truck, etc.)
            {'max_update_rate_hz': 1.0}
        ]
    )

    # Remove the problematic pose_to_odom_bridge for now
    # We'll use the existing odometry from Gazebo plugins instead

    return LaunchDescription([
        declare_world_arg,
        declare_use_gui_config_arg,
        declare_gui_config_arg,
        declare_yolo_class_arg,
        gazebo_resource_path,
        gz_gui_resource_path,
        set_gazebo_model_path,
        yolo_node,
        gazebo_default,
        gazebo_with_gui_config,
        gz_ros2_bridge,
        robot_state_publisher,
        # pose_to_odom_bridge removed
        TimerAction(
            period=10.0,
            actions=[
                      static_tf,
                      # controller,  # DISABLED - using NMPC instead
                      rviz_node
                    ]
        )

    ])
