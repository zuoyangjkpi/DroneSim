from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('target_tracker_distributed_kf')
    config_dir = os.path.join(pkg_dir, 'config')
    default_config_file = os.path.join(config_dir, 'tracker_params.yaml')
    
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the ROS2 parameters file'
    )
    
    robot_id_arg = DeclareLaunchArgument(
        'robot_id',
        default_value='1',
        description='Robot ID for distributed tracking'
    )
    
    num_robots_arg = DeclareLaunchArgument(
        'num_robots',
        default_value='1',
        description='Total number of robots in the system'
    )
    
    # Topics configuration
    pose_topic_arg = DeclareLaunchArgument(
        'pose_topic',
        default_value='/machine_1/pose',
        description='UAV pose topic'
    )
    
    measurement_topic_self_arg = DeclareLaunchArgument(
        'measurement_topic_self',
        default_value='/machine_1/object_detections/projected_to_world',
        description='Self measurement topic'
    )
    
    measurement_topic_suffix_arg = DeclareLaunchArgument(
        'measurement_topic_suffix',
        default_value='object_detections/projected_to_world_network',
        description='Measurement topic suffix for other robots'
    )
    
    # Create the node
    distributed_kf_node = Node(
        package='target_tracker_distributed_kf',
        executable='distributed_kf_node',
        name='distributed_kf3d',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'robotID': LaunchConfiguration('robot_id'),
                'numRobots': LaunchConfiguration('num_robots'),
                'pose_topic': LaunchConfiguration('pose_topic'),
                'measurement_topic_suffix_self': LaunchConfiguration('measurement_topic_self'),
                'measurement_topic_suffix': LaunchConfiguration('measurement_topic_suffix'),
            }
        ],
        output='screen',
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', 'INFO']
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        config_file_arg,
        robot_id_arg,
        num_robots_arg,
        pose_topic_arg,
        measurement_topic_self_arg,
        measurement_topic_suffix_arg,
        distributed_kf_node
    ])
