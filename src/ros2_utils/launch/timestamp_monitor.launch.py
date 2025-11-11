from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    
    # Launch arguments
    declare_log_path = DeclareLaunchArgument(
        'log_file_path',
        default_value='/tmp/timing_analysis.csv',
        description='Path for CSV log file'
    )
    
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Timestamp monitor node
    timestamp_monitor_node = Node(
        package='ros2_utils',
        executable='timestamp_monitor_node',
        name='timestamp_monitor',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_file_path': LaunchConfiguration('log_file_path'),
            'log_to_file': True,
            'sync_tolerance_ms': 100.0,
            'reference_topic': '/firefly_1/xtion/rgb/image_raw',
            'analysis_window_sec': 10.0,
            'show_detailed_stats': False
        }]
    )

    return LaunchDescription([
        declare_log_path,
        declare_use_sim_time,
        timestamp_monitor_node
    ])
