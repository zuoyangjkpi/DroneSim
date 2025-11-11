from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the launch directory
    pkg_dir = get_package_share_directory('tf_from_uav_pose')
    config_dir = os.path.join(pkg_dir, 'config')
    default_config_file = os.path.join(config_dir, 'tf_from_uav_pose_params.yaml')

    # Declare launch arguments (for optional overrides via separate launch files)
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config_file,
        description='Full path to the ROS2 parameters file to use'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Node with proper parameter loading - YAML file contains all topic configurations
    tf_from_uav_pose_node = Node(
        package='tf_from_uav_pose',
        executable='tf_from_uav_pose_node',
        name='tf_from_uav_pose',
        parameters=[
            # # Load ALL parameters from YAML file
            LaunchConfiguration('config_file'),
            # Only override essential launch-time parameters
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        tf_from_uav_pose_node
    ])
