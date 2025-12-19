"""
Launch file for PX4 Bridge Node
Connects DroneSim system to PX4 autopilot via uXRCE-DDS
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Declare arguments
        DeclareLaunchArgument(
            'namespace',
            default_value='',
            description='Namespace for the node'
        ),

        # PX4 Bridge Node
        Node(
            package='px4_bridge',
            executable='px4_bridge_node',
            name='px4_bridge_node',
            namespace=LaunchConfiguration('namespace'),
            output='screen',
            parameters=[{
                'use_sim_time': False,
            }],
            remappings=[
                # Add remappings if needed
            ]
        ),
    ])
