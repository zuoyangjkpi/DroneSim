#!/usr/bin/env python3
"""
Launch file for drone low-level control plugins
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('drone_low_level_controllers')

    # Path to config file
    config_file = os.path.join(pkg_dir, 'config', 'controllers.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Waypoint controller node
    waypoint_controller_node = Node(
        package='drone_low_level_controllers',
        executable='waypoint_controller.py',
        name='waypoint_controller',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Attitude controller node
    attitude_controller_node = Node(
        package='drone_low_level_controllers',
        executable='attitude_controller.py',
        name='attitude_controller',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Gazebo MulticopterVelocityControl adapter node
    multicopter_velocity_control_adapter_node = Node(
        package='drone_low_level_controllers',
        executable='multicopter_velocity_control_adapter.py',
        name='multicopter_velocity_control_adapter',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        waypoint_controller_node,
        attitude_controller_node,
        multicopter_velocity_control_adapter_node,
    ])
