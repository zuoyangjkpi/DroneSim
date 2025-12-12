#!/usr/bin/env python3
"""
Launch file for drone low-level control plugins
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Package directories
    velocity_pkg_dir = get_package_share_directory('drone_low_level_controllers')
    guidance_pkg_dir = get_package_share_directory('drone_guidance_controllers')

    # Path to config files
    guidance_config = os.path.join(guidance_pkg_dir, 'config', 'controllers.yaml')
    velocity_config = os.path.join(velocity_pkg_dir, 'config', 'controllers.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Waypoint controller node
    waypoint_controller_node = Node(
        package='drone_guidance_controllers',
        executable='waypoint_controller',
        name='waypoint_controller',
        parameters=[
            guidance_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Yaw controller node
    yaw_controller_node = Node(
        package='drone_guidance_controllers',
        executable='yaw_controller',
        name='yaw_controller',
        parameters=[
            guidance_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    # Gazebo MulticopterVelocityControl adapter node (OLD - replaced by controller_node)
    # multicopter_velocity_control_adapter_node = Node(
    #     package='drone_low_level_controllers',
    #     executable='multicopter_velocity_control_adapter.py',
    #     name='multicopter_velocity_control_adapter',
    #     parameters=[
    #         velocity_config,
    #         {'use_sim_time': LaunchConfiguration('use_sim_time')}
    #     ],
    #     output='screen'
    # )

    # Cascaded controller node (NEW - replaces Gazebo velocity control plugin)
    controller_node = Node(
        package='drone_low_level_controllers',
        executable='controller_node.py',
        name='multicopter_controller_node',
        parameters=[
            velocity_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    return LaunchDescription([
        use_sim_time_arg,
        waypoint_controller_node,
        yaw_controller_node,
        # multicopter_velocity_control_adapter_node,  # OLD
        controller_node,  # NEW
    ])
