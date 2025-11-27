# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Launch file to run the Path Publisher Node.
    This launch file starts the path publisher node with PID control for path following.
    """

    # ========== Launch Arguments ==========

    declare_path_directory_arg = DeclareLaunchArgument(
        'path_directory',
        default_value='/root/ros2_ws/src/path_tracker/path',
        description='Directory containing CSV path files.'
    )

    declare_path_file_arg = DeclareLaunchArgument(
        'path_file',
        default_value='',
        description='Full path to the CSV file containing the path poses. If empty, will wait for navigation command.'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    # Control parameters
    declare_control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='100.0',
        description='Control loop frequency in Hz'
    )

    declare_distance_threshold_arg = DeclareLaunchArgument(
        'distance_threshold',
        default_value='0.05',
        description='Distance threshold to consider target reached (meters)'
    )

    declare_angle_threshold_arg = DeclareLaunchArgument(
        'angle_threshold',
        default_value='0.05',
        description='Angle threshold to consider target reached (radians)'
    )

    # PID parameters
    declare_kp_x_arg = DeclareLaunchArgument(
        'kp_x',
        default_value='2.0',
        description='Proportional gain for X-axis control'
    )

    declare_kp_y_arg = DeclareLaunchArgument(
        'kp_y',
        default_value='3.0',
        description='Proportional gain for Y-axis control'
    )

    declare_kp_yaw_arg = DeclareLaunchArgument(
        'kp_yaw',
        default_value='4.0',
        description='Proportional gain for Yaw control'
    )

    # Velocity limits
    declare_max_linear_velocity_arg = DeclareLaunchArgument(
        'max_linear_velocity',
        default_value='3.0',
        description='Maximum linear velocity (m/s)'
    )

    declare_max_angular_velocity_arg = DeclareLaunchArgument(
        'max_angular_velocity',
        default_value='4.0',
        description='Maximum angular velocity (rad/s)'
    )

    # ========== Nodes ==========
    
    path_publisher_node = Node(
        package='path_tracker',
        executable='path_publisher_node',
        name='path_publisher_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'path_directory': LaunchConfiguration('path_directory'),
            'path_file': LaunchConfiguration('path_file'),
            'control_rate': LaunchConfiguration('control_rate'),
            'distance_threshold': LaunchConfiguration('distance_threshold'),
            'angle_threshold': LaunchConfiguration('angle_threshold'),
            'kp_x': LaunchConfiguration('kp_x'),
            'kp_y': LaunchConfiguration('kp_y'),
            'kp_yaw': LaunchConfiguration('kp_yaw'),
            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
        }],
    )

    return LaunchDescription([
        # Launch Arguments
        declare_path_directory_arg,
        declare_path_file_arg,
        declare_use_sim_time_arg,
        declare_control_rate_arg,
        declare_distance_threshold_arg,
        declare_angle_threshold_arg,
        declare_kp_x_arg,
        declare_kp_y_arg,
        declare_kp_yaw_arg,
        declare_max_linear_velocity_arg,
        declare_max_angular_velocity_arg,
        
        # Nodes
        path_publisher_node,
    ])
