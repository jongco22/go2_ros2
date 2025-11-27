# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    """
    Launch file to run the PID Path Tracker.
    This launch file starts the tracker node, which receives a path alias (e.g., 'path_1')
    and constructs the full path from a specified directory to control the robot.
    """

    # ========== Launch Arguments ==========

    declare_path_directory_arg = DeclareLaunchArgument(
        'path_directory',
        default_value='/root/ros2_ws/src/path_tracker/path',
        description='The absolute directory where path CSV files are stored.'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_run_tf_to_pose_pub_arg = DeclareLaunchArgument(
        'run_tf_to_pose_pub',
        default_value='true',
        description='If true, run the tf_to_pose_publisher_node.'
    )
     
    # PID 파라미터 인자 선언 (변경 없음)
    declare_kp_x_arg = DeclareLaunchArgument('kp_x', default_value='0.2')
    declare_ki_x_arg = DeclareLaunchArgument('ki_x', default_value='0.0')
    declare_kd_x_arg = DeclareLaunchArgument('kd_x', default_value='0.05')
    declare_kp_y_arg = DeclareLaunchArgument('kp_y', default_value='0.05')
    declare_ki_y_arg = DeclareLaunchArgument('ki_y', default_value='0.0')
    declare_kd_y_arg = DeclareLaunchArgument('kd_y', default_value='0.01')
    declare_kp_yaw_arg = DeclareLaunchArgument('kp_yaw', default_value='0.2')
    declare_ki_yaw_arg = DeclareLaunchArgument('ki_yaw', default_value='0.0')
    declare_kd_yaw_arg = DeclareLaunchArgument('kd_yaw', default_value='0.06')
    declare_max_linear_velocity_arg = DeclareLaunchArgument('max_linear_velocity', default_value='0.6')
    declare_max_angular_velocity_arg = DeclareLaunchArgument('max_angular_velocity', default_value='1.3')
    declare_goal_tolerance_arg = DeclareLaunchArgument('goal_tolerance', default_value='0.15')
    declare_waypoint_arrival_dist_arg = DeclareLaunchArgument('waypoint_arrival_dist', default_value='0.1')
    
    run_tf_to_pose_pub_arg = LaunchConfiguration('run_tf_to_pose_pub')

    # ========== Nodes ==========
    pid_tracker_node = Node(
        package='path_tracker',
        executable='pid_path_tracker_node',
        name='pid_path_tracker_node',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            
            # [핵심 수정 2] C++ 노드가 기대하는 'path_directory' 파라미터를 정확하게 전달합니다.
            'path_directory': LaunchConfiguration('path_directory'),
            'kp_x': LaunchConfiguration('kp_x'),
            'ki_x': LaunchConfiguration('ki_x'),
            'kd_x': LaunchConfiguration('kd_x'),
            'kp_y': LaunchConfiguration('kp_y'),
            'ki_y': LaunchConfiguration('ki_y'),
            'kd_y': LaunchConfiguration('kd_y'),
            'kp_yaw': LaunchConfiguration('kp_yaw'),
            'ki_yaw': LaunchConfiguration('ki_yaw'),
            'kd_yaw': LaunchConfiguration('kd_yaw'),
            # Pass PID parameters from launch arguments to the node

            'max_linear_velocity': LaunchConfiguration('max_linear_velocity'),
            'max_angular_velocity': LaunchConfiguration('max_angular_velocity'),
            'goal_tolerance': LaunchConfiguration('goal_tolerance'),
            'waypoint_arrival_dist': LaunchConfiguration('waypoint_arrival_dist'),
        }],
    )
    
    tf_to_pose_publisher_node = Node(
        package='path_tracker',
        executable='tf_to_pose_publisher_node',
        name='tf_to_pose_publisher_node',
        output='screen',
        parameters=[
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        condition=IfCondition(run_tf_to_pose_pub_arg)
    )

    return LaunchDescription([
        # Launch Arguments
        declare_path_directory_arg,
        declare_use_sim_time_arg,
        declare_kp_x_arg,
        declare_ki_x_arg,
        declare_kd_x_arg,
        declare_kp_y_arg,
        declare_ki_y_arg,
        declare_kd_y_arg,
        declare_kp_yaw_arg,
        declare_ki_yaw_arg,
        declare_kd_yaw_arg,
        declare_max_linear_velocity_arg,
        declare_max_linear_velocity_arg,
        declare_max_angular_velocity_arg,
        declare_goal_tolerance_arg,
        declare_waypoint_arrival_dist_arg,
        declare_run_tf_to_pose_pub_arg,
        
        # Nodes
        pid_tracker_node,
        tf_to_pose_publisher_node,
    ])