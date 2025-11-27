# Copyright (c) 2024, RoboVerse community
# SPDX-License-Identifier: BSD-3-Clause

import os
from typing import List
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument


def load_urdf_content(urdf_path: str) -> str:
    """Load URDF file content"""
    with open(urdf_path, 'r') as file:
        return file.read()


def create_driver_nodes() -> List[Node]:
    """Create the core Go2 hardware driver node."""
    start_mode = LaunchConfiguration('start_mode')
    
    return [
        Node(
            package='unitree_ros2_example',
            executable='go2_sport_client',
            name='go2_sport_client_node',
            output='screen',
            # Use LaunchConfiguration to pass the start_mode argument
            arguments=[start_mode]
        )
    ]


def create_robot_state_nodes() -> List[Node]:
    """Create robot state publisher nodes"""
    package_dir = get_package_share_directory('go2_robot_sdk')
    urdf_path = os.path.join(package_dir, 'urdf', 'go2.urdf')
    use_sim_time = LaunchConfiguration('use_sim_time', default='False')
    
    # Load URDF content
    robot_desc = load_urdf_content(urdf_path)
    
    return [
        # Robot state publisher - subscribes to /joint_states
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='go2_robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
                'ignore_timestamp': False,
            }],
            # No remapping needed - uses /joint_states directly
            arguments=[urdf_path]
        ),
        # Joint states publisher - publishes to /joint_states from /lowstate
        Node(
            package='go2_robot_sdk',
            executable='joint_states_publisher',
            name='joint_states_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
            }],
        ),
        # Joint state publisher GUI for manual control (fallback if /lowstate is not available)
        # Uncomment this if you need to manually set joint positions
        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     output='screen',
        # ),
    ]


def generate_launch_description():
    """Generate the launch description for Go2 robot system"""
    
    # Create launch arguments
    launch_args = [
        DeclareLaunchArgument('start_mode', default_value='0', description='Go2 start mode (0: StandUp, 7: Sit, etc.)'),
        DeclareLaunchArgument('use_sim_time', default_value='False', description='Use simulation time'),
    ]
    
    # Create driver nodes
    driver_nodes = create_driver_nodes()
    
    # Create robot state publisher nodes
    robot_state_nodes = create_robot_state_nodes()
    
    # Combine all elements
    launch_entities = launch_args + driver_nodes + robot_state_nodes
    
    return LaunchDescription(launch_entities)