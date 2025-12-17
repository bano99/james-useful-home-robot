#!/usr/bin/env python3
"""
Launch file for James Robot Arm Cartesian Control

This launch file starts all nodes required for manual Cartesian control
of the AR4-MK3 robot arm via remote control.

Author: James Robot Team
Date: December 2024
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('james_manipulation'),
            'config',
            'arm_cartesian_params.yaml'
        ]),
        description='Path to the configuration file'
    )
    
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    log_level = LaunchConfiguration('log_level')
    
    # Platform Serial Bridge Node
    platform_bridge_node = Node(
        package='james_manipulation',
        executable='platform_serial_bridge',
        name='platform_serial_bridge',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Arm Cartesian Controller Node
    arm_controller_node = Node(
        package='james_manipulation',
        executable='arm_cartesian_controller',
        name='arm_cartesian_controller',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # Teensy Serial Bridge Node
    teensy_bridge_node = Node(
        package='james_manipulation',
        executable='teensy_serial_bridge',
        name='teensy_serial_bridge',
        parameters=[
            config_file,
            {'use_sim_time': use_sim_time}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        use_sim_time_arg,
        log_level_arg,
        
        # Nodes
        platform_bridge_node,
        arm_controller_node,
        teensy_bridge_node,
    ])


if __name__ == '__main__':
    generate_launch_description()