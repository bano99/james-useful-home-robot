#!/usr/bin/env python3
"""
Launch file to view a saved RTAB-Map database without cameras.
Much lighter on resources - only loads the map for visualization.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('james_slam')
    
    # Parameters
    database_path = LaunchConfiguration('database_path')
    
    # RTAB-Map parameters file
    parameters_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='Path to RTAB-Map database to view'
        ),
        
        # RTAB-Map in localization mode (no mapping, just viewing)
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                parameters_file,
                {
                    'use_sim_time': False,
                    'Mem/IncrementalMemory': 'false',  # Localization mode
                    'Mem/InitWMWithAllNodes': 'true',  # Load all nodes
                    'Db/Path': database_path,
                }
            ],
            arguments=['']  # No delete_db_on_start
        ),
    ])
