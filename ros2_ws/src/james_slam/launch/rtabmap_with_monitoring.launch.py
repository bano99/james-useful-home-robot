#!/usr/bin/env python3
"""
RTAB-Map SLAM with Map Quality Monitoring.
Launches RTAB-Map with D435 camera and the map quality monitor node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('james_slam')
    
    # Parameters
    use_sim_time = LaunchConfiguration('use_sim_time')
    localization = LaunchConfiguration('localization')
    database_path = LaunchConfiguration('database_path')
    loop_closure_threshold = LaunchConfiguration('loop_closure_threshold')
    
    # RTAB-Map parameters file for D435-only
    parameters_file = os.path.join(pkg_dir, 'config', 'rtabmap_d435_only.yaml')
    
    return LaunchDescription([
        # Set environment variable for RTAB-Map
        SetEnvironmentVariable('RTABMAP_SYNC_MULTI_RGBD', '0'),
        
        # Launch arguments
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'localization',
            default_value='false',
            description='Start in localization mode (true) or mapping mode (false)'
        ),
        
        DeclareLaunchArgument(
            'database_path',
            default_value='~/.ros/rtabmap.db',
            description='Path to RTAB-Map database'
        ),
        
        DeclareLaunchArgument(
            'loop_closure_threshold',
            default_value='0.5',
            description='Minimum loop closure success rate (0.0-1.0)'
        ),
        
        # RTAB-Map node with visual odometry
        Node(
            package='rtabmap_slam',
            executable='rtabmap',
            name='rtabmap',
            output='screen',
            parameters=[
                parameters_file,
                {
                    'use_sim_time': use_sim_time,
                    'Mem/IncrementalMemory': LaunchConfiguration('localization', default='false'),
                    'Db/Path': database_path,
                }
            ],
            remappings=[
                # D435 camera topics
                ('rgb/image', '/d435/color/image_raw'),
                ('rgb/camera_info', '/d435/color/camera_info'),
                ('depth/image', '/d435/aligned_depth_to_color/image_raw'),
            ],
            arguments=['--delete_db_on_start']  # Remove this after first successful map
        ),
        
        # Visual odometry node
        Node(
            package='rtabmap_odom',
            executable='rgbd_odometry',
            name='rgbd_odometry',
            output='screen',
            parameters=[
                parameters_file,
                {
                    'use_sim_time': use_sim_time,
                    'frame_id': 'd435_link',
                    'publish_tf': True,
                    'wait_for_transform': 0.2,
                }
            ],
            remappings=[
                ('rgb/image', '/d435/color/image_raw'),
                ('rgb/camera_info', '/d435/color/camera_info'),
                ('depth/image', '/d435/aligned_depth_to_color/image_raw'),
            ],
        ),
        
        # Map Quality Monitor node
        Node(
            package='james_slam',
            executable='map_quality_monitor.py',
            name='map_quality_monitor',
            output='screen',
            parameters=[
                {
                    'use_sim_time': use_sim_time,
                    'loop_closure_threshold': loop_closure_threshold,
                    'window_size': 20,
                    'optimization_interval': 300.0,  # 5 minutes
                    'degradation_check_interval': 60.0,  # 1 minute
                }
            ],
        ),
    ])
