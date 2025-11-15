#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
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
    
    # RTAB-Map parameters file
    parameters_file = os.path.join(pkg_dir, 'config', 'rtabmap_params.yaml')
    
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
        
        # RTAB-Map node
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
                
                # T265 odometry
                ('odom', '/t265/pose/sample'),
            ],
            arguments=['--delete_db_on_start']  # Remove this after first successful map
        ),
        
        # RTAB-Map visualization node (optional - requires display)
        # Uncomment to enable visualization
        # Node(
        #     package='rtabmap_viz',
        #     executable='rtabmapviz',
        #     name='rtabmapviz',
        #     output='screen',
        #     parameters=[
        #         parameters_file,
        #         {
        #             'use_sim_time': use_sim_time,
        #         }
        #     ],
        #     remappings=[
        #         ('rgb/image', '/camera/color/image_raw'),
        #         ('rgb/camera_info', '/camera/color/camera_info'),
        #         ('depth/image', '/camera/aligned_depth_to_color/image_raw'),
        #         ('odom', '/camera/pose/sample'),
        #     ],
        # ),
    ])
