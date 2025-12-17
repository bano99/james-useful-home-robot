from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='t265_camera',
            parameters=[{'device_type': 't265'}],
            output='screen'
        )
    ])
