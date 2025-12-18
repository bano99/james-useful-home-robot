import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition
import xacro

def generate_launch_description():
    # 1. Get package and file paths
    james_description_share = FindPackageShare('james_description')
    james_manipulation_share = FindPackageShare('james_manipulation')

    urdf_path = PathJoinSubstitution([james_description_share, 'urdf', 'james.urdf.xacro'])
    srdf_path = PathJoinSubstitution([james_description_share, 'srdf', 'james.srdf.xacro'])
    
    # 2. Process XACRO files
    # Note: Using get_package_share_directory for xacro command line
    from ament_index_python.packages import get_package_share_directory
    
    urdf_file = os.path.join(get_package_share_directory('james_description'), 'urdf', 'james.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    srdf_file = os.path.join(get_package_share_directory('james_description'), 'srdf', 'james.srdf.xacro')
    robot_description_semantic_config = xacro.process_file(srdf_file)
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config.toxml()}

    # 3. Load configurations
    kinematics_yaml = PathJoinSubstitution([james_manipulation_share, 'config', 'moveit', 'kinematics.yaml'])
    ompl_yaml = PathJoinSubstitution([james_manipulation_share, 'config', 'moveit', 'ompl_planning.yaml'])
    joint_limits_yaml = PathJoinSubstitution([james_manipulation_share, 'config', 'moveit', 'joint_limits.yaml'])

    # 4. Define Nodes
    move_group_node = Node(
        package='moveit_ros_move_group',
        executable='move_group',
        output='screen',
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            ompl_yaml,
            joint_limits_yaml,
            {'publish_robot_description_semantic': True},
            {'use_sim_time': LaunchConfiguration('use_sim_time', default='false')}
        ],
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )

    # Static Transform from world to base_footprint if not provided by SLAM
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'base_footprint']
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        move_group_node,
        robot_state_publisher,
        static_tf
    ])
