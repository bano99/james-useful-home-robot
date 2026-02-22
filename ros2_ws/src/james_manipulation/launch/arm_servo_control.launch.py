import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError: # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    print("\n" + "="*60)
    print("  LAUNCHING ARM SERVO CONTROL - VERSION: 2026-01-02-V1")
    print("="*60 + "\n")

    # Define directories
    pkg_james_manipulation = get_package_share_directory('james_manipulation')
    pkg_james_description = get_package_share_directory('james_description')

    # 1. Get URDF via xacro
    urdf_file = os.path.join(pkg_james_description, 'urdf', 'james.urdf.xacro')
    robot_description_config = xacro.process_file(urdf_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    # 2. Get SRDF via xacro
    srdf_file = os.path.join(pkg_james_description, 'srdf', 'james.srdf.xacro')
    robot_description_semantic_config = xacro.process_file(srdf_file)
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config.toxml()}

    # 3. Load other MoveIt configs
    kinematics_yaml = load_yaml('james_manipulation', 'config/moveit/kinematics.yaml')
    joint_limits_yaml = load_yaml('james_manipulation', 'config/moveit/joint_limits.yaml')
    servo_yaml = load_yaml('james_manipulation', 'config/moveit/moveit_servo.yaml')

    # 4. Define Nodes
    
    # Platform Serial Bridge Node
    platform_bridge_node = Node(
        package='james_manipulation',
        executable='platform_serial_bridge',
        name='platform_serial_bridge',
        parameters=[
            PathJoinSubstitution([pkg_james_manipulation, 'config', 'arm_cartesian_params.yaml']),
            {'use_sim_time': False}
        ],
        output='screen'
    )
    
    # Teensy Serial Bridge Node
    teensy_bridge_node = Node(
        package='james_manipulation',
        executable='teensy_serial_bridge',
        name='teensy_serial_bridge',
        parameters=[
            PathJoinSubstitution([pkg_james_manipulation, 'config', 'arm_cartesian_params.yaml']),
            {
                'use_sim_time': False,
                'config_file': PathJoinSubstitution([pkg_james_manipulation, 'config', 'ARconfig.json'])
            }
        ],
        output='screen'
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # MoveIt Servo Node (Composable Node in Foxy)
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer

    servo_node = ComposableNodeContainer(
        name='servo_server_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='moveit_servo',
                plugin='moveit_servo::ServoServer',
                name='servo_server',
                parameters=[
                    servo_yaml,
                    robot_description,
                    robot_description_semantic,
                    kinematics_yaml,
                    joint_limits_yaml,
                ],
            ),
        ],
        output='screen',
    )

    # Arm Servo Teleop Controller (Custom mapping node)
    arm_servo_controller_node = Node(
        package='james_manipulation',
        executable='arm_servo_controller',
        name='arm_servo_controller',
        parameters=[
            PathJoinSubstitution([pkg_james_manipulation, 'config', 'arm_cartesian_params.yaml']),
            {'use_sim_time': False}
        ],
        output='screen'
    )
    
    return LaunchDescription([
        platform_bridge_node,
        teensy_bridge_node,
        robot_state_publisher_node,
        servo_node,
        arm_servo_controller_node,
    ])
