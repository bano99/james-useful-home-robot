import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # UNMISSABLE VERSION CHECK FOR DEBUGGING
    print("\n" + "="*60)
    print("  LAUNCHING ARM CARTESIAN CONTROL - VERSION: 2024-12-21-V6-FIXED")
    print("="*60 + "\n")

    # Define directories
    pkg_james_manipulation = get_package_share_directory('james_manipulation')
    pkg_james_description = get_package_share_directory('james_description')

    # Get URDF via xacro
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("james_description"), "urdf", "james.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}
    
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
    
    ar_config_arg = DeclareLaunchArgument(
        'ar_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('james_manipulation'),
            'config',
            'ARconfig.json'
        ]),
        description='Path to ARconfig.json'
    )
    
    platform_port_arg = DeclareLaunchArgument(
        'platform_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Platform Controller'
    )

    teensy_port_arg = DeclareLaunchArgument(
        'teensy_port',
        default_value='/dev/ttyACM1',
        description='Serial port for Teensy (AR4)'
    )

    enable_auto_detect_arg = DeclareLaunchArgument(
        'enable_auto_detect',
        default_value='true',
        description='Enable serial port auto-detection'
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
    platform_port = LaunchConfiguration('platform_port')
    teensy_port = LaunchConfiguration('teensy_port')
    enable_auto_detect = LaunchConfiguration('enable_auto_detect')
    ar_config = LaunchConfiguration('ar_config')
    
    # Platform Serial Bridge Node
    platform_bridge_node = Node(
        package='james_manipulation',
        executable='platform_serial_bridge',
        name='platform_serial_bridge',
        parameters=[
            config_file,
            {
                'serial_port': platform_port,
                'enable_auto_detect': enable_auto_detect,
                'use_sim_time': use_sim_time
            }
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
            {
                'serial_port': teensy_port,
                'enable_auto_detect': enable_auto_detect,
                'use_sim_time': use_sim_time,
                'config_file': ar_config
            }
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )

    # Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )
    
    return LaunchDescription([
        # Launch arguments
        config_file_arg,
        platform_port_arg,
        teensy_port_arg,
        enable_auto_detect_arg,
        use_sim_time_arg,
        log_level_arg,
        ar_config_arg,
        
        # Nodes
        platform_bridge_node,
        arm_controller_node,
        teensy_bridge_node,
        robot_state_publisher_node,
    ])