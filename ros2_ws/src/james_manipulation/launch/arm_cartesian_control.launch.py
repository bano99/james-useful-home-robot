import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # UNMISSABLE VERSION CHECK FOR DEBUGGING
    print("\n" + "="*60)
    print("  LAUNCHING ARM CARTESIAN CONTROL (IRON GRIP) - VERSION: 2026-01-03-V4")
    print("  BACKEND: MoveGroup IK Service (Robust Mode)")
    print("  STABILITY: JointConstraints (J4/J6 Locked)")
    print("="*60 + "\n")
    
    # Define directories
    pkg_james_manipulation = get_package_share_directory('james_manipulation')
    pkg_james_description = get_package_share_directory('james_description')
    
    # 1. Get URDF via xacro
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
    
    # 2. Get SRDF via xacro
    srdf_path = os.path.join(pkg_james_description, 'srdf', 'james.srdf.xacro')
    import xacro
    robot_description_semantic_config = xacro.process_file(srdf_path)
    robot_description_semantic = {'robot_description_semantic': robot_description_semantic_config.toxml()}

    # 3. Load configurations
    kinematics_yaml = PathJoinSubstitution([pkg_james_manipulation, 'config', 'moveit', 'kinematics.yaml'])
    joint_limits_yaml = PathJoinSubstitution([pkg_james_manipulation, 'config', 'moveit', 'joint_limits.yaml'])
    ompl_yaml = PathJoinSubstitution([pkg_james_manipulation, 'config', 'moveit', 'ompl_planning.yaml'])
    
    # Declare launch arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(pkg_james_manipulation, 'config', 'arm_cartesian_params.yaml'),
        description='Path to the configuration file'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level for all nodes'
    )
    
    # Get launch configurations
    config_file = LaunchConfiguration('config_file')
    log_level = LaunchConfiguration('log_level')
    
    # --- NODES ---

    # 4. MoveGroup Node (REQUIRED for IK Service)
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
            {'use_sim_time': False}
        ],
    )

    # 5. Platform Serial Bridge Node
    platform_bridge_node = Node(
        package='james_manipulation',
        executable='platform_serial_bridge',
        name='platform_serial_bridge',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )
    
    # 6. Teensy Serial Bridge Node
    teensy_bridge_node = Node(
        package='james_manipulation',
        executable='teensy_serial_bridge',
        name='teensy_serial_bridge',
        parameters=[
            config_file,
            {'config_file': os.path.join(pkg_james_manipulation, 'config', 'ARconfig.json')}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen'
    )

    # 7. Robot State Publisher Node
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # 8. Arm Cartesian Controller (The Main Logic)
    # Using the robust arm_cartesian_controller executable that calls IK service
    arm_cartesian_controller_node = Node(
        package='james_manipulation',
        executable='arm_cartesian_controller',
        name='arm_cartesian_controller',
        parameters=[config_file],
        arguments=['--ros-args', '--log-level', log_level],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        
        move_group_node,
        platform_bridge_node,
        teensy_bridge_node,
        robot_state_publisher_node,
        arm_cartesian_controller_node,
    ])