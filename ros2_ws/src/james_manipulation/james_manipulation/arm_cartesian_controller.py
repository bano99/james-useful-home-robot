#!/usr/bin/env python3
"""
Arm Cartesian Controller Node for James Robot

This node receives manual control commands and converts them to Cartesian
movements using inverse kinematics, then sends joint commands to the Teensy.

Author: James Robot Team
Date: December 2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import json
import time
import math


class ArmCartesianController(Node):
    """
    ROS2 node for Cartesian control of AR4-MK3 robot arm
    """
    
    def __init__(self):
        super().__init__('arm_cartesian_controller')
        
        # Declare parameters
        self.declare_parameter('velocity_scale', 0.001)
        self.declare_parameter('rotation_scale', 0.01)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('ik_timeout', 0.05)
        
        # Workspace limits
        self.declare_parameter('workspace_limits.x_min', 0.2)
        self.declare_parameter('workspace_limits.x_max', 0.6)
        self.declare_parameter('workspace_limits.y_min', -0.3)
        self.declare_parameter('workspace_limits.y_max', 0.3)
        self.declare_parameter('workspace_limits.z_min', 0.1)
        self.declare_parameter('workspace_limits.z_max', 0.8)
        
        # MoveIt parameters
        self.declare_parameter('move_group_name', 'ar4_arm')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('end_effector_link', 'gripper_link')
        
        # Get parameters
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.control_rate = self.get_parameter('control_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.ik_timeout = self.get_parameter('ik_timeout').value
        
        # Workspace limits
        self.workspace_limits = {
            'x_min': self.get_parameter('workspace_limits.x_min').value,
            'x_max': self.get_parameter('workspace_limits.x_max').value,
            'y_min': self.get_parameter('workspace_limits.y_min').value,
            'y_max': self.get_parameter('workspace_limits.y_max').value,
            'z_min': self.get_parameter('workspace_limits.z_min').value,
            'z_max': self.get_parameter('workspace_limits.z_max').value,
        }
        
        # State variables
        self.last_command_time = 0.0
        self.current_pose = Pose()
        self.current_joint_state = JointState()
        self.manual_control_active = False
        
        # Initialize current pose to safe position
        self.current_pose.position.x = 0.4
        self.current_pose.position.y = 0.0
        self.current_pose.position.z = 0.3
        self.current_pose.orientation.w = 1.0
        
        # ROS2 subscribers
        self.manual_cmd_sub = self.create_subscription(
            String,
            '/arm/manual_cartesian_cmd',
            self.manual_command_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # ROS2 publishers
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/arm/joint_commands',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/arm/status',
            10
        )
        
        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        # Status timer
        self.status_timer = self.create_timer(
            0.1,  # 10 Hz status updates
            self.publish_status
        )
        
        self.get_logger().info('Arm Cartesian Controller started')
        self.get_logger().info(f'Velocity scale: {self.velocity_scale} m/s per unit')
        self.get_logger().info(f'Rotation scale: {self.rotation_scale} rad/s per unit')
    
    def manual_command_callback(self, msg):
        """Process manual control commands from platform bridge"""
        try:
            data = json.loads(msg.data)
            
            if data.get('type') == 'manual_control':
                self.last_command_time = time.time()
                self.manual_control_active = True
                
                # Extract joystick values
                left_x = data.get('left_x', 0)  # Cartesian Y
                left_y = data.get('left_y', 0)  # Cartesian X  
                left_z = data.get('left_z', 0)  # Cartesian Z
                
                switch_mode = data.get('switch_mode', 'platform')
                
                # Process arm control (left joystick always controls arm)
                self.process_cartesian_command(left_x, left_y, left_z, switch_mode)
                
                self.get_logger().debug(f'Manual command: X={left_y}, Y={left_x}, Z={left_z}')
                
        except json.JSONDecodeError as e:
            self.get_logger().warn(f'Invalid JSON in manual command: {msg.data}')
        except Exception as e:
            self.get_logger().error(f'Error processing manual command: {e}')
    
    def process_cartesian_command(self, joy_x, joy_y, joy_z, switch_mode):
        """Convert joystick input to Cartesian movement"""
        try:
            # Calculate velocity commands
            # Note: Joystick axes may need remapping based on physical setup
            vel_x = joy_y * self.velocity_scale  # Forward/back
            vel_y = joy_x * self.velocity_scale  # Left/right
            vel_z = joy_z * self.velocity_scale  # Up/down
            
            # Update target pose based on velocity
            dt = 1.0 / self.control_rate
            
            new_pose = Pose()
            new_pose.position.x = self.current_pose.position.x + vel_x * dt
            new_pose.position.y = self.current_pose.position.y + vel_y * dt
            new_pose.position.z = self.current_pose.position.z + vel_z * dt
            
            # Keep orientation unchanged for now
            new_pose.orientation = self.current_pose.orientation
            
            # Apply workspace limits
            new_pose = self.apply_workspace_limits(new_pose)
            
            # Update current pose
            self.current_pose = new_pose
            
            # TODO: Call IK solver and publish joint commands
            # For now, just log the target pose
            self.get_logger().debug(
                f'Target pose: X={new_pose.position.x:.3f}, '
                f'Y={new_pose.position.y:.3f}, Z={new_pose.position.z:.3f}'
            )
            
        except Exception as e:
            self.get_logger().error(f'Error processing Cartesian command: {e}')
    
    def apply_workspace_limits(self, pose):
        """Apply workspace safety limits to target pose"""
        limited_pose = Pose()
        limited_pose.orientation = pose.orientation
        
        # Clamp position to workspace limits
        limited_pose.position.x = max(
            self.workspace_limits['x_min'],
            min(self.workspace_limits['x_max'], pose.position.x)
        )
        limited_pose.position.y = max(
            self.workspace_limits['y_min'],
            min(self.workspace_limits['y_max'], pose.position.y)
        )
        limited_pose.position.z = max(
            self.workspace_limits['z_min'],
            min(self.workspace_limits['z_max'], pose.position.z)
        )
        
        return limited_pose
    
    def joint_state_callback(self, msg):
        """Update current joint state from Teensy bridge"""
        self.current_joint_state = msg
        
        # TODO: Update current pose from forward kinematics
        # For now, we maintain pose from commands
    
    def control_loop(self):
        """Main control loop"""
        current_time = time.time()
        
        # Check for command timeout
        if current_time - self.last_command_time > self.command_timeout:
            if self.manual_control_active:
                self.manual_control_active = False
                self.get_logger().info('Manual control timeout - stopping arm')
    
    def publish_status(self):
        """Publish arm status for monitoring"""
        try:
            current_time = time.time()
            command_age = current_time - self.last_command_time
            
            status = {
                'type': 'arm_status',
                'manual_active': self.manual_control_active,
                'command_age': command_age,
                'current_pose': {
                    'x': self.current_pose.position.x,
                    'y': self.current_pose.position.y,
                    'z': self.current_pose.position.z,
                    'qx': self.current_pose.orientation.x,
                    'qy': self.current_pose.orientation.y,
                    'qz': self.current_pose.orientation.z,
                    'qw': self.current_pose.orientation.w,
                },
                'ik_success': True,  # TODO: Update based on actual IK results
                'timestamp': current_time
            }
            
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ArmCartesianController()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()