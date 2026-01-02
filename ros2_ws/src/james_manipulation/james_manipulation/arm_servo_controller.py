#!/usr/bin/env python3
"""
Arm Servo Controller for James Robot
Translates manual joystick commands to MoveIt Servo Twist/Joint commands.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import TwistStamped
from control_msgs.msg import JointJog
import json
import time
import math

class ArmServoController(Node):
    def __init__(self):
        super().__init__('arm_servo_controller')
        
        # Declare parameters
        self.declare_parameter('velocity_scale', 0.5)
        self.declare_parameter('rotation_scale', 0.1)
        self.declare_parameter('planning_frame', 'base_link')
        
        # Publishers for MoveIt Servo
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog, '/servo_node/delta_joint_cmds', 10)
        
        # Subscriber for manual commands
        self.manual_cmd_sub = self.create_subscription(
            String,
            '/arm/manual_cartesian_cmd',
            self.manual_command_callback,
            10
        )
        
        self.planning_frame = self.get_parameter('planning_frame').value
        self.get_logger().info('Arm Servo Controller initialized')

    def manual_command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            joy_lx = data.get('joy_lx', 0.0)
            joy_ly = data.get('joy_ly', 0.0)
            joy_ry = data.get('joy_ry', 0.0)
            joy_rr = data.get('joy_rr', 0.0)
            switch_mode = data.get('switch_mode', 'platform')
            if 'mode' in data:
                switch_mode = 'vertical' if data['mode'] == 1 else 'platform'

            v_scale = self.get_parameter('velocity_scale').value
            r_scale = self.get_parameter('rotation_scale').value

            # 1. Cartesian Translation (X, Y, Z) via Twist
            # These will be handled by the 'ar_translator' group in servo_node
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = self.planning_frame
            
            # Map Joystick to Twist
            # Stick Left -> Robot +X (Left)
            # Stick Forward -> Robot +Y (Forward)
            twist.twist.linear.x = -joy_lx * v_scale
            twist.twist.linear.y = -joy_ly * v_scale
            
            if switch_mode == 'vertical':
                twist.twist.linear.z = joy_ry * v_scale
            else:
                twist.twist.linear.z = 0.0

            # Publish twist if there is translation input
            if abs(joy_lx) > 0.05 or abs(joy_ly) > 0.05 or (switch_mode == 'vertical' and abs(joy_ry) > 0.05):
                self.twist_pub.publish(twist)

            # 2. Joint Jogging (J6)
            # Map joy_rr (rotation) or other axes to specific joints
            if abs(joy_rr) > 0.05:
                jog = JointJog()
                jog.header.stamp = self.get_clock().now().to_msg()
                jog.header.frame_id = self.planning_frame
                
                # If in vertical mode, RR is Yaw (J6 rotation)
                if switch_mode == 'vertical':
                    jog.joint_names = ['arm_joint_6']
                    jog.velocities = [joy_rr * r_scale * 5.0] # Scale for joint speed
                
                self.joint_pub.publish(jog)

        except Exception as e:
            self.get_logger().error(f'Error processing manual command: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = ArmServoController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
