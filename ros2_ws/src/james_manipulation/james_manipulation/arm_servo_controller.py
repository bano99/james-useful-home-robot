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
import tf2_ros
import json
import time
import math

class ArmServoController(Node):
    def __init__(self):
        super().__init__('arm_servo_controller')
        
        # Declare parameters
        self.declare_parameter('velocity_scale', 0.5)
        self.declare_parameter('rotation_scale', 0.1)
        self.declare_parameter('planning_frame', 'arm_base_link')
        
        # TF Listener for diagnostics
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Publishers for MoveIt Servo
        self.twist_pub = self.create_publisher(TwistStamped, '/servo_server/delta_twist_cmds', 10)
        self.joint_pub = self.create_publisher(JointJog, '/servo_server/delta_joint_cmds', 10)
        
        # Publisher for raw bridge commands (to enable BM1)
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        
        # Subscriber for manual commands
        self.manual_cmd_sub = self.create_subscription(
            String,
            '/arm/manual_cartesian_cmd',
            self.manual_command_callback,
            10
        )
        
        self.planning_frame = self.get_parameter('planning_frame').value
        self.last_bm_sent = 0
        self.last_input_time = 0
        self.is_moving = False
        
        # Timer for inactivity cleanup (BM0, ST)
        self.cleanup_timer = self.create_timer(0.2, self.cleanup_callback)
        self.get_logger().info('Arm Servo Controller initialized')

    def cleanup_callback(self):
        """Reverts to precise mode (BM0) and stops the arm after inactivity"""
        if self.is_moving and (time.time() - self.last_input_time > 0.5):
            self.get_logger().info('Inactivity detected. Reverting to Precise Mode (BM0) and sending STOP.')
            self.raw_cmd_pub.publish(String(data="BM0"))
            time.sleep(0.05)
            self.raw_cmd_pub.publish(String(data="ST"))
            self.is_moving = False
            self.last_bm_sent = 0

    def manual_command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            # Support both 'lx' and 'joy_lx' formats
            joy_lx = data.get('lx', data.get('joy_lx', 0.0))
            joy_ly = data.get('ly', data.get('joy_ly', 0.0))
            joy_ry = data.get('ry', data.get('joy_ry', 0.0))
            joy_rr = data.get('rr', data.get('joy_rr', 0.0))
            switch_mode = data.get('mode', data.get('switch_mode', 'platform'))
            
            if isinstance(switch_mode, int):
                switch_mode = 'vertical' if switch_mode == 1 else 'platform'

            # User requested 0.05 deadzone
            active_input = abs(joy_lx) > 0.05 or abs(joy_ly) > 0.05 or abs(joy_ry) > 0.05 or abs(joy_rr) > 0.05

            if active_input:
                self.last_input_time = time.time()
                
                # Only enable Blending Mode (BM1) if input is significant (> 0.1)
                # This prevents drift from triggering the high-latency mode prematurely
                if not self.is_moving and (abs(joy_lx) > 0.1 or abs(joy_ly) > 0.1 or abs(joy_ry) > 0.1 or abs(joy_rr) > 0.1):
                    self.get_logger().info('Significant movement detected. Enabling Blending Mode (BM1).')
                    self.raw_cmd_pub.publish(String(data="BM1"))
                    self.is_moving = True
                    self.last_bm_sent = time.time()
                
                now = time.time()
                if now - self.last_bm_sent > 3.0:
                    self.raw_cmd_pub.publish(String(data="BM1"))
                    self.last_bm_sent = now

                # [JAMES:DIAG] Log Current Pose for debugging
                try:
                    trans = self.tf_buffer.lookup_transform('arm_base_link', 'arm_ee_link', rclpy.time.Time())
                    self.get_logger().info(f"Current EE Pose: X:{trans.transform.translation.x:.3f}, Y:{trans.transform.translation.y:.3f}, Z:{trans.transform.translation.z:.3f}", throttle_duration_sec=2.0)
                except Exception:
                    pass

                self.get_logger().info(f'Joystick Active: LX:{joy_lx:.2f}, LY:{joy_ly:.2f}, RY:{joy_ry:.2f}, RR:{joy_rr:.2f}, Mode:{switch_mode}', throttle_duration_sec=0.5)

            v_scale = self.get_parameter('velocity_scale').value
            r_scale = self.get_parameter('rotation_scale').value

            # 1. Cartesian Translation (X, Y, Z) via Twist
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = self.planning_frame
            twist.twist.linear.x = -joy_lx * v_scale
            twist.twist.linear.y = -joy_ly * v_scale
            
            if switch_mode == 'vertical':
                twist.twist.linear.z = joy_ry * v_scale
            else:
                twist.twist.linear.z = 0.0

            # Publish twist if there is translation input (> 0.05)
            if abs(joy_lx) > 0.05 or abs(joy_ly) > 0.05 or (switch_mode == 'vertical' and abs(joy_ry) > 0.05):
                self.get_logger().info(f'SENT Twist: X:{twist.twist.linear.x:.3f}, Y:{twist.twist.linear.y:.3f}, Z:{twist.twist.linear.z:.3f}', throttle_duration_sec=0.2)
                self.twist_pub.publish(twist)

            # 2. Joint Jogging (J6)
            if abs(joy_rr) > 0.05:
                jog = JointJog()
                jog.header.stamp = self.get_clock().now().to_msg()
                jog.header.frame_id = self.planning_frame
                
                if switch_mode == 'vertical':
                    jog.joint_names = ['arm_joint_6']
                    jog.velocities = [joy_rr * r_scale * 5.0]
                
                self.get_logger().info(f'SENT Jog: {jog.joint_names} -> {jog.velocities[0]:.3f}', throttle_duration_sec=0.2)
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
