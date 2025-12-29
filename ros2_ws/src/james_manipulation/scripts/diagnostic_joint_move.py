import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
import sys
import math

class JointDiagnostic(Node):
    def __init__(self):
        super().__init__('joint_diagnostic')
        self.joint_cmd_pub = self.create_publisher(JointState, '/arm/joint_commands', 10)
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.callback, 10)
        self.current_joints = None
        self.joint_names = [
            'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 
            'arm_joint_4', 'arm_joint_5', 'arm_joint_6'
        ]
        
    def callback(self, msg):
        self.current_joints = msg.position

    def send_raw(self, text):
        msg = String()
        msg.data = text
        self.raw_cmd_pub.publish(msg)

    def move_joint(self, index, delta_deg):
        while self.current_joints is None:
            self.get_logger().info("Waiting for joint states...")
            rclpy.spin_once(self, timeout_sec=0.5)
            
        target_rad = list(self.current_joints)
        target_rad[index] += math.radians(delta_deg)
        
        # 1. Enable Blending
        self.send_raw("BM1")
        time.sleep(0.2)

        # 2. Send JointState command
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = target_rad
        msg.velocity = [20.0] # Speed
        
        self.get_logger().info(f"Moving Joint {index+1} by {delta_deg} degrees...")
        self.joint_cmd_pub.publish(msg)

        time.sleep(2.0)
        
        # 3. Stop
        self.send_raw("BM0")
        self.send_raw("ST")

import math

def main():
    rclpy.init()
    diag = JointDiagnostic()
    
    if len(sys.argv) < 3:
        print("Usage: python3 diagnostic_joint_move.py <joint_index 1-6> <delta_degrees>")
        return

    idx = int(sys.argv[1]) - 1
    delta = float(sys.argv[2])
    
    diag.move_joint(idx, delta)
    time.sleep(1.0)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
