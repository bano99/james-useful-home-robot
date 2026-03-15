#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class JoystickSim(Node):
    def __init__(self):
        super().__init__('joystick_simulator')
        self.cmd_pub = self.create_publisher(String, '/arm/manual_cartesian_cmd', 10)
        
    def send_command(self, lx=0.0, ly=0.0, ry=0.0, rx=0.0, rr=0.0, mode='platform'):
        msg = {
            'type': 'manual_control',
            'armed': True,
            'lx': lx,
            'ly': ly,
            'ry': ry,
            'rx': rx,
            'rr': rr,
            'switch_mode': mode
        }
        ros_msg = String()
        ros_msg.data = json.dumps(msg)
        self.cmd_pub.publish(ros_msg)
        self.get_logger().info(f"Published: Y:{ly} X:{lx} Z:{ry}")

    def test_sequence(self):
        self.get_logger().info("--- STARTING REPRODUCIBLE JOYSTICK TEST ---")
        time.sleep(1.0) # wait for controller idle

        # Test 1: Pure Forward Y axis Translation
        self.get_logger().info("Test 1: Forward Y (ly=1.0) for 2 seconds")
        for _ in range(20):
            self.send_command(ly=1.0)
            time.sleep(0.1)
        
        self.get_logger().info("Test 1: Stopping")
        for _ in range(5):
            self.send_command(ly=0.0)
            time.sleep(0.1)
            
        time.sleep(2.0)

        # Test 2: Diagonal X/Y Translation
        self.get_logger().info("Test 2: Diagonal X/Y (lx=1.0, ly=1.0) for 2 seconds")
        for _ in range(20):
            self.send_command(lx=1.0, ly=1.0)
            time.sleep(0.1)
        
        self.get_logger().info("Test 2: Stopping")
        for _ in range(5):
            self.send_command(lx=0.0, ly=0.0)
            time.sleep(0.1)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickSim()
    try:
        node.test_sequence()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
