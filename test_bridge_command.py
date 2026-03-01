#!/usr/bin/env python3
"""
Quick script to send commands through the ROS2 bridge
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time

class CommandSender(Node):
    def __init__(self):
        super().__init__('command_sender')
        self.pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        time.sleep(0.5)  # Wait for publisher to be ready
        
    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.get_logger().info(f'Sending: {cmd}')
        self.pub.publish(msg)
        time.sleep(0.1)

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 test_bridge_command.py <command>")
        print("Examples:")
        print("  python3 test_bridge_command.py RP")
        print("  python3 test_bridge_command.py DS")
        sys.exit(1)
    
    command = sys.argv[1]
    
    rclpy.init()
    sender = CommandSender()
    sender.send_command(command)
    
    print(f"\nCommand '{command}' sent through bridge.")
    print("Check bridge output for response.")
    
    rclpy.shutdown()

if __name__ == '__main__':
    main()
