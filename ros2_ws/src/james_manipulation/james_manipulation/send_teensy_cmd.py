#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import threading
import time

class TeensyCommander(Node):
    def __init__(self):
        super().__init__('teensy_commander')
        self.pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        # No subscription needed for fire-and-forget

    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        print(f"TX: {cmd}")

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 send_teensy_cmd.py \"COMMAND\"")
        return

    cmd = sys.argv[1]
    rclpy.init()
    node = TeensyCommander()
    
    # Wait briefly for connection to establish
    time.sleep(0.2)
    
    node.send_command(cmd)
    
    # Wait for publish to flush
    time.sleep(0.5)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
