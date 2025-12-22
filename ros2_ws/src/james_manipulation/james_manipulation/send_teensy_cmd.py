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
        self.sub = self.create_subscription(String, '/arm/teensy_raw_rx', self.rx_callback, 10)
        self.response_received = False
    
    def rx_callback(self, msg):
        print(f"RX: {msg.data}")
        self.response_received = True

    def send_command(self, cmd):
        msg = String()
        msg.data = cmd
        self.pub.publish(msg)
        print(f"TX: {cmd}")
        self.response_received = False

def main():
    if len(sys.argv) < 2:
        print("Usage: python3 send_teensy_cmd.py \"COMMAND\"")
        print("Example: python3 send_teensy_cmd.py \"RJA0B0...\"")
        return

    cmd = sys.argv[1]
    
    rclpy.init()
    node = TeensyCommander()
    
    # Wait for connection
    time.sleep(0.5)
    
    node.send_command(cmd)
    
    print("Waiting for response (Ctrl+C to quit)...")
    start_time = time.time()
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            # Optional: Exit after a few seconds if you just want a quick fire-and-forget
            # But user asked to "see response", so we keep listening.
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
