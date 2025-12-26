#!/usr/bin/env python3
"""
Verification Script for AR4 Motion Smoothing
This script sends a sequence of small joint commands to verify that the
blending mechanism is working (i.e., no shivering/stops between moves).
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import time
import sys

class SmoothingVerifier(Node):
    def __init__(self):
        super().__init__('smoothing_verifier')
        self.joint_pub = self.create_publisher(JointState, '/arm/joint_commands', 10)
        self.raw_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, 10)
        
        self.current_joints = None
        self.get_logger().info('Smoothing Verifier Node Started')

    def joint_cb(self, msg):
        self.current_joints = msg

    def send_raw(self, cmd):
        msg = String()
        msg.data = cmd
        self.raw_pub.publish(msg)
        self.get_logger().info(f'Sent Raw Command: {cmd}')

    def run_test(self, joint_idx=0, direction=1):
        self.get_logger().info('Waiting for initial joint states...')
        while rclpy.ok() and self.current_joints is None:
            rclpy.spin_once(self, timeout_sec=0.1)

        # 1. Enable Blending
        self.send_raw('BM1')
        time.sleep(0.5)

        # 2. Sequence of small moves
        start_pos = list(self.current_joints.position)
        names = self.current_joints.name
        
        steps = 45 # ~3 seconds at 15Hz
        step_size = math.radians(0.4) * direction # 0.4 degree steps
        rate = 15 # 15 Hz
        
        joint_name = names[joint_idx] if joint_idx < len(names) else f"J{joint_idx+1}"
        self.get_logger().info(f'--- Starting sequence for {joint_name} ---')
        self.get_logger().info(f'Steps: {steps}, Size: {math.degrees(step_size):.2f} deg, Rate: {rate}Hz')
        
        for i in range(steps):
            if not rclpy.ok(): break
            
            cmd = JointState()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.name = names
            
            new_pos = list(start_pos)
            new_pos[joint_idx] += (i + 1) * step_size
            cmd.position = new_pos
            
            self.joint_pub.publish(cmd)
            time.sleep(1.0 / rate)

        self.get_logger().info(f'--- Sequence for {joint_name} complete ---')
        time.sleep(0.5)

    def finish_test(self):
        # 3. Disable Blending and Stop
        self.send_raw('BM0')
        self.send_raw('ST')
        self.get_logger().info('Test Finished.')

def main():
    rclpy.init()
    node = SmoothingVerifier()
    try:
        # Test J1 (Base) - Left then Right
        node.get_logger().info("TEST 1: J1 Base Smoothness")
        node.run_test(joint_idx=0, direction=1)
        node.run_test(joint_idx=0, direction=-1)
        
        # Test J2 (Shoulder) - Forward then Backward
        node.get_logger().info("TEST 2: J2 Shoulder Smoothness")
        node.run_test(joint_idx=1, direction=1)
        node.run_test(joint_idx=1, direction=-1)
        
        node.finish_test()

    except KeyboardInterrupt:
        node.finish_test()
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
