#!/usr/bin/env python3
"""
Recover from XJ_LIMIT Error

When XJ commands are being rejected due to limit checks, this script
uses RJ commands (which still work) to move the arm to a safe position.

Usage:
    python3 recover_from_xj_limit.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import math
import time


class XJRecovery(Node):
    def __init__(self):
        super().__init__('xj_recovery')
        
        # Publishers
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # State
        self.current_positions = [None] * 9
        self.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 
                           'arm_joint_4', 'arm_joint_5', 'arm_joint_6',
                           'arm_joint_7', 'arm_joint_8', 'arm_joint_9']
        self.received_state = False
        
        self.get_logger().info('XJ Recovery Node Started')
    
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        self.received_state = True
    
    def send_raw(self, cmd):
        """Send raw command to Teensy"""
        msg = String()
        msg.data = cmd
        self.raw_cmd_pub.publish(msg)
        self.get_logger().info(f'→ {cmd}')
    
    def move_to_safe_position(self):
        """Move arm to safe home position using RJ command"""
        if not self.received_state or any(p is None for p in self.current_positions[:6]):
            self.get_logger().error('No valid joint states received!')
            return False
        
        current_deg = [math.degrees(p) if p is not None else 0.0 for p in self.current_positions[:9]]
        
        self.get_logger().info('='*70)
        self.get_logger().info('CURRENT POSITION')
        self.get_logger().info('='*70)
        for i in range(6):
            self.get_logger().info(f'  J{i+1}: {current_deg[i]:7.2f}°')
        self.get_logger().info('='*70)
        self.get_logger().info('')
        
        # Define safe home position (all zeros)
        safe_pos = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        
        self.get_logger().info('Moving to safe home position (all joints at 0°)...')
        self.get_logger().info('')
        
        # Build RJ command
        cmd = "RJ"
        cmd += f"A{safe_pos[0]:.2f}"
        cmd += f"B{safe_pos[1]:.2f}"
        cmd += f"C{safe_pos[2]:.2f}"
        cmd += f"D{safe_pos[3]:.2f}"
        cmd += f"E{safe_pos[4]:.2f}"
        cmd += f"F{safe_pos[5]:.2f}"
        cmd += f"J7{safe_pos[6]:.2f}"
        cmd += f"J8{safe_pos[7]:.2f}"
        cmd += f"J9{safe_pos[8]:.2f}"
        cmd += "Sp25Ac15Dc15Rm80W0Lm111111"
        
        self.send_raw(cmd)
        self.get_logger().info('✓ Recovery command sent')
        self.get_logger().info('')
        self.get_logger().info('Waiting for movement to complete...')
        
        # Wait for movement
        time.sleep(5.0)
        
        # Spin to get updated position
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Show final position
        final_deg = [math.degrees(p) if p is not None else 0.0 for p in self.current_positions[:6]]
        
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('FINAL POSITION')
        self.get_logger().info('='*70)
        for i in range(6):
            self.get_logger().info(f'  J{i+1}: {final_deg[i]:7.2f}°')
        self.get_logger().info('='*70)
        self.get_logger().info('')
        self.get_logger().info('✓ Recovery complete! XJ commands should work now.')
        
        return True
    
    def run(self):
        """Main execution"""
        self.get_logger().info('Waiting for joint states...')
        
        # Wait for initial state
        timeout = 5.0
        start_time = time.time()
        while not self.received_state:
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout waiting for joint states!')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
        
        self.get_logger().info('✓ Received joint states')
        self.get_logger().info('')
        
        # Move to safe position
        return self.move_to_safe_position()


def main(args=None):
    rclpy.init(args=args)
    recovery = XJRecovery()
    
    try:
        recovery.run()
        time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        recovery.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
