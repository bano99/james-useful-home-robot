#!/usr/bin/env python3
"""
XJ Limit Debugger

This script helps debug XJ_LIMIT errors by:
1. Showing current joint positions
2. Decoding XJ_LIMIT bitmap to show which joints are at fault
3. Checking joint limits from ARconfig.json
4. Suggesting recovery movements

Usage:
    python3 debug_xj_limits.py
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
import math
import time


class XJLimitDebugger(Node):
    def __init__(self):
        super().__init__('xj_limit_debugger')
        
        # Publishers
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.collision_sub = self.create_subscription(
            String, '/teensy/collision_status', self.status_callback, 10)
        
        # State
        self.current_positions = [None] * 9  # J1-J9
        self.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 
                           'arm_joint_4', 'arm_joint_5', 'arm_joint_6',
                           'arm_joint_7', 'arm_joint_8', 'arm_joint_9']
        self.received_state = False
        self.last_xj_limit = None
        
        # Load joint limits from config
        self.joint_limits = self.load_joint_limits()
        
        self.get_logger().info('XJ Limit Debugger Started')
    
    def load_joint_limits(self):
        """Load joint limits from ARconfig.json"""
        try:
            with open('ros2_ws/src/james_manipulation/config/ARconfig.json', 'r') as f:
                config = json.load(f)
            
            limits = {}
            for i in range(1, 10):
                pos_key = f'J{i}PosLim'
                neg_key = f'J{i}NegLim'
                limits[f'J{i}'] = {
                    'pos': float(config.get(pos_key, 0.0)),
                    'neg': float(config.get(neg_key, 0.0))
                }
            
            self.get_logger().info('✓ Loaded joint limits from ARconfig.json')
            return limits
        except Exception as e:
            self.get_logger().error(f'Failed to load joint limits: {e}')
            return {}
    
    def joint_state_callback(self, msg):
        """Update current joint positions"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        self.received_state = True
    
    def status_callback(self, msg):
        """Capture XJ_LIMIT messages"""
        if 'XJ_LIMIT' in msg.data:
            self.last_xj_limit = msg.data
            self.get_logger().info(f'← {msg.data}')
    
    def decode_xj_limit(self, bitmap_str):
        """Decode XJ_LIMIT bitmap to show which joints are at fault"""
        # bitmap_str is like "000100" (6 bits for J1-J6)
        faults = []
        for i, bit in enumerate(reversed(bitmap_str)):
            if bit == '1':
                faults.append(f'J{i+1}')
        return faults
    
    def check_xj_movement(self, target_deg):
        """Check if an XJ movement would trigger limit faults (simulating Teensy logic)"""
        faults = []
        
        for i in range(6):
            current = math.degrees(self.current_positions[i])
            target = target_deg[i]
            joint_name = f'J{i+1}'
            
            if joint_name not in self.joint_limits:
                continue
            
            limits = self.joint_limits[joint_name]
            min_angle = -limits['neg']
            max_angle = limits['pos']
            
            # Check if target would exceed limits
            if target > max_angle:
                faults.append({
                    'joint': joint_name,
                    'current': current,
                    'target': target,
                    'limit': max_angle,
                    'type': 'MAX',
                    'overshoot': target - max_angle
                })
            elif target < min_angle:
                faults.append({
                    'joint': joint_name,
                    'current': current,
                    'target': target,
                    'limit': min_angle,
                    'type': 'MIN',
                    'overshoot': min_angle - target
                })
        
        return faults
    
    def send_raw(self, cmd):
        """Send raw command to Teensy"""
        msg = String()
        msg.data = cmd
        self.raw_cmd_pub.publish(msg)
        self.get_logger().info(f'→ {cmd}')
    
    def analyze_current_state(self):
        """Analyze current joint positions and limits"""
        if not self.received_state or any(p is None for p in self.current_positions[:6]):
            self.get_logger().error('No valid joint states received yet')
            return
        
        self.get_logger().info('='*70)
        self.get_logger().info('CURRENT JOINT STATE ANALYSIS')
        self.get_logger().info('='*70)
        
        # Show current positions and limits
        for i in range(6):
            pos_deg = math.degrees(self.current_positions[i])
            joint_name = f'J{i+1}'
            
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                pos_lim = limits['pos']  # Maximum positive angle
                neg_lim = limits['neg']  # Maximum negative angle (as positive number)
                
                # The actual range is: -neg_lim to +pos_lim
                min_angle = -neg_lim
                max_angle = pos_lim
                
                # Calculate margin to limits
                margin_to_max = max_angle - pos_deg
                margin_to_min = pos_deg - min_angle
                
                # Determine status
                status = '✓ OK'
                if pos_deg > max_angle:
                    status = f'❌ EXCEEDS MAX (over by {pos_deg - max_angle:.1f}°)'
                elif pos_deg < min_angle:
                    status = f'❌ EXCEEDS MIN (under by {min_angle - pos_deg:.1f}°)'
                elif margin_to_max < 5.0:
                    status = f'⚠ NEAR MAX LIMIT (margin: {margin_to_max:.1f}°)'
                elif margin_to_min < 5.0:
                    status = f'⚠ NEAR MIN LIMIT (margin: {margin_to_min:.1f}°)'
                
                self.get_logger().info(
                    f'{joint_name}: {pos_deg:7.2f}° | '
                    f'Range: [{min_angle:7.2f}° to {max_angle:7.2f}°] | {status}'
                )
            else:
                self.get_logger().info(f'{joint_name}: {pos_deg:7.2f}° | Limits: UNKNOWN')
        
        self.get_logger().info('='*70)
        
        # Decode last XJ_LIMIT if available
        if self.last_xj_limit:
            bitmap = self.last_xj_limit.split(':')[1]
            faults = self.decode_xj_limit(bitmap)
            self.get_logger().info(f'Last XJ_LIMIT: {bitmap} → Faulted joints: {", ".join(faults)}')
            self.get_logger().info('='*70)
    
    def test_xj_command(self, target_deg_str):
        """Test if an XJ command would be accepted"""
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('XJ COMMAND TEST')
        self.get_logger().info('='*70)
        
        # Parse target angles from string like "A-68.12B0.38C-27.40D4.54E11.29F104.00"
        import re
        matches = re.findall(r'([A-F])([-\d.]+)', target_deg_str)
        target_deg = [0.0] * 6
        
        for letter, value in matches:
            idx = ord(letter) - ord('A')
            target_deg[idx] = float(value)
        
        self.get_logger().info(f'Testing XJ command: {target_deg_str}')
        self.get_logger().info('')
        
        # Check for limit violations
        faults = self.check_xj_movement(target_deg)
        
        if faults:
            self.get_logger().info('❌ COMMAND WOULD BE REJECTED:')
            self.get_logger().info('')
            for fault in faults:
                self.get_logger().info(
                    f"  {fault['joint']}: {fault['current']:.2f}° → {fault['target']:.2f}° "
                    f"exceeds {fault['type']} limit ({fault['limit']:.2f}°) "
                    f"by {fault['overshoot']:.2f}°"
                )
        else:
            self.get_logger().info('✓ Command should be accepted (all targets within limits)')
        
        self.get_logger().info('='*70)
    
    def suggest_recovery(self):
        """Suggest recovery movements to get away from limits"""
        if not self.received_state or any(p is None for p in self.current_positions[:6]):
            return
        
        self.get_logger().info('')
        self.get_logger().info('RECOVERY SUGGESTIONS:')
        self.get_logger().info('-'*70)
        
        suggestions = []
        
        for i in range(6):
            pos_deg = math.degrees(self.current_positions[i])
            joint_name = f'J{i+1}'
            
            if joint_name in self.joint_limits:
                limits = self.joint_limits[joint_name]
                pos_lim = limits['pos']
                neg_lim = limits['neg']
                
                # The actual range is: -neg_lim to +pos_lim
                min_angle = -neg_lim
                max_angle = pos_lim
                
                margin_to_max = max_angle - pos_deg
                margin_to_min = pos_deg - min_angle
                
                # If near maximum limit, suggest moving toward center
                if margin_to_max < 10.0 or pos_deg > max_angle:
                    safe_pos = pos_deg - 15.0
                    suggestions.append((joint_name, safe_pos, f'Move away from max limit'))
                
                # If near minimum limit, suggest moving toward center
                elif margin_to_min < 10.0 or pos_deg < min_angle:
                    safe_pos = pos_deg + 15.0
                    suggestions.append((joint_name, safe_pos, f'Move away from min limit'))
        
        if suggestions:
            self.get_logger().info('Joints near limits detected. Suggested recovery:')
            self.get_logger().info('')
            
            # Build RJ command to move to safe position
            current_deg = [math.degrees(p) if p is not None else 0.0 for p in self.current_positions[:9]]
            target_deg = current_deg.copy()
            
            for joint_name, safe_pos, reason in suggestions:
                joint_idx = int(joint_name[1]) - 1
                target_deg[joint_idx] = safe_pos
                self.get_logger().info(f'  {joint_name}: {current_deg[joint_idx]:.1f}° → {safe_pos:.1f}° ({reason})')
            
            self.get_logger().info('')
            self.get_logger().info('Recovery command (RJ):')
            
            cmd = "RJ"
            cmd += f"A{target_deg[0]:.2f}"
            cmd += f"B{target_deg[1]:.2f}"
            cmd += f"C{target_deg[2]:.2f}"
            cmd += f"D{target_deg[3]:.2f}"
            cmd += f"E{target_deg[4]:.2f}"
            cmd += f"F{target_deg[5]:.2f}"
            cmd += f"J7{target_deg[6]:.2f}"
            cmd += f"J8{target_deg[7]:.2f}"
            cmd += f"J9{target_deg[8]:.2f}"
            cmd += "Sp25Ac15Dc15Rm80W0Lm111111"
            
            self.get_logger().info(f'  {cmd}')
            self.get_logger().info('')
            
            response = input('Send recovery command? (y/n): ')
            if response.lower() == 'y':
                self.send_raw(cmd)
                self.get_logger().info('✓ Recovery command sent')
                time.sleep(3.0)
                
                # Spin to get updated position
                for _ in range(10):
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                self.get_logger().info('')
                self.analyze_current_state()
        else:
            self.get_logger().info('✓ All joints have sufficient margin from limits')
        
        self.get_logger().info('-'*70)
    
    def run(self):
        """Main loop"""
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
        
        # Analyze current state
        self.analyze_current_state()
        
        # Test the failed XJ command
        self.test_xj_command('A-68.1190B0.3770C-27.3990D4.5350E11.2870F104.0009')
        
        # Suggest recovery if needed
        self.suggest_recovery()
        
        return True


def main(args=None):
    rclpy.init(args=args)
    debugger = XJLimitDebugger()
    
    try:
        debugger.run()
    except KeyboardInterrupt:
        pass
    finally:
        debugger.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
