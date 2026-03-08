#!/usr/bin/env python3
"""
XJ Bug Reproduction Script

This script simulates joystick behavior to reproduce the "stuck" bug where
XJ commands stop working after some time.

The script:
1. Checks if Teensy is calibrated
2. If NOT calibrated: Sets calibration flag and initializes position (NO MOVEMENT)
3. If calibrated: Runs joystick simulation with random XJ movements

Usage:
    python3 test_xj_bug_reproduction.py
    
    Or with options:
    python3 test_xj_bug_reproduction.py --max-movements 50 --duration 2.5
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String, Int32
import time
import random
import argparse
import sys
import math
from datetime import datetime


class XJBugTester(Node):
    def __init__(self, max_movements=None, movement_duration=2.0):
        super().__init__('xj_bug_tester')
        
        # Publishers
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        self.packet_count_sub = self.create_subscription(
            Int32, '/teensy/packet_count', self.packet_count_callback, 10)
        
        # Subscribe to collision status to capture GS response
        self.collision_sub = self.create_subscription(
            String, '/teensy/collision_status', self.status_callback, 10)
        
        # State tracking
        self.current_positions = [None] * 6  # radians
        self.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 
                           'arm_joint_4', 'arm_joint_5', 'arm_joint_6']
        self.received_state = False
        self.packet_count = 0
        self.last_status = None
        
        # Initial position: All joints 0°, J3=90°
        self.init_position_deg = [0.0, 0.0, 90.0, 0.0, 0.0, 0.0]
        self.init_position_rad = [math.radians(x) for x in self.init_position_deg]
        
        # Test parameters
        self.max_movements = max_movements
        self.movement_duration = movement_duration
        self.max_deviation = 15.0  # degrees
        
        # Motion profile
        self.speed = 3.0
        self.accel = 10.0
        self.decel = 10.0
        self.ramp = 10.0
        
        # Statistics
        self.move_count = 0
        self.start_time = None
        self.is_calibrated = False
        
        self.get_logger().info('XJ Bug Tester Node Started')
    
    def joint_state_callback(self, msg):
        """Update current joint positions from joint_states topic"""
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        self.received_state = True
    
    def packet_count_callback(self, msg):
        """Track hardware packet count"""
        self.packet_count = msg.data
    
    def status_callback(self, msg):
        """Capture status messages from Teensy"""
        self.last_status = msg.data
        if 'State:' in msg.data or 'XJ_LIMIT' in msg.data or 'BLENDING' in msg.data:
            self.get_logger().info(f'← {msg.data}')
    
    def send_raw(self, cmd):
        """Send raw command to Teensy through bridge"""
        msg = String()
        msg.data = cmd
        self.raw_cmd_pub.publish(msg)
        self.get_logger().info(f'→ {cmd}')
    
    def wait_for_feedback(self, timeout=5.0):
        """Wait for at least one hardware packet from Teensy"""
        start_count = self.packet_count
        start_time = time.time()
        self.get_logger().info('Waiting for Teensy feedback...')
        
        while self.packet_count <= start_count:
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout waiting for Teensy feedback!')
                return False
            rclpy.spin_once(self, timeout_sec=0.1)
            if not rclpy.ok():
                return False
        
        self.get_logger().info('✓ Received Teensy feedback')
        return True
    
    def check_calibration_status(self):
        """Check if Teensy is calibrated by sending GS command and parsing response"""
        self.get_logger().info('Checking calibration status...')
        self.last_status = None
        self.send_raw('GS')
        
        # Wait for response
        start_time = time.time()
        while time.time() - start_time < 1.0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.last_status and 'State:' in self.last_status:
                break
        
        if self.last_status and 'State:' in self.last_status:
            # Parse: State:1,0,0,0,0,0,0,0,0,0
            # State[0] = configured, State[1] = calibrated
            try:
                state_str = self.last_status.split('State:')[1]
                states = [int(x) for x in state_str.split(',')]
                
                if len(states) > 1:
                    if states[1] == 1:
                        self.is_calibrated = True
                        self.get_logger().info('✓ Teensy is CALIBRATED (State[1]=1)')
                        return True
                    else:
                        self.is_calibrated = False
                        self.get_logger().warn('⚠ Teensy is NOT CALIBRATED (State[1]=0)')
                        return False
            except Exception as e:
                self.get_logger().error(f'Failed to parse GS response: {e}')
        
        # Fallback: check if we have valid joint states
        if self.received_state and all(p is not None for p in self.current_positions):
            self.get_logger().warn('Could not get GS response, but joint_states are valid')
            self.is_calibrated = True
            return True
        
        self.is_calibrated = False
        return False
    
    def initialize_calibration(self):
        """Set calibration flag and initialize joint positions (NO MOVEMENT)"""
        self.get_logger().info('='*60)
        self.get_logger().info('INITIALIZING CALIBRATION')
        self.get_logger().info('='*60)
        self.get_logger().info('Target: J1=0° J2=0° J3=90° J4=0° J5=0° J6=0°')
        self.get_logger().info('IMPORTANT: Arm must be manually positioned at these angles!')
        self.get_logger().info('')
        
        # Wait for user confirmation
        input("Press ENTER when arm is positioned correctly, or Ctrl+C to abort...")
        
        # Send GSA1B1 to mark as calibrated
        self.get_logger().info('Marking Teensy as calibrated...')
        self.send_raw('GSA1B1')
        time.sleep(0.2)
        
        # Trigger position update by sending SS (Spline Stop)
        self.send_raw('SS')
        time.sleep(0.2)
        
        # Wait for feedback
        if not self.wait_for_feedback(timeout=3.0):
            self.get_logger().error('Failed to get feedback after calibration')
            return False
        
        self.is_calibrated = True
        self.get_logger().info('✓ Calibration initialized')
        self.get_logger().info(f'Current position: {[math.degrees(p) for p in self.current_positions]}')
        
        return True
    
    def generate_random_movement(self):
        """Generate random movement target (forward/backward/left/right)"""
        movement_type = random.choice(['left', 'right', 'forward', 'backward'])
        
        # Start from current position (in degrees)
        current_deg = [math.degrees(p) if p is not None else self.init_position_deg[i] 
                      for i, p in enumerate(self.current_positions)]
        target_deg = current_deg.copy()
        
        # Generate primary movement with randomness
        primary_delta = random.uniform(8.0, 12.0)
        if random.random() < 0.5:
            primary_delta = -primary_delta
        
        # Apply movement
        if movement_type == 'left':
            target_deg[0] -= primary_delta  # J1
            target_deg[1] += primary_delta * 0.3  # J2
            target_deg[2] -= primary_delta * 0.2  # J3
        elif movement_type == 'right':
            target_deg[0] += primary_delta
            target_deg[1] -= primary_delta * 0.3
            target_deg[2] += primary_delta * 0.2
        elif movement_type == 'forward':
            target_deg[1] += primary_delta  # J2
            target_deg[2] -= primary_delta * 0.5  # J3
        else:  # backward
            target_deg[1] -= primary_delta
            target_deg[2] += primary_delta * 0.5
        
        # Check limits (deviation from INIT position, not absolute)
        for i in range(6):
            deviation = abs(target_deg[i] - self.init_position_deg[i])
            if deviation > self.max_deviation:
                self.get_logger().warn(
                    f'Movement would exceed limits (J{i+1}: target={target_deg[i]:.1f}°, '
                    f'init={self.init_position_deg[i]:.1f}°, deviation={deviation:.1f}° > {self.max_deviation}°)'
                )
                # Return to init position instead
                return self.init_position_deg.copy(), 'return_home'
        
        return target_deg, movement_type
    
    def send_xj_command(self, target_deg):
        """Send XJ command with target joint angles"""
        cmd = "XJ"
        cmd += f"A{target_deg[0]:.2f}"
        cmd += f"B{target_deg[1]:.2f}"
        cmd += f"C{target_deg[2]:.2f}"
        cmd += f"D{target_deg[3]:.2f}"
        cmd += f"E{target_deg[4]:.2f}"
        cmd += f"F{target_deg[5]:.2f}"
        cmd += f"Sp{self.speed:.2f}"
        cmd += f"Ac{self.accel:.2f}"
        cmd += f"Dc{self.decel:.2f}"
        cmd += f"Rm{self.ramp:.2f}"
        
        self.send_raw(cmd)
    
    def execute_movement_sequence(self):
        """Execute one complete movement: BM1 -> XJ -> BM0"""
        self.move_count += 1
        
        self.get_logger().info('-'*60)
        self.get_logger().info(f'MOVEMENT #{self.move_count} - {datetime.now().strftime("%H:%M:%S")}')
        self.get_logger().info('-'*60)
        
        # Generate random movement
        target_deg, movement_type = self.generate_random_movement()
        
        # Get current position (use init if not available yet)
        current_deg = [math.degrees(p) if p is not None else self.init_position_deg[i] 
                      for i, p in enumerate(self.current_positions)]
        
        self.get_logger().info(f'Movement type: {movement_type}')
        self.get_logger().info(f'Current: J1={current_deg[0]:.1f}° J2={current_deg[1]:.1f}° J3={current_deg[2]:.1f}°')
        self.get_logger().info(f'Target:  J1={target_deg[0]:.1f}° J2={target_deg[1]:.1f}° J3={target_deg[2]:.1f}°')
        
        # Step 1: Enable blending mode (BM1)
        self.get_logger().info('[1/3] Starting Move -> Initial Priming (BM1)')
        self.send_raw('BM1')
        time.sleep(0.05)
        
        # Step 2: Send XJ commands (simulate continuous joystick)
        self.get_logger().info('[2/3] Joystick Active -> Pushing Target (XJ)')
        
        num_commands = int(self.movement_duration / 0.1)  # Every 100ms
        for i in range(num_commands):
            # Interpolate between current and target
            progress = (i + 1) / num_commands
            interpolated = []
            for j in range(6):
                interpolated.append(current_deg[j] + (target_deg[j] - current_deg[j]) * progress)
            
            self.send_xj_command(interpolated)
            time.sleep(0.1)
            
            # Spin to process callbacks
            rclpy.spin_once(self, timeout_sec=0.0)
        
        # Step 3: Stop and disable blending mode (BM0)
        self.get_logger().info('[3/3] Joystick Idle -> Sending STOP (BM0)')
        self.send_raw('BM0')
        time.sleep(0.05)
        self.send_raw('ST')
        time.sleep(0.2)
        
        # Spin to get updated position
        for _ in range(5):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        # Log actual position
        actual_deg = [math.degrees(p) if p is not None else 0.0 for p in self.current_positions]
        self.get_logger().info(f'Actual:  J1={actual_deg[0]:.1f}° J2={actual_deg[1]:.1f}° J3={actual_deg[2]:.1f}°')
        
        # Calculate elapsed time
        if self.start_time:
            elapsed = time.time() - self.start_time
            self.get_logger().info(f'Elapsed: {elapsed:.1f}s ({self.move_count} movements)')
    
    def run_test(self):
        """Main test loop"""
        self.get_logger().info('='*60)
        self.get_logger().info('XJ BUG REPRODUCTION TEST')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Movement duration: {self.movement_duration}s')
        self.get_logger().info(f'Max deviation: ±{self.max_deviation}°')
        self.get_logger().info(f'Max movements: {self.max_movements if self.max_movements else "unlimited"}')
        self.get_logger().info('='*60)
        
        # Trigger initial feedback
        self.send_raw('SS')
        
        # Wait for initial feedback
        if not self.wait_for_feedback(timeout=5.0):
            self.get_logger().error('No feedback from Teensy. Is the bridge running?')
            return False
        
        # Check calibration status
        if not self.check_calibration_status():
            # Not calibrated - initialize
            if not self.initialize_calibration():
                return False
        else:
            self.get_logger().info('✓ Teensy is already calibrated')
            self.get_logger().info(f'Current position: {[math.degrees(p) for p in self.current_positions]}')
        
        self.get_logger().info('')
        self.get_logger().info('✓ Initialization complete!')
        self.get_logger().info('Starting movement test...')
        self.get_logger().info('Press Ctrl+C to stop')
        self.get_logger().info('')
        
        self.start_time = time.time()
        
        try:
            while rclpy.ok():
                if self.max_movements and self.move_count >= self.max_movements:
                    self.get_logger().info(f'✓ Completed {self.max_movements} movements!')
                    break
                
                self.execute_movement_sequence()
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            self.get_logger().info('⚠ Test interrupted by user')
        except Exception as e:
            self.get_logger().error(f'✗ Test failed: {e}')
            import traceback
            traceback.print_exc()
        finally:
            # Stop the arm
            self.get_logger().info('Stopping arm...')
            self.send_raw('BM0')
            time.sleep(0.05)
            self.send_raw('ST')
            
            # Print statistics
            if self.start_time:
                elapsed = time.time() - self.start_time
                self.get_logger().info('='*60)
                self.get_logger().info('TEST STATISTICS')
                self.get_logger().info('='*60)
                self.get_logger().info(f'Total movements: {self.move_count}')
                self.get_logger().info(f'Total time: {elapsed:.1f}s ({elapsed/60:.1f} minutes)')
                if self.move_count > 0:
                    self.get_logger().info(f'Average: {elapsed/self.move_count:.1f}s per movement')
                self.get_logger().info('='*60)
        
        return True


def main(args=None):
    parser = argparse.ArgumentParser(description='XJ Bug Reproduction Test')
    parser.add_argument('--max-movements', type=int, default=None,
                       help='Maximum number of movements (default: unlimited)')
    parser.add_argument('--duration', type=float, default=2.0,
                       help='Movement duration in seconds (default: 2.0)')
    
    parsed_args = parser.parse_args()
    
    rclpy.init(args=args)
    tester = XJBugTester(
        max_movements=parsed_args.max_movements,
        movement_duration=parsed_args.duration
    )
    
    try:
        success = tester.run_test()
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()

