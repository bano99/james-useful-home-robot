#!/usr/bin/env python3
"""
XJ Bug Reproduction Script

This script simulates joystick behavior to reproduce the "stuck" bug where
XJ commands stop working after some time.

The script:
1. Initializes Teensy to a fixed position (all joints 0°, J3=90°)
2. Sends random XJ movements (forward/backward/left/right)
3. Each movement follows the pattern: BM1 -> XJ commands -> BM0
4. Continues until the bug is reproduced or manually stopped

Usage:
    python3 test_xj_bug_reproduction.py /dev/ttyACM1
    
    Or through the bridge (if running):
    python3 test_xj_bug_reproduction.py --use-bridge
"""

import serial
import time
import random
import argparse
import sys
from datetime import datetime


class TeensyXJTester:
    def __init__(self, port, baudrate=115200, use_bridge=False):
        self.port = port
        self.baudrate = baudrate
        self.use_bridge = use_bridge
        self.ser = None
        
        # Initial position: All joints 0°, J3=90°
        self.init_position = {
            'J1': 0.0,
            'J2': 0.0,
            'J3': 90.0,
            'J4': 0.0,
            'J5': 0.0,
            'J6': 0.0
        }
        
        # Current position tracking
        self.current_position = self.init_position.copy()
        
        # Motion parameters
        self.max_deviation = 15.0  # Max deviation from init position
        self.primary_move_range = (8.0, 12.0)  # Primary joint moves ±10° ±2°
        self.movement_duration = 2.0  # seconds
        
        # Motion profile (matching bridge defaults)
        self.speed = 3.0
        self.accel = 10.0
        self.decel = 10.0
        self.ramp = 10.0
        
        # Statistics
        self.move_count = 0
        self.start_time = None
        self.last_response = None
        
    def connect(self):
        """Connect to Teensy via serial"""
        if self.use_bridge:
            print("Using ROS2 bridge mode - not implemented yet")
            sys.exit(1)
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            time.sleep(2.0)  # Wait for Teensy to initialize
            print(f"✓ Connected to Teensy on {self.port}")
            
            # Clear any pending data
            self.ser.reset_input_buffer()
            self.ser.reset_output_buffer()
            
            return True
        except Exception as e:
            print(f"✗ Failed to connect: {e}")
            return False
    
    def send_command(self, cmd, wait_response=True, timeout=2.0):
        """Send command to Teensy and optionally wait for response"""
        if not self.ser:
            print("✗ Not connected!")
            return None
        
        try:
            # Send command
            cmd_bytes = (cmd + '\n').encode('utf-8')
            self.ser.write(cmd_bytes)
            print(f"→ {cmd}")
            
            if not wait_response:
                return None
            
            # Wait for response
            start = time.time()
            response = ""
            while (time.time() - start) < timeout:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        response = line
                        print(f"← {line}")
                        self.last_response = line
                        return line
                time.sleep(0.01)
            
            print(f"⚠ No response (timeout after {timeout}s)")
            return None
            
        except Exception as e:
            print(f"✗ Error sending command: {e}")
            return None
    
    def initialize_position(self):
        """Initialize Teensy to fixed position using UP command"""
        print("\n" + "="*60)
        print("INITIALIZING TEENSY TO FIXED POSITION")
        print("="*60)
        print("Target: J1=0° J2=0° J3=90° J4=0° J5=0° J6=0°")
        print("Please ensure arm is approximately in this position!")
        print()
        
        # Send UP command to update position
        # UP command format: UPA<J1>B<J2>C<J3>D<J4>E<J5>F<J6>
        cmd = f"UPA{self.init_position['J1']:.2f}"
        cmd += f"B{self.init_position['J2']:.2f}"
        cmd += f"C{self.init_position['J3']:.2f}"
        cmd += f"D{self.init_position['J4']:.2f}"
        cmd += f"E{self.init_position['J5']:.2f}"
        cmd += f"F{self.init_position['J6']:.2f}"
        
        response = self.send_command(cmd, wait_response=True, timeout=3.0)
        
        if response:
            print("✓ Position initialized")
            return True
        else:
            print("✗ Failed to initialize position")
            return False
    
    def request_position(self):
        """Request current position from Teensy (RP command)"""
        response = self.send_command("RP", wait_response=True, timeout=2.0)
        if response and response.startswith('A'):
            # Parse position: A<j1>B<j2>C<j3>...
            try:
                parts = response.split('G')[0]  # Get joint part before cartesian
                self.current_position['J1'] = float(parts.split('A')[1].split('B')[0])
                self.current_position['J2'] = float(parts.split('B')[1].split('C')[0])
                self.current_position['J3'] = float(parts.split('C')[1].split('D')[0])
                self.current_position['J4'] = float(parts.split('D')[1].split('E')[0])
                self.current_position['J5'] = float(parts.split('E')[1].split('F')[0])
                self.current_position['J6'] = float(parts.split('F')[1])
                return True
            except Exception as e:
                print(f"⚠ Failed to parse position: {e}")
                return False
        return False
    
    def check_limits(self, target_position):
        """Check if target position exceeds max deviation from init"""
        for joint in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']:
            deviation = abs(target_position[joint] - self.init_position[joint])
            if deviation > self.max_deviation:
                return False, joint, deviation
        return True, None, 0
    
    def generate_random_movement(self):
        """Generate random movement (forward/backward/left/right)"""
        # Choose movement type
        movement_type = random.choice(['left', 'right', 'forward', 'backward'])
        
        # Start from current position
        target = self.current_position.copy()
        
        # Generate primary movement with randomness
        primary_delta = random.uniform(*self.primary_move_range)
        if random.random() < 0.5:
            primary_delta = -primary_delta
        
        # Apply movement
        if movement_type == 'left':
            # Left: J1-, J2+, J3-
            target['J1'] -= primary_delta
            target['J2'] += primary_delta * 0.3  # Secondary movement
            target['J3'] -= primary_delta * 0.2
        elif movement_type == 'right':
            # Right: J1+, J2-, J3+
            target['J1'] += primary_delta
            target['J2'] -= primary_delta * 0.3
            target['J3'] += primary_delta * 0.2
        elif movement_type == 'forward':
            # Forward: J2+, J3-
            target['J2'] += primary_delta
            target['J3'] -= primary_delta * 0.5
        else:  # backward
            # Backward: J2-, J3+
            target['J2'] -= primary_delta
            target['J3'] += primary_delta * 0.5
        
        # Check limits
        ok, joint, deviation = self.check_limits(target)
        if not ok:
            print(f"⚠ Movement would exceed limits ({joint}: {deviation:.1f}° > {self.max_deviation}°)")
            # Return to init position instead
            return self.init_position.copy(), 'return_home'
        
        return target, movement_type
    
    def send_xj_command(self, target_position):
        """Send XJ command to move to target position"""
        cmd = "XJ"
        cmd += f"A{target_position['J1']:.2f}"
        cmd += f"B{target_position['J2']:.2f}"
        cmd += f"C{target_position['J3']:.2f}"
        cmd += f"D{target_position['J4']:.2f}"
        cmd += f"E{target_position['J5']:.2f}"
        cmd += f"F{target_position['J6']:.2f}"
        cmd += f"Sp{self.speed:.2f}"
        cmd += f"Ac{self.accel:.2f}"
        cmd += f"Dc{self.decel:.2f}"
        cmd += f"Rm{self.ramp:.2f}"
        
        # Send without waiting for response (XJ is fire-and-forget)
        self.send_command(cmd, wait_response=False)
    
    def execute_movement_sequence(self):
        """Execute one complete movement sequence: BM1 -> XJ -> BM0"""
        self.move_count += 1
        
        print("\n" + "-"*60)
        print(f"MOVEMENT #{self.move_count} - {datetime.now().strftime('%H:%M:%S')}")
        print("-"*60)
        
        # Generate random movement
        target, movement_type = self.generate_random_movement()
        
        print(f"Movement type: {movement_type}")
        print(f"Current: J1={self.current_position['J1']:.1f}° J2={self.current_position['J2']:.1f}° J3={self.current_position['J3']:.1f}°")
        print(f"Target:  J1={target['J1']:.1f}° J2={target['J2']:.1f}° J3={target['J3']:.1f}°")
        
        # Step 1: Enable blending mode (BM1)
        print("\n[1/3] Starting Move -> Initial Priming (BM1)")
        self.send_command("BM1", wait_response=False)
        time.sleep(0.05)
        
        # Step 2: Send XJ commands (simulate continuous joystick input)
        print("[2/3] Joystick Active -> Pushing Target (XJ)")
        
        # Send multiple XJ commands over the movement duration
        num_commands = int(self.movement_duration / 0.1)  # Every 100ms
        for i in range(num_commands):
            # Interpolate between current and target
            progress = (i + 1) / num_commands
            interpolated = {}
            for joint in ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']:
                interpolated[joint] = self.current_position[joint] + \
                                     (target[joint] - self.current_position[joint]) * progress
            
            self.send_xj_command(interpolated)
            time.sleep(0.1)
        
        # Step 3: Stop and disable blending mode (BM0)
        print("[3/3] Joystick Idle -> Sending STOP (BM0)")
        self.send_command("BM0", wait_response=False)
        time.sleep(0.05)
        self.send_command("ST", wait_response=False)
        time.sleep(0.1)
        
        # Update current position
        self.current_position = target.copy()
        
        # Request actual position to verify
        print("\nVerifying position...")
        if self.request_position():
            print(f"Actual:  J1={self.current_position['J1']:.1f}° J2={self.current_position['J2']:.1f}° J3={self.current_position['J3']:.1f}°")
        
        # Calculate elapsed time
        if self.start_time:
            elapsed = time.time() - self.start_time
            print(f"\nElapsed time: {elapsed:.1f}s ({self.move_count} movements)")
    
    def run_test(self, max_movements=None):
        """Run the test loop"""
        print("\n" + "="*60)
        print("XJ BUG REPRODUCTION TEST")
        print("="*60)
        print(f"Port: {self.port}")
        print(f"Movement duration: {self.movement_duration}s")
        print(f"Max deviation: ±{self.max_deviation}°")
        print(f"Max movements: {max_movements if max_movements else 'unlimited'}")
        print("="*60)
        
        if not self.connect():
            return False
        
        if not self.initialize_position():
            return False
        
        print("\n✓ Initialization complete!")
        print("\nStarting movement test...")
        print("Press Ctrl+C to stop\n")
        
        self.start_time = time.time()
        
        try:
            while True:
                if max_movements and self.move_count >= max_movements:
                    print(f"\n✓ Completed {max_movements} movements without failure!")
                    break
                
                self.execute_movement_sequence()
                
                # Small delay between movements
                time.sleep(0.5)
                
        except KeyboardInterrupt:
            print("\n\n⚠ Test interrupted by user")
        except Exception as e:
            print(f"\n\n✗ Test failed with error: {e}")
            import traceback
            traceback.print_exc()
        finally:
            # Stop the arm
            print("\nStopping arm...")
            self.send_command("BM0", wait_response=False)
            time.sleep(0.05)
            self.send_command("ST", wait_response=False)
            
            # Print statistics
            if self.start_time:
                elapsed = time.time() - self.start_time
                print("\n" + "="*60)
                print("TEST STATISTICS")
                print("="*60)
                print(f"Total movements: {self.move_count}")
                print(f"Total time: {elapsed:.1f}s ({elapsed/60:.1f} minutes)")
                print(f"Average time per movement: {elapsed/self.move_count:.1f}s")
                print(f"Last response: {self.last_response}")
                print("="*60)
            
            if self.ser:
                self.ser.close()
                print("\n✓ Serial connection closed")


def main():
    parser = argparse.ArgumentParser(description='XJ Bug Reproduction Test')
    parser.add_argument('port', nargs='?', default='/dev/ttyACM1',
                       help='Serial port (default: /dev/ttyACM1)')
    parser.add_argument('--use-bridge', action='store_true',
                       help='Use ROS2 bridge instead of direct serial')
    parser.add_argument('--max-movements', type=int, default=None,
                       help='Maximum number of movements (default: unlimited)')
    parser.add_argument('--duration', type=float, default=2.0,
                       help='Movement duration in seconds (default: 2.0)')
    
    args = parser.parse_args()
    
    tester = TeensyXJTester(args.port, use_bridge=args.use_bridge)
    tester.movement_duration = args.duration
    
    success = tester.run_test(max_movements=args.max_movements)
    
    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
