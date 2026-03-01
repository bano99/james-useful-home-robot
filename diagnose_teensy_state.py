#!/usr/bin/env python3
"""
Teensy State Diagnostic Tool
=============================
This script diagnoses Teensy responsiveness issues by:
1. Querying STATUS command for internal state
2. Testing simple movement (J6 +10 degrees)
3. Testing blending mode movement (XJ command)
4. Comparing before/after states to identify issues

Usage:
  python diagnose_teensy_state.py /dev/ttyACM0        # First run after calibration
  python diagnose_teensy_state.py /dev/ttyACM0 --compare state_20250301_143022.json
"""

import serial
import time
import json
import sys
import argparse
from datetime import datetime
from pathlib import Path


class TeensyDiagnostic:
    def __init__(self, port, baudrate=115200, timeout=2.0):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None
        
    def connect(self):
        """Connect to Teensy"""
        print(f"Connecting to {self.port}...")
        self.ser = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
        time.sleep(0.5)  # Wait for connection to stabilize
        self.ser.reset_input_buffer()
        print("Connected.")
        
    def disconnect(self):
        """Disconnect from Teensy"""
        if self.ser and self.ser.is_open:
            self.ser.close()
            print("Disconnected.")
    
    def send_command(self, cmd):
        """Send command and return response"""
        if not self.ser or not self.ser.is_open:
            raise RuntimeError("Serial port not open")
        
        self.ser.reset_input_buffer()
        cmd_str = cmd if cmd.endswith('\n') else cmd + '\n'
        self.ser.write(cmd_str.encode())
        self.ser.flush()
        
        # Wait for response
        time.sleep(0.1)
        response = []
        start_time = time.time()
        
        while (time.time() - start_time) < self.timeout:
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if line:
                    response.append(line)
                    # For most commands, one line is enough
                    if not cmd.startswith('DS'):
                        break
            else:
                time.sleep(0.05)
        
        return response
    
    def get_status(self):
        """Get Teensy STATUS diagnostic info"""
        print("Querying STATUS...")
        response = self.send_command('DS')
        
        status = {
            'timestamp': datetime.now().isoformat(),
            'raw_response': response,
            'parsed': {}
        }
        
        # Parse STATUS response
        for line in response:
            if ':' in line:
                key, value = line.split(':', 1)
                status['parsed'][key.strip()] = value.strip()
        
        return status
    
    def get_position(self):
        """Request current position"""
        print("Requesting position (RP)...")
        response = self.send_command('RP')
        return response
    
    def test_simple_movement(self):
        """Test simple J6 movement (+10 degrees)"""
        print("\nTesting SIMPLE movement (J6 +10 deg)...")
        
        # Get current position first
        pos_before = self.get_position()
        print(f"  Position before: {pos_before}")
        
        # Send simple move command for J6 +10 degrees
        # Format: MJA<J1>B<J2>C<J3>D<J4>E<J5>F<J6>Ss<speed>Ac<acc>Dc<dec>Rm<ramp>
        # We only move J6, so we need current positions for others
        # Simplified: just send relative J6 movement
        cmd = "MJA0B0C0D0E0F10Ss20Ac10Dc10Rm20"
        print(f"  Sending: {cmd}")
        
        response = self.send_command(cmd)
        print(f"  Response: {response}")
        
        # Wait for movement to complete
        time.sleep(2.0)
        
        # Get position after
        pos_after = self.get_position()
        print(f"  Position after: {pos_after}")
        
        return {
            'command': cmd,
            'response': response,
            'position_before': pos_before,
            'position_after': pos_after
        }
    
    def test_blending_movement(self):
        """Test blending mode movement (XJ command)"""
        print("\nTesting BLENDING movement (XJ command, J6 +10 deg)...")
        
        # Enable blending mode
        print("  Enabling blending mode...")
        blend_resp = self.send_command('BM1')
        print(f"  Blending response: {blend_resp}")
        
        # Get current position
        pos_before = self.get_position()
        print(f"  Position before: {pos_before}")
        
        # Send XJ command for J6 +10 degrees
        # Format: XJA<J1>B<J2>C<J3>D<J4>E<J5>F<J6>Ss<speed>Ac<acc>Dc<dec>Rm<ramp>
        cmd = "XJA0B0C0D0E0F10Ss20Ac10Dc10Rm20"
        print(f"  Sending: {cmd}")
        
        response = self.send_command(cmd)
        print(f"  Response: {response}")
        
        # Wait for movement
        time.sleep(2.0)
        
        # Get position after
        pos_after = self.get_position()
        print(f"  Position after: {pos_after}")
        
        # Disable blending mode
        print("  Disabling blending mode...")
        self.send_command('BM0')
        
        return {
            'command': cmd,
            'response': response,
            'position_before': pos_before,
            'position_after': pos_after
        }
    
    def run_full_diagnostic(self):
        """Run complete diagnostic suite"""
        print("\n" + "="*60)
        print("TEENSY DIAGNOSTIC TEST")
        print("="*60)
        
        results = {
            'test_time': datetime.now().isoformat(),
            'port': self.port,
            'status': None,
            'simple_movement': None,
            'blending_movement': None
        }
        
        try:
            self.connect()
            
            # 1. Get STATUS
            results['status'] = self.get_status()
            print(f"\nSTATUS received: {len(results['status']['raw_response'])} lines")
            
            # 2. Test simple movement
            results['simple_movement'] = self.test_simple_movement()
            
            # 3. Test blending movement
            results['blending_movement'] = self.test_blending_movement()
            
            # 4. Get final STATUS
            results['status_after'] = self.get_status()
            
        finally:
            self.disconnect()
        
        return results
    
    def save_results(self, results, filename=None):
        """Save results to JSON file"""
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"teensy_state_{timestamp}.json"
        
        filepath = Path(filename)
        with open(filepath, 'w') as f:
            json.dump(results, f, indent=2)
        
        print(f"\n{'='*60}")
        print(f"Results saved to: {filepath.absolute()}")
        print(f"{'='*60}")
        
        return filepath


def compare_states(file1, file2):
    """Compare two diagnostic state files and show differences"""
    print("\n" + "="*60)
    print("COMPARING DIAGNOSTIC STATES")
    print("="*60)
    
    with open(file1, 'r') as f:
        state1 = json.load(f)
    with open(file2, 'r') as f:
        state2 = json.load(f)
    
    print(f"\nState 1: {state1['test_time']}")
    print(f"State 2: {state2['test_time']}")
    
    # Compare STATUS fields
    print("\n--- STATUS COMPARISON ---")
    status1 = state1.get('status', {}).get('parsed', {})
    status2 = state2.get('status', {}).get('parsed', {})
    
    all_keys = set(status1.keys()) | set(status2.keys())
    
    differences = []
    for key in sorted(all_keys):
        val1 = status1.get(key, 'N/A')
        val2 = status2.get(key, 'N/A')
        
        if val1 != val2:
            differences.append((key, val1, val2))
            print(f"  {key}:")
            print(f"    Before: {val1}")
            print(f"    After:  {val2}")
    
    if not differences:
        print("  No differences in STATUS fields")
    
    # Compare movement responses
    print("\n--- MOVEMENT TEST COMPARISON ---")
    
    def compare_movement(name, mv1, mv2):
        print(f"\n{name}:")
        if mv1 and mv2:
            if mv1.get('response') != mv2.get('response'):
                print(f"  Response changed:")
                print(f"    Before: {mv1.get('response')}")
                print(f"    After:  {mv2.get('response')}")
            else:
                print(f"  Response: {mv1.get('response')}")
        elif mv1 and not mv2:
            print("  WARNING: Test succeeded before but failed after!")
        elif not mv1 and mv2:
            print("  Test now succeeds (was failing before)")
    
    compare_movement("Simple Movement", 
                    state1.get('simple_movement'), 
                    state2.get('simple_movement'))
    
    compare_movement("Blending Movement", 
                    state1.get('blending_movement'), 
                    state2.get('blending_movement'))
    
    # Summary
    print("\n" + "="*60)
    print("DIAGNOSTIC SUMMARY")
    print("="*60)
    
    if differences:
        print(f"\nFound {len(differences)} STATUS field differences")
        print("\nKey findings:")
        for key, val1, val2 in differences:
            print(f"  - {key} changed from '{val1}' to '{val2}'")
    else:
        print("\nNo STATUS differences detected.")
        print("This suggests the issue may be:")
        print("  - Serial communication problem")
        print("  - Command parsing issue")
        print("  - Timing/interrupt conflict")
    
    print("\n" + "="*60)


def main():
    parser = argparse.ArgumentParser(
        description='Diagnose Teensy responsiveness issues',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # First run (after calibration, when working)
  python diagnose_teensy_state.py /dev/ttyACM0
  
  # Second run (after Teensy stops responding)
  python diagnose_teensy_state.py /dev/ttyACM0
  
  # Compare two states
  python diagnose_teensy_state.py --compare teensy_state_20250301_143022.json teensy_state_20250301_144530.json
        """
    )
    
    parser.add_argument('port', nargs='?', help='Serial port (e.g., /dev/ttyACM0, COM3)')
    parser.add_argument('--compare', nargs=2, metavar=('FILE1', 'FILE2'),
                       help='Compare two diagnostic state files')
    parser.add_argument('--baudrate', type=int, default=115200,
                       help='Serial baudrate (default: 115200)')
    parser.add_argument('--timeout', type=float, default=2.0,
                       help='Serial timeout in seconds (default: 2.0)')
    
    args = parser.parse_args()
    
    if args.compare:
        compare_states(args.compare[0], args.compare[1])
        return
    
    if not args.port:
        parser.error("Serial port required (unless using --compare)")
    
    # Run diagnostic
    diag = TeensyDiagnostic(args.port, args.baudrate, args.timeout)
    results = diag.run_full_diagnostic()
    filepath = diag.save_results(results)
    
    print("\nNext steps:")
    print("  1. If Teensy stops responding, run this script again")
    print(f"  2. Compare states: python {sys.argv[0]} --compare {filepath} <new_state_file>")


if __name__ == '__main__':
    main()
