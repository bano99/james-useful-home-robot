#!/usr/bin/env python3
"""
Test script for servo serial control via ESP32
Tests the text-based command protocol
"""

import serial
import time
import sys

def test_servo_commands(port='/dev/ttyUSB0', baudrate=115200):
    """Test servo control commands"""
    
    print(f"Connecting to {port} at {baudrate} baud...")
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)  # Wait for ESP32 to initialize
        
        # Clear any startup messages
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        print("\n" + "="*50)
        print("Starting Servo Control Tests")
        print("="*50 + "\n")
        
        # Test 1: Help command
        print("Test 1: Help command")
        ser.write(b"?\n")
        time.sleep(0.5)
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        # Test 2: Ping servo
        print("\nTest 2: Ping servo ID 1")
        ser.write(b"#1PING\n")
        time.sleep(0.5)
        while ser.in_waiting:
            response = ser.readline().decode('utf-8', errors='ignore').strip()
            print(response)
        
        # Test 3: Move to center position
        print("\nTest 3: Move servo 1 to center (2047) at speed 1000")
        ser.write(b"#1P2047S1000\n")
        time.sleep(0.5)
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        time.sleep(2)  # Wait for movement
        
        # Test 4: Read position
        print("\nTest 4: Read current position")
        ser.write(b"#1P?\n")
        time.sleep(0.5)
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        # Test 5: Move to 0 position
        print("\nTest 5: Move servo 1 to 0 position at speed 500")
        ser.write(b"#1P0S500\n")
        time.sleep(0.5)
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        time.sleep(2)  # Wait for movement
        
        # Test 6: Move to max position
        print("\nTest 6: Move servo 1 to max (4095) at speed 500")
        ser.write(b"#1P4095S500\n")
        time.sleep(0.5)
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        time.sleep(3)  # Wait for movement
        
        # Test 7: Return to center
        print("\nTest 7: Return to center")
        ser.write(b"#1P2047S1000\n")
        time.sleep(0.5)
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        # Test 8: Disable torque
        print("\nTest 8: Disable torque (servo can be moved manually)")
        ser.write(b"#1T0\n")
        time.sleep(0.5)
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        print("\n>>> Servo torque disabled. Try moving it manually.")
        input("Press Enter to continue...")
        
        # Test 9: Enable torque
        print("\nTest 9: Enable torque")
        ser.write(b"#1T1\n")
        time.sleep(0.5)
        while ser.in_waiting:
            print(ser.readline().decode('utf-8', errors='ignore').strip())
        
        print("\n" + "="*50)
        print("All tests completed!")
        print("="*50)
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        print(f"\nMake sure:")
        print(f"  1. ESP32 is connected to {port}")
        print(f"  2. You have permission to access the port")
        print(f"  3. No other program is using the port")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        ser.close()
        sys.exit(0)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        # Try common ports
        import os
        if os.name == 'nt':  # Windows
            port = 'COM3'  # Adjust as needed
        else:  # Linux/Mac
            port = '/dev/ttyUSB0'
    
    print(f"Using port: {port}")
    print("If this is wrong, run: python test_serial_control.py <your_port>")
    print()
    
    test_servo_commands(port)
