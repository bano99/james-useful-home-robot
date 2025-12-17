#!/usr/bin/env python3
"""
Test script for gripper controller serial commands
Tests all commands without requiring actual hardware
"""

import serial
import time
import sys

# Configuration
PORT = 'COM3'  # Change to your port (COM3, /dev/ttyUSB0, etc.)
BAUD = 115200
TIMEOUT = 1

def test_connection(ser):
    """Test basic connection"""
    print("\n=== Testing Connection ===")
    ser.write(b'?\n')
    time.sleep(0.1)
    response = ser.read(ser.in_waiting).decode('utf-8', errors='ignore')
    if response:
        print("✓ Connection OK")
        print(response)
        return True
    else:
        print("✗ No response")
        return False

def test_sensor_commands(ser):
    """Test sensor commands"""
    print("\n=== Testing Sensor Commands ===")
    
    commands = [
        ('SS', 'Sensor Status'),
        ('RD', 'Read Distance'),
        ('RO', 'Read Orientation'),
        ('RC', 'Read Calibration'),
    ]
    
    for cmd, desc in commands:
        print(f"\nTesting {desc} ({cmd})...")
        ser.write(f'{cmd}\n'.encode())
        time.sleep(0.1)
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  Response: {response}")
        if response == "ERROR":
            print("  (Expected - sensors not connected)")

def test_torque_commands(ser):
    """Test torque commands"""
    print("\n=== Testing Torque Commands ===")
    
    # Test setting torque levels
    for level in range(1, 6):
        print(f"\nSetting torque level {level}...")
        ser.write(f'TL{level}\n'.encode())
        time.sleep(0.1)
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"  Response: {response}")
    
    # Test getting torque level
    print("\nGetting current torque level...")
    ser.write(b'GT\n')
    time.sleep(0.1)
    response = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  Response: {response}")
    
    # Test torque info
    print("\nGetting torque info...")
    ser.write(b'TI\n')
    time.sleep(0.1)
    response = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  Response: {response}")
    
    # Test increase/decrease
    print("\nTesting torque increase...")
    ser.write(b'T+\n')
    time.sleep(0.1)
    response = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  Response: {response}")
    
    print("\nTesting torque decrease...")
    ser.write(b'T-\n')
    time.sleep(0.1)
    response = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  Response: {response}")
    
    # Test get load
    print("\nGetting gripper load...")
    ser.write(b'GL\n')
    time.sleep(0.1)
    response = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  Response: {response}")
    if response == "ERROR":
        print("  (Expected - servo not connected)")

def test_gripper_commands(ser):
    """Test gripper commands"""
    print("\n=== Testing Gripper Commands ===")
    
    # Test gripper status
    print("\nGetting gripper status...")
    ser.write(b'GS\n')
    time.sleep(0.1)
    response = ser.readline().decode('utf-8', errors='ignore').strip()
    print(f"  Response: {response}")
    if response == "ERROR":
        print("  (Expected - servo not connected)")
    
    # Test gripper open
    print("\nTesting gripper open...")
    ser.write(b'GO\n')
    time.sleep(0.1)
    # Read multiple lines (command echoes and response)
    while ser.in_waiting:
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        if response:
            print(f"  Response: {response}")
    
    # Test gripper close
    print("\nTesting gripper close...")
    ser.write(b'GC\n')
    time.sleep(0.1)
    while ser.in_waiting:
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        if response:
            print(f"  Response: {response}")
    
    # Test gripper position
    print("\nTesting gripper position (GP2000)...")
    ser.write(b'GP2000\n')
    time.sleep(0.1)
    while ser.in_waiting:
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        if response:
            print(f"  Response: {response}")

def test_system_commands(ser):
    """Test system commands"""
    print("\n=== Testing System Commands ===")
    
    print("\nGetting system info...")
    ser.write(b'INFO\n')
    time.sleep(0.2)
    while ser.in_waiting:
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        if response:
            print(f"  {response}")

def interactive_mode(ser):
    """Interactive command mode"""
    print("\n=== Interactive Mode ===")
    print("Type commands (or 'quit' to exit):")
    print("Examples: RD, TL3, GO, GC, ?, INFO")
    print()
    
    while True:
        try:
            cmd = input("Command> ").strip()
            if cmd.lower() in ['quit', 'exit', 'q']:
                break
            
            if not cmd:
                continue
            
            ser.write(f'{cmd}\n'.encode())
            time.sleep(0.1)
            
            # Read all available responses
            while ser.in_waiting:
                response = ser.readline().decode('utf-8', errors='ignore').strip()
                if response:
                    print(response)
        
        except KeyboardInterrupt:
            print("\nExiting...")
            break

def main():
    """Main test function"""
    print("=================================")
    print("Gripper Controller Test Script")
    print("=================================")
    print(f"Port: {PORT}")
    print(f"Baud: {BAUD}")
    print()
    
    try:
        # Open serial connection
        print("Opening serial port...")
        ser = serial.Serial(PORT, BAUD, timeout=TIMEOUT)
        time.sleep(2)  # Wait for ESP32 to reset
        print("✓ Serial port opened")
        
        # Clear any startup messages
        ser.reset_input_buffer()
        
        # Run tests
        if not test_connection(ser):
            print("\n✗ Connection failed. Check port and baud rate.")
            return
        
        test_sensor_commands(ser)
        test_torque_commands(ser)
        test_gripper_commands(ser)
        test_system_commands(ser)
        
        # Interactive mode
        print("\n=================================")
        choice = input("Enter interactive mode? (y/n): ")
        if choice.lower() == 'y':
            interactive_mode(ser)
        
        # Close connection
        ser.close()
        print("\n✓ Test complete")
        
    except serial.SerialException as e:
        print(f"\n✗ Serial error: {e}")
        print(f"Make sure {PORT} is correct and not in use")
        sys.exit(1)
    
    except KeyboardInterrupt:
        print("\n\nTest interrupted by user")
        sys.exit(0)

if __name__ == '__main__':
    main()
