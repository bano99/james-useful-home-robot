#!/usr/bin/env python3
"""
Quick script to test Teensy connectivity
"""
import serial
import time
import sys

if len(sys.argv) < 2:
    print("Usage: python3 check_teensy_connection.py /dev/ttyACM0")
    sys.exit(1)

port = sys.argv[1]
print(f"Testing connection to {port}...")

try:
    ser = serial.Serial(port, 115200, timeout=2.0)
    time.sleep(1.0)
    ser.reset_input_buffer()
    
    print("Sending RP command...")
    ser.write(b'RP\n')
    ser.flush()
    
    time.sleep(0.5)
    
    if ser.in_waiting > 0:
        response = ser.readline().decode('utf-8', errors='ignore').strip()
        print(f"SUCCESS! Received: {response[:80]}")
        print("\nTeeensy is responding. You can run the diagnostic script.")
    else:
        print("NO RESPONSE from Teensy!")
        print("\nPossible issues:")
        print("  1. Serial bridge is running and intercepting commands")
        print("     Solution: Stop the bridge first")
        print("  2. Wrong serial port")
        print("     Solution: Try other /dev/ttyACM* ports")
        print("  3. Teensy is hung/crashed")
        print("     Solution: Power cycle the Teensy")
    
    ser.close()
    
except serial.SerialException as e:
    print(f"ERROR: Could not open serial port: {e}")
    print("\nPossible issues:")
    print("  1. Port is already in use (bridge running?)")
    print("  2. Permission denied (try: sudo usermod -a -G dialout $USER)")
    print("  3. Wrong port name")
except Exception as e:
    print(f"ERROR: {e}")
