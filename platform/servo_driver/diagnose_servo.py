#!/usr/bin/env python3
"""
Quick diagnostic script for servo issues
"""

import serial
import time
import sys

def diagnose(port='/dev/ttyUSB0', baudrate=115200):
    """Run diagnostic tests"""
    
    print("="*60)
    print("SERVO DIAGNOSTIC TOOL")
    print("="*60)
    
    try:
        ser = serial.Serial(port, baudrate, timeout=2)
        time.sleep(2)
        
        # Clear buffer
        while ser.in_waiting:
            ser.readline()
        
        print("\n1. Testing PING (checking if servo responds)...")
        ser.write(b"#1PING\n")
        time.sleep(0.5)
        response = ""
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"   {line}")
            response += line
        
        if "responded to PING" in response:
            print("   ✅ Servo ID 1 is connected and responding")
        else:
            print("   ❌ Servo ID 1 did not respond")
            print("   → Check power supply")
            print("   → Check wiring (GPIO18/19)")
            print("   → Try different servo ID (#2PING, #3PING, etc.)")
            return
        
        print("\n2. Reading current position...")
        ser.write(b"#1P?\n")
        time.sleep(0.5)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"   {line}")
        
        print("\n3. Enabling torque...")
        ser.write(b"#1T1\n")
        time.sleep(0.5)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"   {line}")
        
        print("\n4. Setting servo mode (position control)...")
        ser.write(b"#1M0\n")
        time.sleep(0.5)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"   {line}")
        
        print("\n5. Attempting to move to position 0...")
        ser.write(b"#1P0S500\n")
        time.sleep(1)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"   {line}")
        
        print("\n   → Did the servo move? (y/n): ", end='')
        moved = input().lower()
        
        if moved == 'y':
            print("   ✅ Servo is working!")
        else:
            print("   ❌ Servo did not move")
            print("\n   Possible issues:")
            print("   1. Insufficient power supply (need 6-12V, 2-3A)")
            print("   2. Mechanical obstruction")
            print("   3. Servo already at position 0")
            print("   4. Faulty servo")
        
        print("\n6. Attempting to move to position 4095...")
        ser.write(b"#1P4095S500\n")
        time.sleep(2)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"   {line}")
        
        print("\n   → Did the servo move? (y/n): ", end='')
        moved = input().lower()
        
        if moved == 'y':
            print("   ✅ Servo is working!")
        else:
            print("   ❌ Servo still not moving")
        
        print("\n7. Reading final position...")
        ser.write(b"#1P?\n")
        time.sleep(0.5)
        while ser.in_waiting:
            line = ser.readline().decode('utf-8', errors='ignore').strip()
            print(f"   {line}")
        
        print("\n" + "="*60)
        print("DIAGNOSTIC COMPLETE")
        print("="*60)
        
        print("\nIf servo still doesn't move:")
        print("1. Check power supply voltage (6-12V)")
        print("2. Check power supply current capacity (2-3A minimum)")
        print("3. Verify wiring: GPIO18→Servo RX, GPIO19→Servo TX")
        print("4. Test servo with web interface")
        print("5. Try manually moving servo (torque should resist)")
        
        ser.close()
        
    except serial.SerialException as e:
        print(f"Error: {e}")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\n\nDiagnostic interrupted")
        ser.close()
        sys.exit(0)

if __name__ == "__main__":
    if len(sys.argv) > 1:
        port = sys.argv[1]
    else:
        import os
        if os.name == 'nt':
            port = 'COM3'
        else:
            port = '/dev/ttyUSB0'
    
    print(f"Using port: {port}")
    print("If wrong, run: python diagnose_servo.py <your_port>\n")
    
    diagnose(port)
