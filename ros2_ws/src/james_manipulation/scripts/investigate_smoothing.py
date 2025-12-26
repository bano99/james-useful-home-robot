#!/usr/bin/env python3
import serial
import time
import argparse
import sys
import datetime
import math


def get_timestamp():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

def main():
    parser = argparse.ArgumentParser(description='Investigate AR4 Motion Smoothing via Direct Serial')
    parser.add_argument('--port', type=str, default='/dev/ttyACM1', help='Serial port (e.g. COM3 or /dev/ttyACM0)')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--dist', type=float, default=10.0, help='Total degrees to move J2 (can be negative)')
    parser.add_argument('--segments', type=int, default=20, help='Number of segments to split the move into')
    parser.add_argument('--interval', type=int, default=100, help='Milliseconds between sent commands')
    parser.add_argument('--speed', type=float, default=5.0, help='Speed value (Sp)')
    parser.add_argument('--accel', type=float, default=10.0, help='Accel value (Ac)')
    parser.add_argument('--decel', type=float, default=10.0, help='Decel value (Dc)')
    parser.add_argument('--ramp', type=float, default=10.0, help='Ramp value (Rm)')
    
    args = parser.parse_args()

    print("[{0}] Connecting to {1} at {2}...".format(get_timestamp(), args.port, args.baud))
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except Exception as e:
        print("Error opening serial port: {0}".format(e))
        return

    time.sleep(2) # Wait for Teensy reboot if any
    
    # 1. Get current position
    print("[{0}] Syncing position (RP)...".format(get_timestamp()))
    ser.write(b"RP\n")
    start_time = time.time()
    current_pos = None
    while time.time() - start_time < 2.0:
        line = ser.readline().decode('utf-8', errors='ignore').strip()
        if line:
            print("[{0}] RX: {1}".format(get_timestamp(), line))
            if line.startswith('A') and 'B' in line and 'C' in line:
                # Parse: A0.000B20.420C-33.741D-9.317E4.783F9.863G
                parts = {}
                markers = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
                for i in range(6):
                    start = line.find(markers[i]) + 1
                    end = line.find(markers[i+1])
                    parts[markers[i]] = float(line[start:end])
                current_pos = parts
                break
    
    if not current_pos:
        print("Failed to get initial position. Is the robot on and configured?")
        ser.close()
        return

    print("[{0}] Initial Position: {1}".format(get_timestamp(), current_pos))

    # 2. Enable Blending
    print("[{0}] Enabling Blending (BM1)...".format(get_timestamp()))
    ser.write(b"BM1\n")
    time.sleep(0.1)
    while ser.in_waiting:
        print("[{0}] RX: {1}".format(get_timestamp(), ser.readline().decode('utf-8').strip()))

    # 3. Calculate path
    j2_start = current_pos['B']
    j2_target = j2_start + args.dist
    step_inc = args.dist / args.segments

    print("--- STARTING SWEEP ---")
    print("Target: J2 from {0:.3f} to {1:.3f}".format(j2_start, j2_target))
    print("Segments: {0}, Interval: {1}ms, Speed: {2}%".format(args.segments, args.interval, args.speed))
    print("----------------------")

    try:
        for i in range(args.segments):
            next_j2 = j2_start + (i + 1) * step_inc
            
            # Format XJ command
            cmd = "XJA{0:.4f}B{1:.4f}C{2:.4f}D{3:.4f}E{4:.4f}F{5:.4f}Sp{6:.2f}Ac{7:.2f}Dc{8:.2f}Rm{9:.2f}\n".format(
                current_pos['A'], next_j2, current_pos['C'], current_pos['D'], current_pos['E'], current_pos['F'],
                args.speed, args.accel, args.decel, args.ramp
            )
            
            # Send
            ser.write(cmd.encode('utf-8'))
            print("[{0}] TX: {1}".format(get_timestamp(), cmd.strip()))
            
            # Wait and listen
            end_wait = time.time() + (args.interval / 1000.0)
            while time.time() < end_wait:
                if ser.in_waiting:
                    rx_line = ser.readline().decode('utf-8', errors='ignore').strip()
                    if rx_line:
                        print("[{0}] RX: {1}".format(get_timestamp(), rx_line))
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nInterrupted by user.")

    # 4. Cleanup
    print("[{0}] Cleaning up (BM0, ST)...".format(get_timestamp()))
    ser.write(b"BM0\n")
    time.sleep(0.05)
    ser.write(b"ST\n")
    time.sleep(0.1)
    while ser.in_waiting:
         print("[{0}] RX: {1}".format(get_timestamp(), ser.readline().decode('utf-8').strip()))
    
    ser.close()
    print("[{0}] Closed.".format(get_timestamp()))

if __name__ == "__main__":
    main()
