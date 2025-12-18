#!/usr/bin/env python3
"""
Serial Multimeter - Frequency and Protocol Verifier
Usage: python3 serial_multimeter.py --port /dev/ttyUSB0 --baud 115200 --mode binary --size 21
"""

import argparse
import serial
import time
import struct
import sys
import threading

def main():
    parser = argparse.ArgumentParser(description='Serial Data Frequency Verifier')
    parser.add_argument('--port', type=str, default='/dev/ttyACM0', help='Serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    parser.add_argument('--mode', type=str, default='binary', choices=['binary', 'text'], help='Protocol mode')
    parser.add_argument('--size', type=int, default=21, help='Packet size for binary mode')
    parser.add_argument('--header', type=str, default='0xAA', help='Header byte (hex string, e.g. 0xAA)')
    
    args = parser.parse_args()
    
    header_byte = int(args.header, 16)
    
    print(f"Starting Serial Multimeter on {args.port} @ {args.baud} baud")
    print(f"Mode: {args.mode}, Size: {args.size if args.mode == 'binary' else 'N/A'}")
    
    try:
        ser = serial.Serial(args.port, args.baud, timeout=0.1)
    except serial.SerialException as e:
        print(f"Error opening port: {e}")
        return

    msg_count = 0
    byte_count = 0
    start_time = time.time()
    last_print = start_time
    
    buffer = b""
    
    # Running flag
    running = True

    def print_stats():
        nonlocal msg_count, byte_count, last_print
        while running:
            time.sleep(1.0)
            now = time.time()
            dt = now - last_print
            hz = msg_count / dt
            kbps = (byte_count * 8) / 1000 / dt
            
            print(f"Rate: {hz:6.1f} Hz | Throughput: {kbps:6.1f} Kbps | Messages: {msg_count}")
            
            msg_count = 0
            byte_count = 0
            last_print = now

    stat_thread = threading.Thread(target=print_stats)
    stat_thread.daemon = True
    stat_thread.start()

    try:
        while True:
            if ser.in_waiting > 0:
                chunk = ser.read(ser.in_waiting)
                byte_count += len(chunk)
                
                if args.mode == 'binary':
                    buffer += chunk
                    while len(buffer) >= args.size:
                        # Find header
                        start_idx = buffer.find(header_byte)
                        if start_idx == -1:
                            # Discard garbage if no header found
                            buffer = b"" 
                            break
                        
                        if start_idx > 0:
                            # Alignment correction
                            buffer = buffer[start_idx:]
                        
                        if len(buffer) >= args.size:
                            # We have a full packet candidate
                            # Optional: Check Checksum if you know the offset. 
                            # For generic multimeter, we just count "framed" packets.
                            msg_count += 1
                            buffer = buffer[args.size:]
                        else:
                            break
                            
                else: # Text mode
                    buffer += chunk
                    if b'\n' in buffer:
                        lines = buffer.split(b'\n')
                        # The last part is incomplete, keep it
                        msg_count += len(lines) - 1
                        buffer = lines[-1]
                        
            else:
                time.sleep(0.001)

    except KeyboardInterrupt:
        print("\nStopping...")
        running = False
    except Exception as e:
        print(f"\nError: {e}")
        running = False
    finally:
        ser.close()

if __name__ == "__main__":
    main()
