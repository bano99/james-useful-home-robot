#!/usr/bin/env python3
"""
Platform Serial Bridge Node for James Robot

This node interfaces with the Platform Controller via USB Serial to receive
manual control commands and forward them to the arm cartesian controller.

Author: James Robot Team
Date: December 2024
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial
import json
import threading
import time
import struct
import glob
import os as _os


class PlatformSerialBridge(Node):
    """
    ROS2 node that bridges USB Serial communication with Platform Controller
    """
    
    def __init__(self):
        super().__init__('platform_serial_bridge')
        self.get_logger().info('=====================================================')
        self.get_logger().info('Platform Serial Bridge - VERSION: 2024-12-21-V5-PARSED')
        self.get_logger().info('=====================================================')
        
        # Declare ALL parameters first
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('connection_timeout', 1.0)
        self.declare_parameter('enable_auto_detect', True)
        
        # Debug Parameters
        self.get_logger().info('--- PARAMETER DEBUG START ---')
        for param in self._parameters.values():
            self.get_logger().info(f'[{param.name}]: val={param.value} (type: {type(param.value).__name__})')
        self.get_logger().info('--- PARAMETER DEBUG END ---')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.connection_timeout = self.get_parameter('connection_timeout').value
        
        enable_raw = self.get_parameter('enable_auto_detect').value
        if isinstance(enable_raw, str):
            self.enable_auto_detect = enable_raw.lower() == 'true'
        else:
            self.enable_auto_detect = bool(enable_raw)
            
        self.get_logger().info(f"FINAL - serial_port: {self.serial_port}")
        self.get_logger().info(f"FINAL - enable_auto_detect: {self.enable_auto_detect}")
        
        # Initialize serial connection
        self.serial_conn = None
        self.connected = False
        self.last_command_time = time.time()
        self.message_buffer = b""  # Binary buffer
        self.serial_lock = threading.Lock()
        
        # Binary protocol
        self.PACKET_SIZE = 21
        self.PACKET_START = 0xAA
        
        # Publishers and subscribers
        self.arm_command_pub = self.create_publisher(String, '/arm/manual_cartesian_cmd', 10)
        self.status_pub = self.create_publisher(String, '/platform_bridge/status', 10)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Start serial communication thread
        self.serial_thread = threading.Thread(target=self.serial_communication_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
    
    def connect_serial(self):
        if self.enable_auto_detect:
            self.get_logger().info("Searching for Platform Controller via auto-detect...")
            discovered_port = self.discover_port()
            if discovered_port:
                self.serial_port = discovered_port
            else:
                self.get_logger().warn(f"Auto-detection failed, using {self.serial_port}")

        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            self.serial_conn = serial.Serial(port=self.serial_port, baudrate=self.baud_rate, timeout=self.timeout)
            self.connected = True
            self.get_logger().info(f'CONNECTED to Platform Controller on {self.serial_port}')
            return True
        except Exception as e:
            self.connected = False
            self.get_logger().warn(f'CONNECTION FAILED on {self.serial_port}: {e}')
            return False

    def discover_port(self):
        if _os.name == 'nt':
            candidate_ports = [f'COM{i}' for i in range(1, 21)]
        else:
            candidate_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            
        for port in candidate_ports:
            try:
                test_conn = serial.Serial(port, 115200, timeout=0.1)
                start_time = time.time()
                while time.time() - start_time < 0.5:
                    if test_conn.in_waiting >= self.PACKET_SIZE:
                        byte = test_conn.read(1)
                        if ord(byte) == self.PACKET_START:
                            self.get_logger().info(f"Platform Controller found on {port}")
                            test_conn.close()
                            return port
                test_conn.close()
            except Exception: continue
        return None

    def serial_communication_loop(self):
        while rclpy.ok():
            try:
                if not self.connected:
                    if not self.connect_serial():
                        time.sleep(1.0)
                        continue
                
                with self.serial_lock:
                    if self.serial_conn and self.serial_conn.in_waiting > 0:
                        data = self.serial_conn.read(self.serial_conn.in_waiting)
                        self.process_binary_data(data)
                time.sleep(0.01)
            except Exception as e:
                self.get_logger().error(f'Serial Loop Error: {e}')
                self.connected = False
                time.sleep(1.0)

    def process_binary_data(self, data):
        self.message_buffer += data
        while len(self.message_buffer) >= self.PACKET_SIZE:
            start_idx = self.message_buffer.find(bytes([self.PACKET_START]))
            if start_idx == -1:
                self.message_buffer = b""
                break
            if start_idx > 0:
                self.message_buffer = self.message_buffer[start_idx:]
            if len(self.message_buffer) < self.PACKET_SIZE:
                break
                
            packet = self.message_buffer[:self.PACKET_SIZE]
            self.message_buffer = self.message_buffer[self.PACKET_SIZE:]
            self.handle_packet(packet)

    def handle_packet(self, packet):
        try:
            # Simple unpack logic
            # [0xAA][type][left_x][left_y][left_z][right_x][right_y][right_rot][mode][gripper][timestamp][checksum]
            p_type = packet[1]
            if p_type == 1: # Manual Control
                lx, ly, lz, rx, ry, rr = struct.unpack('<hhhhhh', packet[2:14])
                mode = packet[14]
                cmd = {
                    'type': 'manual_control',
                    'lx': lx/1000.0, 'ly': ly/1000.0, 'lz': lz/1000.0,
                    'rx': rx/1000.0, 'ry': ry/1000.0, 'rr': rr/1000.0,
                    'mode': mode
                }
                msg = String()
                msg.data = json.dumps(cmd)
                self.arm_command_pub.publish(msg)
        except Exception as e:
            self.get_logger().warn(f"Packet handle error: {e}")

    def publish_status(self):
        msg = String()
        msg.data = json.dumps({'connected': self.connected, 'ts': time.time()})
        self.status_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = PlatformSerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if 'node' in locals(): node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()