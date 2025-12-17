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


class PlatformSerialBridge(Node):
    """
    ROS2 node that bridges USB Serial communication with Platform Controller
    """
    
    def __init__(self):
        super().__init__('platform_serial_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('connection_timeout', 1.0)
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.connection_timeout = self.get_parameter('connection_timeout').value
        
        # Initialize serial connection
        self.serial_conn = None
        self.connected = False
        self.last_command_time = time.time()
        self.message_buffer = b""  # Binary buffer
        
        # Binary protocol: 16 bytes total
        # [0xAA][type][left_x][left_y][left_z][right_x][right_y][right_rot][mode][gripper][timestamp][checksum]
        # 1 + 1 + 2 + 2 + 2 + 2 + 2 + 2 + 1 + 1 + 4 + 1 = 21 bytes
        self.PACKET_SIZE = 21
        self.PACKET_START = 0xAA
        
        # ROS2 publishers and subscribers
        self.arm_command_pub = self.create_publisher(
            String, 
            '/arm/manual_cartesian_cmd', 
            10
        )
        
        self.arm_status_sub = self.create_subscription(
            String,
            '/arm/status',
            self.arm_status_callback,
            10
        )
        
        # Status publisher for monitoring
        self.status_pub = self.create_publisher(
            String,
            '/platform_bridge/status',
            10
        )
        
        # Timer for status publishing
        self.status_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_status
        )
        
        # Start serial communication thread
        self.serial_thread = threading.Thread(target=self.serial_communication_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info(f'Platform Serial Bridge started on {self.serial_port}')
    
    def connect_serial(self):
        """Establish serial connection to Platform Controller"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            
            self.connected = True
            self.get_logger().info(f'Connected to Platform Controller on {self.serial_port}')
            return True
            
        except Exception as e:
            self.connected = False
            self.get_logger().warn(f'Failed to connect to {self.serial_port}: {e}')
            return False
    
    def serial_communication_loop(self):
        """Main serial communication loop running in separate thread"""
        while rclpy.ok():
            try:
                if not self.connected:
                    if self.connect_serial():
                        time.sleep(0.1)
                    else:
                        time.sleep(1.0)  # Wait before retry
                        continue
                
                # Read data from Platform Controller
                if self.serial_conn and self.serial_conn.in_waiting > 0:
                    try:
                        data = self.serial_conn.read(self.serial_conn.in_waiting)
                        self.message_buffer += data
                        
                        # Process complete packets
                        while len(self.message_buffer) >= self.PACKET_SIZE:
                            # Find packet start
                            start_idx = self.message_buffer.find(self.PACKET_START)
                            if start_idx == -1:
                                self.message_buffer = b""
                                break
                            
                            # Remove data before packet start
                            if start_idx > 0:
                                self.message_buffer = self.message_buffer[start_idx:]
                            
                            # Check if we have a complete packet
                            if len(self.message_buffer) >= self.PACKET_SIZE:
                                packet = self.message_buffer[:self.PACKET_SIZE]
                                self.message_buffer = self.message_buffer[self.PACKET_SIZE:]
                                self.process_binary_packet(packet)
                            else:
                                break
                                
                    except Exception as e:
                        self.get_logger().warn(f'Error reading serial data: {e}')
                
                time.sleep(0.001)  # Small delay to prevent CPU spinning
                
            except Exception as e:
                self.get_logger().error(f'Serial communication error: {e}')
                self.connected = False
                time.sleep(1.0)
    
    def process_binary_packet(self, packet):
        """Process binary packet from Platform Controller"""
        try:
            # Unpack binary data
            unpacked = struct.unpack('<BBhhhhhBBIB', packet)
            start, msg_type, left_x, left_y, left_z, right_x, right_y, right_rot, mode, gripper, timestamp, checksum = unpacked
            
            # Verify checksum (simple sum of all bytes except checksum)
            calc_checksum = sum(packet[:-1]) & 0xFF
            if calc_checksum != checksum:
                self.get_logger().debug(f'Checksum mismatch: {calc_checksum} != {checksum}')
                return
            
            if msg_type == 1:  # manual_control
                # Convert to JSON for compatibility with existing arm controller
                data = {
                    'type': 'manual_control',
                    'left_x': left_x,
                    'left_y': left_y, 
                    'left_z': left_z,
                    'right_x': right_x,
                    'right_y': right_y,
                    'right_rot': right_rot,
                    'switch_mode': 'vertical' if mode == 1 else 'horizontal',
                    'gripper_pot': gripper,
                    'timestamp': timestamp
                }
                
                # Update last command time
                self.last_command_time = time.time()
                
                # Forward to arm cartesian controller
                msg = String()
                msg.data = json.dumps(data)
                self.arm_command_pub.publish(msg)
                
                self.get_logger().debug(f'Forwarded command: {data}')
            
        except struct.error as e:
            self.get_logger().warn(f'Invalid binary packet: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing binary packet: {e}')
    
    def arm_status_callback(self, msg):
        """Callback for arm status messages to forward back to Platform Controller"""
        try:
            if self.connected and self.serial_conn:
                # Forward status back to Platform Controller
                self.serial_conn.write((msg.data + '\n').encode('utf-8'))
                self.get_logger().debug(f'Sent status to platform: {msg.data}')
                
        except Exception as e:
            self.get_logger().warn(f'Error sending status to platform: {e}')
    
    def publish_status(self):
        """Publish bridge status for monitoring"""
        try:
            current_time = time.time()
            command_age = current_time - self.last_command_time
            
            status = {
                'type': 'bridge_status',
                'connected': self.connected,
                'serial_port': self.serial_port,
                'last_command_age': command_age,
                'commands_active': command_age < self.connection_timeout,
                'timestamp': current_time
            }
            
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')
    
    def destroy_node(self):
        """Clean shutdown"""
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = PlatformSerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()