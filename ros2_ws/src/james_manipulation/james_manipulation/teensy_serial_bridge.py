#!/usr/bin/env python3
"""
Teensy Serial Bridge Node for James Robot

This node interfaces with the Teensy 4.1 (AR4 controller) via USB Serial
to send joint commands and receive joint state feedback.

Author: James Robot Team
Date: December 2024
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import serial
import threading
import time
import re
import math
import json


class TeensySerialBridge(Node):
    """
    ROS2 node that bridges USB Serial communication with Teensy 4.1 AR4 controller
    """
    
    def __init__(self):
        super().__init__('teensy_serial_bridge')
        
        # Declare parameters
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('mock_hardware', False)
        self.declare_parameter('robot_model', 'mk3')
        self.declare_parameter('firmware_version', '2.1.0')
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.mock_hardware = self.get_parameter('mock_hardware').value
        self.robot_model = self.get_parameter('robot_model').value
        self.firmware_version = self.get_parameter('firmware_version').value
        
        # Initialize serial connection
        self.serial_conn = None
        self.connected = False
        self.initialized = False
        self.last_joint_state = JointState()
        self.last_command_time = time.time()
        self.serial_lock = threading.Lock()
        
        # Joint labels used by the firmware (A=J1, B=J2, C=J3, D=J4, E=J5, F=J6)
        self.joint_labels = ['A', 'B', 'C', 'D', 'E', 'F']
        self.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6']
        
        # Initialize joint state message
        self.last_joint_state.name = self.joint_names
        self.last_joint_state.position = [0.0] * 6
        self.last_joint_state.velocity = [0.0] * 6
        self.last_joint_state.effort = [0.0] * 6
        
        # ROS2 publishers and subscribers
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            '/arm/joint_commands',
            self.joint_command_callback,
            10
        )
        
        self.raw_cmd_sub = self.create_subscription(
            String,
            '/arm/teensy_raw_cmd',
            self.raw_command_callback,
            10
        )
        
        # Status publisher
        self.status_pub = self.create_publisher(
            String,
            '/teensy_bridge/status',
            10
        )
        
        # Timer for publishing joint states
        self.joint_state_timer = self.create_timer(
            1.0 / self.publish_rate,
            self.publish_joint_state
        )
        
        # Timer for status publishing
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz status updates
            self.publish_status
        )
        
        # Start serial communication thread
        self.serial_thread = threading.Thread(target=self.serial_communication_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info(f'Teensy Serial Bridge started on {self.serial_port} (Model: {self.robot_model})')
    
    def connect_serial(self):
        """Establish serial connection to Teensy 4.1"""
        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            
            self.connected = True
            self.initialized = False
            self.get_logger().info(f'Connected to Teensy on {self.serial_port}')
            
            # Send initialization command
            self.send_initialization()
            
            return True
            
        except Exception as e:
            self.connected = False
            self.get_logger().warn(f'Failed to connect to {self.serial_port}: {e}')
            return False

    def send_initialization(self):
        """Send calibration/initialization message to robot"""
        # Format: STA<version>B<model>
        command = f"STA{self.firmware_version}B{self.robot_model}\n"
        with self.serial_lock:
            if self.connected and self.serial_conn:
                self.serial_conn.write(command.encode('utf-8'))
                self.get_logger().info(f'Sent initialization: {command.strip()}')

    def serial_communication_loop(self):
        """Main serial communication loop running in separate thread"""
        while rclpy.ok():
            try:
                if not self.connected:
                    if self.connect_serial():
                        # Wait a bit longer for Teensy to boot after reset on connection
                        time.sleep(2.0)
                        self.send_initialization()
                        time.sleep(0.5)
                    else:
                        time.sleep(1.0)
                        continue
                
                # Periodically retry initialization if not yet initialized
                now = time.time()
                if not self.initialized and (now % 5.0 < 0.1):
                    self.get_logger().info("Handshake not complete, retrying...")
                    self.send_initialization()
                
                # Periodically request position/velocity if not using broadcast
                # (Note: firmware stateTRAJ loop needs query or broadcast? 
                #  Looking at firmware loop, it only responds to commands. 
                #  Actually, it processes commands as they come. 
                #  Usually, we query JP to get position.)
                
                # Query joint positions every few iterations to keep state updated
                # However, usually the firmware might be modified to broadcast.
                # In ycheng517 firmware, JP/JV are query-based.
                
                # Request positions periodicially
                now = time.time()
                if self.initialized and (now % 0.1 < 0.01): # roughly 10Hz query
                   self.query_state()

                # Read data from Teensy
                with self.serial_lock:
                    if self.serial_conn and self.serial_conn.in_waiting > 0:
                        try:
                            line = self.serial_conn.readline().decode('utf-8').strip()
                            if line:
                                self.process_teensy_message(line)
                        except UnicodeDecodeError:
                            pass
                
                time.sleep(0.005)  
                
            except serial.SerialException as e:
                self.get_logger().error(f'Serial connection lost on {self.serial_port}: {e}')
                self.connected = False
                self.initialized = False
                if self.serial_conn:
                    try:
                        self.serial_conn.close()
                    except:
                        pass
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f'Serial communication error: {e}')
                self.connected = False
                time.sleep(1.0)
    
    def query_state(self):
        """Query joint positions and velocities"""
        with self.serial_lock:
            if self.connected and self.serial_conn:
                self.serial_conn.write(b"JP\n")
                # self.serial_conn.write(b"JV\n") # Option to query velocity too

    def process_teensy_message(self, message):
        """Process incoming message from Teensy (joint state feedback)"""
        try:
            # Parse AR4 joint state response format (ycheng517 protocol)
            # JP initialization ack: "STA1B1C2.1.0Dmk3"
            if message.startswith('ST'):
                if 'A1' in message and 'C1' in message:
                    self.initialized = True
                    self.get_logger().info(f'Handshake complete: {message}')
                else:
                    self.get_logger().error(f'Handshake failed (Version/Model mismatch): {message}')
                    self.initialized = False
            
            # Joint position update: "JPA<val>B<val>C<val>D<val>E<val>F<val>"
            elif message.startswith('JP'):
                joint_data = message[2:]
                self.parse_joint_feedback(joint_data, 'position')
                
            # Joint velocity update: "JVA<val>B<val>C<val>D<val>E<val>F<val>"
            elif message.startswith('JV'):
                joint_data = message[2:]
                self.parse_joint_feedback(joint_data, 'velocity')
            
            # Estop status: "ES0" or "ES1"
            elif message.startswith('ES'):
                estop = message[2:] == '1'
                if estop:
                    self.get_logger().error('EMERGENCY STOP DETECTED')
                
            # Internal debug: "DB: ..."
            elif message.startswith('DB:'):
                self.get_logger().debug(f'Teensy Debug: {message[3:].strip()}')
            
            # Error: "ER: ..."
            elif message.startswith('ER:'):
                self.get_logger().error(f'Teensy Error: {message[3:].strip()}')
            
            else:
                # Log other messages for debugging
                self.get_logger().info(f'Teensy message: {message}')
                
        except Exception as e:
            self.get_logger().warn(f'Error parsing Teensy message: {message} - {e}')
    
    def parse_joint_feedback(self, data, field):
        """Helper to parse A...B... format into joint states"""
        self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
        for i, label in enumerate(self.joint_labels):
            start_idx = data.find(label)
            if start_idx == -1: continue
            
            # Find next label or end
            next_label_idx = -1
            if i < len(self.joint_labels) - 1:
                next_label_idx = data.find(self.joint_labels[i+1], start_idx + 1)
            
            val_str = data[start_idx+1:next_label_idx] if next_label_idx != -1 else data[start_idx+1:]
            try:
                val = float(val_str)
                if field == 'position':
                    self.last_joint_state.position[i] = math.radians(val)
                elif field == 'velocity':
                    self.last_joint_state.velocity[i] = math.radians(val)
            except ValueError:
                pass

    def joint_command_callback(self, msg):
        """Send joint commands to Teensy"""
        try:
            if not self.connected or not self.serial_conn:
                self.get_logger().warn('Cannot send joint command - Teensy not connected', throttle_duration_sec=2.0)
                return
            
            if not self.initialized:
                self.get_logger().warn('Cannot send joint command - Waiting for handshake', throttle_duration_sec=2.0)
                return

            self.last_command_time = time.time()
            
            # Format joint command (ycheng517 protocol: MTA<val>B<val>...)
            if len(msg.position) >= 6:
                cmd_parts = []
                for i in range(6):
                    angle_deg = math.degrees(msg.position[i])
                    cmd_parts.append(f'{self.joint_labels[i]}{angle_deg:.4f}')
                
                command = 'MT' + ''.join(cmd_parts) + '\n'
                
                with self.serial_lock:
                    if self.connected and self.serial_conn:
                        self.serial_conn.write(command.encode('utf-8'))
                        self.get_logger().debug(f'Sent MT command: {command.strip()}')
                    
                    if self.mock_hardware:
                        self.last_joint_state.position = list(msg.position)
                        self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
            
            # Also handle velocity commands if provided
            if len(msg.velocity) >= 6 and any(v != 0.0 for v in msg.velocity):
                cmd_parts = []
                for i in range(6):
                    vel_deg = math.degrees(msg.velocity[i])
                    cmd_parts.append(f'{self.joint_labels[i]}{vel_deg:.4f}')
                
                command = 'MV' + ''.join(cmd_parts) + '\n'
                with self.serial_lock:
                    if self.connected and self.serial_conn:
                        self.serial_conn.write(command.encode('utf-8'))
            
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write to Teensy failed: {e}')
            self.connected = False
        except Exception as e:
            self.get_logger().error(f'Error sending joint command: {e}')
            
    def raw_command_callback(self, msg):
        """Send raw command string to Teensy"""
        try:
            if not self.connected or not self.serial_conn:
                if not self.mock_hardware:
                    self.get_logger().warn('Cannot send raw command - Teensy not connected')
                    return
            
            command = msg.data
            if not command.endswith('\n'):
                command += '\n'
                
            self.get_logger().info(f'Sending raw command to Teensy: {command.strip()}')
            
            with self.serial_lock:
                if self.connected and self.serial_conn:
                    self.serial_conn.write(command.encode('utf-8'))
                
                # Simple mock for debugging
                if self.mock_hardware:
                    if command.startswith('MT'):
                         self.process_teensy_message('STack') # Mock init
                         self.process_teensy_message('JP' + command[2:])
                
        except Exception as e:
            self.get_logger().error(f'Error sending raw command: {e}')
    
    def publish_joint_state(self):
        """Publish current joint state"""
        try:
            if self.connected:
                # Update timestamp
                self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
                self.joint_state_pub.publish(self.last_joint_state)
                
        except Exception as e:
            self.get_logger().error(f'Error publishing joint state: {e}')
    
    def publish_status(self):
        """Publish bridge status for monitoring"""
        try:
            current_time = time.time()
            command_age = current_time - self.last_command_time
            
            status = {
                'type': 'teensy_bridge_status',
                'connected': self.connected,
                'initialized': self.initialized,
                'serial_port': self.serial_port,
                'last_command_age': command_age,
                'joint_state_valid': len(self.last_joint_state.position) == 6,
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
        node = TeensySerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
