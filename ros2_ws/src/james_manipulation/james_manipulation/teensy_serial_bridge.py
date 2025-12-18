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
        
        # Get parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.mock_hardware = self.get_parameter('mock_hardware').value
        
        # Initialize serial connection
        self.serial_conn = None
        self.connected = False
        self.last_joint_state = JointState()
        self.last_command_time = time.time()
        self.serial_lock = threading.Lock()
        
        # Joint names for AR4-MK3 (matching URDF prefix)
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
        
        self.get_logger().info(f'Teensy Serial Bridge started on {self.serial_port}')
    
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
            self.get_logger().info(f'Connected to Teensy on {self.serial_port}')
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
    
    def process_teensy_message(self, message):
        """Process incoming message from Teensy (joint state feedback)"""
        try:
            # Parse AR4 joint state response format
            # Expected format: "JS J1:45.5 J2:30.2 J3:60.1 J4:0.0 J5:45.0 J6:0.0"
            if message.startswith('JS '):
                joint_data = message[3:]  # Remove 'JS ' prefix
                
                # Parse joint values using regex
                joint_pattern = r'J(\d+):([-+]?\d*\.?\d+)'
                matches = re.findall(joint_pattern, joint_data)
                
                if len(matches) == 6:
                    # Update joint state
                    self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
                    
                    for i, (joint_num, angle_str) in enumerate(matches):
                        joint_index = int(joint_num) - 1  # Convert to 0-based index
                        if 0 <= joint_index < 6:
                            # Convert degrees to radians
                            angle_rad = math.radians(float(angle_str))
                            self.last_joint_state.position[joint_index] = angle_rad
                    
                    self.get_logger().debug(f'Updated joint state: {matches}')
                else:
                    self.get_logger().warn(f'Invalid joint state format: {message}')
            
            else:
                # Log other messages for debugging
                self.get_logger().debug(f'Teensy message: {message}')
                
        except Exception as e:
            self.get_logger().warn(f'Error parsing Teensy message: {message} - {e}')
    
    def joint_command_callback(self, msg):
        """Send joint commands to Teensy"""
        try:
            if not self.connected or not self.serial_conn:
                self.get_logger().warn('Cannot send joint command - Teensy not connected')
                return
            
            self.last_command_time = time.time()
            
            # Format joint command for AR4 protocol
            # Expected format: "MJ J1:45.5 J2:30.2 J3:60.1 J4:0.0 J5:45.0 J6:0.0"
            if len(msg.position) >= 6:
                joint_angles = []
                for i in range(6):
                    # Convert radians to degrees
                    angle_deg = math.degrees(msg.position[i])
                    joint_angles.append(f'J{i+1}:{angle_deg:.2f}')
                
                command = 'MJ ' + ' '.join(joint_angles) + '\n'
                
                # Send command to Teensy
                with self.serial_lock:
                    if self.connected and self.serial_conn:
                        self.serial_conn.write(command.encode('utf-8'))
                        self.get_logger().debug(f'Sent joint command: {command.strip()}')
                    
                    # If mocking, loop back the command as current state
                    if self.mock_hardware:
                        self.last_joint_state.position = list(msg.position)
                        self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
            else:
                self.get_logger().warn(f'Invalid joint command - expected 6 joints, got {len(msg.position)}')
                
        except serial.SerialException as e:
            self.get_logger().warn(f'Serial write to Teensy failed: {e}')
            self.connected = False
        except Exception as e:
            self.get_logger().error(f'Error sending joint command: {e}')
    
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