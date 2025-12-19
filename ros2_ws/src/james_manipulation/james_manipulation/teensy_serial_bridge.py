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
        self.declare_parameter('baud_rate', 9600) # Default for commercial firmware
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('publish_rate', 20.0) # Lowered for 9600 baud
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
        self.initialized = True # Commercial firmware doesn't need handshake
        self.last_joint_state = JointState()
        self.last_command_time = time.time()
        self.serial_lock = threading.Lock()
        
        # Joint labels used by commercial firmware (A-F for J1-J6, P-R for J7-J9)
        self.joint_labels = ['A', 'B', 'C', 'D', 'E', 'F']
        self.aux_labels = ['P', 'Q', 'R'] # J7, J8, J9 indices
        self.joint_names = [
            'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 
            'arm_joint_4', 'arm_joint_5', 'arm_joint_6',
            'aux_joint_7', 'aux_joint_8', 'aux_joint_9'
        ]
        
        # Initialize joint state message
        self.last_joint_state.name = self.joint_names
        self.last_joint_state.position = [0.0] * 9
        self.last_joint_state.velocity = [0.0] * 9
        self.last_joint_state.effort = [0.0] * 9
        
        # ROS2 publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_cmd_sub = self.create_subscription(JointState, '/arm/joint_commands', self.joint_command_callback, 10)
        self.raw_cmd_sub = self.create_subscription(String, '/arm/teensy_raw_cmd', self.raw_command_callback, 10)
        self.status_pub = self.create_publisher(String, '/teensy_bridge/status', 10)
        
        # Timer for publishing joint states
        self.joint_state_timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_state)
        
        # Timer for status publishing
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Start serial communication thread
        self.serial_thread = threading.Thread(target=self.serial_communication_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info(f'Teensy Serial Bridge (Commercial v6.3) started on {self.serial_port} at {self.baud_rate} baud')
    
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
                        time.sleep(2.0) # Wait for boot logs
                        self.send_configuration() # Critical for setting motor directions!
                    else:
                        time.sleep(1.0)
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
                self.get_logger().error(f'Serial connection lost: {e}')
                self.connected = False
                if self.serial_conn:
                    try: self.serial_conn.close()
                    except: pass
                time.sleep(1.0)
            except Exception as e:
                self.get_logger().error(f'Serial error: {e}')
                self.connected = False
                time.sleep(1.0)

    def send_configuration(self):
        """Send the UP command to configure motor directions and parameters"""
        # Values extracted from AR4.py defaults
        # Motor Directions (G-O)
        J1MotDir = 0
        J2MotDir = 1
        J3MotDir = 1
        J4MotDir = 1
        J5MotDir = 1
        J6MotDir = 1
        J7MotDir = 1
        J8MotDir = 1
        J9MotDir = 1
        
        # Calibration Directions (P-X)
        J1CalDir = 0
        J2CalDir = 1
        J3CalDir = 1
        J4CalDir = 1
        J5CalDir = 1
        J6CalDir = 1
        J7CalDir = 1
        J8CalDir = 1
        J9CalDir = 1
        
        # Limits (Using generic defaults, user might need to tune these)
        J1PosLim, J1NegLim = 150, -150
        J2PosLim, J2NegLim = 100, -100
        J3PosLim, J3NegLim = 100, -100
        J4PosLim, J4NegLim = 165, -165
        J5PosLim, J5NegLim = 105, -105
        J6PosLim, J6NegLim = 1000, -1000 # J6 has wide range usually
        
        # Steps/Deg (k-p) - Copied from typical AR4 config
        J1StepDeg = 88.888
        J2StepDeg = 55.555
        J3StepDeg = 55.555
        J4StepDeg = 48.888
        J5StepDeg = 43.720
        J6StepDeg = 44.444
        
        # Construct UP string
        # A-F: Tool transform (0 default)
        cmd = "UP"
        cmd += "A0B0C0D0E0F0" 
        cmd += f"G{J1MotDir}H{J2MotDir}I{J3MotDir}J{J4MotDir}K{J5MotDir}L{J6MotDir}M{J7MotDir}N{J8MotDir}O{J9MotDir}"
        cmd += f"P{J1CalDir}Q{J2CalDir}R{J3CalDir}S{J4CalDir}T{J5CalDir}U{J6CalDir}V{J7CalDir}W{J8CalDir}X{J9CalDir}" 
        cmd += f"Y{J1PosLim}Z{J1NegLim}a{J2PosLim}b{J2NegLim}c{J3PosLim}d{J3NegLim}e{J4PosLim}f{J4NegLim}g{J5PosLim}h{J5NegLim}i{J6PosLim}j{J6NegLim}"
        cmd += f"k{J1StepDeg:.3f}l{J2StepDeg:.3f}m{J3StepDeg:.3f}n{J4StepDeg:.3f}o{J5StepDeg:.3f}p{J6StepDeg:.3f}"
        
        # Remainder (Encoders, DH) - sending defaults 
        cmd += "q1r1s1t1u1v1" # Encoder multipliers
        cmd += "w0x0y0z0!0@0" # theta DH
        cmd += "#0$0%0^0&0*0" # alpha DH
        cmd += "(0)0+0=0,0_0" # d DH
        cmd += "<0>0?0{0}0~0" # a DH
        cmd += "\n"
        
        with self.serial_lock:
            if self.connected and self.serial_conn:
                self.serial_conn.write(cmd.encode('utf-8'))
                self.get_logger().info('Sent UP configuration command to Teensy')
    
    def process_teensy_message(self, message):
        """Process incoming message from Teensy (Joint state: A...B...C...)"""
        try:
            # Commercial firmware broadcasts position automatically 
            # Format: A<J1>B<J2>C<J3>D<J4>E<J5>F<J6>G<X>H<Y>I<Z>J<Rz>K<Ry>L<Rx>M<SV>N<DB>O<FG>P<J7>Q<J8>R<J9>
            if message.startswith('A') and 'B' in message and 'C' in message:
                self.parse_commercial_feedback(message)
            
            # Internal debug: "DB: ..."
            elif message.startswith('DB:'):
                self.get_logger().debug(f'Teensy Debug: {message[3:].strip()}')
            
            # Error: "ER" or "EL..."
            elif message.startswith('ER'):
                self.get_logger().error('Teensy Kinematic Error reported')
            elif message.startswith('EL'):
                self.get_logger().error(f'Teensy Axis Limit Fault: {message}')
            elif message.startswith('EB') or message.startswith('ESTOP'):
                self.get_logger().error('EMERGENCY STOP ACTIVE')
            
            else:
                self.get_logger().info(f'Teensy: {message}')
                
        except Exception as e:
            self.get_logger().warn(f'Error parsing message: {message} - {e}')
    
    def parse_commercial_feedback(self, data):
        """Parse the long A...R string from commercial firmware"""
        self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
        
        # Mapping markers to joint indices
        # A=0, B=1, C=2, D=3, E=4, F=5, G=X, H=Y, I=Z, J=Rz, K=Ry, L=Rx, M=SV, N=DB, O=FG, P=6, Q=7, R=8
        
        # Standard joints A-F
        markers = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
        for i in range(6):
            m_start = data.find(markers[i])
            m_end = data.find(markers[i+1])
            if m_start != -1 and m_end != -1:
                try:
                    val = float(data[m_start+1:m_end])
                    self.last_joint_state.position[i] = math.radians(val)
                except ValueError: pass

        # Aux joints P-R (J7-J9)
        aux_markers = ['P', 'Q', 'R']
        for i, m in enumerate(aux_markers):
            m_start = data.find(m)
            if m_start == -1: continue
            # Find next if exists
            next_m = -1
            if i < 2: next_m = data.find(aux_markers[i+1])
            
            val_str = data[m_start+1:next_m] if next_m != -1 else data[m_start+1:]
            try:
                val = float(val_str)
                self.last_joint_state.position[6+i] = val # Linear or deg? Usually 6-8 are linear?
            except ValueError: pass

    def joint_command_callback(self, msg):
        """Send Joint commands via RJ prefix"""
        if not self.connected or not self.serial_conn: return

        # Command format: RJ A<J1>B<J2>C<J3>D<J4>E<J5>F<J6>J7<J7>J8<J8>J9<J9>Sp<val>Ac<val>Dc<val>Rm<val>W0Lm000000
        # For simplicity, we use default speed/accel/ramp if not provided in extra fields
        speed = 10.0
        accel = 10.0
        decel = 10.0
        ramp = 10.0
        
        cmd = "RJ"
        for i in range(min(len(msg.position), 6)):
            val = math.degrees(msg.position[i])
            cmd += f"{self.joint_labels[i]}{val:.4f}"
        
        # J7-J9
        j7 = msg.position[6] if len(msg.position) > 6 else 0.0
        j8 = msg.position[7] if len(msg.position) > 7 else 0.0
        j9 = msg.position[8] if len(msg.position) > 8 else 0.0
        cmd += f"J7{j7:.4f}J8{j8:.4f}J9{j9:.4f}"
        
        # Speed/Accel params
        cmd += f"Sp{speed:.2f}Ac{accel:.2f}Dc{decel:.2f}Rm{ramp:.2f}W0Lm000000\n"
        
        with self.serial_lock:
            self.serial_conn.write(cmd.encode('utf-8'))
            self.get_logger().debug(f'Sent RJ: {cmd.strip()}')

    def raw_command_callback(self, msg):
        """Send raw command string to Teensy"""
        if not self.connected or not self.serial_conn: return
        
        command = msg.data
        if not command.endswith('\n'): command += '\n'
            
        self.get_logger().info(f'Sending raw command: {command.strip()}')
        with self.serial_lock:
            self.serial_conn.write(command.encode('utf-8'))
    
    def publish_joint_state(self):
        """Publish current joint state"""
        if self.connected:
            self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_pub.publish(self.last_joint_state)
    
    def publish_status(self):
        """Publish bridge status"""
        status = {
            'type': 'teensy_bridge_status',
            'connected': self.connected,
            'baud': self.baud_rate,
            'timestamp': time.time()
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)
    
    def destroy_node(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TeensySerialBridge()
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        if 'node' in locals(): node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
