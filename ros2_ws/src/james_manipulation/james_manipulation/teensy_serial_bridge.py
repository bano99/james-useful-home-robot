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
from std_msgs.msg import String, Int32
import serial
import threading
import time
import re
import math
import json
from datetime import datetime
import os
import glob
from ament_index_python.packages import get_package_share_directory


class TeensySerialBridge(Node):
    """
    ROS2 node that bridges USB Serial communication with Teensy 4.1 AR4 controller
    """
    
    def __init__(self):
        super().__init__('teensy_serial_bridge')
        
        # Default config path - use ROS2 package share directory
        try:
            package_share_dir = get_package_share_directory('james_manipulation')
            default_config_path = os.path.join(package_share_dir, 'config', 'ARconfig.json')
        except Exception:
            package_dir = os.path.dirname(os.path.abspath(__file__))
            default_config_path = os.path.join(os.path.dirname(package_dir), 'config', 'ARconfig.json')

        # Declare ALL parameters first
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 9600)
        self.declare_parameter('timeout', 0.1)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('mock_hardware', False)
        self.declare_parameter('config_file', default_config_path)
        self.declare_parameter('enable_auto_detect', True)
        self.declare_parameter('send_up_on_startup', True)
        
        # Log raw parameter values for debugging
        all_params = self._parameters
        self.get_logger().info(f"--- PARAMETER DEBUG ---")
        for name, param in all_params.items():
            self.get_logger().info(f"Param '{name}': value={param.value}, type={param.type_}")
        
        # Get and process parameters
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.timeout = self.get_parameter('timeout').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.mock_hardware = self.get_parameter('mock_hardware').value
        self.config_file = self.get_parameter('config_file').value
        
        enable_raw = self.get_parameter('enable_auto_detect').value
        if isinstance(enable_raw, str):
            self.enable_auto_detect = enable_raw.lower() == 'true'
        else:
            self.enable_auto_detect = bool(enable_raw)

        send_up_raw = self.get_parameter('send_up_on_startup').value
        if isinstance(send_up_raw, str):
            self.send_up_on_startup = send_up_raw.lower() == 'true'
        else:
            self.send_up_on_startup = bool(send_up_raw)
            
        self.get_logger().info(f"Effective serial_port: {self.serial_port}")
        self.get_logger().info(f"Effective enable_auto_detect: {self.enable_auto_detect}")
        self.get_logger().info(f"Effective send_up_on_startup: {self.send_up_on_startup}")
        
        # Load configuration
        self.config = {}
        self.load_config()
        
        # Initialize serial connection
        self.serial_conn = None
        self.connected = False
        self.initialized = True # Commercial firmware doesn't need handshake
        self.last_joint_state = JointState()
        self.last_command_time = time.time()
        self.serial_lock = threading.Lock()
        
        # Publishers
        self.collision_pub = self.create_publisher(String, '/teensy/collision_status', 10)
        
        # Setup logging
        log_dir = os.path.expanduser('~/teensy_logs')
        os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file = os.path.join(log_dir, f'teensy_serial_{timestamp}.log')
        self.log_file = open(log_file, 'w', buffering=1)  # Line buffered
        self.get_logger().info(f'Logging serial communication to: {log_file}')
        self.log_file.write(f'=== Teensy Serial Bridge Log Started at {datetime.now()} ===\n')
        self.log_file.write(f'Port: {self.serial_port}, Baud: {self.baud_rate}\n\n')
        
        # Joint labels used by commercial firmware (A-F for J1-J6)
        self.joint_labels = ['A', 'B', 'C', 'D', 'E', 'F']
        self.joint_names = [
            'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 
            'arm_joint_4', 'arm_joint_5', 'arm_joint_6'
        ]
        
        # Initialize joint state message
        self.last_joint_state.name = self.joint_names
        self.last_joint_state.position = [0.0] * 6
        self.last_joint_state.velocity = [0.0] * 6
        self.last_joint_state.effort = [0.0] * 6
        self.first_data_received = False
        
        # ROS2 publishers and subscribers
        self.joint_state_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.joint_cmd_sub = self.create_subscription(JointState, '/arm/joint_commands', self.joint_command_callback, 10)
        self.raw_cmd_sub = self.create_subscription(String, '/arm/teensy_raw_cmd', self.raw_command_callback, 10)
        self.status_pub = self.create_publisher(String, '/teensy_bridge/status', 10)
        self.collision_pub = self.create_publisher(String, '/teensy/collision_status', 10)
        # Timer for publishing joint states (Heartbeat)
        self.publish_rate = 20.0
        self.joint_state_timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_state)
        
        # Timer for status publishing
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Packet count for synchronization
        self.packet_count_pub = self.create_publisher(Int32, '/teensy/packet_count', 10)
        self.packet_count = 0
        
        # Start serial communication thread
        self.serial_thread = threading.Thread(target=self.serial_communication_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
        
        self.get_logger().info(f'Teensy Serial Bridge (Commercial v6.3) started on {self.serial_port} at {self.baud_rate} baud')
    
    def connect_serial(self):
        """Establish serial connection with optional auto-detection"""
        if self.enable_auto_detect:
            self.get_logger().info("Auto-detect enabled. Searching for Teensy (AR4 Firmware)...")
            discovered_port = self.discover_port()
            if discovered_port:
                self.serial_port = discovered_port
                self.get_logger().info(f"Teensy discovered on {self.serial_port}")
            else:
                self.get_logger().warn(f"Auto-detection failed, falling back to {self.serial_port}")

        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            
            self.connected = True
            self.get_logger().info(f'Connected to Teensy on {self.serial_port} at {self.baud_rate} baud')
            return True
            
        except Exception as e:
            self.connected = False
            self.get_logger().warn(f'Failed to connect to {self.serial_port}: {e}')
            return False

    def discover_port(self):
        """Scan available serial ports to identify the Teensy (AR4 Firmware)"""
        if os.name == 'nt':
            candidate_ports = [f'COM{i}' for i in range(1, 21)]
        else:
            candidate_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            
        for port in candidate_ports:
            self.get_logger().info(f"Probing port {port}...")
            try:
                # Ar4 firmware usually broadcasts at 9600. 
                # Some custom versions might be at 115200.
                for br in [9600, 115200]:
                    test_conn = serial.Serial(port, br, timeout=0.1)
                    start_time = time.time()
                    while time.time() - start_time < 0.5:
                        if test_conn.in_waiting > 0:
                            line = test_conn.readline().decode('utf-8', errors='ignore').strip()
                            # Look for position broadcast starting with A and containing B,C markers
                            if line.startswith('A') and 'B' in line and 'C' in line:
                                self.get_logger().info(f"Identified Teensy on {port} at {br} baud")
                                self.baud_rate = br  # Update baud rate if different
                                test_conn.close()
                                return port
                    test_conn.close()
            except Exception:
                continue
        return None

    def serial_communication_loop(self):
        """Main serial communication loop running in separate thread"""
        while rclpy.ok():
            try:
                if not self.connected:
                    if self.connect_serial():
                        time.sleep(2.0) # Wait for boot logs
                        if self.send_up_on_startup:
                            self.send_configuration() # Critical for setting motor directions!
                        else:
                            self.get_logger().info('Skipping automatic UP configuration command as requested')
                    else:
                        time.sleep(1.0)
                        continue
                
                # Read data from Teensy
                with self.serial_lock:
                    if self.serial_conn and self.serial_conn.in_waiting > 0:
                        try:
                            line = self.serial_conn.readline().decode('utf-8').strip()
                            if line:
                                # Log received data
                                self.log_file.write(f'[{datetime.now().strftime("%H:%M:%S.%f")[:-3]}] RX: {line}\n')
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

    def destroy_node(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
        super().destroy_node()
        
    def load_config(self):
        """Load configuration from JSON file"""
        try:
            with open(self.config_file, 'r') as f:
                self.config = json.load(f)
            self.get_logger().info(f'Loaded configuration from {self.config_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config file {self.config_file}: {e}')
            self.config = {}

    def send_configuration(self):
        """Send the UP command to configure motor directions and parameters from loaded config"""
        if not self.config:
            self.get_logger().warn('No configuration loaded, skipping initialization command.')
            return

        def get_val(key, default, cast=int):
            try:
                val = self.config.get(key, default)
                return cast(val)
            except:
                return default

        # Motor Directions (G-O)
        J1MotDir = get_val('J1MotDir', 0)
        J2MotDir = get_val('J2MotDir', 1)
        J3MotDir = get_val('J3MotDir', 1)
        J4MotDir = get_val('J4MotDir', 1)
        J5MotDir = get_val('J5MotDir', 1)
        J6MotDir = get_val('J6MotDir', 1)
        J7MotDir = get_val('J7MotDir', 1)
        J8MotDir = get_val('J8MotDir', 1)
        J9MotDir = get_val('J9MotDir', 1)
        
        # Calibration Directions (P-X)
        J1CalDir = get_val('J1CalDir', 0)
        J2CalDir = get_val('J2CalDir', 1)
        J3CalDir = get_val('J3CalDir', 1)
        J4CalDir = get_val('J4CalDir', 1)
        J5CalDir = get_val('J5CalDir', 1)
        J6CalDir = get_val('J6CalDir', 1)
        J7CalDir = get_val('J7CalDir', 1)
        J8CalDir = get_val('J8CalDir', 1)
        J9CalDir = get_val('J9CalDir', 1)
        
        # Limits
        J1PosLim = get_val('J1PosLim', 150)
        J1NegLim = get_val('J1NegLim', -150)
        J2PosLim = get_val('J2PosLim', 100)
        J2NegLim = get_val('J2NegLim', -100)
        J3PosLim = get_val('J3PosLim', 100)
        J3NegLim = get_val('J3NegLim', -100)
        J4PosLim = get_val('J4PosLim', 165)
        J4NegLim = get_val('J4NegLim', -165)
        J5PosLim = get_val('J5PosLim', 105)
        J5NegLim = get_val('J5NegLim', -105)
        J6PosLim = get_val('J6PosLim', 1000)
        J6NegLim = get_val('J6NegLim', -1000)
        
        # Steps/Deg (k-p) - Use float for precision
        J1StepDeg = get_val('J1StepDeg', 88.888, float)
        J2StepDeg = get_val('J2StepDeg', 55.555, float)
        J3StepDeg = get_val('J3StepDeg', 55.555, float)
        J4StepDeg = get_val('J4StepDeg', 48.888, float)
        J5StepDeg = get_val('J5StepDeg', 43.720, float)
        J6StepDeg = get_val('J6StepDeg', 44.444, float)

        # Encoder Multipliers (q-v)
        J1EncMult = get_val('J1EncMult', 1)
        J2EncMult = get_val('J2EncMult', 1)
        J3EncMult = get_val('J3EncMult', 1)
        J4EncMult = get_val('J4EncMult', 1)
        J5EncMult = get_val('J5EncMult', 1)
        J6EncMult = get_val('J6EncMult', 1)

        # DH Parameters (w-~)
        J1tDH = get_val('J1\u0398DHpar', 0, float)
        J2tDH = get_val('J2\u0398DHpar', -90, float)
        J3tDH = get_val('J3\u0398DHpar', 0, float)
        J4tDH = get_val('J4\u0398DHpar', 0, float)
        J5tDH = get_val('J5\u0398DHpar', 0, float)
        J6tDH = get_val('J6\u0398DHpar', 180, float)

        J1aDH = get_val('J1\u03b1DHpar', 0, float)
        J2aDH = get_val('J2\u03b1DHpar', -90, float)
        J3aDH = get_val('J3\u03b1DHpar', 0, float)
        J4aDH = get_val('J4\u03b1DHpar', -90, float)
        J5aDH = get_val('J5\u03b1DHpar', 90, float)
        J6aDH = get_val('J6\u03b1DHpar', -90, float)

        J1dDH = get_val('J1dDHpar', 0, float)
        J2dDH = get_val('J2dDHpar', 0, float)
        J3dDH = get_val('J3dDHpar', 0, float)
        J4dDH = get_val('J4dDHpar', 0, float)
        J5dDH = get_val('J5dDHpar', 0, float)
        J6dDH = get_val('J6dDHpar', 0, float)

        J1lDH = get_val('J1aDHpar', 0, float) # 'a' param or link length
        J2lDH = get_val('J2aDHpar', 0, float)
        J3lDH = get_val('J3aDHpar', 0, float)
        J4lDH = get_val('J4aDHpar', 0, float)
        J5lDH = get_val('J5aDHpar', 0, float)
        J6lDH = get_val('J6aDHpar', 0, float)

        # Construct UP string
        # A-F: Tool transform (0 default)
        cmd = "UP"
        cmd += "A0B0C0D0E0F0" 
        cmd += f"G{J1MotDir}H{J2MotDir}I{J3MotDir}J{J4MotDir}K{J5MotDir}L{J6MotDir}M{J7MotDir}N{J8MotDir}O{J9MotDir}"
        cmd += f"P{J1CalDir}Q{J2CalDir}R{J3CalDir}S{J4CalDir}T{J5CalDir}U{J6CalDir}V{J7CalDir}W{J8CalDir}X{J9CalDir}" 
        cmd += f"Y{J1PosLim}Z{J1NegLim}a{J2PosLim}b{J2NegLim}c{J3PosLim}d{J3NegLim}e{J4PosLim}f{J4NegLim}g{J5PosLim}h{J5NegLim}i{J6PosLim}j{J6NegLim}"
        cmd += f"k{J1StepDeg:.3f}l{J2StepDeg:.3f}m{J3StepDeg:.3f}n{J4StepDeg:.3f}o{J5StepDeg:.3f}p{J6StepDeg:.3f}"
        
        # Encoders
        cmd += f"q{J1EncMult}r{J2EncMult}s{J3EncMult}t{J4EncMult}u{J5EncMult}v{J6EncMult}"
        
        # DH Parameters
        cmd += f"w{J1tDH}x{J2tDH}y{J3tDH}z{J4tDH}!{J5tDH}@{J6tDH}" # Theta
        cmd += f"#{J1aDH}${J2aDH}%{J3aDH}^{J4aDH}&{J5aDH}*{J6aDH}" # Alpha
        cmd += f"({J1dDH}){J2dDH}+{J3dDH}={J4dDH},{J5dDH}_{J6dDH}" # D
        cmd += f"<{J1lDH}>{J2lDH}?{J3lDH}{{{J4lDH}}}{J5lDH}~{J6lDH}" # A (Link Length)
        
        cmd += "\n"
        
        with self.serial_lock:
            if self.connected and self.serial_conn:
                self.serial_conn.write(cmd.encode('utf-8'))
                # Log sent data
                self.log_file.write(f'[{datetime.now().strftime("%H:%M:%S.%f")[:-3]}] TX: {cmd.strip()}\n')
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
        # A=0, B=1, C=2, D=3, E=4, F=5, G=X, H=Y, I=Z, J=Rz, K=Ry, L=Rx, M=SV, N=DB, O=FG
        
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

        # Parse Status/Collision flag 'O'
        o_start = data.find('O')
        if o_start != -1:
            # The O field is usually ONO or OECXXXXXX
            # Markers after O are P, Q, R or end
            next_markers = ['P', 'Q', 'R']
            o_end = -1
            for m in next_markers:
                m_pos = data.find(m, o_start + 1)
                if m_pos != -1:
                    if o_end == -1 or m_pos < o_end:
                        o_end = m_pos
            
            o_val = data[o_start+1:o_end] if o_end != -1 else data[o_start+1:]
            o_val = o_val.strip()
            
            if o_val.startswith('EC'):
                collision_msg = String()
                collision_msg.data = o_val
                self.collision_pub.publish(collision_msg)
                self.get_logger().warning(f'COLLISION DETECTED: {o_val}')
        
        self.first_data_received = True
        self.packet_count += 1
        
        # Publish hardware packet count for synchronization
        pc_msg = Int32()
        pc_msg.data = self.packet_count
        self.packet_count_pub.publish(pc_msg)

        # Also publish joint state immediately
        self.publish_joint_state()

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
        
        # Add J7-J9 dummy values to satisfy the Teensy firmware's fragile parser
        cmd += "J70.0000J80.0000J90.0000"
        
        # Speed/Accel params
        # Using Open Loop (Lm111111111) for ALL joints to prevent axis limit/collision resets
        # Lm string must be 9 digits for this firmware version
        cmd += f"Sp{speed:.2f}Ac{accel:.2f}Dc{decel:.2f}Rm{ramp:.2f}W0Lm111111111\n"
        
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
            # Log sent data
            self.log_file.write(f'[{datetime.now().strftime("%H:%M:%S.%f")[:-3]}] TX: {command.strip()}\n')
    
    def publish_joint_state(self):
        """Publish current joint state (Heartbeat)"""
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
        if self.log_file:
            self.log_file.write(f'\n=== Log Ended at {datetime.now()} ===\n')
            self.log_file.close()
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
