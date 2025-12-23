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
import os as _os
import glob
from ament_index_python.packages import get_package_share_directory


class TeensySerialBridge(Node):
    """
    ROS2 node that bridges USB Serial communication with Teensy 4.1 AR4 controller
    """
    
    def __init__(self):
        super().__init__('teensy_serial_bridge')
        self.get_logger().info('=====================================================')
        self.get_logger().info('Teensy Serial Bridge - VERSION: 2024-12-21-V5-PARSED')
        self.get_logger().info('=====================================================')
        
        # Default config path - use ROS2 package share directory
        try:
            package_share_dir = get_package_share_directory('james_manipulation')
            default_config_path = _os.path.join(package_share_dir, 'config', 'ARconfig.json')
        except Exception:
            package_dir = _os.path.dirname(_os.path.abspath(__file__))
            default_config_path = _os.path.join(_os.path.dirname(package_dir), 'config', 'ARconfig.json')

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
        self.get_logger().info('--- PARAMETER DEBUG START ---')
        for param in self._parameters.values():
            self.get_logger().info(f'[{param.name}]: val={param.value} (type: {type(param.value).__name__})')
        self.get_logger().info('--- PARAMETER DEBUG END ---')
        
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
            self.enable_auto_detect = enable_raw.lower() in ('true', '1', 'yes', 'on')
        else:
            self.enable_auto_detect = bool(enable_raw)

        send_up_raw = self.get_parameter('send_up_on_startup').value
        if isinstance(send_up_raw, str):
            self.send_up_on_startup = send_up_raw.lower() in ('true', '1', 'yes', 'on')
        else:
            self.send_up_on_startup = bool(send_up_raw)
            
        self.get_logger().info(f"FINAL - serial_port: {self.serial_port}")
        self.get_logger().info(f"FINAL - enable_auto_detect: {self.enable_auto_detect}")
        
        # Load configuration
        self.config = {}
        self.load_config()
        
        # Initialize serial connection
        self.serial_conn = None
        self.connected = False
        self.initialized = True
        self.last_joint_state = JointState()
        self.last_command_time = time.time()
        self.serial_lock = threading.Lock()
        
        # Setup logging folder
        log_dir = _os.path.expanduser('~/teensy_logs')
        _os.makedirs(log_dir, exist_ok=True)
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        log_file_path = _os.path.join(log_dir, f'teensy_serial_{timestamp}.log')
        self.log_file = open(log_file_path, 'w', buffering=1)
        self.get_logger().info(f'Logging serial communication to: {log_file_path}')
        
        # Joint labels
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
        self.packet_count_pub = self.create_publisher(Int32, '/teensy/packet_count', 10)
        self.raw_rx_pub = self.create_publisher(String, '/arm/teensy_raw_rx', 10) # For debug CLI
        self.packet_count = 0
        
        # Timer for publishing joint states
        self.joint_state_timer = self.create_timer(1.0 / self.publish_rate, self.publish_joint_state)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Start serial communication thread
        self.serial_thread = threading.Thread(target=self.serial_communication_loop)
        self.serial_thread.daemon = True
        self.serial_thread.start()
    
    def connect_serial(self):
        """Establish serial connection with optional auto-detection"""
        if self.enable_auto_detect:
            self.get_logger().info("Searching for Teensy via auto-detect...")
            discovered_port = self.discover_port()
            if discovered_port:
                self.serial_port = discovered_port
            else:
                self.get_logger().warn(f"Auto-detection failed, using {self.serial_port}")

        try:
            if self.serial_conn and self.serial_conn.is_open:
                self.serial_conn.close()
            
            self.serial_conn = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            self.connected = True
            self.get_logger().info(f'CONNECTED to Teensy on {self.serial_port}')
            
            # Check Firmware State using GS command
            time.sleep(2.0) # Wait for reboot/init
            self.serial_conn.write(b'GS\n')
            time.sleep(0.1) 
            
            # Read state response
            is_configured = False
            start_time = time.time()
            while time.time() - start_time < 0.5:
                if self.serial_conn.in_waiting:
                    resp = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
                    if resp.startswith("State:"):
                        try:
                            states = [int(v) for v in resp.replace("State:", "").split(',')]
                            if len(states) > 0 and states[0] == 1:
                                is_configured = True
                                self.get_logger().info("Teensy ALREADY CONFIGURED (State[0]=1). Skipping UP command.")
                            if len(states) > 1 and states[1] == 0:
                                self.get_logger().warn("⚠️ ROBOT NOT CALIBRATED (State[1]=0) ⚠️")
                        except ValueError: pass
                    break
                time.sleep(0.05)
            
            # Smart Configuration Logic
            if is_configured:
                self.send_up_on_startup = False
            else:
                self.get_logger().info("Teensy NOT configured. Will send UP command.")
                # Ensure flag is True so main loop sends confirmation
                # Note: parameter object is read-only, we rely on the instance var check in loop
                # If param was false, we force it true here for this session?
                # User preference 'send_up_on_startup' logic:
                # If user said NO, we respect NO? Or do we enforce YES if unconfigured?
                # Assuming safety: If unconfigured, we MUST send UP or robot is useless.
                self.send_up_on_startup = True

            # Send RP command to sync state (Read Position)
            self.serial_conn.write(b'RP\n')
            self.get_logger().info('Sent RP (Read Position) to sync state')
            
            return True
        except Exception as e:
            self.connected = False
            self.get_logger().warn(f'CONNECTION FAILED on {self.serial_port}: {e}')
            return False

    def discover_port(self):
        """Scan available serial ports"""
        if _os.name == 'nt':
            candidate_ports = [f'COM{i}' for i in range(1, 21)]
        else:
            candidate_ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
            
        for port in candidate_ports:
            self.get_logger().info(f"Probing {port}...")
            try:
                for br in [9600, 115200]:
                    test_conn = serial.Serial(port, br, timeout=0.1)
                    
                    # Send Echo Test Message (TM)
                    # The firmware echoes back the message after TM
                    check_msg = "__TEENSY_CHECK__"
                    test_conn.write(f"TM{check_msg}\n".encode())
                    test_conn.flush()
                    
                    start_time = time.time()
                    buffer = ""
                    
                    while time.time() - start_time < 0.5:
                        if test_conn.in_waiting > 0:
                            try:
                                chunk = test_conn.read(test_conn.in_waiting).decode('utf-8', errors='ignore')
                                buffer += chunk
                                if check_msg in buffer:
                                    self.get_logger().info(f"Teensy found on {port} at {br} baud")
                                    self.baud_rate = br
                                    test_conn.close()
                                    return port
                            except Exception: pass
                        time.sleep(0.01)
                    test_conn.close()
            except Exception: continue
        return None

    def serial_communication_loop(self):
        """Main serial loop"""
        while rclpy.ok():
            try:
                if not self.connected:
                    if self.connect_serial():
                        time.sleep(2.0)
                        if self.send_up_on_startup:
                            self.send_configuration()
                    else:
                        time.sleep(1.0)
                        continue
                
                with self.serial_lock:
                    if self.serial_conn and self.serial_conn.in_waiting > 0:
                        try:
                            line = self.serial_conn.readline().decode('utf-8').strip()
                            if line:
                                self.log_file.write(f'[{datetime.now().strftime("%H:%M:%S.%f")[:-3]}] RX: {line}\n')
                                self.raw_rx_pub.publish(String(data=line)) # Publish raw for Debug CLI
                                self.process_teensy_message(line)
                        except UnicodeDecodeError: pass
                time.sleep(0.005)
            except Exception as e:
                self.get_logger().error(f'Serial Loop Error: {e}')
                self.connected = False
                time.sleep(1.0)

    def load_config(self):
        try:
            with open(self.config_file, 'r') as f:
                self.config = json.load(f)
            self.get_logger().info(f'Loaded config: {self.config_file}')
        except Exception as e:
            self.get_logger().error(f'Config Load Error: {e}')

    def send_configuration(self):
        if not self.config: return
        def get_val(key, default, cast=int):
            try: return cast(self.config.get(key, default))
            except: return default

        # Motor Directions & Parameters
        cmd = "UP"
        cmd += "A0B0C0D0E0F0" 
        cmd += f"G{get_val('J1MotDir', 0)}H{get_val('J2MotDir', 1)}I{get_val('J3MotDir', 1)}J{get_val('J4MotDir', 1)}K{get_val('J5MotDir', 1)}L{get_val('J6MotDir', 1)}M{get_val('J7MotDir', 1)}N{get_val('J8MotDir', 1)}O{get_val('J9MotDir', 1)}"
        cmd += f"P{get_val('J1CalDir', 0)}Q{get_val('J2CalDir', 1)}R{get_val('J3CalDir', 1)}S{get_val('J4CalDir', 1)}T{get_val('J5CalDir', 1)}U{get_val('J6CalDir', 1)}V{get_val('J7CalDir', 1)}W{get_val('J8CalDir', 1)}X{get_val('J9CalDir', 1)}" 
        cmd += f"Y{get_val('J1PosLim', 150)}Z{get_val('J1NegLim', -150)}a{get_val('J2PosLim', 100)}b{get_val('J2NegLim', -100)}c{get_val('J3PosLim', 100)}d{get_val('J3NegLim', -100)}e{get_val('J4PosLim', 165)}f{get_val('J4NegLim', -165)}g{get_val('J5PosLim', 105)}h{get_val('J5NegLim', -105)}i{get_val('J6PosLim', 1000)}j{get_val('J6NegLim', -1000)}"
        cmd += f"k{get_val('J1StepDeg', 88.888, float):.3f}l{get_val('J2StepDeg', 55.555, float):.3f}m{get_val('J3StepDeg', 55.555, float):.3f}n{get_val('J4StepDeg', 48.888, float):.3f}o{get_val('J5StepDeg', 43.720, float):.3f}p{get_val('J6StepDeg', 44.444, float):.3f}"
        cmd += f"q{get_val('J1EncMult', 1)}r{get_val('J2EncMult', 1)}s{get_val('J3EncMult', 1)}t{get_val('J4EncMult', 1)}u{get_val('J5EncMult', 1)}v{get_val('J6EncMult', 1)}"
        
        # DH Parameters
        theta_key = '\u0398DHpar'
        alpha_key = '\u03b1DHpar'
        cmd += f"w{get_val('J1'+theta_key, 0, float)}x{get_val('J2'+theta_key, -90, float)}y{get_val('J3'+theta_key, 0, float)}z{get_val('J4'+theta_key, 0, float)}!{get_val('J5'+theta_key, 0, float)}@{get_val('J6'+theta_key, 180, float)}"
        cmd += f"#{get_val('J1'+alpha_key, 0, float)}${get_val('J2'+alpha_key, -90, float)}%{get_val('J3'+alpha_key, 0, float)}^{get_val('J4'+alpha_key, -90, float)}&{get_val('J5'+alpha_key, 90, float)}*{get_val('J6'+alpha_key, -90, float)}"
        cmd += f"({get_val('J1dDHpar', 0, float)}){get_val('J2dDHpar', 0, float)}+{get_val('J3dDHpar', 0, float)}={get_val('J4dDHpar', 0, float)},{get_val('J5dDHpar', 0, float)}_{get_val('J6dDHpar', 0, float)}"
        cmd += f"<{get_val('J1aDHpar', 0, float)}>{get_val('J2aDHpar', 0, float)}?{get_val('J3aDHpar', 0, float)}{{{get_val('J4aDHpar', 0, float)}}}{get_val('J5aDHpar', 0, float)}~{get_val('J6aDHpar', 0, float)}"
        cmd += "\n"
        
        with self.serial_lock:
            if self.connected and self.serial_conn:
                self.log_file.write(f'[{datetime.now().strftime("%H:%M:%S.%f")[:-3]}] TX: {cmd}')
                self.serial_conn.write(cmd.encode('utf-8'))
                self.get_logger().info('Sent configuration (UP) to Teensy')
                
                # Mark as Configured (State[0] = 1)
                time.sleep(0.1)
                set_state_cmd = "GSA0B1\n"
                self.serial_conn.write(set_state_cmd.encode('utf-8'))
                self.get_logger().info('Sent GSA0B1 (Mark Configured)')

    def process_teensy_message(self, message):
        try:
            if message.startswith('A') and 'B' in message and 'C' in message:
                self.parse_feedback(message)
            elif message.startswith('EL'):
                self.get_logger().error(f'Axis Limit Fault: {message}')
            elif message.startswith('ER'):
                self.get_logger().error('Kinematic Error')
        except Exception as e:
            self.get_logger().warn(f'Parse Error: {e}')
    
    def parse_feedback(self, data):
        self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
        markers = ['A', 'B', 'C', 'D', 'E', 'F', 'G']
        for i in range(6):
            m_s = data.find(markers[i])
            if m_s != -1:
                # Find end marker
                if i < 5:
                    m_e = data.find(markers[i+1])
                else:
                    # For last element, search for G, if not found use end of string
                    m_e = data.find(markers[i+1])
                    if m_e == -1:
                        m_e = len(data)
                
                # Extract value
                if m_e != -1 and m_e > m_s:
                    val_str = data[m_s+1:m_e]
                    try: 
                        self.last_joint_state.position[i] = math.radians(float(val_str))
                    except ValueError: 
                        self.get_logger().warn(f"Parse Error J{i+1}: '{val_str}'")
        
        self.first_data_received = True
        self.packet_count += 1
        pc_msg = Int32()
        pc_msg.data = self.packet_count
        self.packet_count_pub.publish(pc_msg)
        self.publish_joint_state()

    def joint_command_callback(self, msg):
        if not self.connected or not self.serial_conn: return
        cmd = "RJ"
        for i in range(min(len(msg.position), 6)):
            cmd += f"{self.joint_labels[i]}{math.degrees(msg.position[i]):.4f}"
        cmd += "J70.0000J80.0000J90.0000Sp60.00Ac200.00Dc200.00Rm80.00W0Lm111111111\n"
        with self.serial_lock:
            self.log_file.write(f'[{datetime.now().strftime("%H:%M:%S.%f")[:-3]}] TX: {cmd}')
            self.serial_conn.write(cmd.encode('utf-8'))

    def raw_command_callback(self, msg):
        if not self.connected or not self.serial_conn: return
        cmd = msg.data.strip() + "\n"
        with self.serial_lock:
            self.log_file.write(f'[{datetime.now().strftime("%H:%M:%S.%f")[:-3]}] TX: {cmd}')
            self.serial_conn.write(cmd.encode('utf-8'))

    def publish_joint_state(self):
        if self.connected:
            self.last_joint_state.header.stamp = self.get_clock().now().to_msg()
            self.joint_state_pub.publish(self.last_joint_state)
    
    def publish_status(self):
        msg = String()
        msg.data = json.dumps({'connected': self.connected, 'baud': self.baud_rate, 'ts': time.time()})
        self.status_pub.publish(msg)

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
