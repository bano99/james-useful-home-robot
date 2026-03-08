#!/usr/bin/env python3
"""
Send Joint Limit Configuration to Teensy

This script reads limits from ARconfig.json and sends them to the Teensy
to ensure the firmware has the correct limit values.

Usage:
    python3 send_limit_config.py
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class LimitConfigSender(Node):
    def __init__(self):
        super().__init__('limit_config_sender')
        
        # Publisher
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        
        # Load config
        self.config = self.load_config()
        
        self.get_logger().info('Limit Config Sender Started')
    
    def load_config(self):
        """Load joint limits from ARconfig.json"""
        try:
            with open('ros2_ws/src/james_manipulation/config/ARconfig.json', 'r') as f:
                config = json.load(f)
            self.get_logger().info('✓ Loaded ARconfig.json')
            return config
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            return None
    
    def send_raw(self, cmd):
        """Send raw command to Teensy"""
        msg = String()
        msg.data = cmd
        self.raw_cmd_pub.publish(msg)
        self.get_logger().info(f'→ {cmd}')
    
    def send_limit_config(self):
        """Send LL command with joint limits"""
        if not self.config:
            self.get_logger().error('No config loaded!')
            return False
        
        self.get_logger().info('='*70)
        self.get_logger().info('SENDING JOINT LIMIT CONFIGURATION')
        self.get_logger().info('='*70)
        
        # Build LL command
        # Format: LLA<J1pos>B<J1neg>C<J2pos>D<J2neg>E<J3pos>F<J3neg>
        #         G<J4pos>H<J4neg>I<J5pos>J<J5neg>K<J6pos>L<J6neg>
        
        cmd = "LL"
        cmd += f"A{self.config['J1PosLim']}"
        cmd += f"B{self.config['J1NegLim']}"
        cmd += f"C{self.config['J2PosLim']}"
        cmd += f"D{self.config['J2NegLim']}"
        cmd += f"E{self.config['J3PosLim']}"
        cmd += f"F{self.config['J3NegLim']}"
        cmd += f"G{self.config['J4PosLim']}"
        cmd += f"H{self.config['J4NegLim']}"
        cmd += f"I{self.config['J5PosLim']}"
        cmd += f"J{self.config['J5NegLim']}"
        cmd += f"K{self.config['J6PosLim']}"
        cmd += f"L{self.config['J6NegLim']}"
        
        self.get_logger().info('Joint Limits:')
        self.get_logger().info(f'  J1: -{self.config["J1NegLim"]}° to +{self.config["J1PosLim"]}°')
        self.get_logger().info(f'  J2: -{self.config["J2NegLim"]}° to +{self.config["J2PosLim"]}°')
        self.get_logger().info(f'  J3: -{self.config["J3NegLim"]}° to +{self.config["J3PosLim"]}°')
        self.get_logger().info(f'  J4: -{self.config["J4NegLim"]}° to +{self.config["J4PosLim"]}°')
        self.get_logger().info(f'  J5: -{self.config["J5NegLim"]}° to +{self.config["J5PosLim"]}°')
        self.get_logger().info(f'  J6: -{self.config["J6NegLim"]}° to +{self.config["J6PosLim"]}°')
        self.get_logger().info('')
        
        self.send_raw(cmd)
        self.get_logger().info('✓ Limit configuration sent')
        self.get_logger().info('='*70)
        
        return True
    
    def run(self):
        """Main execution"""
        # Wait a moment for publisher to be ready
        time.sleep(0.5)
        
        # Send configuration
        success = self.send_limit_config()
        
        if success:
            self.get_logger().info('')
            self.get_logger().info('Configuration sent successfully!')
            self.get_logger().info('The Teensy should now use the correct joint limits.')
            self.get_logger().info('')
            self.get_logger().info('You can now retry your XJ commands.')
        
        return success


def main(args=None):
    rclpy.init(args=args)
    sender = LimitConfigSender()
    
    try:
        sender.run()
        time.sleep(1.0)  # Give time for message to be sent
    except KeyboardInterrupt:
        pass
    finally:
        sender.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
