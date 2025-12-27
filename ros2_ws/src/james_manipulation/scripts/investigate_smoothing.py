#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
import argparse
import sys
import datetime
import math
import threading

def get_timestamp():
    return datetime.datetime.now().strftime("%H:%M:%S.%f")[:-3]

class SmoothingInvestigator(Node):
    def __init__(self, args):
        super().__init__('smoothing_investigator')
        self.args = args
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, '/arm/joint_commands', 10)
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        
        # Subscriber for Sync
        self.current_pos = None
        self.sync_event = threading.Event()
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.joint_names = [
            'arm_joint_1', 'arm_joint_2', 'arm_joint_3', 
            'arm_joint_4', 'arm_joint_5', 'arm_joint_6'
        ]

    def joint_state_callback(self, msg):
        if self.current_pos is None:
            # First message received, capture it
            self.current_pos = list(msg.position)
            print(f"[{get_timestamp()}] Initial Position Synced: {self.current_pos}")
            self.sync_event.set()

    def run_investigation(self):
        print(f"[{get_timestamp()}] Waiting for /joint_states to sync...")
        if not self.sync_event.wait(timeout=5.0):
            print("Failed to sync initial position from /joint_states. Is the bridge running?")
            return

        # 1. Enable Blending
        print(f"[{get_timestamp()}] TX: BM1 (Enable Blending)")
        bm1_msg = String()
        bm1_msg.data = "BM1"
        self.raw_cmd_pub.publish(bm1_msg)
        time.sleep(0.5) # Wait for bridge to process

        # 2. Calculate and Send Sweep
        j2_start = self.current_pos[1] # arm_joint_2 is at index 1
        dist_rad = math.radians(self.args.dist)
        step_inc = dist_rad / self.args.segments

        print("--- STARTING ROS SWEEP ---")
        print(f"Target: J2 dist {self.args.dist} deg over {self.args.segments} segments")
        print(f"Interval: {self.args.interval}ms, Speed: {self.args.speed}%")
        print("--------------------------")

        try:
            for i in range(self.args.segments):
                # Prepare message
                cmd_msg = JointState()
                cmd_msg.header.stamp = self.get_clock().now().to_msg()
                cmd_msg.name = self.joint_names
                
                # Update J2, keep others same
                new_pos = list(self.current_pos)
                new_pos[1] = j2_start + (i + 1) * step_inc
                cmd_msg.position = new_pos
                
                # Speed is passed in velocity[0] (V9 convention)
                cmd_msg.velocity = [float(self.args.speed)]
                
                # Publish
                self.joint_cmd_pub.publish(cmd_msg)
                print(f"[{get_timestamp()}] TX J2: {math.degrees(new_pos[1]):.4f} (Segment {i+1}/{self.args.segments})")
                
                # Wait
                time.sleep(self.args.interval / 1000.0)

            # 3. Disable Blending Immediately
            time.sleep(3.0)
            print(f"[{get_timestamp()}] TX: BM0 (Disable Blending)")
            bm0_msg = String()
            bm0_msg.data = "BM0"
            self.raw_cmd_pub.publish(bm0_msg)

            # 4. Wait 3 seconds then STOP
            print(f"[{get_timestamp()}] Waiting 3.0 seconds before STOP...")
            time.sleep(3.0)
            
            print(f"[{get_timestamp()}] TX: ST (Stop)")
            st_msg = String()
            st_msg.data = "ST"
            self.raw_cmd_pub.publish(st_msg)

        except KeyboardInterrupt:
            print("\nInterrupted by user.")
            st_msg = String()
            st_msg.data = "ST"
            self.raw_cmd_pub.publish(st_msg)
        except Exception as e:
            print(f"Error in investigation: {e}")
        finally:
            print(f"[{get_timestamp()}] Investigation Complete.")
            # We don't call shutdown here, main() does it

def main():
    parser = argparse.ArgumentParser(description='Investigate AR4 Motion Smoothing via ROS2 Bridge')
    parser.add_argument('--dist', type=float, default=10.0, help='Total degrees to move J2')
    parser.add_argument('--segments', type=int, default=20, help='Number of segments')
    parser.add_argument('--interval', type=int, default=100, help='Milliseconds between commands')
    parser.add_argument('--speed', type=float, default=5.0, help='Speed value (Sp)')
    
    # These are in the original but not used in RJ/XJ unless bridge is modified, 
    # but we keep the args for CLI compatibility
    parser.add_argument('--accel', type=float, default=10.0, help='Accel value (Ac)')
    parser.add_argument('--decel', type=float, default=10.0, help='Decel value (Dc)')
    parser.add_argument('--ramp', type=float, default=10.0, help='Ramp value (Rm)')
    
    args = parser.parse_args()

    rclpy.init()
    node = SmoothingInvestigator(args)
    
    # Run business logic in a separate thread so we can use rclpy.spin or just sleep
    # but rclpy basics: we need to spin to get the /joint_states
    thread = threading.Thread(target=node.run_investigation, daemon=True)
    thread.start()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
