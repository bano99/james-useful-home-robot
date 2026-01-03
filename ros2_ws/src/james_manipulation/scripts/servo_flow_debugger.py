#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int8
from geometry_msgs.msg import TwistStamped
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
import time
from datetime import datetime

def get_timestamp():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

class ServoFlowDebugger(Node):
    def __init__(self):
        super().__init__('servo_flow_debugger')
        self.get_logger().info("==========================================")
        self.get_logger().info("  MOVEIT SERVO FLOW DEBUGGER (FOXY)       ")
        self.get_logger().info("==========================================")

        # Counters
        self.counts = {
            'joy_in': 0,
            'twist_out': 0,
            'servo_status': 0,
            'traj_out': 0,
            'joint_states': 0
        }
        
        self.last_status = -1
        
        # 1. Platform Bridge -> Controller (Manual Cartesian Command)
        self.create_subscription(String, '/arm/manual_cartesian_cmd', self.joy_cb, 10)
        
        # 2. Controller -> Servo Server (Delta Twist)
        self.create_subscription(TwistStamped, '/servo_server/delta_twist_cmds', self.twist_cb, 10)
        
        # 3. Servo Server -> Bridge (Joint Trajectory)
        self.create_subscription(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', self.traj_cb, 10)
        
        # 4. Servo Server Status
        self.create_subscription(Int8, '/servo_server/status', self.status_cb, 10)
        
        # 5. Bridge -> Everywhere (Joint States)
        self.create_subscription(JointState, '/joint_states', self.js_cb, 10)
        
        # Timer for summary
        self.create_timer(1.0, self.summary_cb)
        
        self.get_logger().info("Listening to Servo pipeline topics...")

    def joy_cb(self, msg): self.counts['joy_in'] += 1
    def twist_cb(self, msg): self.counts['twist_out'] += 1
    def traj_cb(self, msg): 
        self.counts['traj_out'] += 1
        self.get_logger().info(f"[{get_timestamp()}] TRAJECTORY EMITTED: {len(msg.points)} points", throttle_duration_sec=2.0)
        
    def status_cb(self, msg): 
        self.counts['servo_status'] += 1
        if msg.data != self.last_status:
            status_map = {0: "NO_CONTROL", 1: "CONTROLLING", 2: "JOINT_BOUND", 3: "SINGULARITY", 4: "COLLISION"}
            label = status_map.get(msg.data, f"UNKNOWN({msg.data})")
            self.get_logger().info(f"[{get_timestamp()}] SERVO STATUS CHANGED: {msg.data} ({label})")
            self.last_status = msg.data
            
    def js_cb(self, msg): self.counts['joint_states'] += 1

    def summary_cb(self):
        c = self.counts
        self.get_logger().info(
            f"STATS (per sec): Joy:{c['joy_in']} -> Twist:{c['twist_out']} | Servo Status:{c['servo_status']} -> Traj:{c['traj_out']} | JS:{c['joint_states']}"
        )
        # Reset relative counters
        for k in self.counts: self.counts[k] = 0

def main():
    rclpy.init()
    node = ServoFlowDebugger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
