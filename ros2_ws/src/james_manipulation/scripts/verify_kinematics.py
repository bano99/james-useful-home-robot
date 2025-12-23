#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import RobotState
import math
import time

from rclpy.qos import qos_profile_sensor_data

class KinematicsVerifier(Node):
    def __init__(self):
        super().__init__('kinematics_verifier')
        
        # Service Clients
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # Wait for services
        self.get_logger().info('Waiting for MoveIt services...')
        self.fk_client.wait_for_service()
        self.ik_client.wait_for_service()
        self.get_logger().info('Services available.')
        
        # Subscription with Sensor Data QoS (Best Effort)
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        self.current_joints = None
        
        # Timer
        self.timer = self.create_timer(2.0, self.verify_loop)

    def joint_cb(self, msg):
        self.current_joints = msg

    def verify_loop(self):
        if not self.current_joints:
            self.get_logger().info('Waiting for joint states... (Topic: /joint_states)')
            # Debug: List topics
            topics = self.get_topic_names_and_types()
            found = [t for t, _ in topics if 'joint_states' in t]
            if found:
                self.get_logger().info(f'Found joint state topics: {found}. Please update script if needed.')
            return
            
        # 1. Compute FK
        fk_req = GetPositionFK.Request()
        fk_req.header.frame_id = 'base_link'
        fk_req.fk_link_names = ['tool_link'] # Or ee_link
        # Note: We need to know the EE link name. Assuming 'tool_link' or 'gripper_link' or 'ee_link'.
        # Let's try 'arm_ee_link' based on xacro. Will verify in logs.
        fk_req.fk_link_names = ['arm_ee_link']
        
        state = RobotState()
        state.joint_state = self.current_joints
        fk_req.robot_state = state
        
        future_fk = self.fk_client.call_async(fk_req)
        rclpy.spin_until_future_complete(self, future_fk)
        resp_fk = future_fk.result()
        
        if resp_fk.error_code.val != 1:
            self.get_logger().error(f'FK Failed: {resp_fk.error_code.val}')
            return
            
        pose = resp_fk.pose_stamped[0]
        self.get_logger().info(f'Current Pose (FK): X={pose.pose.position.x:.3f}, Y={pose.pose.position.y:.3f}, Z={pose.pose.position.z:.3f}')
        
        # 2. Compute IK (Round Trip)
        ik_req = GetPositionIK.Request()
        ik_req.ik_request.group_name = 'manipulator' # Default MoveIt group
        ik_req.ik_request.robot_state = state # Seed with CURRENT joints
        ik_req.ik_request.avoid_collisions = True 
        ik_req.ik_request.pose_stamped = pose # Ask for EXACT pose back
        ik_req.ik_request.timeout = rclpy.duration.Duration(seconds=0.1).to_msg()
        
        future_ik = self.ik_client.call_async(ik_req)
        rclpy.spin_until_future_complete(self, future_ik)
        resp_ik = future_ik.result()
        
        if resp_ik.error_code.val != 1:
            self.get_logger().warn(f'IK Failed for Current Pose! Code: {resp_ik.error_code.val}')
            return
            
        # 3. Compare J (Input) vs J' (Output)
        j_in = dict(zip(self.current_joints.name, self.current_joints.position))
        j_out = dict(zip(resp_ik.solution.joint_state.name, resp_ik.solution.joint_state.position))
        
        diffs = []
        names = []
        for name in j_in:
            if name in j_out:
                d = math.degrees(j_out[name] - j_in[name])
                if abs(d) > 0.5:
                    diffs.append(f'{name}: {d:.2f}')
        
        if diffs:
            self.get_logger().warn(f'STABILITY FAILURE: IK returned different joints for current pose!\n Deltas: {", ".join(diffs)}')
        else:
            self.get_logger().info('STABLE: IK returned current state (Round trip successful).')

def main():
    rclpy.init()
    node = KinematicsVerifier()
    node.verify_loop() # Run once or loop
    # rclpy.spin(node) # Use internal spin loop
    rclpy.shutdown()

if __name__ == '__main__':
    main()
