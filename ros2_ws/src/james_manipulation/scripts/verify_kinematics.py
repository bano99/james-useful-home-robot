#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.srv import GetPositionFK, GetPositionIK
from moveit_msgs.msg import RobotState
from rclpy.qos import qos_profile_sensor_data
import math
import time

class KinematicsVerifier(Node):
    def __init__(self):
        super().__init__('kinematics_verifier')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        # Best effort subscription for hardware joint states
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        self.current_joints = None

    def joint_cb(self, msg):
        self.current_joints = msg

def main():
    rclpy.init()
    node = KinematicsVerifier()
    
    print("Waiting for Services...")
    node.fk_client.wait_for_service()
    node.ik_client.wait_for_service()
    print("Services Available.")

    try:
        while rclpy.ok():
            # 1. Wait for data
            print("Listening for Joint States...", end='\r')
            rclpy.spin_once(node, timeout_sec=1.0)
            if not node.current_joints:
                continue
                
            print(f"\nReceived Joints: {node.current_joints.name[0]}... (Total {len(node.current_joints.position)})")
            
            # 2. Call FK
            fk_req = GetPositionFK.Request()
            fk_req.header.frame_id = 'base_link'
            fk_req.fk_link_names = ['arm_ee_link']
            state = RobotState()
            state.joint_state = node.current_joints
            fk_req.robot_state = state
            
            future_fk = node.fk_client.call_async(fk_req)
            while rclpy.ok() and not future_fk.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                
            resp_fk = future_fk.result()
            if not resp_fk or resp_fk.error_code.val != 1:
                print(f"FK Failed! Code: {resp_fk.error_code.val if resp_fk else 'None'}")
                time.sleep(1.0)
                continue

            pose = resp_fk.pose_stamped[0].pose
            print(f"Current FK Pose: X={pose.position.x:.3f}, Y={pose.position.y:.3f}, Z={pose.position.z:.3f}")

            # 3. Call IK
            ik_req = GetPositionIK.Request()
            ik_req.ik_request.group_name = 'manipulator'
            ik_req.ik_request.robot_state = state # Seed = Current
            ik_req.ik_request.avoid_collisions = True
            ik_req.ik_request.pose_stamped = resp_fk.pose_stamped[0]
            ik_req.ik_request.timeout.sec = 0
            ik_req.ik_request.timeout.nanosec = 100000000 # 0.1s
            
            future_ik = node.ik_client.call_async(ik_req)
            while rclpy.ok() and not future_ik.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                
            resp_ik = future_ik.result()
            if not resp_ik or resp_ik.error_code.val != 1:
                print(f"IK Failed (Invalid Pose for Solver?) Code: {resp_ik.error_code.val if resp_ik else 'None'}")
            else:
                # 4. Compare
                j_in = dict(zip(node.current_joints.name, node.current_joints.position))
                j_out = dict(zip(resp_ik.solution.joint_state.name, resp_ik.solution.joint_state.position))
                
                deltas = []
                for k, v in j_in.items():
                    if k in j_out:
                         diff = math.degrees(j_out[k] - v)
                         if abs(diff) > 2.0:
                             deltas.append(f"{k}: {diff:.1f} deg")
                
                if deltas:
                     print(f"!!! STABILITY FAILURE !!! Solver moved joints: {', '.join(deltas)}")
                else:
                     print("STABLE: IK Solution matches Current State.")
            
            time.sleep(2.0)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

