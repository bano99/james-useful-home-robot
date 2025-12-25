#!/usr/bin/env python3
print("--- Script Starting ---", flush=True)

import sys
try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from moveit_msgs.srv import GetPositionFK, GetPositionIK
    from moveit_msgs.msg import RobotState
    from rclpy.qos import qos_profile_sensor_data
    import math
    import time
    print("Imports Successful.", flush=True)
except Exception as e:
    print(f"IMPORT ERROR: {e}", flush=True)
    sys.exit(1)

class KinematicsVerifier(Node):
    def __init__(self):
        super().__init__('kinematics_verifier')
        self.fk_client = self.create_client(GetPositionFK, '/compute_fk')
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        self.joint_sub = self.create_subscription(JointState, '/joint_states', self.joint_cb, qos_profile_sensor_data)
        self.current_joints = None

    def joint_cb(self, msg):
        self.current_joints = msg

def main():
    try:
        rclpy.init()
        print("Node Initialized.", flush=True)
        node = KinematicsVerifier()
        
        print("Waiting for FK Service...", flush=True)
        if not node.fk_client.wait_for_service(timeout_sec=5.0):
            print("TIMEOUT: FK Service not found. Is MoveIt running?", flush=True)
            return

        print("Waiting for IK Service...", flush=True)
        if not node.ik_client.wait_for_service(timeout_sec=5.0):
            print("TIMEOUT: IK Service not found.", flush=True)
            return
            
        print("Services Connected. Starting Loop.", flush=True)

        while rclpy.ok():
            print("Listening for Joint States...", flush=True)
            # Spin a bit to catch messages
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.1)
                if node.current_joints:
                    break
            
            if not node.current_joints:
                print("  No joints received yet. (Check Namespace?)", flush=True)
                time.sleep(1.0)
                continue
            
            print(f"Received Joints: {node.current_joints.name[0]}... ({len(node.current_joints.position)} joints)", flush=True)
            
            # 1. FK
            fk_req = GetPositionFK.Request()
            fk_req.header.frame_id = 'base_link'
            fk_req.fk_link_names = ['arm_link_6']
            state = RobotState()
            state.joint_state = node.current_joints
            fk_req.robot_state = state
            
            future_fk = node.fk_client.call_async(fk_req)
            while rclpy.ok() and not future_fk.done():
                rclpy.spin_once(node, timeout_sec=0.1)
            
            if not future_fk.result() or future_fk.result().error_code.val != 1:
                print("FK Computation Failed.", flush=True)
                continue
                
            pose = future_fk.result().pose_stamped[0]
            print(f"Current Pose (link_6): X={pose.pose.position.x:.3f}, Y={pose.pose.position.y:.3f}, Z={pose.pose.position.z:.3f}", flush=True)
            
            # 2. IK
            ik_req = GetPositionIK.Request()
            ik_req.ik_request.group_name = 'ar_manipulator'
            ik_req.ik_request.robot_state = state
            ik_req.ik_request.avoid_collisions = True
            ik_req.ik_request.ik_link_name = 'arm_link_6'
            ik_req.ik_request.pose_stamped = pose
            ik_req.ik_request.timeout.sec = 0
            ik_req.ik_request.timeout.nanosec = 100000000 # 0.1s
            
            future_ik = node.ik_client.call_async(ik_req)
            while rclpy.ok() and not future_ik.done():
                rclpy.spin_once(node, timeout_sec=0.1)
                
            if not future_ik.result() or future_ik.result().error_code.val != 1:
                print(f"IK Failed with Collisions Enabled! (Code: {future_ik.result().error_code.val if future_ik.result() else 'None'})", flush=True)
                print("Retrying with collision avoidance DISABLED...", flush=True)
                
                ik_req.ik_request.avoid_collisions = False
                future_ik_retry = node.ik_client.call_async(ik_req)
                while rclpy.ok() and not future_ik_retry.done():
                    rclpy.spin_once(node, timeout_sec=0.1)
                
                if not future_ik_retry.result() or future_ik_retry.result().error_code.val != 1:
                     print("FULL FAILURE: Even without collisions, IK cannot solve this pose. (Limits?)", flush=True)
                     # Print Joint Values
                     print("Current Joints (Deg):", flush=True)
                     for n, p in zip(node.current_joints.name, node.current_joints.position):
                         print(f"  {n}: {math.degrees(p):.2f}", flush=True)
                     continue
                else:
                     print("SUCCESS (No Collision): The pose is valid but requires ignoring collisions.", flush=True)
                     future_ik = future_ik_retry # Use this result for comparison
            
            # Compare logic continues...
            if True: #Indent block equivalent
                # Compare
                sol = future_ik.result().solution.joint_state
                j_in = dict(zip(node.current_joints.name, node.current_joints.position))
                j_out = dict(zip(sol.name, sol.position))
                
                deltas = []
                for name, val in j_in.items():
                    if name in j_out:
                        diff = math.degrees(j_out[name] - val)
                        if abs(diff) > 2.0:
                            deltas.append(f"{name}: {diff:.1f}")
                
                if deltas:
                    print(f"!!! INSTABILITY DETECTED !!! Solver Jumped: {', '.join(deltas)}", flush=True)
                    print("Input Joints (Deg):", flush=True)
                    for n, p in zip(node.current_joints.name, node.current_joints.position):
                    print(f"  {n}: {math.degrees(p):.2f}", flush=True)
                else:
                    print("STABLE: Solution matches current state.", flush=True)
            
            time.sleep(2.0)

    except KeyboardInterrupt:
        print("\nExiting...", flush=True)
    finally:
        rclpy.shutdown()

if __name__ == '__main__':
    main()
