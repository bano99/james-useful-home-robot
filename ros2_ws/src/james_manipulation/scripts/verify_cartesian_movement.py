#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from std_msgs.msg import String
from moveit_msgs.srv import GetPositionIK
from moveit_msgs.msg import Constraints, JointConstraint
import tf2_ros
import time
import argparse
import math
import threading
import sys
import signal
from datetime import datetime

def get_timestamp():
    return datetime.now().strftime("%H:%M:%S.%f")[:-3]

class CartesianMovementVerifier(Node):
    def __init__(self, args):
        super().__init__('cartesian_movement_verifier')
        self.args = args
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(JointState, '/arm/joint_commands', 10)
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        
        # IK Client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # TF Setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Subscriber for Sync
        self.current_joint_state = None
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
        
        self.stop_requested = False

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        if not self.sync_event.is_set():
            self.sync_event.set()

    def get_ik(self, target_pose):
        if not self.ik_client.service_is_ready():
            self.get_logger().error('IK Service NOT READY')
            return None
            
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.args.group
        req.ik_request.robot_state.joint_state = self.current_joint_state
        req.ik_request.avoid_collisions = False # Keep relaxed for verification
        
        # [JAMES:MOD] Add Joint Constraints to lock specific joints if requested
        if self.args.lock:
            constraints = Constraints()
            # We want to lock J4 and J6 to their current seed position
            joints_to_lock = ['arm_joint_4', 'arm_joint_6']
            for name in joints_to_lock:
                if name in self.current_joint_state.name:
                    idx = self.current_joint_state.name.index(name)
                    jc = JointConstraint()
                    jc.joint_name = name
                    jc.position = self.current_joint_state.position[idx]
                    jc.tolerance_above = 0.1 # ~5 degrees as per briefing
                    jc.tolerance_below = 0.1
                    jc.weight = 1.0 # High priority
                    constraints.joint_constraints.append(jc)
            req.ik_request.constraints = constraints
            # self.get_logger().info(f"Applying JointConstraints to lock J4 and J6 (tolerance: 0.1 rad)")
        
        # Log seed joints for debugging failing IK
        j_str = ", ".join([f"{n}:{p:.3f}" for n, p in zip(self.current_joint_state.name, self.current_joint_state.position)])
        # self.get_logger().info(f"IK Seed Joints: {j_str}") 
        
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "base_link"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = target_pose
        req.ik_request.pose_stamped = pose_stamped
        
        # Use a synchronous call with timeout since we are in a MultiThreadedExecutor
        future = self.ik_client.call_async(req)
        # Instead of spin_until_future_complete, we wait on the future
        # which is safe in a separate thread if using MultiThreadedExecutor
        start_wait = time.time()
        while rclpy.ok() and not future.done() and (time.time() - start_wait < 2.0):
            time.sleep(0.01)
            
        if future.done():
            res = future.result()
            if res.error_code.val == res.error_code.SUCCESS:
                return res.solution.joint_state
            else:
                self.get_logger().error(f'IK Failed with error code: {res.error_code.val}')
        else:
            self.get_logger().error('IK Service timed out')
        return None

    def run_test(self, executor):
        try:
            self._do_run_test()
        except KeyboardInterrupt:
            print("\nTest interrupted.")
        except Exception as e:
            print(f"Error during test: {e}")
        finally:
            print("Shutting down test...")
            executor.shutdown()

    def _do_run_test(self):
        print(f"[{get_timestamp()}] Initializing Cartesian Test...")
        print(f"Waiting for /joint_states and IK service...")
        if not self.sync_event.wait(timeout=5.0):
            print("Failed to sync initial position. Is the bridge running?")
            return
        
        if not self.ik_client.wait_for_service(timeout_sec=5.0):
            print("IK Service not found. Is MoveIt running?")
            return

        # 1. Get current EE pose via TF
        print(f"[{get_timestamp()}] Looking up current transform (base_link -> arm_link_6)...")
        start_pose = None
        for i in range(5):
            try:
                trans = self.tf_buffer.lookup_transform('base_link', 'arm_link_6', rclpy.time.Time(), rclpy.duration.Duration(seconds=1.0))
                start_pose = Pose()
                start_pose.position.x = trans.transform.translation.x
                start_pose.position.y = trans.transform.translation.y
                start_pose.position.z = trans.transform.translation.z
                start_pose.orientation = trans.transform.rotation
                break
            except Exception as e:
                print(f"TF lookup attempt {i+1} failed: {e}")
                time.sleep(0.5)
        
        if not start_pose:
            print("Failed to get current pose via TF after multiple attempts.")
            return

        print(f"[{get_timestamp()}] Start Pose: X={start_pose.position.x:.3f}, Y={start_pose.position.y:.3f}, Z={start_pose.position.z:.3f}")

        # 2. Calculate Segments (Units: mm to m)
        dx_m = self.args.dx / 1000.0
        dy_m = self.args.dy / 1000.0
        dz_m = self.args.dz / 1000.0
        
        segments = []
        for i in range(1, self.args.segments + 1):
            ratio = i / float(self.args.segments)
            p = Pose()
            p.position.x = start_pose.position.x + dx_m * ratio
            p.position.y = start_pose.position.y + dy_m * ratio
            p.position.z = start_pose.position.z + dz_m * ratio
            p.orientation = start_pose.orientation
            segments.append(p)

        # 3. Solve IK for all segments first (Safety)
        print(f"[{get_timestamp()}] Solving IK for {len(segments)} segments...")
        joint_targets = []
        for i, p in enumerate(segments):
            if self.stop_requested: return
            sol = self.get_ik(p)
            if sol:
                joint_targets.append(sol)
                if (i+1) % 5 == 0 or i == 0:
                    print(f"  Solved {i+1}/{len(segments)}...")
            else:
                print(f"[{get_timestamp()}] IK Failed for segment {i+1}. Target was X={p.position.x:.3f}, Y={p.position.y:.3f}, Z={p.position.z:.3f}")
                print("Hint: Check if the target is within reach and not self-colliding.")
                return

        # 4. Execute
        print(f"[{get_timestamp()}] IK Solved. Enabling Blending (BM1) and executing...")
        self.raw_cmd_pub.publish(String(data="BM1"))
        time.sleep(1.0) # Wait for BM1 to take effect

        for i, sol in enumerate(joint_targets):
            if self.stop_requested: break
            cmd_msg = JointState()
            cmd_msg.header.stamp = self.get_clock().now().to_msg()
            cmd_msg.name = self.joint_names
            cmd_msg.position = sol.position
            cmd_msg.velocity = [float(self.args.speed)]
            self.joint_cmd_pub.publish(cmd_msg)
            if (i+1) % 5 == 0 or i == 0:
                 print(f"[{get_timestamp()}] Sent segment {i+1}/{self.args.segments}")
            time.sleep(0.12) # ~8Hz pump

        print(f"[{get_timestamp()}] Move complete. Waiting for robot to settle...")
        time.sleep(2.0)
        self.raw_cmd_pub.publish(String(data="BM0"))
        self.raw_cmd_pub.publish(String(data="ST"))
        print(f"[{get_timestamp()}] Test Finished.")

def main():
    parser = argparse.ArgumentParser(description='Verify Cartesian Movement')
    parser.add_argument('--dx', type=float, default=0.0, help='X offset in mm')
    parser.add_argument('--dy', type=float, default=0.0, help='Y offset in mm')
    parser.add_argument('--dz', type=float, default=0.0, help='Z offset in mm')
    parser.add_argument('--segments', type=int, default=10, help='Number of segments')
    parser.add_argument('--speed', type=float, default=15.0, help='Speed value (%)')
    parser.add_argument('--group', type=str, default='ar_manipulator', help='MoveIt planning group')
    parser.add_argument('--lock', action='store_true', help='Enable J4/J6 locking via JointConstraints')
    
    args = parser.parse_args()
    rclpy.init()
    node = CartesianMovementVerifier(args)
    
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    # Run test in thread
    thread = threading.Thread(target=node.run_test, args=(executor,), daemon=True)
    thread.start()
    
    def signal_handler(sig, frame):
        print("\nCTRL+C detected! Stopping...")
        node.stop_requested = True
        executor.shutdown()
        # Non-graceful exit if it hangs
        time.sleep(0.5)
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
