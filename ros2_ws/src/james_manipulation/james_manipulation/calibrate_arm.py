#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import time
import math

class ArmCalibrator(Node):
    def __init__(self):
        super().__init__('arm_calibrator')
        
        self.declare_parameter('step_by_step', False)
        self.step_by_step = self.get_parameter('step_by_step').value
        
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)
        
        self.current_positions = [0.0] * 6
        self.joint_names = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5', 'arm_joint_6']
        self.received_state = False
        
        self.get_logger().info('Arm Calibrator Node Started')

    def joint_state_callback(self, msg):
        for i, name in enumerate(self.joint_names):
            if name in msg.name:
                idx = msg.name.index(name)
                self.current_positions[i] = msg.position[idx]
        self.received_state = True

    def wait_for_joints(self, target_positions, tolerance=0.02, timeout=30.0):
        """Wait until joints reach target positions (in radians)"""
        self.get_logger().info(f'Waiting for joints to reach {target_positions}...')
        start_time = time.time()
        while rclpy.ok():
            if time.time() - start_time > timeout:
                self.get_logger().error('Timeout waiting for joints to reach target!')
                return False
            
            reached = True
            for i in range(len(target_positions)):
                if target_positions[i] is not None:
                    if abs(self.current_positions[i] - target_positions[i]) > tolerance:
                        reached = False
                        break
            
            if reached:
                self.get_logger().info('Target reached.')
                return True
            
            rclpy.spin_once(self, timeout_sec=0.1)

    def send_raw(self, cmd):
        msg = String()
        msg.data = cmd
        self.raw_cmd_pub.publish(msg)
        self.get_logger().info(f'Sent: {cmd}')

    def confirm_step(self, description):
        """Pause for user confirmation if step_by_step is enabled"""
        if self.step_by_step:
            self.get_logger().info(f'---')
            self.get_logger().info(f'PROMPT: {description}')
            input("Press ENTER to proceed or Ctrl+C to stop...")
            self.get_logger().info(f'Proceeding...')

    def run_calibration(self):
        self.get_logger().info('Starting Custom Homing Sequence...')
        if self.step_by_step:
            self.get_logger().info('STEP-BY-STEP MODE ACTIVE')
        
        # Wait for first joint state
        while rclpy.ok() and not self.received_state:
            self.get_logger().info('Waiting for initial joint states...', throttle_duration_sec=2.0)
            rclpy.spin_once(self, timeout_sec=0.1)

        # 1. Joint 6 Home
        self.confirm_step('Home Joint 6')
        self.send_raw('HM J6')
        time.sleep(2.0) # Buffer for homing to start
        # Wait for J6 to be approx 0 (assuming HM resets to 0)
        self.wait_for_joints([None, None, None, None, None, 0.0], timeout=60.0)
        
        # Move J6 to -90
        self.confirm_step('Move J6 to -90 degrees')
        self.send_raw('MJ J1:0 J2:0 J3:0 J4:0 J5:0 J6:-90')
        self.wait_for_joints([None, None, None, None, None, math.radians(-90)])

        # 2. Joint 5 Home
        self.confirm_step('Home Joint 5')
        self.send_raw('HM J5')
        time.sleep(2.0)
        self.wait_for_joints([None, None, None, None, 0.0, None])
        
        # Move J5 to 45
        self.confirm_step('Move J5 to 45 degrees')
        self.send_raw('MJ J1:0 J2:0 J3:0 J4:0 J5:45 J6:-90')
        self.wait_for_joints([None, None, None, None, math.radians(45), math.radians(-90)])

        # 3. Joint 4 Home
        self.confirm_step('Home Joint 4')
        self.send_raw('HM J4')
        time.sleep(2.0)
        self.wait_for_joints([None, None, None, 0.0, None, None])

        # 4. Joint 3 Home
        self.confirm_step('Home Joint 3')
        self.send_raw('HM J3')
        time.sleep(2.0)
        self.wait_for_joints([None, None, 0.0, None, None, None])
        
        # Move J3 to 90
        self.confirm_step('Move Joint 3 to 90 degrees')
        self.send_raw('MJ J1:0 J2:0 J3:90 J4:0 J5:45 J6:-90')
        self.wait_for_joints([None, None, math.radians(90), None, None, None])

        # 5. Joint 1 Home
        self.confirm_step('Home Joint 1')
        self.send_raw('HM J1')
        time.sleep(2.0)
        self.wait_for_joints([0.0, None, None, None, None, None])
        
        # Move J1 to 90
        self.confirm_step('Move Joint 1 to 90 degrees')
        self.send_raw('MJ J1:90 J2:0 J3:90 J4:0 J5:45 J6:-90')
        self.wait_for_joints([math.radians(90), None, None, None, None, None])

        # 6. Joint 2 Home
        self.confirm_step('Home Joint 2 (Final Joint)')
        self.get_logger().info('Initializing Joint 2 (last one)...')
        self.send_raw('HM J2')
        time.sleep(2.0)
        self.wait_for_joints([None, 0.0, None, None, None, None])

        # 7. Move Joint 1 and 3 back to 0
        self.confirm_step('Move Joint 1 and 3 back to 0')
        self.get_logger().info('Moving Joint 1 and 3 back to 0...')
        self.send_raw('MJ J1:0 J2:0 J3:0 J4:0 J5:45 J6:-90') # Keeping J5/J6 safe
        self.wait_for_joints([0.0, None, 0.0, None, None, None])
        
        # Final Move to Default 0
        self.confirm_step('Return all joints to zero')
        self.send_raw('MJ J1:0 J2:0 J3:0 J4:0 J5:0 J6:0')
        self.wait_for_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

        self.get_logger().info('Calibration Sequence Complete!')

def main(args=None):
    rclpy.init(args=args)
    calibrator = ArmCalibrator()
    try:
        calibrator.run_calibration()
    except KeyboardInterrupt:
        pass
    finally:
        calibrator.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
