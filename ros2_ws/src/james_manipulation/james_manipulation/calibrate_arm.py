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
        self.get_logger().info('Starting Commercial Firmware Calibration Sequence (LL)...')
        
        # Calibration strings for each joint (LL protocol)
        # Sequence: J6 -> J5 -> J4 -> J3 -> J1 -> Move J1 -> J2 -> Zeroing
        
        # 1. Joint 6
        self.confirm_step('Calibrate Joint 6')
        self.send_raw('LLA0B0C0D0E0F1G0H0I0J0K0L90M0N45O-90P0Q0R0')
        # Wait for J6 to settle at its calibration position (usually -90 unless offset)
        # Firmware moveJ response/sendRobotPos will update state
        time.sleep(5.0) 
        
        # 2. Joint 5
        self.confirm_step('Calibrate Joint 5')
        self.send_raw('LLA0B0C0D0E1F0G0H0I0J0K0L90M0N45O-90P0Q0R0')
        time.sleep(5.0)

        # 3. Joint 4
        self.confirm_step('Calibrate Joint 4')
        self.send_raw('LLA0B0C0D1E0F0G0H0I0J0K0L90M0N45O-90P0Q0R0')
        time.sleep(5.0)

        # 4. Joint 3
        self.confirm_step('Calibrate Joint 3')
        self.send_raw('LLA0B0C1D0E0F0G0H0I0J0K0L90M0N45O-90P0Q0R0')
        time.sleep(5.0)

        # 5. Joint 1
        self.confirm_step('Calibrate Joint 1')
        self.send_raw('LLA1B0C0D0E0F0G0H0I0J0K0L90M0N45O-90P0Q0R0')
        time.sleep(5.0)

        # INTERMEDIATE MOVE: J1 to 45 deg
        self.confirm_step('Move J1 to 45 degrees before J2 calibration')
        # We use a raw RJ command here to be precise
        # Format: RJ A45.0B<cur>C<cur>D<cur>E<cur>F<cur>J7...
        # For simplicity, we can just send the move to 45 while keeping others at their current known safe calibration angles
        # J1=45, J2=0, J3=90, J4=0, J5=45, J6=-90 (Standard calibration pose targets)
        self.send_raw('RJ A45.0000B0.0000C90.0000D0.0000E45.0000F-90.0000J70.0000J80.0000J90.0000Sp10.00Ac10.00Dc10.00Rm10.00W0Lm000000')
        self.wait_for_joints([math.radians(45), None, None, None, None, None], timeout=30.0)

        # 6. Joint 2
        self.confirm_step('Calibrate Joint 2')
        self.send_raw('LLA0B1C0D0E0F0G0H0I0J0K0L90M0N45O-90P0Q0R0')
        time.sleep(5.0)

        # 7. Final Zeroing
        self.confirm_step('Move all joints to zero position')
        self.send_raw('RJ A0.0000B0.0000C0.0000D0.0000E0.0000F0.0000J70.0000J80.0000J90.0000Sp10.00Ac10.00Dc10.00Rm10.00W0Lm000000')
        self.wait_for_joints([0.0, 0.0, 0.0, 0.0, 0.0, 0.0], timeout=60.0)

        self.get_logger().info('Commercial Calibration Sequence Complete!')

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
