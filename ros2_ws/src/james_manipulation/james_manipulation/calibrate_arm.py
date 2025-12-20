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

    def wait_for_joints(self, target_positions, tolerance=0.05, timeout=30.0):
        """Wait until joints reach target positions (in radians)"""
        self.get_logger().info(f'Waiting for joints to reach target...')
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

    def move_joints(self, j1=None, j2=None, j3=None, j4=None, j5=None, j6=None, speed=25, accel=15, decel=15, ramp=80):
        """Send RJ command with all 6 joint positions in degrees
        Use current position for any joint set to None"""
        
        # Wait a moment for latest feedback
        rclpy.spin_once(self, timeout_sec=0.1)
        
        # Convert current positions from radians to degrees
        current_deg = [math.degrees(pos) for pos in self.current_positions]
        
        # Use provided values or fall back to current position
        j1_target = j1 if j1 is not None else current_deg[0]
        j2_target = j2 if j2 is not None else current_deg[1]
        j3_target = j3 if j3 is not None else current_deg[2]
        j4_target = j4 if j4 is not None else current_deg[3]
        j5_target = j5 if j5 is not None else current_deg[4]
        j6_target = j6 if j6 is not None else current_deg[5]
        
        cmd = f'RJA{j1_target:.3f}B{j2_target:.3f}C{j3_target:.3f}D{j4_target:.3f}E{j5_target:.3f}F{j6_target:.3f}'
        cmd += f'J70.00J80.00J90.00Sp{speed}Ac{accel}Dc{decel}Rm{ramp}W0Lm111000'
        self.send_raw(cmd)

    def run_calibration(self):
        self.get_logger().info('Starting Commercial Firmware Calibration Sequence...')
        self.get_logger().info('New Sequence: J6→90°, J5→0°, J4, J3→-85°, J1→-45°, J3→45°, J2, J1→0°')
        
        # Wait a moment for joint state feedback to start
        time.sleep(1.0)
        
        # Track current known positions (in degrees) after each calibration
        # These are the calibration offsets from ARconfig.json
        cal_positions = {
            'J1': 0.0,   # J1calOff
            'J2': 0.0,   # J2calOff
            'J3': 90.0,  # J3calOff
            'J4': 0.0,   # J4calOff
            'J5': 45.0,  # J5calOff
            'J6': -90.0  # J6calOff
        }
        
        # 1. Calibrate Joint 6, then move to 90 deg
        self.confirm_step('Calibrate Joint 6')
        self.send_raw('LLA0B0C0D0E0F1G0H0I0J0.0K-26.7L0.0M0.0N0.0O0.0P0.0Q0.0R0.0')
        time.sleep(5.0)
        
        self.confirm_step('Move J6 to 90 degrees')
        self.move_joints(j6=90.0, speed=50)
        self.wait_for_joints([None, None, None, None, None, math.radians(90)], timeout=15.0)
        cal_positions['J6'] = 90.0
        
        # 2. Calibrate Joint 5, then move to 0 deg
        self.confirm_step('Calibrate Joint 5')
        self.send_raw('LLA0B0C0D0E1F0G0H0I0J0.0K-26.7L0.0M0.0N0.0O0.0P0.0Q0.0R0.0')
        time.sleep(5.0)
        
        self.confirm_step('Move J5 to 0 degrees')
        self.move_joints(j5=0.0, speed=50)
        self.wait_for_joints([None, None, None, None, math.radians(0), None], timeout=15.0)
        cal_positions['J5'] = 0.0

        # 3. Calibrate Joint 4
        self.confirm_step('Calibrate Joint 4')
        self.send_raw('LLA0B0C0D1E0F0G0H0I0J0.0K-26.7L0.0M0.0N0.0O0.0P0.0Q0.0R0.0')
        time.sleep(5.0)
        cal_positions['J4'] = 0.0

        # 4. Calibrate Joint 3, then move to -85 deg
        self.confirm_step('Calibrate Joint 3')
        self.send_raw('LLA0B0C1D0E0F0G0H0I0J0.0K-26.7L0.0M0.0N0.0O0.0P0.0Q0.0R0.0')
        time.sleep(5.0)
        
        self.confirm_step('Move J3 to -85 degrees')
        self.move_joints(j3=-85.0, speed=50)
        self.wait_for_joints([None, None, math.radians(-85), None, None, None], timeout=15.0)
        cal_positions['J3'] = -85.0

        # 5. Calibrate Joint 1, then move to -45 deg
        self.confirm_step('Calibrate Joint 1')
        self.send_raw('LLA1B0C0D0E0F0G0H0I0J0.0K-26.7L0.0M0.0N0.0O0.0P0.0Q0.0R0.0')
        time.sleep(5.0)
        
        self.confirm_step('Move J1 to -45 degrees')
        self.move_joints(j1=-45.0)
        self.wait_for_joints([math.radians(-45), None, None, None, None, None], timeout=15.0)
        cal_positions['J1'] = -45.0

        # 6. Move J3 to 35 deg
        self.confirm_step('Move J3 to 35 degrees')
        self.move_joints(j3=35.0, speed=50)
        self.wait_for_joints([None, None, math.radians(35), None, None, None], timeout=15.0)
        cal_positions['J3'] = 35.0

        # 7. Calibrate Joint 2
        self.confirm_step('Calibrate Joint 2')
        self.send_raw('LLA0B1C0D0E0F0G0H0I0J0.0K-26.7L0.0M0.0N0.0O0.0P0.0Q0.0R0.0')
        time.sleep(5.0)
        
        self.confirm_step('Move J2 to 0 degrees')
        self.move_joints(j2=0.0)
        self.wait_for_joints([None, math.radians(0), None, None, None, None], timeout=15.0)
        cal_positions['J2'] = 0.0

        # 8. Move J1 to 0 deg (final safe position)
        self.confirm_step('Move J1 to 0 degrees (final safe position)')
        self.move_joints(j1=0.0)
        self.wait_for_joints([math.radians(0), None, None, None, None, None], timeout=15.0)

        self.get_logger().info('Calibration Sequence Complete!')
        self.get_logger().info(f'Final position: J1=0°, J2={cal_positions["J2"]}°, J3={cal_positions["J3"]}°, J4={cal_positions["J4"]}°, J5={cal_positions["J5"]}°, J6={cal_positions["J6"]}°')

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
