from datetime import datetime
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
import tf2_ros
from moveit_msgs.srv import GetPositionIK
import json
import time
import math
import threading


class ArmCartesianController(Node):
    """
    ROS2 node for Cartesian control of AR4-MK3 robot arm using MoveIt2 IK
    With robust Deadzone and Idle-Sync for bumpless transfer.
    """
    
    def __init__(self):
        super().__init__('arm_cartesian_controller')
        
        # Declare parameters (values are loaded from config/arm_cartesian_params.yaml)
        self.declare_parameter('velocity_scale', 0.0)
        self.declare_parameter('rotation_scale', 0.0)
        self.declare_parameter('control_rate', 0.0)
        self.declare_parameter('command_timeout', 0.0)
        self.declare_parameter('ik_timeout', 0.0)
        self.declare_parameter('joystick_deadzone', 0.0)
        self.declare_parameter('movement_lead', 1.0) # ai keeps wanting to set this to 1, event though it's not used because it coms from yaml file
        
        # Workspace limits
        self.declare_parameter('workspace_limits.x_min', 0.0)
        self.declare_parameter('workspace_limits.x_max', 0.0)
        self.declare_parameter('workspace_limits.y_min', 0.0)
        self.declare_parameter('workspace_limits.y_max', 0.0)
        self.declare_parameter('workspace_limits.z_min', 0.0)
        self.declare_parameter('workspace_limits.z_max', 0.0)
        
        # MoveIt parameters
        self.declare_parameter('move_group_name', "")
        self.declare_parameter('planning_frame', "")
        self.declare_parameter('end_effector_link', "")
        
        # Get parameters
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.control_rate = self.get_parameter('control_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.ik_timeout = self.get_parameter('ik_timeout').value
        self.deadzone = self.get_parameter('joystick_deadzone').value
        self.movement_lead = self.get_parameter('movement_lead').value
        
        self.planning_frame = self.get_parameter('planning_frame').value
        self.ee_link = self.get_parameter('end_effector_link').value
        self.group_name = self.get_parameter('move_group_name').value
        
        # Workspace limits
        self.workspace_limits = {
            'x_min': self.get_parameter('workspace_limits.x_min').value,
            'x_max': self.get_parameter('workspace_limits.x_max').value,
            'y_min': self.get_parameter('workspace_limits.y_min').value,
            'y_max': self.get_parameter('workspace_limits.y_max').value,
            'z_min': self.get_parameter('workspace_limits.z_min').value,
            'z_max': self.get_parameter('workspace_limits.z_max').value,
        }
        
        # State variables
        self.last_command_time = 0.0
        self.current_target_pose = Pose()
        self.current_joint_state = JointState()
        self.manual_control_active = False
        self.tf_synced = False
        self.joint_state_received = False
        self.ik_success = False
        
        # TF2 setup
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # IK Service client
        self.ik_client = self.create_client(GetPositionIK, '/compute_ik')
        
        # ROS2 publishers and subscribers
        self.manual_cmd_sub = self.create_subscription(
            String,
            '/arm/manual_cartesian_cmd',
            self.manual_command_callback,
            10
        )
        
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            '/arm/joint_commands',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            '/arm/status',
            10
        )
        

        # [JAMES:MOD] Publisher for Raw Commands (for STOP)
        self.raw_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)


        # Control timer
        self.control_timer = self.create_timer(
            1.0 / self.control_rate,
            self.control_loop
        )
        
        # Status timer
        self.status_timer = self.create_timer(
            0.5,  # 2 Hz status updates
            self.publish_status
        )

        # Pending velocity commands
        self.pending_v_x = 0.0
        self.pending_v_y = 0.0
        self.pending_v_z = 0.0
        self.pending_v_yaw = 0.0
        self.joy_lx = 0.0
        self.joy_ly = 0.0
        self.joy_ry = 0.0
        self.joy_rr = 0.0
        self.dynamic_sp = 5.0 # Default speed
        self.idle_start_time = None
        self.blending_active = False # [JAMES:MOD] Track firmware blending state
        self.last_sync_pose = Pose() # Track actual position for leash
        self.log_throttle_map = {} # Track last log times for manual throttling
        
        self.get_logger().info('Arm Cartesian Controller initialized (Idle-Sync Logic Enabled)')
        self.get_logger().info(f'EE Link: {self.ee_link}, Planning Frame: {self.planning_frame}')

    def log(self, msg, level='info', throttle=0.0):
        """Custom logger with manual throttle to avoid ROS2 filter errors"""
        now = time.time()
        if throttle > 0:
            key = msg[:20] 
            last_time = self.log_throttle_map.get(key, 0)
            if now - last_time < throttle:
                return
            self.log_throttle_map[key] = now

        # Stripped custom timestamps to let ROS handle it cleanly
        if level == 'info':
            self.get_logger().info(msg)
        elif level == 'warn':
            self.get_logger().warn(msg)
        elif level == 'error':
            self.get_logger().error(msg)

    def apply_deadzone(self, val):
        if abs(val) < self.deadzone:
            return 0.0
        return val

    def manual_command_callback(self, msg):
        """Process manual control commands from platform bridge"""
        try:
            data = json.loads(msg.data)
            
            if data.get('type') == 'manual_control':
                if not self.joint_state_received:
                    self.log('Manual cmd received but NO joint_state_received yet', level='warn', throttle=2.0)
                    return

                # Always active if receiving data, but velocities might be zero
                self.manual_control_active = True
                self.last_command_time = self.get_clock().now().nanoseconds / 1e9
                
                # Extract and Apply Deadzone (Support both key formats)
                joy_lx = self.apply_deadzone(data.get('lx', data.get('left_x', 0)))
                joy_ly = self.apply_deadzone(data.get('ly', data.get('left_y', 0)))
                joy_lz = self.apply_deadzone(data.get('lz', data.get('left_z', 0)))
                joy_rx = self.apply_deadzone(data.get('rx', data.get('right_x', 0)))
                joy_ry = self.apply_deadzone(data.get('ry', data.get('right_y', 0)))
                joy_rr = self.apply_deadzone(data.get('rr', data.get('right_rot', 0)))
                
                # Handle mode mapping
                switch_mode = data.get('switch_mode', 'platform')
                if 'mode' in data:
                    switch_mode = 'vertical' if data['mode'] == 1 else 'platform'
                
                # Calculate velocity commands
                self.joy_lx, self.joy_ly, self.joy_ry, self.joy_rr = joy_lx, joy_ly, joy_ry, joy_rr
                self.pending_v_x = -joy_lx * self.velocity_scale
                self.pending_v_y = joy_ly * self.velocity_scale
                
                if switch_mode == 'vertical':
                    self.pending_v_z = joy_ry * self.velocity_scale
                    self.pending_v_yaw = joy_rr * self.rotation_scale
                else:
                    self.pending_v_z = 0.0
                    self.pending_v_yaw = 0.0
                
                # DEBUG: Log inputs and calculated velocities
                self.log(f'INPUT: lx={joy_lx:.2f}, ly={joy_ly:.2f} -> Vx={self.pending_v_x:.3f}, Vy={self.pending_v_y:.3f}, Vz={self.pending_v_z:.3f} Mode={switch_mode}')
                
        except Exception as e:
            self.log(f'Error processing manual command: {e}', level='error')

    def joint_state_callback(self, msg):
        self.current_joint_state = msg
        self.joint_state_received = True

        # [JAMES:MOD] SELF-CLOCKING PULSE (V15)
        # Every time we receive a state update (Teensy ACK), we generate the NEXT segment
        if self.manual_control_active and self.blending_active:
             self.produce_next_segment()

    def sync_pose_to_actual(self, loud=False):
        """Initialize current_target_pose from the actual arm position via TF"""
        try:
            now = rclpy.time.Time()
            transform = self.tf_buffer.lookup_transform(
                self.planning_frame,
                self.ee_link,
                now,
                timeout=rclpy.duration.Duration(seconds=0.2)
            )
            
            self.current_target_pose.position.x = transform.transform.translation.x
            self.current_target_pose.position.y = transform.transform.translation.y
            self.current_target_pose.position.z = transform.transform.translation.z
            self.current_target_pose.orientation = transform.transform.rotation

            # [JAMES:MOD] Update last_sync_pose as well
            self.last_sync_pose.position.x = transform.transform.translation.x
            self.last_sync_pose.position.y = transform.transform.translation.y
            self.last_sync_pose.position.z = transform.transform.translation.z
            self.last_sync_pose.orientation = transform.transform.rotation
            
            self.tf_synced = True
            # Log synchronization periodically to verify we have the correct starting pose
            if loud:
                self.log(f'SYNC: TF Pose -> X={self.current_target_pose.position.x:.3f}, Y={self.current_target_pose.position.y:.3f}, Z={self.current_target_pose.position.z:.3f}')
            else:
                 self.log(f'SYNC: TF Pose -> X={self.current_target_pose.position.x:.3f}, Y={self.current_target_pose.position.y:.3f}, Z={self.current_target_pose.position.z:.3f}', throttle=2.0)
            return True
        except Exception as e:
            if loud:
                self.log(f'Could not sync pose from TF: {e}', level='warn')
            self.tf_synced = False
            return False

    def control_loop(self):
        """Watchdog loop at 10Hz - Handles only STOP logic and Idle syncing"""
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Timeout Check
        if current_time - self.last_command_time > self.command_timeout:
            self.manual_control_active = False
        
        is_idle = (abs(self.pending_v_x) + abs(self.pending_v_y) + abs(self.pending_v_z) + abs(self.pending_v_yaw)) < 1e-6
        
        if not self.manual_control_active or is_idle:
            if self.idle_start_time is None:
                self.idle_start_time = current_time
            
            # Stop Hysteresis
            if self.blending_active and (current_time - self.idle_start_time > 0.3):
                self.log('Joystick Idle for >0.3s -> Sending STOP (BM0)')
                stop_msg = String()
                stop_msg.data = "BM0"
                self.raw_cmd_pub.publish(stop_msg)
                stop_msg.data = "ST"
                self.raw_cmd_pub.publish(stop_msg)
                self.blending_active = False
                self.manual_control_active = False 
            
            if not self.blending_active:
                if not self.sync_pose_to_actual(loud=False):
                    self.log('Sync Pose Failed (TF issue?)', level='warn', throttle=2.0)
            return

        # Start Move logic (Initial Push)
        self.idle_start_time = None
        if not self.blending_active:
             self.log('Starting Move -> Initial Priming Burst (BM1 + 3x Segments)')
             if not self.sync_pose_to_actual(loud=True):
                 self.log('Cannot start move: Initial sync failed', level='warn')
                 return
             
             start_msg = String()
             start_msg.data = "BM1"
             self.raw_cmd_pub.publish(start_msg)
             self.blending_active = True
             
             # Prime the Teensy buffer with 3 initial segments to start the closed loop
             for _ in range(3):
                  self.produce_next_segment()
        return
 
    def produce_next_segment(self):
        """Closed-loop Producer: Generates one 'Beefy' segment and sends to bridge (V15)"""
        # Dynamic Speed Scaling
        joy_magnitude = math.sqrt(self.joy_lx**2 + self.joy_ly**2 + self.joy_ry**2 + self.joy_rr**2)
        joy_magnitude = min(1.0, joy_magnitude)
        self.dynamic_sp = 1.0 + (joy_magnitude * 29.0) # 1% to 30%
        
        # IK Failure Recovery -> Re-syncing carrot to actual
        if not self.ik_success:
             self.log('IK Failure Recovery -> Re-syncing carrot to actual', level='warn')
             if not self.sync_pose_to_actual(loud=True):
                 return
             self.ik_success = True

        # Calculate Delta: 0.2s of movement per segment (Fixed Lead)
        # In V15, we define segments by 'work duration', not 'ROS clock rate'.
        # dt = (duration of path segment in seconds)
        dt = 0.2 * self.movement_lead 
        
        new_x = self.current_target_pose.position.x + self.pending_v_x * dt
        new_y = self.current_target_pose.position.y + self.pending_v_y * dt
        new_z = self.current_target_pose.position.z + self.pending_v_z * dt
        
        # Carrot Leash (V15: Tightened to 2cm for strict local stability)
        max_dist = 0.02
        dx = new_x - self.last_sync_pose.position.x
        dy = new_y - self.last_sync_pose.position.y
        dz = new_z - self.last_sync_pose.position.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if dist > max_dist:
             scale = max_dist / dist
             self.current_target_pose.position.x = self.last_sync_pose.position.x + dx * scale
             self.current_target_pose.position.y = self.last_sync_pose.position.y + dy * scale
             self.current_target_pose.position.z = self.last_sync_pose.position.z + dz * scale
        else:
             self.current_target_pose.position.x = new_x
             self.current_target_pose.position.y = new_y
             self.current_target_pose.position.z = new_z

        if abs(self.pending_v_yaw) > 1e-6:
            self.apply_yaw_step(self.pending_v_yaw * dt)

        # Apply Workspace Limits
        self.current_target_pose = self.apply_workspace_limits(self.current_target_pose)

        # Submit IK request asynchronously
        self.call_ik_service_async()

    def apply_yaw_step(self, delta_yaw):
        cos_y = math.cos(delta_yaw / 2.0)
        sin_y = math.sin(delta_yaw / 2.0)
        dqz = sin_y
        dqw = cos_y
        qx = self.current_target_pose.orientation.x
        qy = self.current_target_pose.orientation.y
        qz = self.current_target_pose.orientation.z
        qw = self.current_target_pose.orientation.w
        self.current_target_pose.orientation.w = qw * dqw - qx * 0 - qy * 0 - qz * dqz
        self.current_target_pose.orientation.x = qw * 0 + qx * dqw + qy * dqz - qz * 0
        self.current_target_pose.orientation.y = qw * 0 - qx * dqz + qy * dqw + qz * 0
        self.current_target_pose.orientation.z = qw * dqz + qx * 0 - qy * 0 + qz * dqw
        norm = math.sqrt(self.current_target_pose.orientation.x**2 + self.current_target_pose.orientation.y**2 + self.current_target_pose.orientation.z**2 + self.current_target_pose.orientation.w**2)
        self.current_target_pose.orientation.x /= norm
        self.current_target_pose.orientation.y /= norm
        self.current_target_pose.orientation.z /= norm
        self.current_target_pose.orientation.w /= norm

    def apply_workspace_limits(self, pose):
        pose.position.x = max(self.workspace_limits['x_min'], min(self.workspace_limits['x_max'], pose.position.x))
        pose.position.y = max(self.workspace_limits['y_min'], min(self.workspace_limits['y_max'], pose.position.y))
        pose.position.z = max(self.workspace_limits['z_min'], min(self.workspace_limits['z_max'], pose.position.z))
        return pose

    def call_ik_service_async(self):
        if not self.ik_client.service_is_ready():
            self.get_logger().error('IK Service /compute_ik NOT READY', throttle_duration_sec=2.0)
            return
        req = GetPositionIK.Request()
        req.ik_request.group_name = self.group_name
        req.ik_request.robot_state.joint_state = self.current_joint_state
        req.ik_request.avoid_collisions = True
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = self.planning_frame
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose = self.current_target_pose
        req.ik_request.pose_stamped = pose_stamped
        req.ik_request.timeout = rclpy.duration.Duration(seconds=self.ik_timeout).to_msg()
        future = self.ik_client.call_async(req)
        future.add_done_callback(self.ik_callback)

    def ik_callback(self, future):
        try:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                self.ik_success = True

                # KINEMATICS DEBUG: Measure the jump
                if self.current_joint_state:
                    tgt = response.solution.joint_state.position
                    cur = self.current_joint_state.position
                    # Safe zip (names might not match order, but usually do in MoveIt)
                    if len(tgt) == len(cur):
                         diffs = [math.degrees(t - c) for t, c in zip(tgt, cur)]
                         # Log if any joint moves more than 2 degrees
                         if any(abs(d) > 2.0 for d in diffs):
                             self.log(f'LARGE JUMP: J1={diffs[0]:.1f}, J2={diffs[1]:.1f}, J3={diffs[2]:.1f}', level='warn')

                cmd_msg = JointState()
                cmd_msg.header.stamp = self.get_clock().now().to_msg()
                cmd_msg.name = response.solution.joint_state.name
                cmd_msg.position = response.solution.joint_state.position
                
                # [JAMES:MOD] Pass dynamic speed in the velocity field (V9)
                cmd_msg.velocity = [self.dynamic_sp]
                
                self.joint_cmd_pub.publish(cmd_msg)
            else:
                self.log(f'IK FAILED: Error Code {response.error_code.val}', level='warn')
                self.ik_success = False # Trigger re-sync in next loop
        except Exception as e:
            self.log(f'Error in ik_callback: {e}', level='error')

    def publish_status(self):
        status = {
            'type': 'arm_status',
            'manual_active': self.manual_control_active,
            'ik_success': self.ik_success,
            'target_pose': {
                'x': round(self.current_target_pose.position.x, 3),
                'y': round(self.current_target_pose.position.y, 3),
                'z': round(self.current_target_pose.position.z, 3)
            },
            'timestamp': self.get_clock().now().nanoseconds / 1e9
        }
        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmCartesianController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt: pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
