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
    """
    
    def __init__(self):
        super().__init__('arm_cartesian_controller')
        
        # Declare parameters
        self.declare_parameter('velocity_scale', 0.001)
        self.declare_parameter('rotation_scale', 0.01)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('command_timeout', 0.5)
        self.declare_parameter('ik_timeout', 0.05)
        
        # Workspace limits
        self.declare_parameter('workspace_limits.x_min', 0.2)
        self.declare_parameter('workspace_limits.x_max', 0.6)
        self.declare_parameter('workspace_limits.y_min', -0.3)
        self.declare_parameter('workspace_limits.y_max', 0.3)
        self.declare_parameter('workspace_limits.z_min', 0.1)
        self.declare_parameter('workspace_limits.z_max', 0.8)
        
        # MoveIt parameters
        self.declare_parameter('move_group_name', 'ar4_arm')
        self.declare_parameter('planning_frame', 'base_link')
        self.declare_parameter('end_effector_link', 'gripper_link')
        
        # Get parameters
        self.velocity_scale = self.get_parameter('velocity_scale').value
        self.rotation_scale = self.get_parameter('rotation_scale').value
        self.control_rate = self.get_parameter('control_rate').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.ik_timeout = self.get_parameter('ik_timeout').value
        
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
        
        self.get_logger().info('Arm Cartesian Controller initialized')
        self.get_logger().info(f'EE Link: {self.ee_link}, Planning Frame: {self.planning_frame}')

    def manual_command_callback(self, msg):
        """Process manual control commands from platform bridge"""
        try:
            data = json.loads(msg.data)
            
            if data.get('type') == 'manual_control':
                # Critical: Safety checks before activating control
                if not self.joint_state_received:
                    self.get_logger().warn('Ignoring manual command: Joint states not yet received', throttle_duration_sec=2.0)
                    return

                # If this is the start of manual control, or not yet synced, try to sync
                if not self.tf_synced:
                    if not self.sync_pose_to_actual():
                        # Don't activate if we can't sync Current Pose
                        return
                
                self.last_command_time = self.get_clock().now().nanoseconds / 1e9
                self.manual_control_active = True
                
                # Extract joystick values
                # Left Joystick: arm forward/back (X), left/right (Y), up/down (Z)
                # Map from Remote: left_y -> Cartesian X, left_x -> Cartesian Y, left_z -> Cartesian Z
                joy_lx = data.get('left_x', 0)
                joy_ly = data.get('left_y', 0)
                joy_lz = data.get('left_z', 0)
                
                # Right Joystick: rotation or extra Z if in vertical mode
                joy_rx = data.get('right_x', 0)
                joy_ry = data.get('right_y', 0)
                joy_rr = data.get('right_rot', 0)
                
                switch_mode = data.get('switch_mode', 'platform')
                
                # Calculate velocity commands
                self.pending_v_x = joy_ly * self.velocity_scale
                self.pending_v_y = joy_lx * self.velocity_scale
                self.pending_v_z = joy_lz * self.velocity_scale
                
                # If switch is 'vertical' (OFF), Right joystick can also control arm
                if switch_mode == 'vertical':
                    # Add right_y to Z if desired, or use for rotation
                    self.pending_v_z += joy_ry * self.velocity_scale
                    self.pending_v_yaw = joy_rr * self.rotation_scale
                else:
                    self.pending_v_yaw = 0.0
                
        except Exception as e:
            self.get_logger().error(f'Error processing manual command: {e}')

    def joint_state_callback(self, msg):
        """Update current joint state from bridge feedback"""
        self.current_joint_state = msg
        self.joint_state_received = True

    def sync_pose_to_actual(self):
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
            
            self.tf_synced = True
            self.get_logger().info(f'Synchronized target pose to actual arm position: {self.current_target_pose.position.x:.3f}, {self.current_target_pose.position.y:.3f}, {self.current_target_pose.position.z:.3f}')
            return True
        except Exception as e:
            self.get_logger().warn(f'Could not sync pose from TF: {e}', throttle_duration_sec=1.0)
            self.tf_synced = False
            return False

    def control_loop(self):
        """Main control loop at control_rate"""
        if not self.manual_control_active:
            # Maybe keep syncing to actual when inactive to avoid jump on start
            # But only if TF is available
            return

        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # Check for command timeout
        if current_time - self.last_command_time > self.command_timeout:
            self.manual_control_active = False
            self.get_logger().info('Manual control timeout - arm stationary')
            return

        # 1. Update target pose
        dt = 1.0 / self.control_rate
        
        self.current_target_pose.position.x += self.pending_v_x * dt
        self.current_target_pose.position.y += self.pending_v_y * dt
        self.current_target_pose.position.z += self.pending_v_z * dt
        
        # Apply rudimentary rotation (Yaw only) if needed
        if abs(self.pending_v_yaw) > 0.001:
            self.apply_yaw_step(self.pending_v_yaw * dt)

        # 2. Apply Workspace Limits
        self.current_target_pose = self.apply_workspace_limits(self.current_target_pose)

        # 3. Solve IK
        self.call_ik_service_async()

    def apply_yaw_step(self, delta_yaw):
        """Rudimentary Yaw rotation update (quaternion math)"""
        # Approximating a small rotation around Z axis
        # q_orig * q_delta
        cos_y = math.cos(delta_yaw / 2.0)
        sin_y = math.sin(delta_yaw / 2.0)
        
        # Delta quaternion (0, 0, sin, cos)
        dqx = 0.0
        dqy = 0.0
        dqz = sin_y
        dqw = cos_y
        
        # Current quaternion
        qx = self.current_target_pose.orientation.x
        qy = self.current_target_pose.orientation.y
        qz = self.current_target_pose.orientation.z
        qw = self.current_target_pose.orientation.w
        
        # Result = q * dq
        self.current_target_pose.orientation.w = qw * dqw - qx * dqx - qy * dqy - qz * dqz
        self.current_target_pose.orientation.x = qw * dqx + qx * dqw + qy * dqz - qz * dqy
        self.current_target_pose.orientation.y = qw * dqy - qx * dqz + qy * dqw + qz * dqx
        self.current_target_pose.orientation.z = qw * dqz + qx * dqy - qy * dqx + qz * dqw
        
        # Normalize
        norm = math.sqrt(self.current_target_pose.orientation.x**2 + 
                         self.current_target_pose.orientation.y**2 + 
                         self.current_target_pose.orientation.z**2 + 
                         self.current_target_pose.orientation.w**2)
        self.current_target_pose.orientation.x /= norm
        self.current_target_pose.orientation.y /= norm
        self.current_target_pose.orientation.z /= norm
        self.current_target_pose.orientation.w /= norm

    def apply_workspace_limits(self, pose):
        """Clamp target pose within safe workspace"""
        pose.position.x = max(self.workspace_limits['x_min'], min(self.workspace_limits['x_max'], pose.position.x))
        pose.position.y = max(self.workspace_limits['y_min'], min(self.workspace_limits['y_max'], pose.position.y))
        pose.position.z = max(self.workspace_limits['z_min'], min(self.workspace_limits['z_max'], pose.position.z))
        return pose

    def call_ik_service_async(self):
        """Asynchronously call MoveIt2 IK service"""
        if not self.ik_client.service_is_ready():
            self.get_logger().debug('IK service not ready', throttle_duration_sec=1.0)
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
        """Handle IK service response"""
        try:
            response = future.result()
            if response.error_code.val == response.error_code.SUCCESS:
                self.ik_success = True
                # Publish joint commands
                cmd_msg = JointState()
                cmd_msg.header.stamp = self.get_clock().now().to_msg()
                cmd_msg.name = response.solution.joint_state.name
                cmd_msg.position = response.solution.joint_state.position
                self.joint_cmd_pub.publish(cmd_msg)
            else:
                self.ik_success = False
                self.get_logger().debug(f'IK failed: {response.error_code.val}', throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().error(f'IK service call failed: {e}')

    def publish_status(self):
        """Publish node status for other nodes or telemetry"""
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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
