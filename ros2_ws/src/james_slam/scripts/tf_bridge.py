#!/usr/bin/env python3
"""
Simple TF bridge to connect T265 odometry frame to D435 camera frame.
This allows RTAB-Map to work without a full robot TF tree.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class TFBridge(Node):
    def __init__(self):
        super().__init__('tf_bridge')
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publish TF at 30Hz
        self.timer = self.create_timer(1.0/30.0, self.publish_transforms)
        
        self.get_logger().info('TF Bridge started - publishing odom_frame -> d435_color_optical_frame')
    
    def publish_transforms(self):
        # Create transform from odom_frame to d435_color_optical_frame
        # This is a static identity transform since both cameras are rigidly mounted
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom_frame'
        t.child_frame_id = 'd435_color_optical_frame'
        
        # Identity transform (no translation or rotation)
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = TFBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
