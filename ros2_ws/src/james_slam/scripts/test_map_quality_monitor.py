#!/usr/bin/env python3
"""
Test script for Map Quality Monitor

This script simulates RTAB-Map info messages to test the map quality monitor
without requiring actual SLAM to be running.
"""

import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import Info
from geometry_msgs.msg import Transform
from std_msgs.msg import Header
import time
import random


class MapQualityMonitorTester(Node):
    """
    Publishes simulated RTAB-Map info messages for testing.
    """
    
    def __init__(self):
        super().__init__('map_quality_monitor_tester')
        
        # Publisher for simulated info messages
        self.info_pub = self.create_publisher(
            Info,
            '/rtabmap/info',
            10
        )
        
        # Timer to publish test messages
        self.timer = self.create_timer(1.0, self.publish_test_info)
        
        self.message_count = 0
        self.node_id = 1
        
        self.get_logger().info('Map Quality Monitor Tester started')
        self.get_logger().info('Publishing simulated RTAB-Map info messages...')
    
    def publish_test_info(self):
        """
        Publish a simulated info message.
        
        Simulates different scenarios:
        - Normal operation with occasional loop closures
        - Some loop closures succeed, some fail
        """
        msg = Info()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        # Simulate node count increasing
        self.node_id += 1
        msg.nodes = self.node_id
        
        # Simulate loop closure every 5 messages
        if self.message_count % 5 == 0 and self.message_count > 0:
            # Simulate loop closure detection
            msg.loop_closure_id = self.node_id
            msg.ref_id = max(1, self.node_id - random.randint(5, 15))
            
            # Simulate success/failure (70% success rate)
            if random.random() < 0.7:
                # Success: positive covariance
                msg.loop_closure_transform = Transform()
                msg.loop_closure_transform.covariance = [1.0] * 36
                self.get_logger().info(
                    f'Simulating successful loop closure: {msg.loop_closure_id} -> {msg.ref_id}'
                )
            else:
                # Failure: zero covariance
                msg.loop_closure_transform = Transform()
                msg.loop_closure_transform.covariance = [0.0] * 36
                self.get_logger().info(
                    f'Simulating failed loop closure: {msg.loop_closure_id} -> {msg.ref_id}'
                )
        else:
            # No loop closure
            msg.loop_closure_id = 0
            msg.ref_id = 0
        
        self.info_pub.publish(msg)
        self.message_count += 1
        
        # Log progress
        if self.message_count % 10 == 0:
            self.get_logger().info(f'Published {self.message_count} test messages')


def main(args=None):
    rclpy.init(args=args)
    
    node = MapQualityMonitorTester()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
