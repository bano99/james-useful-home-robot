#!/usr/bin/env python3
"""
Integration test for Map Quality Monitor

Tests the map quality monitor's ability to:
1. Track loop closure success rates
2. Detect map degradation
3. Publish diagnostics correctly
"""

import unittest
import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import Info
from diagnostic_msgs.msg import DiagnosticArray
from geometry_msgs.msg import Transform
from std_msgs.msg import Header
import time
import threading


class TestMapQualityMonitor(unittest.TestCase):
    """
    Test suite for Map Quality Monitor
    """
    
    @classmethod
    def setUpClass(cls):
        """Initialize ROS2"""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2"""
        rclpy.shutdown()
    
    def setUp(self):
        """Set up test fixtures"""
        self.node = Node('test_map_quality_monitor')
        
        # Publisher for test info messages
        self.info_pub = self.node.create_publisher(
            Info,
            '/rtabmap/info',
            10
        )
        
        # Subscriber for diagnostics
        self.diagnostics_received = []
        self.diag_sub = self.node.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )
        
        # Spin node in background thread
        self.spin_thread = threading.Thread(target=self.spin_node, daemon=True)
        self.spin_thread.start()
        
        # Wait for connections
        time.sleep(1.0)
    
    def tearDown(self):
        """Clean up"""
        self.node.destroy_node()
    
    def spin_node(self):
        """Spin node in background"""
        while rclpy.ok():
            rclpy.spin_once(self.node, timeout_sec=0.1)
    
    def diagnostics_callback(self, msg):
        """Store received diagnostics"""
        self.diagnostics_received.append(msg)
    
    def publish_info_message(self, loop_closure_id=0, ref_id=0, success=True):
        """
        Publish a test info message
        
        Args:
            loop_closure_id: ID of loop closure (0 = no loop closure)
            ref_id: Reference node ID
            success: Whether loop closure succeeded
        """
        msg = Info()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        
        msg.loop_closure_id = loop_closure_id
        msg.ref_id = ref_id
        msg.nodes = 10
        
        if loop_closure_id > 0:
            msg.loop_closure_transform = Transform()
            if success:
                # Success: positive covariance
                msg.loop_closure_transform.covariance = [1.0] * 36
            else:
                # Failure: zero covariance
                msg.loop_closure_transform.covariance = [0.0] * 36
        
        self.info_pub.publish(msg)
        time.sleep(0.1)  # Allow time for processing
    
    def test_successful_loop_closures(self):
        """Test that successful loop closures are tracked correctly"""
        # Clear any previous diagnostics
        self.diagnostics_received.clear()
        
        # Publish several successful loop closures
        for i in range(5):
            self.publish_info_message(
                loop_closure_id=i+1,
                ref_id=max(1, i-2),
                success=True
            )
        
        # Wait for diagnostics
        time.sleep(2.0)
        
        # Should have received diagnostics
        self.assertGreater(len(self.diagnostics_received), 0,
                          "Should receive diagnostic messages")
        
        # Check latest diagnostic
        latest_diag = self.diagnostics_received[-1]
        slam_status = None
        for status in latest_diag.status:
            if 'SLAM' in status.name or 'Map Quality' in status.name:
                slam_status = status
                break
        
        self.assertIsNotNone(slam_status, "Should have SLAM status in diagnostics")
        
        # With all successful loop closures, status should be OK
        # Note: May be WARN if insufficient data, but should not be ERROR
        self.assertNotEqual(slam_status.level, 2,  # ERROR level
                           "Status should not be ERROR with successful loop closures")
    
    def test_failed_loop_closures(self):
        """Test that failed loop closures trigger warnings"""
        # Clear any previous diagnostics
        self.diagnostics_received.clear()
        
        # Publish several failed loop closures
        for i in range(10):
            self.publish_info_message(
                loop_closure_id=i+1,
                ref_id=max(1, i-2),
                success=False
            )
        
        # Wait for diagnostics
        time.sleep(2.0)
        
        # Should have received diagnostics
        self.assertGreater(len(self.diagnostics_received), 0,
                          "Should receive diagnostic messages")
        
        # Check latest diagnostic
        latest_diag = self.diagnostics_received[-1]
        slam_status = None
        for status in latest_diag.status:
            if 'SLAM' in status.name or 'Map Quality' in status.name:
                slam_status = status
                break
        
        self.assertIsNotNone(slam_status, "Should have SLAM status in diagnostics")
        
        # With all failed loop closures, status should be WARN or ERROR
        self.assertGreaterEqual(slam_status.level, 1,  # WARN or ERROR
                               "Status should be WARN or ERROR with failed loop closures")
    
    def test_mixed_loop_closures(self):
        """Test with mixed success/failure rates"""
        # Clear any previous diagnostics
        self.diagnostics_received.clear()
        
        # Publish mixed loop closures (60% success)
        for i in range(10):
            success = (i % 5) < 3  # 3 out of 5 succeed
            self.publish_info_message(
                loop_closure_id=i+1,
                ref_id=max(1, i-2),
                success=success
            )
        
        # Wait for diagnostics
        time.sleep(2.0)
        
        # Should have received diagnostics
        self.assertGreater(len(self.diagnostics_received), 0,
                          "Should receive diagnostic messages")
        
        # Verify diagnostics contain expected keys
        latest_diag = self.diagnostics_received[-1]
        slam_status = None
        for status in latest_diag.status:
            if 'SLAM' in status.name or 'Map Quality' in status.name:
                slam_status = status
                break
        
        self.assertIsNotNone(slam_status, "Should have SLAM status in diagnostics")
        
        # Check that key metrics are present
        keys = [kv.key for kv in slam_status.values]
        self.assertIn('Total Loop Closures', keys,
                     "Should report total loop closures")
        self.assertIn('Successful Loop Closures', keys,
                     "Should report successful loop closures")


def main():
    """Run tests"""
    unittest.main()


if __name__ == '__main__':
    main()
