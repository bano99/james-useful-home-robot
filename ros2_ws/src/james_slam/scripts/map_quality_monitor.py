#!/usr/bin/env python3
"""
Map Quality Monitor Node for James Robot

This node monitors the quality of the RTAB-Map SLAM system by:
- Tracking loop closure success rate
- Detecting map degradation
- Triggering map optimization when needed

Validates Requirements 2.4: THE Main Brain SHALL update the SLAM map 
continuously during operation to reflect environmental changes.
"""

import rclpy
from rclpy.node import Node
from rtabmap_msgs.msg import Info, MapData
from std_srvs.srv import Empty
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from collections import deque
import time


class MapQualityMonitor(Node):
    """
    Monitors RTAB-Map quality metrics and triggers optimization when needed.
    """
    
    # Thresholds for map quality assessment
    LOOP_CLOSURE_SUCCESS_THRESHOLD = 0.5  # 50% success rate minimum
    LOOP_CLOSURE_WINDOW_SIZE = 20  # Track last 20 loop closure attempts
    MAP_OPTIMIZATION_INTERVAL = 300.0  # 5 minutes between optimizations
    DEGRADATION_CHECK_INTERVAL = 60.0  # Check every minute
    
    def __init__(self):
        super().__init__('map_quality_monitor')
        
        # Declare parameters
        self.declare_parameter('loop_closure_threshold', self.LOOP_CLOSURE_SUCCESS_THRESHOLD)
        self.declare_parameter('window_size', self.LOOP_CLOSURE_WINDOW_SIZE)
        self.declare_parameter('optimization_interval', self.MAP_OPTIMIZATION_INTERVAL)
        self.declare_parameter('degradation_check_interval', self.DEGRADATION_CHECK_INTERVAL)
        
        # Get parameters
        self.loop_closure_threshold = self.get_parameter('loop_closure_threshold').value
        self.window_size = self.get_parameter('window_size').value
        self.optimization_interval = self.get_parameter('optimization_interval').value
        self.degradation_check_interval = self.get_parameter('degradation_check_interval').value
        
        # State tracking
        self.loop_closure_attempts = deque(maxlen=self.window_size)
        self.last_optimization_time = time.time()
        self.total_loop_closures = 0
        self.successful_loop_closures = 0
        self.last_info_time = None
        self.map_nodes_count = 0
        self.last_map_nodes_count = 0
        
        # Subscribe to RTAB-Map info topic
        self.info_sub = self.create_subscription(
            Info,
            '/rtabmap/info',
            self.info_callback,
            10
        )
        
        # Subscribe to RTAB-Map map data for additional metrics
        self.map_sub = self.create_subscription(
            MapData,
            '/rtabmap/mapData',
            self.map_callback,
            10
        )
        
        # Publisher for diagnostics
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray,
            '/diagnostics',
            10
        )
        
        # Service client for triggering map optimization
        self.optimize_client = self.create_client(
            Empty,
            '/rtabmap/trigger_new_map'
        )
        
        # Timer for periodic degradation checks
        self.degradation_timer = self.create_timer(
            self.degradation_check_interval,
            self.check_map_degradation
        )
        
        self.get_logger().info('Map Quality Monitor initialized')
        self.get_logger().info(f'Loop closure threshold: {self.loop_closure_threshold}')
        self.get_logger().info(f'Window size: {self.window_size}')
        self.get_logger().info(f'Optimization interval: {self.optimization_interval}s')
    
    def info_callback(self, msg):
        """
        Process RTAB-Map info messages to track loop closures.
        
        The Info message contains statistics about the SLAM system including
        loop closure attempts and successes.
        """
        self.last_info_time = time.time()
        
        # Extract loop closure information
        # loopClosureId > 0 indicates a loop closure was detected
        if msg.loop_closure_id > 0:
            self.total_loop_closures += 1
            
            # Check if loop closure was accepted (not rejected)
            # A rejected loop closure has a transform covariance that's too high
            loop_closure_accepted = msg.loop_closure_transform.covariance[0] > 0
            
            if loop_closure_accepted:
                self.successful_loop_closures += 1
                self.loop_closure_attempts.append(1)  # Success
                self.get_logger().info(
                    f'Loop closure accepted: {msg.loop_closure_id} -> {msg.ref_id}'
                )
            else:
                self.loop_closure_attempts.append(0)  # Failure
                self.get_logger().warn(
                    f'Loop closure rejected: {msg.loop_closure_id} -> {msg.ref_id}'
                )
        
        # Update map node count
        self.map_nodes_count = msg.nodes
        
        # Publish diagnostics
        self.publish_diagnostics()
        
        # Check if optimization is needed
        self.check_optimization_needed()
    
    def map_callback(self, msg):
        """
        Process RTAB-Map map data messages for additional quality metrics.
        """
        # Track map size changes
        current_nodes = len(msg.nodes)
        if current_nodes != self.last_map_nodes_count:
            self.get_logger().debug(f'Map nodes: {current_nodes}')
            self.last_map_nodes_count = current_nodes
    
    def get_loop_closure_success_rate(self):
        """
        Calculate the loop closure success rate over the recent window.
        
        Returns:
            float: Success rate between 0.0 and 1.0, or None if insufficient data
        """
        if len(self.loop_closure_attempts) == 0:
            return None
        
        successes = sum(self.loop_closure_attempts)
        total = len(self.loop_closure_attempts)
        return successes / total
    
    def get_overall_loop_closure_rate(self):
        """
        Calculate the overall loop closure success rate since node start.
        
        Returns:
            float: Success rate between 0.0 and 1.0, or None if no attempts
        """
        if self.total_loop_closures == 0:
            return None
        
        return self.successful_loop_closures / self.total_loop_closures
    
    def check_map_degradation(self):
        """
        Periodic check for map quality degradation.
        
        Degradation indicators:
        - Low loop closure success rate
        - No new loop closures in extended period
        - Stale info messages (SLAM may have crashed)
        """
        current_time = time.time()
        
        # Check if we're receiving info messages
        if self.last_info_time is None:
            self.get_logger().warn('No RTAB-Map info messages received yet')
            return
        
        time_since_last_info = current_time - self.last_info_time
        if time_since_last_info > 30.0:  # No info for 30 seconds
            self.get_logger().error(
                f'RTAB-Map info messages stale ({time_since_last_info:.1f}s). '
                'SLAM may have stopped!'
            )
            return
        
        # Check loop closure success rate
        recent_rate = self.get_loop_closure_success_rate()
        if recent_rate is not None and recent_rate < self.loop_closure_threshold:
            self.get_logger().warn(
                f'Map degradation detected: Loop closure success rate '
                f'({recent_rate:.2%}) below threshold ({self.loop_closure_threshold:.2%})'
            )
            
            # Consider triggering optimization if rate is very low
            if recent_rate < 0.3:  # Less than 30% success
                self.get_logger().warn('Severe map degradation - considering optimization')
                self.trigger_map_optimization()
    
    def check_optimization_needed(self):
        """
        Check if map optimization should be triggered based on time interval.
        """
        current_time = time.time()
        time_since_optimization = current_time - self.last_optimization_time
        
        if time_since_optimization >= self.optimization_interval:
            self.get_logger().info(
                f'Periodic map optimization due ({time_since_optimization:.1f}s elapsed)'
            )
            self.trigger_map_optimization()
    
    def trigger_map_optimization(self):
        """
        Trigger RTAB-Map graph optimization.
        
        Note: RTAB-Map performs optimization automatically, but we can trigger
        additional optimization by calling the trigger_new_map service or by
        publishing to /rtabmap/trigger_new_map topic.
        """
        if not self.optimize_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('Map optimization service not available')
            return
        
        self.get_logger().info('Triggering map optimization...')
        
        request = Empty.Request()
        future = self.optimize_client.call_async(request)
        future.add_done_callback(self.optimization_callback)
        
        self.last_optimization_time = time.time()
    
    def optimization_callback(self, future):
        """
        Handle the response from map optimization service.
        """
        try:
            future.result()
            self.get_logger().info('Map optimization completed successfully')
        except Exception as e:
            self.get_logger().error(f'Map optimization failed: {str(e)}')
    
    def publish_diagnostics(self):
        """
        Publish diagnostic information about map quality.
        """
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        status = DiagnosticStatus()
        status.name = 'SLAM Map Quality'
        status.hardware_id = 'rtabmap'
        
        # Determine overall status
        recent_rate = self.get_loop_closure_success_rate()
        overall_rate = self.get_overall_loop_closure_rate()
        
        if recent_rate is None:
            status.level = DiagnosticStatus.OK
            status.message = 'Insufficient data for quality assessment'
        elif recent_rate < 0.3:
            status.level = DiagnosticStatus.ERROR
            status.message = f'Severe map degradation (success rate: {recent_rate:.2%})'
        elif recent_rate < self.loop_closure_threshold:
            status.level = DiagnosticStatus.WARN
            status.message = f'Map quality degraded (success rate: {recent_rate:.2%})'
        else:
            status.level = DiagnosticStatus.OK
            status.message = f'Map quality good (success rate: {recent_rate:.2%})'
        
        # Add key-value pairs with detailed metrics
        status.values.append(KeyValue(
            key='Total Loop Closures',
            value=str(self.total_loop_closures)
        ))
        status.values.append(KeyValue(
            key='Successful Loop Closures',
            value=str(self.successful_loop_closures)
        ))
        
        if recent_rate is not None:
            status.values.append(KeyValue(
                key='Recent Success Rate',
                value=f'{recent_rate:.2%}'
            ))
        
        if overall_rate is not None:
            status.values.append(KeyValue(
                key='Overall Success Rate',
                value=f'{overall_rate:.2%}'
            ))
        
        status.values.append(KeyValue(
            key='Map Nodes',
            value=str(self.map_nodes_count)
        ))
        
        time_since_optimization = time.time() - self.last_optimization_time
        status.values.append(KeyValue(
            key='Time Since Last Optimization',
            value=f'{time_since_optimization:.1f}s'
        ))
        
        diag_array.status.append(status)
        self.diagnostics_pub.publish(diag_array)


def main(args=None):
    rclpy.init(args=args)
    
    node = MapQualityMonitor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
