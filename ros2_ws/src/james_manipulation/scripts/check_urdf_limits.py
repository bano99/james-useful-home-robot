#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
import xml.etree.ElementTree as ET
import math

def main():
    rclpy.init()
    node = Node('urdf_limit_checker')
    
    # Create client to getting robot_description from move_group or robot_state_publisher
    # Usually robot_state_publisher holds the truth.
    client = node.create_client(GetParameters, '/robot_state_publisher/get_parameters')
    print("Waiting for /robot_state_publisher/get_parameters...", flush=True)
    if not client.wait_for_service(timeout_sec=5.0):
        print("Service not available. Trying /move_group...", flush=True)
        client = node.create_client(GetParameters, '/move_group/get_parameters')
        if not client.wait_for_service(timeout_sec=5.0):
            print("Failed to find parameter service.", flush=True)
            return

    req = GetParameters.Request()
    req.names = ['robot_description']
    
    future = client.call_async(req)
    rclpy.spin_until_future_complete(node, future)
    
    result = future.result()
    if not result or not result.values:
        print("Failed to retrieve robot_description.", flush=True)
        return
        
    urdf_string = result.values[0].string_value
    print(f"Got URDF ({len(urdf_string)} bytes). Parsing...", flush=True)
    
    root = ET.fromstring(urdf_string)
    
    for joint in root.findall('joint'):
        name = joint.get('name')
        if 'arm_joint' in name:
            limit = joint.find('limit')
            if limit is not None:
                lower = float(limit.get('lower', 0.0))
                upper = float(limit.get('upper', 0.0))
                print(f"Joint: {name}")
                print(f"  Result in URDF (Rad): [{lower:.4f}, {upper:.4f}]")
                print(f"  Result in URDF (Deg): [{math.degrees(lower):.1f}, {math.degrees(upper):.1f}]")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
