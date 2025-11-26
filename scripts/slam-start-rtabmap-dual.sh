#!/bin/bash
# Start RTAB-Map with D435 + T265 (higher accuracy)

cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash

echo "Starting RTAB-Map SLAM (D435 + T265 odometry)..."
ros2 launch james_slam rtabmap.launch.py
