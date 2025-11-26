#!/bin/bash
# Start RTAB-Map with D435 visual odometry only

cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash

echo "Starting RTAB-Map SLAM (D435 visual odometry)..."
ros2 launch james_slam rtabmap_d435_only.launch.py
