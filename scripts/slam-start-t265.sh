#!/bin/bash
# Start T265 tracking camera for odometry

cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash

echo "Starting T265 tracking camera..."
ros2 launch realsense2_camera rs_launch.py \
  device_type:=t265 \
  camera_name:=t265 \
  enable_pose:=true
