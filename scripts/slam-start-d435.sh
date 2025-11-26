#!/bin/bash
# Start D435-only SLAM (lower power, visual odometry)

cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash

echo "Starting D435 camera..."
ros2 launch realsense2_camera rs_launch.py \
  device_type:=d435 \
  camera_name:=d435 \
  enable_depth:=true \
  enable_color:=true \
  align_depth.enable:=true \
  depth_module.profile:=640x480x30 \
  rgb_camera.profile:=640x480x30
