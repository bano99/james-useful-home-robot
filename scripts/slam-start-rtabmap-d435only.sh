#!/bin/bash
# Start RTAB-Map with D435 visual odometry only
# Usage: ./slam-start-rtabmap-d435only.sh [images]
#   images - Enable image storage (uses more memory)

cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash

if [ "$1" = "images" ]; then
    echo "Starting RTAB-Map SLAM (D435 visual odometry) with IMAGE STORAGE..."
    echo "⚠️  Warning: This uses more memory!"
    ros2 launch james_slam rtabmap_d435_only.launch.py \
        Mem/ImageKept:=true \
        Mem/BinDataKept:=true
else
    echo "Starting RTAB-Map SLAM (D435 visual odometry)..."
    echo "ℹ️  Tip: Add 'images' parameter to save images for better visualization"
    ros2 launch james_slam rtabmap_d435_only.launch.py
fi
