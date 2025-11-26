#!/bin/bash
# Start RTAB-Map with D435 + T265 (higher accuracy)
# Usage: ./slam-start-rtabmap-dual.sh [images]
#   images - Enable image storage (uses more memory)

cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash

if [ "$1" = "images" ]; then
    echo "Starting RTAB-Map SLAM (D435 + T265 odometry) with IMAGE STORAGE..."
    echo "⚠️  Warning: This uses more memory!"
    ros2 launch james_slam rtabmap.launch.py \
        Mem/ImageKept:=true \
        Mem/BinDataKept:=true
else
    echo "Starting RTAB-Map SLAM (D435 + T265 odometry)..."
    echo "ℹ️  Tip: Add 'images' parameter to save images for better visualization"
    ros2 launch james_slam rtabmap.launch.py
fi
