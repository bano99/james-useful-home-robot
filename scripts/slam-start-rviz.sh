#!/bin/bash
# Start RViz2 for live SLAM visualization

cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash

echo "Starting RViz2..."
echo "Configure: Fixed Frame = 'map', Add topic '/grid_prob_map'"
rviz2
