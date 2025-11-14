# James SLAM Package

RTAB-Map SLAM configuration for James home robot using RealSense D435 (RGB-D) and T265 (odometry).

## Features

- **Optimized for Jetson Nano**: Reduced memory footprint and computational requirements
- **Sensor Fusion**: Combines D435 visual data with T265 odometry
- **Indoor Mapping**: Configured for home environments with 5cm grid resolution
- **Traversable Obstacles**: Ignores obstacles < 10cm height (carpets, cables, yoga mats)
- **Loop Closure**: Automatic loop closure detection for consistent maps

## Prerequisites

```bash
# Install RTAB-Map
sudo apt install ros-foxy-rtabmap-ros

# Cameras should be running
ros2 launch realsense2_camera rs_launch.py device_type:=d435 enable_color:=true enable_depth:=true align_depth.enable:=true
ros2 launch realsense2_camera rs_launch.py device_type:=t265 enable_pose:=true
```

## Building

```bash
cd ~/james-useful-home-robot/ros2_ws
colcon build --packages-select james_slam
source install/setup.bash
```

## Usage

### 1. Start Cameras

Terminal 1 - D435:
```bash
ros2 launch realsense2_camera rs_launch.py \
    device_type:=d435 \
    rgb_camera.profile:=640,480,30 \
    depth_module.profile:=640,480,30 \
    enable_color:=true \
    enable_depth:=true \
    align_depth.enable:=true
```

Terminal 2 - T265:
```bash
ros2 launch realsense2_camera rs_launch.py \
    device_type:=t265 \
    enable_pose:=true \
    enable_fisheye1:=false \
    enable_fisheye2:=false
```

### 2. Start RTAB-Map

Terminal 3:
```bash
source ~/james-useful-home-robot/ros2_ws/install/setup.bash
ros2 launch james_slam rtabmap.launch.py
```

### 3. Drive Robot to Build Map

Use your remote control to drive James around the environment. RTAB-Map will build a 3D map automatically.

### 4. View Map

```bash
# In another terminal
ros2 run rviz2 rviz2
```

In RViz2:
1. Set Fixed Frame to `map`
2. Add → By topic → `/rtabmap/grid_map` → Map
3. Add → By topic → `/rtabmap/cloud_map` → PointCloud2
4. Add → TF to see coordinate frames

## Localization Mode

Once you have a good map, you can switch to localization mode (no new mapping):

```bash
ros2 launch james_slam rtabmap.launch.py localization:=true
```

## Saving and Loading Maps

Maps are automatically saved to `~/.ros/rtabmap.db`

To start fresh (delete existing map):
```bash
rm ~/.ros/rtabmap.db
```

To backup your map:
```bash
cp ~/.ros/rtabmap.db ~/james_maps/home_map_$(date +%Y%m%d).db
```

## Troubleshooting

### No map appearing
- Check that both cameras are publishing: `ros2 topic hz /camera/color/image_raw` and `ros2 topic hz /camera/pose/sample`
- Verify T265 odometry is working: `ros2 topic echo /camera/pose/sample`
- Check RTAB-Map is receiving data: `ros2 topic echo /rtabmap/info`

### Poor map quality
- Drive slower to give RTAB-Map time to process
- Ensure good lighting conditions
- Point camera at textured surfaces (not blank walls)
- Check for loop closures in RTAB-Map info

### Out of memory on Jetson Nano
- Reduce `Kp/MaxFeatures` in config (currently 200)
- Reduce `Mem/STMSize` (currently 10)
- Close other applications
- Consider using swap space

## Configuration

Edit `config/rtabmap_params.yaml` to adjust RTAB-Map parameters.

Key parameters:
- `Grid/CellSize`: Map resolution (default 5cm)
- `Grid/RangeMax`: Maximum sensor range (default 5m)
- `Kp/MaxFeatures`: Features per frame (default 200)
- `Mem/STMSize`: Short-term memory size (default 10)

## Topics

### Subscribed
- `/camera/color/image_raw` - RGB image from D435
- `/camera/color/camera_info` - Camera calibration
- `/camera/aligned_depth_to_color/image_raw` - Depth image from D435
- `/camera/pose/sample` - Odometry from T265

### Published
- `/rtabmap/grid_map` - 2D occupancy grid
- `/rtabmap/cloud_map` - 3D point cloud map
- `/rtabmap/info` - RTAB-Map statistics
- `/tf` - Map transforms

## Next Steps

After successful mapping:
1. Integrate with Nav2 for autonomous navigation (Task 8)
2. Use map for object detection and localization (Task 9)
3. Enable manipulation planning with map as collision geometry (Task 12)
