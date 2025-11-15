# James SLAM Package

RTAB-Map SLAM configuration for James home robot using RealSense D435 (RGB-D) and T265 (odometry).

## Quick Start

```bash
# Terminal 1 - T265
ros2 launch realsense2_camera rs_launch.py device_type:=t265 camera_name:=t265 enable_pose:=true

# Terminal 2 - D435
ros2 launch realsense2_camera rs_launch.py device_type:=d435 camera_name:=d435 \
  enable_depth:=true enable_color:=true align_depth.enable:=true \
  depth_module.profile:=640x480x30 rgb_camera.profile:=640x480x30

# Terminal 3 - RTAB-Map
cd ~/james-useful-home-robot/ros2_ws && source install/setup.bash
ros2 launch james_slam rtabmap.launch.py

# Terminal 4 - Visualization (optional)
rviz2
```

## Features

- **Optimized for Jetson Nano**: Reduced memory footprint and computational requirements
- **Sensor Fusion**: Combines D435 visual data with T265 odometry
- **Indoor Mapping**: Configured for home environments with 5cm grid resolution
- **Traversable Obstacles**: Ignores obstacles < 10cm height (carpets, cables, yoga mats)
- **Loop Closure**: Automatic loop closure detection for consistent maps
- **TF Bridge**: Custom node handles frame transforms between cameras

## Prerequisites

```bash
# Install RTAB-Map
sudo apt install ros-foxy-rtabmap-ros

# Install RViz2 for visualization
sudo apt install ros-foxy-rviz2
```

## Building

```bash
cd ~/james-useful-home-robot/ros2_ws
colcon build --packages-select james_slam
source install/setup.bash
```

## Usage

### 1. Start Cameras

**Terminal 1 - T265 (Tracking Camera):**
```bash
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
    device_type:=t265 \
    camera_name:=t265 \
    enable_pose:=true
```

**Terminal 2 - D435 (Depth Camera):**
```bash
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash
ros2 launch realsense2_camera rs_launch.py \
    device_type:=d435 \
    camera_name:=d435 \
    enable_depth:=true \
    enable_color:=true \
    align_depth.enable:=true \
    depth_module.profile:=640x480x30 \
    rgb_camera.profile:=640x480x30
```

**Note**: Using separate `camera_name` parameters prevents topic conflicts between cameras.

### 2. Start RTAB-Map SLAM

**Terminal 3:**
```bash
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash
ros2 launch james_slam rtabmap.launch.py
```

You should see RTAB-Map start and begin processing camera data. The TF bridge node will automatically connect the camera frames.

### 3. Drive Robot to Build Map

Use your remote control to drive James around the environment. RTAB-Map will build a 3D map automatically.

### 4. Visualize Map (Optional)

**Terminal 4 - RViz2:**
```bash
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash
rviz2
```

**Configure RViz2:**
1. **Set Fixed Frame**: Change "Fixed Frame" to `map` in Global Options
2. **Add 2D Map**: 
   - Click "Add" → "By topic" → `/map` → "Map"
3. **Add 3D Point Cloud** (optional):
   - Click "Add" → "By topic" → `/cloud_map` → "PointCloud2"
4. **Add Camera View** (optional):
   - Click "Add" → "By topic" → `/d435/color/image_raw` → "Image"
5. **Add TF**:
   - Click "Add" → "TF"

You'll see the map being built in real-time as James moves around!

## Viewing Saved Maps (Offline)

The Jetson Nano can struggle running cameras + RTAB-Map + RViz2 simultaneously. Here are two ways to view your map after a mapping session:

### Option 1: RTAB-Map Database Viewer (Recommended)

Stop all nodes and use the standalone viewer:

```bash
# View the saved map
rtabmap-databaseViewer ~/.ros/rtabmap.db
```

This lightweight GUI lets you:
- Explore the 3D point cloud
- View pose graph and loop closures
- Export maps (PLY, PCD, OctoMap)
- Analyze mapping quality

### Option 2: View in RViz2 (Lighter Load)

```bash
# Launch map viewer (no cameras needed)
ros2 launch james_slam view_map.launch.py

# In another terminal, launch RViz2
rviz2
```

Configure RViz2 as described in section 4 above.

## Localization Mode

Once you have a good map, you can switch to localization mode (no new mapping):

```bash
ros2 launch james_slam rtabmap.launch.py localization:=true
```

## Saving and Loading Maps

Maps are automatically saved to `~/.ros/rtabmap.db`

### Backup Your Map
```bash
# Create maps directory
mkdir -p ~/james_maps

# Backup current map with timestamp
cp ~/.ros/rtabmap.db ~/james_maps/home_map_$(date +%Y%m%d_%H%M).db
```

### Start Fresh Mapping
```bash
# Delete existing map to start over
rm ~/.ros/rtabmap.db
```

### Load a Specific Map
```bash
# Copy a backup to active location
cp ~/james_maps/home_map_20241115_1030.db ~/.ros/rtabmap.db

# Or specify path in launch
ros2 launch james_slam rtabmap.launch.py database_path:=~/james_maps/my_map.db
```

### Export Map for Other Tools
```bash
# Open database viewer
rtabmap-databaseViewer ~/.ros/rtabmap.db

# Then: File → Export → Choose format:
# - Point Cloud (PLY, PCD) - for CloudCompare, MeshLab
# - OctoMap (.ot) - for motion planning
# - 2D Grid Map (PGM) - for Nav2
```

## Troubleshooting

### No map appearing
- **Check cameras are publishing**:
  ```bash
  ros2 topic hz /d435/color/image_raw
  ros2 topic hz /t265/pose/sample
  ```
- **Verify RTAB-Map is running**:
  ```bash
  ros2 topic list | grep map
  ros2 topic echo /info
  ```
- **Check TF bridge is running**:
  ```bash
  ros2 node list | grep tf_bridge
  ```

### Camera resolution mismatch error
If you see `imageWidth/depthWidth == imageHeight/depthHeight not met`, ensure both cameras use matching resolutions:
```bash
# Both should be 640x480
depth_module.profile:=640x480x30
rgb_camera.profile:=640x480x30
```

### TF frame errors
The TF bridge node automatically publishes transforms between `odom_frame` and `d435_color_optical_frame`. If you see TF errors, check:
```bash
ros2 run tf2_ros tf2_echo odom_frame d435_color_optical_frame
```

### Poor map quality
- Drive slower (< 0.5 m/s) to give RTAB-Map time to process
- Ensure good lighting conditions
- Point camera at textured surfaces (not blank walls)
- Check for loop closures: `ros2 topic echo /info | grep "Loop closure"`
- Move in loops to enable loop closure detection

### Out of memory on Jetson Nano
- Reduce `Kp/MaxFeatures` in config (currently 200)
- Reduce `Mem/STMSize` (currently 10)
- Close other applications
- Enable swap space:
  ```bash
  sudo fallocate -l 4G /swapfile
  sudo chmod 600 /swapfile
  sudo mkswap /swapfile
  sudo swapon /swapfile
  ```

## Configuration

Edit `config/rtabmap_params.yaml` to adjust RTAB-Map parameters.

Key parameters:
- `Grid/CellSize`: Map resolution (default 5cm)
- `Grid/RangeMax`: Maximum sensor range (default 5m)
- `Kp/MaxFeatures`: Features per frame (default 200)
- `Mem/STMSize`: Short-term memory size (default 10)

## Topics

### Subscribed
- `/d435/color/image_raw` - RGB image from D435
- `/d435/color/camera_info` - Camera calibration
- `/d435/aligned_depth_to_color/image_raw` - Depth image from D435
- `/t265/pose/sample` - Odometry from T265

### Published
- `/map` - 2D occupancy grid for navigation
- `/mapData` - RTAB-Map internal map data
- `/mapGraph` - Pose graph for loop closures
- `/cloud_map` - 3D point cloud map
- `/cloud_ground` - Ground plane point cloud
- `/cloud_obstacles` - Obstacle point cloud
- `/grid_prob_map` - Probabilistic occupancy grid
- `/info` - RTAB-Map statistics (nodes, loop closures, etc.)
- `/global_pose` - Robot pose in map frame
- `/tf` and `/tf_static` - Transform tree

## Next Steps

After successful mapping:
1. Integrate with Nav2 for autonomous navigation (Task 8)
2. Use map for object detection and localization (Task 9)
3. Enable manipulation planning with map as collision geometry (Task 12)
