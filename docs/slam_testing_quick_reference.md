# SLAM Testing Quick Reference

Quick commands for testing SLAM functionality. See `slam_testing_guide.md` for detailed procedures.

## Prerequisites Checklist

```bash
# Check ROS2 is sourced
ros2 topic list

# If not, source workspace
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash

# Make scripts executable (first time only)
chmod +x ~/james-useful-home-robot/scripts/*.sh
```

## Quick Start - Interactive Helper

```bash
cd ~/james-useful-home-robot
./scripts/slam-test-helper.sh
```

This provides an interactive menu for all testing operations.

## Manual Commands

### Phase 1: Start SLAM System

**Terminal 1 - D435 Camera:**
```bash
cd ~/james-useful-home-robot
./scripts/slam-start-d435.sh
```

**Terminal 2 - RTAB-Map:**
```bash
cd ~/james-useful-home-robot
./scripts/slam-start-rtabmap-d435only.sh
```

**Terminal 3 - Monitor (optional):**
```bash
# Watch loop closures
ros2 topic echo /info | grep "Loop closure"

# Or check map update rate
ros2 topic hz /map
```

### Phase 2: Verify System Running

```bash
# Check all topics
ros2 topic list

# Check camera is publishing
ros2 topic hz /d435/color/image_raw

# Check RTAB-Map is publishing
ros2 topic hz /map

# Get RTAB-Map statistics
ros2 topic echo /info --once
```

### Phase 3: Drive and Map

**Drive robot manually using remote control:**
- Speed: < 0.3 m/s (walking pace)
- Look at textured surfaces (walls with pictures, furniture)
- Complete loops (return to start multiple times)
- Pause at doorways and room centers

**Monitor progress:**
```bash
# Watch for loop closures
ros2 topic echo /info | grep -i "loop"

# Check number of nodes
ros2 topic echo /info | grep -i "nodes"
```

### Phase 4: Save Map

```bash
# Backup with timestamp
cd ~/james-useful-home-robot
./scripts/slam-backup-map.sh

# Or manual backup
mkdir -p ~/james_maps
cp ~/.ros/rtabmap.db ~/james_maps/home_map_$(date +%Y%m%d_%H%M).db
```

### Phase 5: View Map

```bash
# Stop all ROS2 nodes first (Ctrl+C in each terminal)

# View current map
./scripts/slam-view-map.sh

# View specific backup
./scripts/slam-view-map.sh ~/james_maps/home_map_20241124_1430.db
```

### Phase 6: Test Loop Closure

**Restart SLAM with existing map:**
```bash
# Terminal 1 - D435
./scripts/slam-start-d435.sh

# Terminal 2 - RTAB-Map (loads existing map)
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash
ros2 launch james_slam rtabmap_d435_only.launch.py

# Terminal 3 - Monitor loop closures
ros2 topic echo /info | grep "Loop closure"
```

**Drive a loop:**
- Start at recognizable location
- Drive to another room
- Return to start via different path
- Watch for loop closure detection

### Phase 7: Test Relocalization

**Start in localization mode:**
```bash
# Terminal 1 - D435
./scripts/slam-start-d435.sh

# Terminal 2 - RTAB-Map (localization only)
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash
ros2 launch james_slam rtabmap_d435_only.launch.py localization:=true

# Terminal 3 - Monitor localization
ros2 topic echo /info | grep -i "hypothesis"
```

**Test relocalization:**
- Move robot to different location (manually)
- Drive slowly and rotate 360°
- Should localize within 30 seconds

## Common Checks

### Is camera working?
```bash
ros2 topic list | grep d435
ros2 topic hz /d435/color/image_raw
```

### Is RTAB-Map running?
```bash
ros2 node list | grep rtabmap
ros2 topic hz /map
```

### How many loop closures?
```bash
ros2 topic echo /info --once | grep -i "loop"
```

### How many nodes in map?
```bash
ros2 topic echo /info --once | grep -i "nodes"
```

### What's the map file size?
```bash
ls -lh ~/.ros/rtabmap.db
```

### List all map backups?
```bash
ls -lh ~/james_maps/
```

## Troubleshooting Quick Fixes

### Camera not detected
```bash
# Check USB connection
lsusb | grep Intel

# Restart camera
# Ctrl+C in camera terminal, then restart
./scripts/slam-start-d435.sh
```

### RTAB-Map not publishing
```bash
# Check if node is running
ros2 node list

# Check for errors
ros2 node info /rtabmap

# Restart RTAB-Map
# Ctrl+C in RTAB-Map terminal, then restart
./scripts/slam-start-rtabmap-d435only.sh
```

### Tracking lost frequently
```bash
# Drive slower
# Improve lighting
# Point camera at textured surfaces
# Check camera is stable (no vibration)
```

### No loop closures
```bash
# Drive more overlapping paths
# Return to start multiple times
# Ensure good visual features in environment
```

### Out of memory
```bash
# Close RViz and other applications
# Reduce features in config
# Enable swap space:
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

## Map Management Commands

```bash
# Backup current map
./scripts/slam-backup-map.sh

# List backups
ls -lh ~/james_maps/

# Load specific backup
cp ~/james_maps/home_map_20241124_1430.db ~/.ros/rtabmap.db

# Delete current map (start fresh)
rm ~/.ros/rtabmap.db

# View map
./scripts/slam-view-map.sh

# Export map (use database viewer)
rtabmap-databaseViewer ~/.ros/rtabmap.db
# Then: File → Export → Choose format
```

## Success Criteria

**Map Building:**
- ✅ Map covers all target areas
- ✅ Walls are straight and continuous
- ✅ Room dimensions accurate within 5%
- ✅ No duplicate geometry

**Loop Closure:**
- ✅ At least 5 loop closures detected
- ✅ Loop closures at expected locations
- ✅ Map quality improves after loop closure
- ✅ No false loop closures

**Relocalization:**
- ✅ Localizes within 60 seconds
- ✅ Position accuracy within 20 cm
- ✅ Works from multiple starting locations
- ✅ Success rate > 66%

**Map Persistence:**
- ✅ Map saves automatically
- ✅ Map loads correctly
- ✅ Backups work properly
- ✅ Map statistics are reasonable

## Next Steps

After successful SLAM testing:

1. ✅ Save final map: `./scripts/slam-backup-map.sh`
2. ✅ Document any issues in test report
3. ✅ Proceed to Task 8: Navigation System
4. ✅ Use saved map for navigation testing

## References

- **Full Testing Guide:** `docs/slam_testing_guide.md`
- **SLAM Package README:** `ros2_ws/src/james_slam/README.md`
- **SLAM Scripts README:** `scripts/SLAM_SCRIPTS_README.md`
- **Requirements:** `.kiro/specs/james-home-robot/requirements.md` (2.1, 2.3, 2.4)
- **Design:** `.kiro/specs/james-home-robot/design.md` (SLAM section)
