# SLAM Quick Start Scripts

Convenient shell scripts for running SLAM on the Jetson Nano.

## Installation

```bash
cd ~/james-useful-home-robot
chmod +x scripts/slam-*.sh
```

## D435-Only Mode (Recommended - Lower Power)

**Terminal 1 - D435 Camera:**
```bash
~/james-useful-home-robot/scripts/slam-start-d435.sh
```

**Terminal 2 - RTAB-Map:**
```bash
# Without images (lower memory, faster)
~/james-useful-home-robot/scripts/slam-start-rtabmap-d435only.sh

# With images (for better visualization in database viewer)
~/james-useful-home-robot/scripts/slam-start-rtabmap-d435only.sh images
```

**Terminal 3 - RViz2 (optional):**
```bash
~/james-useful-home-robot/scripts/slam-start-rviz.sh
```

## D435 + T265 Mode (Higher Accuracy)

**Terminal 1 - T265:**
```bash
~/james-useful-home-robot/scripts/slam-start-t265.sh
```

**Terminal 2 - D435:**
```bash
~/james-useful-home-robot/scripts/slam-start-d435.sh
```

**Terminal 3 - RTAB-Map:**
```bash
# Without images (lower memory, faster)
~/james-useful-home-robot/scripts/slam-start-rtabmap-dual.sh

# With images (for better visualization in database viewer)
~/james-useful-home-robot/scripts/slam-start-rtabmap-dual.sh images
```

**Terminal 4 - RViz2 (optional):**
```bash
~/james-useful-home-robot/scripts/slam-start-rviz.sh
```

## Map Management

**Backup current map:**
```bash
~/james-useful-home-robot/scripts/slam-backup-map.sh
```

**View saved map:**
```bash
# View current map
~/james-useful-home-robot/scripts/slam-view-map.sh

# View specific backup
~/james-useful-home-robot/scripts/slam-view-map.sh ~/james_maps/home_map_20241124_1430.db
```

## Image Storage

**Without `images` parameter (default):**
- Lower memory usage
- Faster processing
- Map suitable for navigation
- Database viewer shows sparse points only

**With `images` parameter:**
- Higher memory usage (watch for OOM on Jetson Nano)
- Stores RGB-D images in database
- Database viewer shows full 3D point clouds
- Can export images and dense maps
- Better for visualization and analysis

**When to use images:**
- Creating maps for presentation/visualization
- Need to export point clouds
- Analyzing map quality in detail
- Have adequate memory available

## Tips

- Use `screen` or `tmux` for persistent sessions if PuTTY disconnects
- Close RViz2 during mapping to save resources
- Backup maps after each successful session
- Drive slowly (< 0.3 m/s) for best results
- Start without `images`, add it only if needed

## Troubleshooting

**Scripts not executable:**
```bash
chmod +x ~/james-useful-home-robot/scripts/slam-*.sh
```

**PuTTY keeps disconnecting:**
```bash
# Use screen for persistent sessions
screen -S slam
# Run your script
# Detach: Ctrl+A, then D
# Reattach: screen -r slam
```
