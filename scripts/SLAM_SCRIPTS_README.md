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
~/james-useful-home-robot/scripts/slam-start-rtabmap-d435only.sh
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
~/james-useful-home-robot/scripts/slam-start-rtabmap-dual.sh
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

## Tips

- Use `screen` or `tmux` for persistent sessions if PuTTY disconnects
- Close RViz2 during mapping to save resources
- Backup maps after each successful session
- Drive slowly (< 0.3 m/s) for best results

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
