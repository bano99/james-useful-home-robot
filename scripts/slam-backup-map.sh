#!/bin/bash
# Backup current RTAB-Map database with timestamp

BACKUP_DIR=~/james_maps
MAP_NAME="home_map_$(date +%Y%m%d_%H%M)"

mkdir -p $BACKUP_DIR

if [ -f ~/.ros/rtabmap.db ]; then
    cp ~/.ros/rtabmap.db $BACKUP_DIR/$MAP_NAME.db
    echo "✓ Map backed up to: $BACKUP_DIR/$MAP_NAME.db"
    ls -lh $BACKUP_DIR/$MAP_NAME.db
else
    echo "✗ No map found at ~/.ros/rtabmap.db"
    exit 1
fi
