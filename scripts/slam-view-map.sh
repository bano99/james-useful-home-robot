#!/bin/bash
# View saved RTAB-Map database

if [ -z "$1" ]; then
    # No argument, use default
    MAP_FILE=~/.ros/rtabmap.db
else
    # Use provided map file
    MAP_FILE="$1"
fi

if [ -f "$MAP_FILE" ]; then
    echo "Opening map: $MAP_FILE"
    rtabmap-databaseViewer "$MAP_FILE"
else
    echo "✗ Map file not found: $MAP_FILE"
    echo ""
    echo "Available maps:"
    ls -lh ~/james_maps/*.db 2>/dev/null || echo "  No maps in ~/james_maps/"
    exit 1
fi
