#!/bin/bash
# SLAM Testing Helper Script
# Provides interactive menu for SLAM testing workflow

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Directories
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PROJECT_ROOT="$(dirname "$SCRIPT_DIR")"
MAPS_DIR=~/james_maps
RTABMAP_DB=~/.ros/rtabmap.db

# Create maps directory if it doesn't exist
mkdir -p "$MAPS_DIR"

# Helper functions
print_header() {
    echo -e "${BLUE}========================================${NC}"
    echo -e "${BLUE}$1${NC}"
    echo -e "${BLUE}========================================${NC}"
}

print_success() {
    echo -e "${GREEN}✓ $1${NC}"
}

print_error() {
    echo -e "${RED}✗ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠ $1${NC}"
}

print_info() {
    echo -e "${BLUE}ℹ $1${NC}"
}

# Check if RTAB-Map database exists
check_map_exists() {
    if [ -f "$RTABMAP_DB" ]; then
        return 0
    else
        return 1
    fi
}

# Get map statistics
get_map_stats() {
    if check_map_exists; then
        local size=$(du -h "$RTABMAP_DB" | cut -f1)
        local modified=$(stat -c %y "$RTABMAP_DB" 2>/dev/null || stat -f "%Sm" "$RTABMAP_DB" 2>/dev/null)
        echo "Size: $size, Modified: $modified"
    else
        echo "No map found"
    fi
}

# List available map backups
list_map_backups() {
    print_header "Available Map Backups"
    if [ -d "$MAPS_DIR" ] && [ "$(ls -A $MAPS_DIR/*.db 2>/dev/null)" ]; then
        ls -lh "$MAPS_DIR"/*.db | awk '{print $9, "(" $5 ")"}'
    else
        print_warning "No map backups found in $MAPS_DIR"
    fi
    echo ""
}

# Backup current map
backup_map() {
    if check_map_exists; then
        local timestamp=$(date +%Y%m%d_%H%M%S)
        local backup_name="home_map_${timestamp}.db"
        local backup_path="$MAPS_DIR/$backup_name"
        
        cp "$RTABMAP_DB" "$backup_path"
        print_success "Map backed up to: $backup_path"
        
        local size=$(du -h "$backup_path" | cut -f1)
        print_info "Backup size: $size"
    else
        print_error "No map found at $RTABMAP_DB"
        return 1
    fi
}

# Delete current map
delete_current_map() {
    if check_map_exists; then
        echo -e "${YELLOW}This will delete the current map at $RTABMAP_DB${NC}"
        read -p "Are you sure? (yes/no): " confirm
        if [ "$confirm" = "yes" ]; then
            rm "$RTABMAP_DB"
            print_success "Current map deleted"
        else
            print_info "Deletion cancelled"
        fi
    else
        print_warning "No current map to delete"
    fi
}

# Load a map backup
load_map_backup() {
    list_map_backups
    echo ""
    read -p "Enter map filename (or full path): " map_file
    
    # Check if it's a full path or just filename
    if [ -f "$map_file" ]; then
        backup_path="$map_file"
    elif [ -f "$MAPS_DIR/$map_file" ]; then
        backup_path="$MAPS_DIR/$map_file"
    else
        print_error "Map file not found: $map_file"
        return 1
    fi
    
    # Backup current map if it exists
    if check_map_exists; then
        print_warning "Current map exists. Backing it up first..."
        backup_map
    fi
    
    # Load the backup
    cp "$backup_path" "$RTABMAP_DB"
    print_success "Map loaded from: $backup_path"
}

# View map in database viewer
view_map() {
    if ! command -v rtabmap-databaseViewer &> /dev/null; then
        print_error "rtabmap-databaseViewer not found. Install with:"
        echo "  sudo apt install ros-jazzy-rtabmap-ros"
        return 1
    fi
    
    if [ -n "$1" ]; then
        # View specific map
        if [ -f "$1" ]; then
            print_info "Opening map: $1"
            rtabmap-databaseViewer "$1"
        else
            print_error "Map file not found: $1"
            return 1
        fi
    else
        # View current map
        if check_map_exists; then
            print_info "Opening current map: $RTABMAP_DB"
            rtabmap-databaseViewer "$RTABMAP_DB"
        else
            print_error "No current map found"
            return 1
        fi
    fi
}

# Check ROS2 topics
check_ros_topics() {
    print_header "Checking ROS2 Topics"
    
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Source your workspace first:"
        echo "  cd ~/james-useful-home-robot/ros2_ws"
        echo "  source install/setup.bash"
        return 1
    fi
    
    # Check camera topics
    print_info "Checking D435 camera topics..."
    if ros2 topic list | grep -q "/d435/color/image_raw"; then
        print_success "D435 camera is publishing"
        ros2 topic hz /d435/color/image_raw --once 2>/dev/null || true
    else
        print_warning "D435 camera not detected"
    fi
    
    # Check RTAB-Map topics
    print_info "Checking RTAB-Map topics..."
    if ros2 topic list | grep -q "/map"; then
        print_success "RTAB-Map is running"
        ros2 topic hz /map --once 2>/dev/null || true
    else
        print_warning "RTAB-Map not detected"
    fi
    
    # Check odometry
    print_info "Checking odometry topics..."
    if ros2 topic list | grep -q "/odom"; then
        print_success "Odometry is publishing"
        ros2 topic hz /odom --once 2>/dev/null || true
    else
        print_warning "Odometry not detected"
    fi
    
    echo ""
}

# Monitor loop closures
monitor_loop_closures() {
    print_header "Monitoring Loop Closures"
    print_info "Press Ctrl+C to stop"
    echo ""
    
    if ! ros2 topic list | grep -q "/info"; then
        print_error "RTAB-Map not running. Start it first."
        return 1
    fi
    
    ros2 topic echo /info | grep --line-buffered "loop" || true
}

# Get RTAB-Map statistics
get_rtabmap_stats() {
    print_header "RTAB-Map Statistics"
    
    if ! ros2 topic list | grep -q "/info"; then
        print_error "RTAB-Map not running. Start it first."
        return 1
    fi
    
    print_info "Fetching statistics..."
    ros2 topic echo /info --once 2>/dev/null | grep -E "(nodes|loop_closures|features)" || true
    echo ""
}

# Export map to different formats
export_map() {
    print_header "Export Map"
    print_info "This will open the database viewer."
    print_info "Use File → Export to save in different formats:"
    echo "  - Point Cloud (PLY, PCD) - for CloudCompare, MeshLab"
    echo "  - OctoMap (.ot) - for motion planning"
    echo "  - 2D Grid Map (PGM) - for Nav2"
    echo ""
    
    view_map
}

# Run quick SLAM test
quick_test() {
    print_header "Quick SLAM Test"
    print_info "This will check if SLAM system is ready"
    echo ""
    
    # Check if ROS2 is sourced
    if ! command -v ros2 &> /dev/null; then
        print_error "ROS2 not found. Source your workspace first:"
        echo "  cd ~/james-useful-home-robot/ros2_ws"
        echo "  source install/setup.bash"
        return 1
    fi
    
    # Check camera
    print_info "Checking D435 camera..."
    if ros2 topic list | grep -q "/d435/color/image_raw"; then
        print_success "D435 camera detected"
    else
        print_error "D435 camera not found. Start it with:"
        echo "  ./scripts/slam-start-d435.sh"
        return 1
    fi
    
    # Check RTAB-Map
    print_info "Checking RTAB-Map..."
    if ros2 topic list | grep -q "/map"; then
        print_success "RTAB-Map detected"
    else
        print_error "RTAB-Map not found. Start it with:"
        echo "  ./scripts/slam-start-rtabmap-d435only.sh"
        return 1
    fi
    
    # Check map publishing rate
    print_info "Checking map update rate..."
    local hz=$(ros2 topic hz /map --window 10 2>&1 | grep "average rate" | awk '{print $3}')
    if [ -n "$hz" ]; then
        print_success "Map updating at ${hz} Hz"
    else
        print_warning "Could not determine map update rate"
    fi
    
    # Check current map
    print_info "Checking current map..."
    if check_map_exists; then
        local stats=$(get_map_stats)
        print_success "Map exists: $stats"
    else
        print_warning "No existing map (this is normal for first run)"
    fi
    
    echo ""
    print_success "SLAM system is ready for testing!"
}

# Main menu
show_menu() {
    clear
    print_header "SLAM Testing Helper"
    echo ""
    echo "Current Map: $(get_map_stats)"
    echo ""
    echo "1)  Quick Test - Check if SLAM system is ready"
    echo "2)  Check ROS2 Topics - Verify cameras and RTAB-Map"
    echo "3)  Monitor Loop Closures - Watch for loop closure detection"
    echo "4)  Get RTAB-Map Statistics - View current mapping stats"
    echo ""
    echo "5)  Backup Current Map - Save map with timestamp"
    echo "6)  List Map Backups - Show all saved maps"
    echo "7)  Load Map Backup - Restore a previous map"
    echo "8)  Delete Current Map - Start fresh mapping"
    echo ""
    echo "9)  View Current Map - Open in database viewer"
    echo "10) View Map Backup - Open a specific backup"
    echo "11) Export Map - Export to PLY, PCD, OctoMap, etc."
    echo ""
    echo "12) Open Testing Guide - View full testing documentation"
    echo ""
    echo "0)  Exit"
    echo ""
}

# Main loop
main() {
    while true; do
        show_menu
        read -p "Select option: " choice
        echo ""
        
        case $choice in
            1)
                quick_test
                ;;
            2)
                check_ros_topics
                ;;
            3)
                monitor_loop_closures
                ;;
            4)
                get_rtabmap_stats
                ;;
            5)
                backup_map
                ;;
            6)
                list_map_backups
                ;;
            7)
                load_map_backup
                ;;
            8)
                delete_current_map
                ;;
            9)
                view_map
                ;;
            10)
                list_map_backups
                read -p "Enter map filename: " map_file
                if [ -f "$MAPS_DIR/$map_file" ]; then
                    view_map "$MAPS_DIR/$map_file"
                else
                    print_error "Map not found: $map_file"
                fi
                ;;
            11)
                export_map
                ;;
            12)
                if [ -f "$PROJECT_ROOT/docs/slam_testing_guide.md" ]; then
                    less "$PROJECT_ROOT/docs/slam_testing_guide.md"
                else
                    print_error "Testing guide not found"
                fi
                ;;
            0)
                print_info "Exiting..."
                exit 0
                ;;
            *)
                print_error "Invalid option"
                ;;
        esac
        
        echo ""
        read -p "Press Enter to continue..."
    done
}

# Run main menu
main
