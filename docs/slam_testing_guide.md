# SLAM Testing Guide for James Home Robot

This guide walks you through testing the SLAM system in your home environment to verify that James can build accurate maps, perform loop closure, and relocalize after restart.

## Prerequisites

- [ ] James robot assembled with D435 camera mounted at 120cm height
- [ ] Remote control paired and functional
- [ ] Jetson Nano running with ROS2 installed
- [ ] RTAB-Map package built and sourced
- [ ] Battery charged (at least 80% for full home mapping)
- [ ] Clear path through home (remove obstacles that could damage robot)

## Test Overview

This test validates Requirements 2.1, 2.3, and 2.4:
- **2.1**: SLAM using D435 and T265 cameras to create 3D map
- **2.3**: Use SLAM map to plan collision-free paths
- **2.4**: Update SLAM map continuously during operation

## Test Procedure

### Phase 1: Initial Map Building (30-45 minutes)

#### 1.1 Prepare Environment

**Checklist:**
- [ ] Good lighting throughout home (open curtains, turn on lights)
- [ ] Remove small obstacles from floor (toys, cables, etc.)
- [ ] Identify key areas to map: living room, kitchen, hallways, bedrooms
- [ ] Plan a route that forms loops (for loop closure testing)
- [ ] Have a notepad ready to record observations

**Recommended Route:**
```
Start → Living Room → Kitchen → Hallway → Bedroom 1 → 
Hallway → Bedroom 2 → Hallway → Kitchen → Living Room → Start
```

This creates multiple loops for testing loop closure detection.

#### 1.2 Start SLAM System

**Terminal 1 - Start D435 Camera:**
```bash
cd ~/james-useful-home-robot
./scripts/slam-start-d435.sh
```

Wait for camera to initialize. You should see:
```
[INFO] RealSense Node Is Up!
```

**Terminal 2 - Start RTAB-Map:**
```bash
cd ~/james-useful-home-robot
# Start without images first (lower memory)
./scripts/slam-start-rtabmap-d435only.sh
```

Wait for RTAB-Map to initialize. You should see:
```
[INFO] rtabmap: RTAB-Map started!
[INFO] rgbd_odometry: Visual odometry started!
```

**Verify System is Running:**
```bash
# In a new terminal
ros2 topic hz /map
# Should show ~1-2 Hz

ros2 topic echo /info --once
# Should show RTAB-Map statistics
```

#### 1.3 Drive Through Home

**Instructions:**
1. **Start slowly** - Drive at walking pace (< 0.3 m/s)
2. **Look at textured surfaces** - Point camera at walls with pictures, furniture, not blank walls
3. **Overlap coverage** - Revisit areas from different angles
4. **Complete loops** - Return to starting position multiple times
5. **Pause at key locations** - Stop for 2-3 seconds at doorways and room centers

**What to Record:**
- [ ] Total mapping time: _____ minutes
- [ ] Approximate distance traveled: _____ meters
- [ ] Number of rooms mapped: _____
- [ ] Any areas where tracking was lost: _____
- [ ] Any crashes or collisions: _____

**Signs of Good Mapping:**
- ✅ Smooth robot movement in RViz (if running)
- ✅ Map gradually fills in with consistent geometry
- ✅ No sudden jumps in robot position
- ✅ Walls appear straight and parallel

**Signs of Problems:**
- ❌ Robot position jumps suddenly (tracking lost)
- ❌ Duplicate walls or rooms (poor loop closure)
- ❌ Walls appear wavy or distorted (poor odometry)
- ❌ Large blank areas (insufficient coverage)

#### 1.4 Monitor Loop Closures

While driving, monitor loop closure detection:

```bash
# In a new terminal
ros2 topic echo /info | grep "Loop closure"
```

**Expected Behavior:**
- First loop: Should detect loop closure when returning to start
- Subsequent loops: Should detect 2-3 loop closures per complete circuit
- Loop closure causes brief pause in processing (normal)

**Record:**
- [ ] Number of loop closures detected: _____
- [ ] Were loop closures detected at expected locations? Yes / No
- [ ] Did map quality improve after loop closures? Yes / No

#### 1.5 Stop and Save Map

When mapping is complete:

1. **Stop driving** - Park robot in safe location
2. **Let RTAB-Map process** - Wait 30 seconds for final optimization
3. **Backup the map:**
```bash
cd ~/james-useful-home-robot
./scripts/slam-backup-map.sh
```

This saves the map to `~/james_maps/home_map_YYYYMMDD_HHMM.db`

**Record the map filename:** _____________________________

### Phase 2: Map Quality Verification (10-15 minutes)

#### 2.1 View Map in Database Viewer

Stop all ROS2 nodes (Ctrl+C in each terminal), then:

```bash
cd ~/james-useful-home-robot
./scripts/slam-view-map.sh
```

**What to Check:**

**3D View:**
- [ ] Walls are straight and continuous
- [ ] Rooms have correct dimensions (measure with tape measure if needed)
- [ ] Furniture is recognizable
- [ ] Floor is flat and level
- [ ] Ceiling height is consistent

**Pose Graph:**
- [ ] Click "View" → "Graph View"
- [ ] Nodes form a connected path
- [ ] Loop closure links are visible (green lines)
- [ ] No isolated clusters of nodes

**Statistics:**
- [ ] Click "View" → "Statistics"
- [ ] Total nodes: _____ (should be > 100 for full home)
- [ ] Loop closures: _____ (should be > 5 for good map)
- [ ] Average features per image: _____ (should be > 100)

**Export Map for Analysis:**
```
File → Export → Point Cloud (PLY)
Save as: ~/james_maps/home_map_YYYYMMDD_HHMM.ply
```

#### 2.2 Measure Map Accuracy

**Test 1: Room Dimensions**
- Measure a room with tape measure: _____ m x _____ m
- Measure same room in map viewer: _____ m x _____ m
- Error: _____ % (should be < 5%)

**Test 2: Doorway Width**
- Measure doorway with tape measure: _____ cm
- Measure same doorway in map: _____ cm
- Error: _____ cm (should be < 5 cm)

**Test 3: Furniture Position**
- Place a marker (e.g., colored tape) on floor
- Note position in map
- Move robot, return to marker
- Check if marker is at same position in map
- Position drift: _____ cm (should be < 10 cm)

### Phase 3: Loop Closure Verification (15-20 minutes)

This test verifies that RTAB-Map correctly recognizes when the robot returns to a previously visited location.

#### 3.1 Restart SLAM System

```bash
# Terminal 1 - D435
./scripts/slam-start-d435.sh

# Terminal 2 - RTAB-Map (load existing map)
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash
ros2 launch james_slam rtabmap_d435_only.launch.py localization:=false
```

The system will load the existing map from `~/.ros/rtabmap.db`.

#### 3.2 Drive a New Loop

Drive the robot through a path that overlaps with the previous mapping session:

1. Start at a recognizable location (e.g., living room center)
2. Drive to another room
3. Return to starting location via a different path
4. Monitor for loop closure detection

**Monitor Loop Closure:**
```bash
ros2 topic echo /info | grep "Loop closure"
```

**Expected:**
- Loop closure should be detected within 5 seconds of returning to start
- Map should "snap" into alignment (visible in RViz if running)
- No duplicate geometry should appear

**Record:**
- [ ] Loop closure detected? Yes / No
- [ ] Time to detect: _____ seconds
- [ ] Map quality after loop closure: Good / Fair / Poor
- [ ] Any duplicate walls or rooms? Yes / No

#### 3.3 Test Multiple Loop Closures

Repeat the loop closure test 3 times with different paths:

**Loop 1:**
- Path: _____________________________
- Loop closure detected: Yes / No
- Detection time: _____ seconds

**Loop 2:**
- Path: _____________________________
- Loop closure detected: Yes / No
- Detection time: _____ seconds

**Loop 3:**
- Path: _____________________________
- Loop closure detected: Yes / No
- Detection time: _____ seconds

**Success Criteria:**
- ✅ At least 2 out of 3 loop closures detected
- ✅ Detection time < 10 seconds
- ✅ No false loop closures (incorrect matches)

### Phase 4: Relocalization After Restart (15-20 minutes)

This test verifies that RTAB-Map can relocalize the robot after a complete system restart.

#### 4.1 Stop All Nodes

```bash
# Press Ctrl+C in all terminals
# Wait for clean shutdown
```

#### 4.2 Move Robot to New Location

Manually move the robot to a different location in the mapped area:
- Pick up robot (or drive manually with motors off)
- Place in a different room
- Note the new location: _____________________________

#### 4.3 Restart in Localization Mode

```bash
# Terminal 1 - D435
./scripts/slam-start-d435.sh

# Terminal 2 - RTAB-Map in localization mode
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash
ros2 launch james_slam rtabmap_d435_only.launch.py localization:=true
```

**Localization mode:**
- Does not add new nodes to map
- Only localizes robot within existing map
- Lower computational load

#### 4.4 Verify Relocalization

**Monitor Localization:**
```bash
ros2 topic echo /info --once
```

Look for:
- `Localization: true`
- `Hypothesis: X` (should be > 0 when localized)

**Drive Robot Slowly:**
- Move robot forward 1-2 meters
- Rotate in place 360 degrees
- Move to a recognizable landmark (doorway, furniture)

**Expected Behavior:**
- Within 10-30 seconds, robot should localize
- Position in map should match physical location
- Confidence should increase as robot moves

**Record:**
- [ ] Relocalization successful? Yes / No
- [ ] Time to relocalize: _____ seconds
- [ ] Initial position accuracy: Good / Fair / Poor
- [ ] Position accuracy after movement: Good / Fair / Poor

#### 4.5 Test Relocalization from Multiple Locations

Repeat relocalization test from 3 different starting locations:

**Location 1:** _____________________________
- Relocalization time: _____ seconds
- Success: Yes / No

**Location 2:** _____________________________
- Relocalization time: _____ seconds
- Success: Yes / No

**Location 3:** _____________________________
- Relocalization time: _____ seconds
- Success: Yes / No

**Success Criteria:**
- ✅ At least 2 out of 3 relocalizations successful
- ✅ Relocalization time < 60 seconds
- ✅ Position accuracy within 20 cm

### Phase 5: Map Persistence and Loading (5-10 minutes)

This test verifies that maps can be saved, loaded, and used across sessions.

#### 5.1 Create Multiple Map Backups

```bash
# Backup current map
./scripts/slam-backup-map.sh

# Record filename
# Map 1: _____________________________
```

Drive robot to add more coverage, then backup again:

```bash
./scripts/slam-backup-map.sh
# Map 2: _____________________________
```

#### 5.2 Load Specific Map

```bash
# Stop all nodes
# Copy a specific backup to active location
cp ~/james_maps/home_map_YYYYMMDD_HHMM.db ~/.ros/rtabmap.db

# Restart RTAB-Map
cd ~/james-useful-home-robot/ros2_ws
source install/setup.bash
ros2 launch james_slam rtabmap_d435_only.launch.py localization:=true
```

**Verify:**
- [ ] Map loaded successfully? Yes / No
- [ ] Robot localizes in loaded map? Yes / No
- [ ] Map matches expected coverage? Yes / No

#### 5.3 View Map Statistics

```bash
./scripts/slam-view-map.sh ~/james_maps/home_map_YYYYMMDD_HHMM.db
```

**Record for each backup:**

**Map 1:**
- Nodes: _____
- Loop closures: _____
- File size: _____ MB

**Map 2:**
- Nodes: _____
- Loop closures: _____
- File size: _____ MB

## Test Results Summary

### Overall Assessment

**SLAM Functionality:**
- [ ] Map building: Pass / Fail
- [ ] Loop closure detection: Pass / Fail
- [ ] Relocalization: Pass / Fail
- [ ] Map persistence: Pass / Fail

**Map Quality:**
- [ ] Geometric accuracy: Good / Fair / Poor
- [ ] Coverage completeness: Good / Fair / Poor
- [ ] Consistency: Good / Fair / Poor

**Performance:**
- [ ] Processing speed: Good / Fair / Poor
- [ ] Memory usage: Good / Fair / Poor
- [ ] Battery life during mapping: _____ minutes

### Issues Encountered

List any problems encountered during testing:

1. _____________________________
2. _____________________________
3. _____________________________

### Recommendations

Based on test results:

- [ ] Map quality is sufficient for navigation (proceed to Task 8)
- [ ] Map needs improvement (remap with better coverage)
- [ ] System needs tuning (adjust RTAB-Map parameters)
- [ ] Hardware issues detected (check cameras, motors, etc.)

### Next Steps

- [ ] Save final map for navigation testing
- [ ] Document any parameter changes needed
- [ ] Proceed to Task 8: Navigation System
- [ ] Create additional maps for different areas if needed

## Troubleshooting

### Problem: Tracking Lost Frequently

**Symptoms:**
- Robot position jumps in map
- "Odometry lost" warnings
- Map has duplicate geometry

**Solutions:**
1. Improve lighting (open curtains, add lamps)
2. Drive slower (< 0.2 m/s)
3. Point camera at textured surfaces
4. Reduce `Kp/MaxFeatures` if CPU overloaded
5. Check camera is securely mounted (no vibration)

### Problem: No Loop Closures Detected

**Symptoms:**
- Map drifts over time
- No "Loop closure" messages
- Pose graph has no green links

**Solutions:**
1. Drive more overlapping paths
2. Return to starting location multiple times
3. Increase `Mem/STMSize` for longer memory
4. Ensure good visual features (textured environment)
5. Check `Mem/LoopClosureDetection` is enabled

### Problem: Poor Relocalization

**Symptoms:**
- Robot cannot localize after restart
- Position is incorrect
- Takes > 60 seconds to localize

**Solutions:**
1. Start relocalization from a distinctive location
2. Rotate robot 360° to gather features
3. Drive to a recognizable landmark
4. Increase `Kp/MaxFeatures` for more features
5. Ensure map has good coverage of area

### Problem: Map Quality is Poor

**Symptoms:**
- Walls are wavy or distorted
- Rooms have incorrect dimensions
- Furniture is unrecognizable

**Solutions:**
1. Remap with slower driving speed
2. Ensure camera is level and stable
3. Calibrate camera intrinsics
4. Check for camera lens distortion
5. Verify camera mounting height (should be 120cm)

### Problem: Out of Memory

**Symptoms:**
- RTAB-Map crashes
- "Out of memory" errors
- System becomes unresponsive

**Solutions:**
1. Close RViz and other applications
2. Reduce `Mem/STMSize` (try 5 instead of 10)
3. Reduce `Kp/MaxFeatures` (try 100 instead of 200)
4. Enable swap space (see james_slam README)
5. Map smaller areas at a time

## Validation Checklist

Before proceeding to navigation testing, verify:

- [ ] Map covers all areas where robot will operate
- [ ] Map accuracy is within 5% of actual dimensions
- [ ] Loop closures detected reliably (> 80% success rate)
- [ ] Relocalization works from multiple locations (> 66% success rate)
- [ ] Map persists across system restarts
- [ ] Map file is backed up to `~/james_maps/`
- [ ] No critical issues encountered during testing

## References

- Requirements: `.kiro/specs/james-home-robot/requirements.md` (2.1, 2.3, 2.4)
- Design: `.kiro/specs/james-home-robot/design.md` (SLAM section)
- RTAB-Map Documentation: `ros2_ws/src/james_slam/README.md`
- SLAM Scripts: `scripts/SLAM_SCRIPTS_README.md`

---

**Test Completed By:** _____________________________

**Date:** _____________________________

**Signature:** _____________________________
