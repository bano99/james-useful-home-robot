# Map Quality Monitoring - Troubleshooting Guide

## Common Issues and Solutions

### Issue 1: No Diagnostics Published

**Symptoms:**
- `/diagnostics` topic shows no messages
- `ros2 topic echo /diagnostics` returns nothing

**Diagnosis:**
```bash
# Check if monitor node is running
ros2 node list | grep map_quality

# Check if RTAB-Map is publishing
ros2 topic hz /rtabmap/info

# Check topic connections
ros2 topic info /diagnostics
```

**Solutions:**

1. **Monitor not running:**
   ```bash
   # Start the monitor
   ros2 run james_slam map_quality_monitor.py
   ```

2. **RTAB-Map not publishing:**
   ```bash
   # Check RTAB-Map is running
   ros2 node list | grep rtabmap
   
   # Restart RTAB-Map if needed
   ros2 launch james_slam rtabmap_d435_only.launch.py
   ```

3. **Topic remapping issue:**
   ```bash
   # Check actual topic names
   ros2 topic list | grep info
   
   # If different, remap in launch file or command line
   ros2 run james_slam map_quality_monitor.py \
     --ros-args -r /rtabmap/info:=/actual_topic_name
   ```

### Issue 2: "Insufficient Data" Message

**Symptoms:**
- Diagnostics show "Insufficient data for quality assessment"
- Status remains at this message for extended time

**Diagnosis:**
```bash
# Check if loop closures are being detected
ros2 topic echo /rtabmap/info | grep loop_closure_id
```

**Explanation:**
- This is normal at startup
- Monitor needs at least one loop closure attempt to calculate success rate
- Loop closures only occur when robot revisits previous locations

**Solutions:**

1. **Drive through mapped areas:**
   - Use remote control to drive robot
   - Revisit areas that were mapped earlier
   - Look for distinctive features (furniture, walls with texture)

2. **Check RTAB-Map configuration:**
   ```bash
   # Verify loop closure is enabled
   ros2 param get /rtabmap Mem/LoopClosureDetection
   # Should return: true
   ```

3. **Improve environment:**
   - Add more distinctive features
   - Improve lighting
   - Avoid blank walls

### Issue 3: High Loop Closure Failure Rate

**Symptoms:**
- Success rate consistently below 50%
- Status shows WARN or ERROR
- Many "Loop closure rejected" messages in logs

**Diagnosis:**
```bash
# Monitor loop closure attempts
ros2 topic echo /rtabmap/info | grep -A 5 loop_closure

# Check RTAB-Map statistics
ros2 topic echo /rtabmap/info | grep -E "(loop|rejected)"
```

**Common Causes:**

1. **Robot moving too fast:**
   - Motion blur in images
   - Insufficient feature matching time
   
   **Solution:** Reduce velocity to < 0.3 m/s

2. **Poor lighting:**
   - Underexposed or overexposed images
   - Inconsistent lighting between visits
   
   **Solution:** 
   ```bash
   # Check camera exposure
   ros2 topic echo /d435/color/camera_info
   
   # Adjust camera settings if needed
   ros2 param set /d435 exposure 100
   ```

3. **Dynamic environment:**
   - Objects moved between visits
   - People in different positions
   
   **Solution:** Map during quiet times, accept lower success rate

4. **Incorrect RTAB-Map parameters:**
   - Feature detection too strict
   - Loop closure threshold too high
   
   **Solution:** Tune parameters in `rtabmap_d435_only.yaml`:
   ```yaml
   Kp/MaxFeatures: "300"  # Increase from 200
   Vis/MinInliers: "10"   # Decrease from 15
   ```

### Issue 4: Stale Info Messages

**Symptoms:**
- Error: "RTAB-Map info messages stale (XX.Xs). SLAM may have stopped!"
- No new info messages for > 30 seconds

**Diagnosis:**
```bash
# Check if RTAB-Map is running
ros2 node list | grep rtabmap

# Check CPU usage
top -p $(pgrep -f rtabmap)

# Check for errors in RTAB-Map logs
ros2 node info /rtabmap
```

**Solutions:**

1. **RTAB-Map crashed:**
   ```bash
   # Restart RTAB-Map
   ros2 launch james_slam rtabmap_d435_only.launch.py
   ```

2. **CPU overload:**
   - RTAB-Map may be frozen due to high CPU usage
   - Reduce feature count in config
   - Close other applications
   
   ```bash
   # Check system resources
   htop
   
   # Reduce RTAB-Map load
   ros2 param set /rtabmap Kp/MaxFeatures 150
   ```

3. **Camera stopped:**
   ```bash
   # Check camera topics
   ros2 topic hz /d435/color/image_raw
   ros2 topic hz /d435/aligned_depth_to_color/image_raw
   
   # Restart camera if needed
   ros2 launch realsense2_camera rs_launch.py device_type:=d435
   ```

### Issue 5: Optimization Not Triggering

**Symptoms:**
- Severe degradation detected but no optimization
- "Map optimization service not available" warning

**Diagnosis:**
```bash
# Check if optimization service exists
ros2 service list | grep trigger_new_map

# Try calling service manually
ros2 service call /rtabmap/trigger_new_map std_srvs/srv/Empty
```

**Solutions:**

1. **Service name incorrect:**
   ```bash
   # Find actual service name
   ros2 service list | grep rtabmap
   
   # Update monitor if needed (check source code)
   ```

2. **RTAB-Map not exposing service:**
   - Some RTAB-Map configurations may not expose this service
   - Check RTAB-Map version and configuration
   
   ```bash
   # Check RTAB-Map version
   ros2 pkg list | grep rtabmap
   
   # Verify service is enabled in launch file
   ```

3. **Permissions issue:**
   ```bash
   # Check if service is callable
   ros2 service type /rtabmap/trigger_new_map
   ```

### Issue 6: Monitor Crashes or Restarts

**Symptoms:**
- Monitor node disappears from node list
- Python exceptions in logs
- Automatic restarts

**Diagnosis:**
```bash
# Check for Python errors
ros2 run james_slam map_quality_monitor.py

# Check system logs
journalctl -u ros2* | grep map_quality
```

**Common Causes:**

1. **Missing dependencies:**
   ```bash
   # Install required packages
   sudo apt install ros-jazzy-diagnostic-msgs
   sudo apt install ros-jazzy-rtabmap-msgs
   pip3 install rclpy
   ```

2. **Memory issues:**
   ```bash
   # Check memory usage
   free -h
   
   # Add swap if needed
   sudo fallocate -l 2G /swapfile
   sudo chmod 600 /swapfile
   sudo mkswap /swapfile
   sudo swapon /swapfile
   ```

3. **Python version mismatch:**
   ```bash
   # Check Python version
   python3 --version
   # Should be 3.10+ for ROS2 Jazzy
   ```

### Issue 7: Incorrect Success Rate Calculation

**Symptoms:**
- Success rate doesn't match observed loop closures
- Rate seems stuck or not updating

**Diagnosis:**
```bash
# Enable debug logging
ros2 run james_slam map_quality_monitor.py \
  --ros-args --log-level debug

# Watch for loop closure messages
ros2 topic echo /rtabmap/info | grep loop_closure_id
```

**Solutions:**

1. **Check covariance interpretation:**
   - Monitor considers covariance[0] > 0 as success
   - Verify this matches RTAB-Map behavior
   
   ```bash
   # Examine actual info messages
   ros2 topic echo /rtabmap/info
   ```

2. **Window size too small:**
   - Increase window size for more stable rate
   
   ```bash
   ros2 run james_slam map_quality_monitor.py \
     --ros-args -p window_size:=50
   ```

3. **Timing issues:**
   - Messages arriving out of order
   - Check system time synchronization
   
   ```bash
   # Check system time
   timedatectl
   ```

## Diagnostic Commands Reference

### Check System Status
```bash
# All nodes
ros2 node list

# Monitor specifically
ros2 node info /map_quality_monitor

# All topics
ros2 topic list

# Topic rates
ros2 topic hz /rtabmap/info
ros2 topic hz /diagnostics
```

### Monitor Specific Metrics
```bash
# Loop closure info
ros2 topic echo /rtabmap/info | grep -A 10 loop_closure

# Diagnostic status
ros2 topic echo /diagnostics | grep -A 20 "SLAM Map Quality"

# Monitor logs
ros2 run james_slam map_quality_monitor.py 2>&1 | tee monitor.log
```

### Test Components
```bash
# Test with simulator
ros2 run james_slam test_map_quality_monitor.py

# Manual service call
ros2 service call /rtabmap/trigger_new_map std_srvs/srv/Empty

# Check parameter values
ros2 param list /map_quality_monitor
ros2 param get /map_quality_monitor loop_closure_threshold
```

## Performance Optimization

### Reduce CPU Usage
```yaml
# In rtabmap_d435_only.yaml
Kp/MaxFeatures: "150"  # Reduce from 200
Mem/STMSize: "5"       # Reduce from 10
```

### Reduce Memory Usage
```yaml
# In rtabmap_d435_only.yaml
Mem/ImageKept: "false"     # Don't keep images
Mem/BinDataKept: "false"   # Don't keep raw data
```

### Improve Responsiveness
```bash
# Increase check frequency
ros2 run james_slam map_quality_monitor.py \
  --ros-args -p degradation_check_interval:=30.0
```

## Getting Help

### Collect Debug Information
```bash
# Create debug report
cat > debug_report.txt << EOF
=== Node Status ===
$(ros2 node list)

=== Topic Status ===
$(ros2 topic list)
$(ros2 topic hz /rtabmap/info)
$(ros2 topic hz /diagnostics)

=== Parameter Values ===
$(ros2 param list /map_quality_monitor)

=== Recent Logs ===
$(ros2 run james_slam map_quality_monitor.py 2>&1 | tail -50)
EOF
```

### Enable Verbose Logging
```bash
# Run with debug output
ros2 run james_slam map_quality_monitor.py \
  --ros-args --log-level debug

# Or set via environment
export RCUTILS_CONSOLE_OUTPUT_FORMAT="[{severity}] [{name}]: {message}"
ros2 run james_slam map_quality_monitor.py
```

### Contact Support
When reporting issues, include:
1. ROS2 version: `ros2 --version`
2. RTAB-Map version: `ros2 pkg list | grep rtabmap`
3. Debug report (see above)
4. Steps to reproduce
5. Expected vs actual behavior

## Prevention Best Practices

1. **Regular monitoring:**
   - Check diagnostics periodically
   - Watch for degradation trends
   - Act on warnings early

2. **Proper configuration:**
   - Tune thresholds for your environment
   - Adjust window size based on loop closure frequency
   - Set appropriate optimization intervals

3. **Environment management:**
   - Maintain consistent lighting
   - Minimize dynamic objects during mapping
   - Provide distinctive features for loop closure

4. **System maintenance:**
   - Keep ROS2 and RTAB-Map updated
   - Monitor system resources (CPU, memory)
   - Regular camera calibration checks

5. **Testing:**
   - Test with simulator before deployment
   - Validate in controlled environment first
   - Gradually increase complexity
