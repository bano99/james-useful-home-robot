# Map Quality Monitoring

## Overview

The Map Quality Monitor is a ROS2 node that monitors the health and quality of the RTAB-Map SLAM system. It tracks loop closure success rates, detects map degradation, and triggers optimization when needed.

This component validates **Requirement 2.4**: "THE Main Brain SHALL update the SLAM map continuously during operation to reflect environmental changes."

## Features

### 1. Loop Closure Monitoring

The monitor tracks loop closure attempts and their success/failure rates:

- **Recent Success Rate**: Calculated over a sliding window (default: last 20 attempts)
- **Overall Success Rate**: Calculated since node startup
- **Threshold-based Alerts**: Warns when success rate drops below configurable threshold (default: 50%)

### 2. Map Degradation Detection

The monitor detects several types of map degradation:

- **Low Loop Closure Success Rate**: When recent success rate falls below threshold
- **Severe Degradation**: When success rate drops below 30%
- **Stale Data**: When RTAB-Map stops publishing info messages (potential crash)

### 3. Automatic Optimization

The monitor can trigger map optimization in two scenarios:

- **Periodic**: At regular intervals (default: every 5 minutes)
- **On-Demand**: When severe degradation is detected (success rate < 30%)

### 4. Diagnostics Publishing

The monitor publishes diagnostic information to `/diagnostics` topic:

- Overall map quality status (OK, WARN, ERROR)
- Loop closure statistics
- Map node count
- Time since last optimization

## Usage

### Launch with RTAB-Map

Use the provided launch file to start RTAB-Map with monitoring:

```bash
ros2 launch james_slam rtabmap_with_monitoring.launch.py
```

### Launch Standalone

If RTAB-Map is already running, launch the monitor separately:

```bash
ros2 run james_slam map_quality_monitor.py
```

### Configuration Parameters

The monitor accepts the following parameters:

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `loop_closure_threshold` | float | 0.5 | Minimum acceptable success rate (0.0-1.0) |
| `window_size` | int | 20 | Number of recent attempts to track |
| `optimization_interval` | float | 300.0 | Seconds between periodic optimizations |
| `degradation_check_interval` | float | 60.0 | Seconds between degradation checks |

Example with custom parameters:

```bash
ros2 run james_slam map_quality_monitor.py \
  --ros-args \
  -p loop_closure_threshold:=0.6 \
  -p window_size:=30 \
  -p optimization_interval:=600.0
```

## Testing

### Simulated Testing

Test the monitor without running actual SLAM:

```bash
# Terminal 1: Start the monitor
ros2 run james_slam map_quality_monitor.py

# Terminal 2: Run the test simulator
ros2 run james_slam test_map_quality_monitor.py
```

The test simulator publishes fake RTAB-Map info messages with simulated loop closures (70% success rate).

### Monitor Diagnostics

View diagnostic output:

```bash
ros2 topic echo /diagnostics
```

### Monitor Logs

The monitor logs important events:

- Loop closure successes/failures
- Map degradation warnings
- Optimization triggers
- System health status

## Topics

### Subscribed Topics

- `/rtabmap/info` (rtabmap_msgs/Info): RTAB-Map statistics and loop closure info
- `/rtabmap/mapData` (rtabmap_msgs/MapData): Map graph data

### Published Topics

- `/diagnostics` (diagnostic_msgs/DiagnosticArray): System diagnostics

### Service Clients

- `/rtabmap/trigger_new_map` (std_srvs/Empty): Triggers map optimization

## Architecture

```
┌─────────────────────┐
│   RTAB-Map SLAM     │
│                     │
│  - Loop Closure     │
│  - Map Building     │
│  - Optimization     │
└──────────┬──────────┘
           │ /rtabmap/info
           │ /rtabmap/mapData
           ▼
┌─────────────────────┐
│ Map Quality Monitor │
│                     │
│  - Track Success    │
│  - Detect Issues    │
│  - Trigger Optimize │
└──────────┬──────────┘
           │ /diagnostics
           ▼
┌─────────────────────┐
│  Diagnostic System  │
│  (rqt_robot_monitor)│
└─────────────────────┘
```

## Metrics

### Loop Closure Success Rate

The success rate is calculated as:

```
Success Rate = Successful Loop Closures / Total Loop Closure Attempts
```

A loop closure is considered successful if:
1. RTAB-Map detects a loop closure (`loop_closure_id > 0`)
2. The loop closure is accepted (covariance > 0)

### Quality Thresholds

| Success Rate | Status | Action |
|--------------|--------|--------|
| ≥ 50% | OK | Normal operation |
| 30-50% | WARN | Monitor closely |
| < 30% | ERROR | Trigger optimization |

## Troubleshooting

### No Loop Closures Detected

**Symptoms**: Monitor shows "Insufficient data for quality assessment"

**Possible Causes**:
- Robot hasn't revisited previous locations
- Environment lacks distinctive features
- Camera quality issues

**Solutions**:
- Drive robot through previously mapped areas
- Improve lighting conditions
- Check camera calibration

### High Loop Closure Failure Rate

**Symptoms**: Success rate consistently below 50%

**Possible Causes**:
- Dynamic environment (objects moving)
- Poor lighting conditions
- Camera motion blur
- Incorrect RTAB-Map parameters

**Solutions**:
- Reduce robot velocity
- Improve lighting
- Tune RTAB-Map feature detection parameters
- Check camera exposure settings

### Stale Info Messages

**Symptoms**: "RTAB-Map info messages stale" error

**Possible Causes**:
- RTAB-Map node crashed
- Topic remapping incorrect
- Network issues (if using distributed ROS)

**Solutions**:
- Check if RTAB-Map node is running: `ros2 node list`
- Verify topic connections: `ros2 topic info /rtabmap/info`
- Restart RTAB-Map node

## Integration with Web Dashboard

The diagnostics published by the monitor can be displayed in the web dashboard:

```javascript
// Subscribe to diagnostics
const diagnostics = new ROSLIB.Topic({
  ros: ros,
  name: '/diagnostics',
  messageType: 'diagnostic_msgs/DiagnosticArray'
});

diagnostics.subscribe((message) => {
  const slamStatus = message.status.find(s => s.name === 'SLAM Map Quality');
  if (slamStatus) {
    updateMapQualityDisplay(slamStatus);
  }
});
```

## Future Enhancements

Potential improvements for the map quality monitor:

1. **Adaptive Thresholds**: Adjust thresholds based on environment type
2. **Predictive Degradation**: Use ML to predict degradation before it occurs
3. **Automatic Recovery**: Implement recovery strategies beyond optimization
4. **Map Comparison**: Compare current map with reference map for drift detection
5. **Performance Metrics**: Track SLAM computational performance (CPU, memory)

## References

- [RTAB-Map Documentation](http://wiki.ros.org/rtabmap_ros)
- [ROS2 Diagnostics](https://docs.ros.org/en/rolling/Tutorials/Demos/Logging-and-logger-configuration.html)
- James Robot Design Document: Section "SLAM and Localization"
