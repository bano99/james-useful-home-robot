# Map Quality Monitoring - Quick Reference

## Quick Start

### Launch with Monitoring
```bash
ros2 launch james_slam rtabmap_with_monitoring.launch.py
```

### View Diagnostics
```bash
# Terminal output
ros2 topic echo /diagnostics

# GUI
ros2 run rqt_robot_monitor rqt_robot_monitor
```

### Test Without Hardware
```bash
# Terminal 1
ros2 run james_slam map_quality_monitor.py

# Terminal 2
ros2 run james_slam test_map_quality_monitor.py
```

## Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `loop_closure_threshold` | 0.5 | Min success rate (0.0-1.0) |
| `window_size` | 20 | Recent attempts to track |
| `optimization_interval` | 300.0 | Seconds between optimizations |
| `degradation_check_interval` | 60.0 | Seconds between checks |

## Status Levels

| Success Rate | Status | Meaning |
|--------------|--------|---------|
| ≥ 50% | ✅ OK | Map quality good |
| 30-50% | ⚠️ WARN | Map quality degraded |
| < 30% | ❌ ERROR | Severe degradation |

## Common Commands

### Custom Configuration
```bash
ros2 run james_slam map_quality_monitor.py \
  --ros-args \
  -p loop_closure_threshold:=0.6 \
  -p window_size:=30
```

### Check Node Status
```bash
ros2 node list | grep map_quality
ros2 node info /map_quality_monitor
```

### Monitor Topics
```bash
# Info messages from RTAB-Map
ros2 topic hz /rtabmap/info

# Diagnostics from monitor
ros2 topic hz /diagnostics
```

## Troubleshooting

### No diagnostics appearing
- Check monitor is running: `ros2 node list`
- Verify RTAB-Map is publishing: `ros2 topic hz /rtabmap/info`

### "Insufficient data" message
- Normal at startup
- Need at least one loop closure attempt
- Drive robot through previously mapped areas

### High failure rate
- Reduce robot velocity
- Improve lighting
- Check camera calibration
- Tune RTAB-Map parameters

## Integration

### With Web Dashboard
```javascript
diagnostics.subscribe((msg) => {
  const slam = msg.status.find(s => s.name === 'SLAM Map Quality');
  updateDisplay(slam);
});
```

### With Navigation
Monitor can trigger replanning when map quality degrades:
```python
if map_quality < threshold:
    nav2_client.cancel_goal()
    trigger_replan()
```

## More Information

See full documentation: `ros2_ws/src/james_slam/docs/map_quality_monitoring.md`
