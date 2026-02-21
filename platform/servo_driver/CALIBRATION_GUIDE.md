# Gripper Calibration Guide

## New Commands Added

### Read Load
```
#<ID>L?
```
Returns the current load on the servo (-1000 to +1000).

**Example:**
```
#1L?
→ Servo 1 Load: 125
```

### Auto-Calibrate to Open Position
```
#<ID>CALOPEN
```
Automatically moves gripper toward fully open (position 4095) until hitting mechanical stop.
- Uses low speed (100) for safety
- Monitors load continuously
- Stops when load exceeds 500
- Backs off 50 units from hard stop
- Reports the max safe open position

**Example:**
```
#1CALOPEN
→ Calibrating servo 1 to OPEN position (moving toward 4095)...
→ Starting position: 2047
→ Position: 2097 Load: 120
→ Position: 2147 Load: 135
→ Position: 2197 Load: 145
→ ...
→ Position: 3847 Load: 580
→ Hard stop detected!
→ Calibration complete! Max safe OPEN position: 3797
→ Store this value for your application.
```

### Auto-Calibrate to Close Position
```
#<ID>CALCLOSE
```
Automatically moves gripper toward fully closed (position 0) until hitting mechanical stop.
- Same safety features as CALOPEN
- Moves toward 0 instead of 4095

**Example:**
```
#1CALCLOSE
→ Calibrating servo 1 to CLOSE position (moving toward 0)...
→ Starting position: 2047
→ Position: 1997 Load: 115
→ ...
→ Position: 247 Load: 620
→ Hard stop detected!
→ Calibration complete! Min safe CLOSE position: 297
→ Store this value for your application.
```

## Calibration Procedure

### Step 1: Upload New Firmware
Make sure you have the latest firmware with calibration support.

### Step 2: Calibrate to Open Position
```
#1CALOPEN
```

Watch the output and note the final position. For example:
```
Max safe OPEN position: 3797
```

### Step 3: Calibrate to Close Position
```
#1CALCLOSE
```

Note the final position:
```
Min safe CLOSE position: 297
```

### Step 4: Store Calibration Values
Use these values in your application:
- Open position: 3797
- Close position: 297
- Safe range: 297 to 3797

### Step 5: Test the Range
```
#1P297S500    → Move to closed
#1P3797S500   → Move to open
#1P2047S500   → Move to middle
```

## Tuning Parameters

If calibration doesn't work well, you can adjust these values in `SERIAL_PARSER.h`:

```cpp
const int LOAD_THRESHOLD = 500;      // Increase if too sensitive, decrease if not detecting
const int CALIBRATION_SPEED = 100;   // Slower = safer, faster = quicker calibration
const int CHECK_INTERVAL = 50;       // How often to check load (ms)
const int TIMEOUT_MS = 10000;        // Max time for calibration
const int STEP_SIZE = 50;            // Smaller = more precise, larger = faster
```

### Load Threshold Tuning

Test the load during normal operation:
```
#1P2047S500   → Move to middle
#1L?          → Read load (should be low, e.g., 50-200)
```

Then manually push the gripper against something:
```
#1L?          → Read load (should be high, e.g., 600-1000)
```

Set `LOAD_THRESHOLD` between these values (e.g., 500).

## Safety Features

1. **Low Speed**: Moves at speed 100 (very slow) to avoid damage
2. **Load Monitoring**: Checks every 50ms for excessive load
3. **Timeout**: Stops after 10 seconds even if no hard stop detected
4. **Back-off**: Moves 50 units away from hard stop for safety
5. **Incremental Movement**: Moves in small steps (50 units) for precision

## Integration with ROS2

After calibration, use the values in your ROS2 node:

```python
# Store calibration values
self.gripper_open_pos = 3797
self.gripper_close_pos = 297

# Open gripper
self.serial_port.write(f"#1P{self.gripper_open_pos}S500\n".encode())

# Close gripper
self.serial_port.write(f"#1P{self.gripper_close_pos}S500\n".encode())

# Partial open (50%)
mid_pos = (self.gripper_open_pos + self.gripper_close_pos) // 2
self.serial_port.write(f"#1P{mid_pos}S500\n".encode())
```

## Troubleshooting

### Calibration doesn't detect hard stop
- Increase `LOAD_THRESHOLD` (try 300 or 400)
- Check servo power supply
- Verify mechanical stop exists

### Calibration is too sensitive
- Decrease `LOAD_THRESHOLD` (try 600 or 700)
- Increase `CALIBRATION_SPEED` slightly

### Calibration times out
- Increase `TIMEOUT_MS`
- Check if gripper can actually reach the stop
- Verify servo is moving (try manual command first)

### Gripper damages itself
- Decrease `CALIBRATION_SPEED` (try 50)
- Decrease `LOAD_THRESHOLD` (more sensitive)
- Increase `STEP_SIZE` for faster detection

## Manual Calibration (Alternative)

If auto-calibration doesn't work, you can do it manually:

1. **Move slowly toward open:**
   ```
   #1P3000S100
   #1L?          → Check load
   #1P3500S100
   #1L?          → Check load
   #1P4000S100
   #1L?          → Check load (should spike when hitting stop)
   ```

2. **Note the position where load spikes**

3. **Back off 50-100 units from that position**

4. **Use that as your max open position**

## Command Summary

| Command | Description | Example |
|---------|-------------|---------|
| `#<ID>L?` | Read current load | `#1L?` |
| `#<ID>CALOPEN` | Auto-calibrate to open | `#1CALOPEN` |
| `#<ID>CALCLOSE` | Auto-calibrate to close | `#1CALCLOSE` |
| `#<ID>P?` | Read current position | `#1P?` |
| `#<ID>P<pos>S<speed>` | Move to position | `#1P3797S500` |

## Next Steps

1. Run calibration once
2. Store the values
3. Use them in your application
4. Re-calibrate if you change gripper mechanics
5. Consider storing calibration in ESP32 EEPROM for persistence
