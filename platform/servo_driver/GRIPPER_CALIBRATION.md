# Gripper Calibration to Hard Stop

## Goal
Move gripper to fully open position (max position = 4095) until it hits mechanical stop, using load feedback to detect the limit.

## Strategy

Since higher position = wider open, we want to:
1. Move slowly toward 4095 (fully open)
2. Monitor servo load continuously
3. Stop when load exceeds threshold (hitting hard stop)
4. Record that position as the calibration point

## ST Servo Load Monitoring

The ST servos provide real-time load feedback:
- Load range: -1000 to +1000
- Negative = CCW direction load
- Positive = CW direction load
- When hitting obstacle, load spikes

## Calibration Sequence

### Manual Method (for testing)

1. **Read current position:**
   ```
   #1P?
   ```

2. **Move slowly toward open (increase position):**
   ```
   #1P4095S100
   ```
   (Very slow speed = 100)

3. **Monitor load while moving:**
   - Need to add a command to read load continuously
   - Stop when load > threshold (e.g., 500)

### Automated Calibration Command

We should add a new command: `#<ID>CAL` that:
1. Moves slowly toward max position
2. Reads load every 50ms
3. Stops when load exceeds threshold
4. Returns the calibration position

## Implementation

Let me add this to the parser:

### New Command: `#<ID>CALOPEN` (Calibrate to Open Position)
- Moves toward 4095 slowly
- Monitors load
- Stops at hard stop
- Returns max safe position

### New Command: `#<ID>CALCLOSE` (Calibrate to Closed Position)
- Moves toward 0 slowly
- Monitors load
- Stops at hard stop
- Returns min safe position

### New Command: `#<ID>L?` (Read Load)
- Returns current load value
- Useful for manual calibration

## Load Threshold Tuning

Typical values:
- Normal operation: 50-200
- Light contact: 300-500
- Hard stop: 600-1000

Start with threshold = 500, adjust based on testing.

## Safety Considerations

1. **Low speed** - Use speed 50-100 to avoid damage
2. **Load monitoring** - Check every 50ms
3. **Timeout** - Stop after 5 seconds even if no hard stop
4. **Soft limits** - Store calibrated limits in EEPROM

## Would you like me to implement this?

I can add:
1. `#1L?` - Read current load
2. `#1CALOPEN` - Auto-calibrate to open position
3. `#1CALCLOSE` - Auto-calibrate to closed position
4. Store calibration values

This way you can run calibration once at startup, then use the stored limits for safe operation.
