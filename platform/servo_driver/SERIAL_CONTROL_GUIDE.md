# Serial Control Quick Start Guide

## Problem Solved
The ESP32 servo driver now supports text-based serial commands. Previously, commands like `#1P0S500` were being forwarded to the servos as raw bytes, but the servos only understand the SCServo binary protocol. Now the ESP32 parses these text commands in a dedicated thread and converts them to proper SCServo library calls.

## What Changed
1. Added `SERIAL_PARSER.h` - parses text commands and executes SCServo library functions
2. Updated `BOARD_DEV.h` - integrated parser into the `workingModeSelect()` thread
3. Commands starting with `#` are routed to the servo parser
4. All other commands are handled as sensor commands
5. Everything runs in a non-blocking thread, keeping the main loop free

## Connection Settings
- Baud Rate: 115200
- Format: 8N1
- Line ending: Newline (\n)

## Supported Commands

### Servo Position Control
```
#<ID>P<Position>S<Speed>
```
Example: `#1P2047S1000` - Move servo 1 to center at speed 1000

Parameters:
- ID: 0-253
- Position: 0-4095 (0° to 360°)
- Speed: 0-4000

### Read Position
```
#<ID>P?
```
Example: `#1P?` - Read current position of servo 1

### Torque Control
```
#<ID>T<0|1>
```
Examples:
- `#1T0` - Disable torque (can move manually)
- `#1T1` - Enable torque (holds position)

### Mode Control
```
#<ID>M<0|3>
```
Examples:
- `#1M0` - Servo mode (position control)
- `#1M3` - Motor mode (continuous rotation)

### Ping Servo
```
#<ID>PING
```
Example: `#1PING` - Check if servo 1 is connected

### Help
```
?
```
Shows available commands

## Testing

### Upload New Firmware
1. Open Arduino IDE
2. Load `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino`
3. Select board: ESP32 Dev Module
4. Select port (e.g., COM3 or /dev/ttyUSB0)
5. Click Upload

### Test with Serial Monitor
1. Open Serial Monitor (115200 baud)
2. Set line ending to "Newline"
3. Try commands:
   ```
   ?
   #1PING
   #1P2047S1000
   #1P?
   ```

### Test with Python Script
```bash
python platform/servo_driver/test_serial_control.py
```

Or specify port:
```bash
python platform/servo_driver/test_serial_control.py COM3
python platform/servo_driver/test_serial_control.py /dev/ttyUSB0
```

## Common Issues

### "Command must start with #"
- Make sure servo commands begin with # character
- Check line ending is set to Newline

### "Invalid servo ID"
- ID must be 0-253
- Check servo is configured with correct ID

### "Servo did not respond"
- Check servo power supply
- Verify servo wiring (GPIO18=RX, GPIO19=TX to servo bus)
- Try web interface to confirm servo is working

### No response at all
- Check baud rate is 115200
- Verify ESP32 is connected to correct port
- Make sure firmware was uploaded successfully

## Position Reference

For ST servos (360° range):
- 0° = 0
- 90° = 1024
- 180° = 2047 (center)
- 270° = 3071
- 360° = 4095

Formula: `Position = (Angle / 360) * 4095`

## Example Session

```
>>> ?
--- Sensor Commands: RD, RO, RC, SS, SCAN, INFO, HELP ---
--- Servo Commands: #<ID>P<pos>S<speed>, #<ID>T<0|1>, #<ID>M<0|3>, #<ID>PING ---
--- Example: #1P2047S1000 (move servo 1 to center at speed 1000) ---

>>> #1PING
Servo ID 1 responded to PING

>>> #1P2047S1000
OK: Servo 1 -> Position 2047 @ Speed 1000

>>> #1P?
Servo 1 Position: 2047

>>> #1T0
OK: Servo 1 torque disabled
```

## Integration with ROS2

The `teensy_serial_bridge.py` node can now send these commands directly:

```python
# In your ROS2 node
self.serial_port.write(b"#1P2047S1000\n")
```

No changes needed to your ROS2 code - just upload the new firmware!
