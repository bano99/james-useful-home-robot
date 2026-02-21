# Upload and Test Checklist

Follow these steps to fix your serial control issue:

## ✅ Step 1: Verify Files Are Ready
Check that these files exist and are updated:
- [ ] `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/SERIAL_PARSER.h` (new)
- [ ] `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/BOARD_DEV.h` (modified)
- [ ] `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino` (modified)
- [ ] Include order: SERIAL_PARSER.h comes before BOARD_DEV.h

## ✅ Step 2: Upload Firmware to ESP32

### Arduino IDE Setup
1. [ ] Open Arduino IDE
2. [ ] Go to File → Open
3. [ ] Navigate to: `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino`
4. [ ] Verify `#include "SERIAL_PARSER.h"` is in the includes section

### Board Configuration
5. [ ] Tools → Board → ESP32 Arduino → ESP32 Dev Module
6. [ ] Tools → Port → Select your ESP32 port (e.g., COM3 or /dev/ttyUSB0)
7. [ ] Tools → Upload Speed → 115200

### Upload
8. [ ] Click the Upload button (→)
9. [ ] Wait for "Done uploading" message
10. [ ] Check for any compilation errors (there should be none)

## ✅ Step 3: Test with Serial Monitor

### Open Serial Monitor
1. [ ] Tools → Serial Monitor (or Ctrl+Shift+M)
2. [ ] Set baud rate to: 115200
3. [ ] Set line ending to: "Newline" or "Both NL & CR"

### Test Commands
4. [ ] Type `?` and press Enter
   - Expected: Help message showing available commands

5. [ ] Type `#1PING` and press Enter
   - Expected: "Servo ID 1 responded to PING" (if servo is connected)
   - Or: "Servo ID 1 did not respond" (if not connected/powered)

6. [ ] Type `#1P2047S1000` and press Enter
   - Expected: "OK: Servo 1 -> Position 2047 @ Speed 1000"
   - Servo should move to center position

7. [ ] Type `#1P?` and press Enter
   - Expected: "Servo 1 Position: 2047" (or current position)

8. [ ] Type `#1P0S500` and press Enter
   - Expected: "OK: Servo 1 -> Position 0 @ Speed 500"
   - Servo should move to 0° position

## ✅ Step 4: Test with Python Script (Optional)

### Install Requirements
```bash
pip install pyserial
```

### Run Test Script
```bash
# Windows
python platform/servo_driver/test_serial_control.py COM3

# Linux/Mac
python platform/servo_driver/test_serial_control.py /dev/ttyUSB0
```

Expected: Script runs through all tests and servo moves accordingly

## ✅ Step 5: Test with Your ROS2 Application

Your existing code should now work without changes:
```python
self.serial_port.write(b"#1P2047S1000\n")
```

## Troubleshooting

### ❌ "Command must start with #"
- Solution: Make sure servo commands begin with # character

### ❌ "Servo did not respond"
- Check: Servo power supply is connected and on
- Check: Servo is connected to the correct UART pins (GPIO18/19)
- Check: Servo ID matches the command (default is usually 1)
- Try: Web interface to verify servo works

### ❌ No response at all
- Check: Baud rate is 115200
- Check: Line ending is set to "Newline"
- Check: Correct COM port is selected
- Try: Disconnect and reconnect ESP32

### ❌ Compilation errors
- Check: All .h files are in the same folder as ServoDriver.ino
- Check: Arduino IDE has ESP32 board support installed
- Check: Latest version of SCServo library is installed

### ❌ Upload fails
- Check: ESP32 is connected via USB
- Check: Correct port is selected
- Try: Press and hold BOOT button during upload
- Try: Different USB cable

## Success Criteria

You'll know it's working when:
- ✅ `?` command shows help text
- ✅ `#1PING` gets a response from servo
- ✅ `#1P2047S1000` moves the servo and prints "OK"
- ✅ `#1P?` returns the current position
- ✅ Servo moves smoothly to commanded positions

## Next Steps After Success

1. Test all servos (IDs 1-6 for your gripper)
2. Integrate with ROS2 nodes
3. Test sensor commands (RD, RO, RC, SS)
4. Verify web interface still works
5. Document any servo ID mappings for your setup

## Need Help?

If you're still having issues:
1. Check `platform/servo_driver/SERIAL_CONTROL_GUIDE.md` for detailed troubleshooting
2. Verify servo power supply voltage (should be stable 6-12V)
3. Check servo UART wiring with multimeter
4. Test servos individually with web interface first
5. Review `platform/servo_driver/SERIAL_FIX_SUMMARY.md` for technical details

## Quick Reference

| Command | What it does |
|---------|--------------|
| `?` | Show help |
| `#1PING` | Check if servo 1 responds |
| `#1P2047S1000` | Move servo 1 to center |
| `#1P0S500` | Move servo 1 to 0° |
| `#1P4095S500` | Move servo 1 to 360° |
| `#1P?` | Read servo 1 position |
| `#1T0` | Disable servo 1 torque |
| `#1T1` | Enable servo 1 torque |
| `RD` | Read distance sensor |
| `RO` | Read IMU orientation |
| `SS` | Check sensor status |
