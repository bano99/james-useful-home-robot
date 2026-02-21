# ServoDriver Quick Start Guide

## 🚀 Upload Firmware

**File:** `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino`

1. Open in Arduino IDE
2. Select board: "ESP32 Dev Module"
3. Upload to ESP32 ServoDriver board
4. Open Serial Monitor (115200 baud)

## 📝 Get MAC Address

Watch Serial Monitor for:
```
Gripper MAC: XX:XX:XX:XX:XX:XX
```
**Save this MAC address!** You'll need it for the remote control.

## 🔧 Configure Remote Control

1. Open `platform/remote/remote_control_v2_simple/remote_control_v2_simple.ino`
2. Find line 91:
   ```cpp
   uint8_t gripperBroadcastAddress[] = {0x14, 0x33, 0x5C, 0x25, 0x05, 0x78};
   ```
3. Replace with your Gripper MAC from above
4. Upload to remote control

## 🔧 Configure Gripper (Optional)

If you want status feedback to remote:

1. Get remote MAC from its Serial Monitor
2. Update line 10 in ServoDriver.ino:
   ```cpp
   uint8_t broadcastAddress[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX};
   ```
3. Re-upload firmware

## ✅ Test Connection

### Test 1: ESP-NOW Remote Control
1. Power on both devices
2. Move slider potentiometer on remote
3. Watch Serial Monitor on gripper:
   ```
   ESP-NOW: Move servo 1 to pos 2047 @ speed 1000
   ```
4. Gripper servo should move!

### Test 2: Serial Control
Send commands via Serial Monitor:
```
#1P2047S1000    → Move servo 1 to center
RD              → Read distance sensor
RO              → Read orientation
SS              → Sensor status
?               → Help
```

### Test 3: Dual Control
1. Move gripper with remote slider
2. While moving, send serial command: `RD`
3. Both should work simultaneously!

## 🎮 Control Methods

### Remote Control (Wireless)
- **Slider Potentiometer** (GPIO 16) → Gripper position
- **Range:** 0-4095 (fully open to fully closed)
- **Midpoint:** POT 2060 → Servo 2047
- **Connection indicator:** Green circle on display

### Serial Commands (USB/UART)

**Servo Commands:**
```
#<ID>P<pos>S<speed>    Move servo (e.g., #1P2047S1000)
#<ID>T<0|1>            Torque off/on
#<ID>M<0|3>            Mode: 0=servo, 3=motor
#<ID>PING              Ping servo
```

**Sensor Commands:**
```
RD        Read distance (mm)
RO        Read orientation (degrees)
RC        Read calibration status
SS        Sensor status
SCAN      Scan for servos
INFO      System info
?         Help
```

## 🔍 Troubleshooting

### No ESP-NOW Connection
- ✅ Check MAC addresses are correct
- ✅ Verify DEFAULT_ROLE = 2 in ServoDriver.ino
- ✅ Check both devices are powered on
- ✅ Look for "ESP-NOW: Enabled" in startup message

### Gripper Not Moving
- ✅ Check servo is connected (GPIO 18/19)
- ✅ Verify servo ID is 1
- ✅ Check servo power supply (7.4V)
- ✅ Send `#1PING` to test servo connection

### Sensor Not Working
- ✅ Check I2C connections (GPIO 16/17)
- ✅ Send `SS` to check sensor status
- ✅ Verify sensors are powered (3.3V)

### Serial Commands Not Working
- ✅ Check baud rate is 115200
- ✅ Verify SERIAL_FORWARDING = true
- ✅ Try simple command: `?`

## 📊 Expected Serial Output

```
Gripper MAC: 14:33:5C:25:05:78
MAC:14:33:5C:25:05:78
================================
Gripper Controller Ready
ESP-NOW: Enabled (Follower Mode)
Sensor Commands: RD, RO, RC, SS, SCAN, INFO, ?
Servo Commands: #<ID>P<pos>S<speed>, #<ID>T<0|1>, #<ID>M<0|3>, #<ID>PING
Example: #1P2047S1000 (move servo 1 to center)
================================

[When remote moves slider:]
ESP-NOW: Move servo 1 to pos 2047 @ speed 1000
ESP-NOW: Move servo 1 to pos 2100 @ speed 1000

[When you send serial command:]
#1P2047S1000
OK
```

## 🎯 Common Use Cases

### Manual Operation
Use remote control slider for real-time gripper control

### Autonomous Operation
Send serial commands from Jetson/PC for programmatic control

### Hybrid Operation
Use remote for manual positioning, then switch to autonomous control

### Debugging
Monitor ESP-NOW activity and sensor readings via Serial Monitor

## 📚 More Information

- **Full Documentation:** `docs/espnow_gripper_remote_connection.md`
- **Integration Details:** `platform/servo_driver/ESP_NOW_INTEGRATION.md`
- **Cleanup Guide:** `platform/servo_driver/CLEANUP_CHECKLIST.md`
- **Command Reference:** `platform/servo_driver/SERVO_COMMANDS.md`

## ✨ Features

✅ ESP-NOW wireless control
✅ Serial USB/UART control
✅ VL53L1X distance sensor
✅ BNO055 orientation sensor
✅ Dual control mode (simultaneous)
✅ Thread-safe operation
✅ Detailed debug output
✅ Web server interface
✅ RGB LED indicators

---

**Ready to go!** 🎉 Upload, configure MAC addresses, and start controlling your gripper!
