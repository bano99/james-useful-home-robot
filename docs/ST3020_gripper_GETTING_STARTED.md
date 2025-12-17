# ST3020 Gripper Integration - Getting Started

## Quick Start Guide

This guide will help you start working on the gripper integration **before the hardware arrives**.

---

## What We Have

✅ **Waveshare ESP32 Servo Driver Board** - Ordered, waiting for delivery
✅ **ST3020 Servo** - Ordered, waiting for delivery
✅ **VL53L1X ToF Sensor** - Need to verify if available
✅ **BNO055 IMU Sensor** - Need to verify if available
✅ **Spare ESP32** - Available for testing NOW
✅ **Waveshare Firmware Source Code** - Available at `platform/servo_driver/Servo-Driver-with-ESP32/`
✅ **Remote Control Code** - Available at `platform/remote/`
✅ **Platform Controller Code** - Available at `platform/controller/`

---

## What We Can Do NOW (Before Hardware Arrives)

### Priority 1: Modify Gripper Firmware ⭐⭐⭐

**Goal**: Extend Waveshare firmware with sensor support and torque control

**Location**: `platform/servo_driver/`

**Steps**:
1. Copy `ServoDriver/` to `ServoDriver_Gripper/`
2. Add sensor support (VL53L1X, BNO055)
3. Add 5-level torque control
4. Test on spare ESP32

**Estimated Time**: 3-4 days

---

### Priority 2: Extend Remote Control ⭐⭐

**Goal**: Add gripper control to existing remote control

**Location**: `platform/remote/remote_control_v2_amoled/` (or v2_simple)

**Steps**:
1. Analyze current remote control code
2. Add gripper control variables
3. Add button mapping for gripper
4. Update ESPNOW message structure
5. Add gripper status to display

**Estimated Time**: 2-3 days

---

### Priority 3: Create Test Tools ⭐

**Goal**: Python scripts to test serial communication

**Location**: `platform/gripper/test_scripts/` (new)

**Steps**:
1. Create serial command test script
2. Create protocol documentation
3. Test command parsing

**Estimated Time**: 1-2 days

---

## Step-by-Step: Start with Firmware

### Step 1: Set Up Working Directory

```bash
cd platform/servo_driver
cp -r Servo-Driver-with-ESP32/ServoDriver ServoDriver_Gripper
cd ServoDriver_Gripper
```

### Step 2: Create New Header Files

We'll create three new files:

1. **SENSOR_CTRL.h** - Sensor initialization and reading
2. **TORQUE_CTRL.h** - 5-level torque control
3. **README.md** - Documentation of changes

### Step 3: Modify Main Sketch

Edit `ServoDriver_Gripper.ino` to include new headers and functionality.

---

## File Structure After Modifications

```
platform/servo_driver/
├── Servo-Driver-with-ESP32/          # Original (keep untouched)
│   ├── ServoDriver/
│   └── SCServo/
└── ServoDriver_Gripper/              # Modified version (NEW)
    ├── ServoDriver_Gripper.ino       # Main sketch (MODIFIED)
    ├── STSCTRL.h                     # Servo control (EXISTING)
    ├── SENSOR_CTRL.h                 # Sensor control (NEW)
    ├── TORQUE_CTRL.h                 # Torque control (NEW)
    ├── CONNECT.h                     # Communication (EXISTING)
    ├── BOARD_DEV.h                   # Board peripherals (EXISTING)
    ├── WEBPAGE.h                     # Web interface (EXISTING)
    ├── RGB_CTRL.h                    # RGB LED (EXISTING)
    ├── PreferencesConfig.h           # Config (EXISTING)
    └── README.md                     # Documentation (NEW)
```

---

## Command Protocol Design

### Existing Commands (Waveshare)
- Web interface control
- ESP-NOW wireless control
- USB serial commands for servo

### New Commands (To Add)

#### Sensor Commands
```
RD\n          - Read Distance
              Response: D<mm>\n or ERROR\n
              Example: D245\n (245mm)

RO\n          - Read Orientation
              Response: OX<x>Y<y>Z<z>\n or ERROR\n
              Example: OX45.23Y12.45Z180.00\n

RC\n          - Read Calibration (BNO055)
              Response: CS<sys>G<gyro>A<accel>M<mag>\n or ERROR\n
              Example: CS3G3A3M2\n
```

#### Torque Commands
```
TL<1-5>\n     - Set Torque Level (1=very light, 5=maximum)
              Response: OK\n or ERROR\n
              Example: TL3\n → OK\n

GT\n          - Get Torque Level
              Response: T<level>\n
              Example: T3\n
```

#### Gripper Commands
```
GO\n          - Gripper Open
              Response: OK\n or ERROR\n

GC\n          - Gripper Close
              Response: OK\n or ERROR\n

GP<pos>\n     - Gripper Position (0-4095)
              Response: OK\n or ERROR\n
              Example: GP2000\n

GS\n          - Gripper Status
              Response: P<pos>L<load>T<torque>\n
              Example: P2000L450T3\n
```

---

## Remote Control Integration

### Current Remote Control Versions

1. **v1** - Basic version
2. **v2_amoled** - Full LVGL graphics (complex)
3. **v2_simple** - Simple TFT_eSPI graphics (recommended)

**Recommendation**: Start with `v2_simple` for easier modification.

### ESPNOW Message Structure

**Current** (Platform Control):
```cpp
struct struct_message {
  int x;      // Forward/backward
  int y;      // Left/right
  int rot;    // Rotation
};
```

**Extended** (Add Gripper):
```cpp
struct struct_message {
  int x;           // Forward/backward
  int y;           // Left/right
  int rot;         // Rotation
  int gripper;     // Gripper position (0-4095)
  int torque;      // Torque level (1-5)
  bool grip_open;  // Open command
  bool grip_close; // Close command
};
```

### Button Mapping Ideas

**Option 1: Use existing buttons**
- Button 1: Gripper open
- Button 2: Gripper close
- Long press Button 1: Decrease torque
- Long press Button 2: Increase torque

**Option 2: Use touchscreen**
- Touch zones on AMOLED display
- Slider for gripper position
- Buttons for torque level

---

## Testing Without Hardware

### Test 1: Firmware Compilation
```bash
# Open Arduino IDE
# Select Board: "ESP32 Dev Module"
# Open ServoDriver_Gripper.ino
# Click Verify (compile only)
# Should compile without errors
```

### Test 2: Upload to Spare ESP32
```bash
# Connect spare ESP32 via USB
# Select correct COM port
# Upload firmware
# Open Serial Monitor (115200 baud)
# Test commands (will return ERROR without sensors - OK)
```

### Test 3: Command Testing
```python
# test_serial_commands.py
import serial
import time

ser = serial.Serial('COM3', 115200)  # Adjust COM port
time.sleep(2)

# Test sensor commands (will return ERROR - expected)
ser.write(b'RD\n')
print(ser.readline())  # Should return: ERROR\n

ser.write(b'RO\n')
print(ser.readline())  # Should return: ERROR\n

# Test torque commands (should work)
ser.write(b'TL3\n')
print(ser.readline())  # Should return: OK\n

ser.write(b'GT\n')
print(ser.readline())  # Should return: T3\n
```

---

## Pin Assignment Reference

### Waveshare ESP32 Servo Driver Board

| Pin | Current Use | New Use | Notes |
|-----|-------------|---------|-------|
| GPIO18 | Servo RX | ST3020 RX | Keep as is |
| GPIO19 | Servo TX | ST3020 TX | Keep as is |
| GPIO21 | I2C SDA (OLED) | I2C SDA (OLED + VL53L1X + BNO055) | Shared |
| GPIO22 | I2C SCL (OLED) | I2C SCL (OLED + VL53L1X + BNO055) | Shared |
| GPIO23 | RGB LED | RGB LED | Keep as is |
| GPIO32 | Available | Status LED (optional) | New |
| GPIO33 | Available | Reserved | New |

### I2C Address Map

| Device | Address | Notes |
|--------|---------|-------|
| OLED | 0x3C | Existing |
| VL53L1X | 0x30 | After address change from 0x29 |
| BNO055 | 0x29 | Default |
| PCA9685 | 0x40 | Unused but present |

**No conflicts!** ✅

---

## Next Steps

### Immediate Actions (Today)

1. ✅ Read this guide
2. ✅ Review TODO list (`ST3020_gripper_integration_TODO.md`)
3. ✅ Review analysis document (`ST3020_gripper_integration_analysis.md`)
4. ⬜ Decide which tasks to start with

### Recommended Starting Point

**Start with Task 1.1.2: Add Sensor Support to Firmware**

This is the foundation for everything else. Once sensors are integrated, torque control and remote control will be easier to add.

---

## Questions Before Starting

1. **Which remote control version are you currently using?**
   - v1, v2_amoled, or v2_simple?

2. **Do you have VL53L1X and BNO055 sensors available now?**
   - If yes, we can test sensor integration immediately
   - If no, we can still prepare the code

3. **Which spare ESP32 board do you have?**
   - ESP32-DevKit, LilyGO, or other?
   - Need to know for pin compatibility

4. **Do you want to start with firmware or remote control first?**
   - Firmware is recommended (foundation)
   - But we can do both in parallel

---

## Ready to Start?

Let me know which task you'd like to tackle first, and I'll guide you through it step by step!

**Recommended order**:
1. Create SENSOR_CTRL.h
2. Create TORQUE_CTRL.h
3. Modify ServoDriver_Gripper.ino
4. Test on spare ESP32
5. Extend remote control
6. Create test scripts

---

**Last Updated**: December 6, 2025
**Status**: Ready to begin
**Next Action**: Choose starting task and let's code! 🚀
