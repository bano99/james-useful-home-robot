# Teensy Semi-Reboot Integration Guide

## Overview
The semi-reboot feature allows you to trigger a Teensy firmware reset from the remote control while preserving joint positions and calibration state. This addresses firmware drift issues without requiring manual recalibration.

## Architecture

The semi-reboot command flows through the following chain:

```
Remote Control (ESP32)
    ↓ (ESP-NOW)
Platform Controller (ESP32)
    ↓ (USB Serial - Binary Protocol)
Platform Serial Bridge (ROS2 Python)
    ↓ (ROS2 Topic: /arm/teensy_raw_cmd)
Teensy Serial Bridge (ROS2 Python)
    ↓ (USB Serial - Text Protocol)
Teensy 4.1 (AR4 Controller)
```

## Hardware Setup

### Remote Control Button
- **Pin**: GPIO 42 (leftButtonPin)
- **Type**: Active LOW with internal pullup
- **Function**: Press to trigger semi-reboot

### Visual Feedback
When the button is pressed, the remote display shows:
```
┌──────────────────┐
│   REBOOTING      │
│   TEENSY...      │
└──────────────────┘
```

## Software Components

### 1. Remote Control Firmware
**File**: `platform/remote/remote_control_v2_simple/remote_control_v2_simple.ino`

**Changes**:
- Added `leftButtonPin` (GPIO 42) definition
- Added `trigger_reboot` field to `RemoteControlData` struct
- Implemented `handleSemiRebootButton()` function with debouncing
- Button state tracked with 200ms debounce

**Data Structure**:
```cpp
typedef struct RemoteControlData {
  int right_y, right_x, right_rot;
  int left_y, left_x, left_z;
  bool switch_platform_mode;
  int gripper_pot;
  bool armed;
  bool trigger_reboot;  // NEW: Semi-reboot trigger
} RemoteControlData;
```

### 2. Platform Controller Firmware
**File**: `platform/controller/plattform_controller/plattform_controller.ino`

**Changes**:
- Added `trigger_reboot` field to `RemoteControlData` struct
- Updated binary packet size from 22 to 23 bytes
- Added `trigger_reboot` to packet at byte 17

**Binary Packet Format** (23 bytes):
```
[0]    0xAA           - Start marker
[1]    0x01           - Message type (manual_control)
[2-3]  left_x         - int16_t (little endian)
[4-5]  left_y         - int16_t
[6-7]  left_z         - int16_t
[8-9]  right_x        - int16_t
[10-11] right_y       - int16_t
[12-13] right_rot     - int16_t
[14]   mode           - uint8_t (0=platform, 1=arm)
[15]   gripper_pot    - uint8_t
[16]   armed          - uint8_t (0=disarmed, 1=armed)
[17]   trigger_reboot - uint8_t (0=no, 1=yes)  ← NEW
[18-21] timestamp     - uint32_t (little endian)
[22]   checksum       - uint8_t
```

### 3. Platform Serial Bridge (ROS2)
**File**: `ros2_ws/src/james_manipulation/james_manipulation/platform_serial_bridge.py`

**Changes**:
- Updated `PACKET_SIZE` from 22 to 23 bytes
- Added publisher for `/arm/teensy_raw_cmd` topic
- Modified `handle_packet()` to extract and process `trigger_reboot` field
- Publishes "SR" command to teensy_raw_cmd topic when triggered

**Processing Logic**:
```python
trigger_reboot = packet[17]
if trigger_reboot == 1:
    self.get_logger().warn('⚠️ SEMI-REBOOT TRIGGERED FROM REMOTE ⚠️')
    reboot_msg = String()
    reboot_msg.data = 'SR'
    self.teensy_raw_cmd_pub.publish(reboot_msg)
```

### 4. Teensy Serial Bridge (ROS2)
**File**: `ros2_ws/src/james_manipulation/james_manipulation/teensy_serial_bridge.py`

**Existing Functionality**:
- Already subscribes to `/arm/teensy_raw_cmd` topic
- Forwards raw commands directly to Teensy via serial
- The "SR" command is passed through to the Teensy

**No changes needed** - the existing `raw_command_callback()` handles the SR command.

### 5. Teensy Firmware
**File**: `platform/teensy/AR4_teensy41_sketch_v6.3/AR4_teensy41_sketch_v6.3.ino`

**Changes**:
- Added `#include <EEPROM.h>`
- Implemented `savePositionsAndReset()` function
- Implemented `restorePositionsFromEEPROM()` function
- Added "SR" command handler in command processing loop
- Added restore call in `setup()`

**EEPROM Layout** (41 bytes):
```
Address  Size  Content
0        4     Magic number (0xAB12CD34)
4        4     J1StepM
8        4     J2StepM
12       4     J3StepM
16       4     J4StepM
20       4     J5StepM
24       4     J6StepM
28       4     J7StepM
32       4     J8StepM
36       4     J9StepM
40       1     Calibration flag
```

## Usage Instructions

### From Remote Control
1. Ensure the robot is stationary
2. Press the left button (GPIO 42) on the remote control
3. Display shows "REBOOTING TEENSY..."
4. Wait ~2 seconds for reboot to complete
5. Robot resumes operation with restored positions

### From ROS2 Command Line
You can also trigger the reboot manually:

```bash
# Publish SR command directly to Teensy
ros2 topic pub --once /arm/teensy_raw_cmd std_msgs/msg/String "data: 'SR'"
```

### From Python Script
```python
from std_msgs.msg import String

# In your ROS2 node
teensy_cmd_pub = self.create_publisher(String, '/arm/teensy_raw_cmd', 10)

# Trigger semi-reboot
msg = String()
msg.data = 'SR'
teensy_cmd_pub.publish(msg)
```

## Monitoring and Debugging

### Serial Monitor Output

**Remote Control**:
```
SEMI-REBOOT TRIGGERED!
```

**Platform Controller**:
```
(Binary packet with trigger_reboot=1 sent to Jetson)
```

**Platform Serial Bridge**:
```
[platform_serial_bridge]: ⚠️ SEMI-REBOOT TRIGGERED FROM REMOTE ⚠️
```

**Teensy Serial Bridge**:
```
[teensy_serial_bridge]: TX: SR
```

**Teensy**:
```
POSITIONS_SAVED_RESETTING
(System resets)
POSITIONS_RESTORED_FROM_EEPROM
```

### ROS2 Topic Monitoring

Monitor the command flow:
```bash
# Watch for semi-reboot commands
ros2 topic echo /arm/teensy_raw_cmd

# Monitor Teensy responses
ros2 topic echo /arm/teensy_raw_rx
```

## Safety Considerations

1. **Only trigger when stationary**: Do not press the button during motion
2. **Verify position**: Ensure robot is in a safe position before triggering
3. **Brief interruption**: Expect ~2 seconds of downtime during reset
4. **One-time restore**: Magic number is cleared after restore to prevent unintended reloads
5. **Normal boot preserved**: If no saved data exists, Teensy boots normally

## Troubleshooting

### Button Press Not Detected
- Check GPIO 42 connection
- Verify internal pullup is enabled
- Check debounce timing (200ms)
- Monitor serial output from remote

### Command Not Reaching Teensy
- Verify ESP-NOW connection between remote and platform controller
- Check USB serial connection between platform controller and Jetson
- Monitor ROS2 topics for command flow
- Check platform_serial_bridge is running

### Positions Not Restored
- Check for "POSITIONS_RESTORED_FROM_EEPROM" message on Teensy serial
- Verify EEPROM magic number (should be 0xAB12CD34 before restore)
- Ensure positions were saved before reset
- Check EEPROM library is included in Teensy firmware

### Robot Behavior After Restore
- Verify joint positions match pre-reboot state
- Check calibration flag is set
- Monitor joint state messages on `/joint_states`
- If positions are incorrect, perform full recalibration

## Testing Procedure

1. **Initial Setup**:
   - Calibrate robot normally
   - Move to a known position
   - Record joint positions

2. **Trigger Reboot**:
   - Press left button on remote
   - Observe display feedback
   - Wait for reboot completion

3. **Verify Restoration**:
   - Check Teensy serial for restore message
   - Compare joint positions to recorded values
   - Test robot motion
   - Verify no recalibration needed

4. **Stress Test**:
   - Perform multiple reboot cycles
   - Test at different joint positions
   - Verify EEPROM wear leveling (Teensy 4.1 has 100,000+ write cycles)

## Future Enhancements

Potential improvements:
- Add confirmation dialog on remote display
- Implement reboot counter in EEPROM
- Add automatic reboot on drift detection
- Store additional state (velocity profiles, tool offsets)
- Implement wear leveling for EEPROM writes
- Add reboot history logging

## Related Documentation

- [Teensy Semi-Reboot Guide](../platform/teensy/SEMI_REBOOT_GUIDE.md) - Detailed Teensy implementation
- [Remote Control Documentation](../platform/remote/README.md) - Remote control hardware and firmware
- [Platform Controller Documentation](../platform/controller/README.md) - Platform controller details
- [ROS2 Bridge Architecture](./hardware_interconnection.md) - System communication overview
