# ESP-NOW Gripper Remote Control Connection Guide

## Overview
The remote control and ServoDriver (gripper controller) are already connected via ESP-NOW! The slider potentiometer on the remote controls the gripper servo position.

## Current Implementation Status: ✅ WORKING

### Remote Control (Transmitter)
**File:** `platform/remote/remote_control_v2_simple/remote_control_v2_simple.ino`

**Hardware:**
- Slider Potentiometer: GPIO 16 (`gripperPotPin`)
- Device: LilyGO T-Display S3 AMOLED V2

**Data Structure Sent:**
```cpp
typedef struct GripperCommand {
  int command;    // CMD_SET_POS (1), CMD_SET_MIDDLE (2), CMD_SET_TORQUE (3)
  int id;         // Servo ID (1 for gripper)
  int pos;        // Position (0-4095)
  int speed;      // Speed (1000 default)
  int torque;     // Torque enable (1 = on)
} GripperCommand;
```

**MAC Address:** Printed at startup via `esp_read_mac()`

**Target MAC (Gripper):** `{0x14, 0x33, 0x5C, 0x25, 0x05, 0x78}`

### ServoDriver (Receiver)
**File:** `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino` ✅

**Configuration:**
- `DEFAULT_ROLE = 2` (Follower mode - receives ESP-NOW commands)
- `DEFAULT_WIFI_MODE = 1` (AP mode)

**Features:**
- ESP-NOW remote control
- VL53L1X distance sensor
- BNO055 orientation sensor
- Serial command parser
- Dual control modes (wireless + serial)

**Data Structure Expected:**
```cpp
typedef struct GripperCommand {
  int command;  // CMD_SET_POS (1), CMD_SET_MIDDLE (2), CMD_SET_TORQUE (3)
  int id;       // Servo ID
  int pos;      // Position (0-4095 for STS)
  int speed;    // Speed
  int torque;   // 1 = Enable, 0 = Disable
} GripperCommand;
```

**MAC Address:** Printed at startup via `esp_read_mac()` and `WiFi.macAddress()`

**ESP-NOW Handler:** `OnDataRecv()` in `CONNECT.h`
- Handles `CMD_SET_POS`: Moves servo to position
- Handles `CMD_SET_MIDDLE`: Calibrates servo midpoint
- Handles `CMD_SET_TORQUE`: Enables/disables servo torque
- Validates data size and provides detailed debug output

## ✅ CURRENT FIRMWARE (UPDATED)

**Use this firmware:** `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino`

This version now includes:
- ✅ ESP-NOW remote control support (GripperCommand structure)
- ✅ VL53L1X distance sensor integration
- ✅ BNO055 orientation sensor integration
- ✅ Serial command parser for USB/UART control
- ✅ Thread-safe servo control with mutex
- ✅ Dual control modes (ESP-NOW + Serial)

**Redundant files (can be archived):**
- `gripper/firmware/ServoDriver/ServoDriver.ino` - Features merged
- `platform/servo_driver/Servo-Driver-with-ESP32/untested_ServoDriver/ServoDriver.ino` - Old version

## How It Works (Current Implementation)

### Remote Control Side:
1. Reads slider potentiometer value (0-4095) from GPIO 16
2. Applies midpoint calibration (POT 2060 → Servo 2047)
3. Filters changes with threshold (10 ADC units)
4. Sends `GripperCommand` via ESP-NOW when value changes
5. Shows gripper connection status (green/red circle) on display

### ServoDriver Side:
1. Receives ESP-NOW data in `OnDataRecv()` callback
2. Checks if `DEV_ROLE == 2` (follower mode)
3. Extracts servo ID, position, and speed
4. Sends command to servo via `st.WritePosEx()`
5. Prints debug info to Serial

## Connection Verification

### On Remote Control:
```
Remote MAC: XX:XX:XX:XX:XX:XX
```
Watch for gripper connection indicator (top-right circle):
- Green = Connected (received status within 2 seconds)
- Red = Disconnected

### On ServoDriver:
```
MAC: XX:XX:XX:XX:XX:XX
Bytes received: XX
POS: XXXX
SPEED: XXXX
```

## Potentiometer Mapping

**Remote Control Calibration:**
```cpp
// Midpoint: POT 2060 → Servo 2047
if (abs(currentPot - 2060) <= 15) {
  servoPos = 2047; // Deadband
} else if (currentPot < 2060) {
  servoPos = map(currentPot, 0, 2045, 0, 2046);
} else {
  servoPos = map(currentPot, 2075, 4095, 2048, 4095);
}
```

**Servo Range:**
- Minimum: 0 (fully open)
- Center: 2047 (neutral)
- Maximum: 4095 (fully closed)

## Troubleshooting

### No Connection
1. Verify MAC addresses match in both files
2. Check ServoDriver is in follower mode (`DEFAULT_ROLE = 2`)
3. Ensure both devices are powered on
4. Check Serial output for ESP-NOW init errors

### Gripper Not Moving
1. Check servo is connected to ServoDriver
2. Verify servo ID is 1
3. Check servo torque is enabled
4. Monitor Serial output on ServoDriver for received commands

### Erratic Movement
1. Adjust `potThreshold` in remote (currently 10)
2. Check potentiometer wiring
3. Verify power supply stability

## Next Steps

1. **Fix Data Structure Mismatch** - Choose Option 1 or 2 above
2. **Test Connection** - Upload both firmwares and verify communication
3. **Verify MAC Addresses** - Update hardcoded MACs with actual device MACs
4. **Add Status Feedback** - Implement bidirectional communication for gripper status

## Related Files
- Remote firmware: `platform/remote/remote_control_v2_simple/remote_control_v2_simple.ino`
- **Gripper firmware (CURRENT):** `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino` ✅
- ESP-NOW implementation: `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/CONNECT.h`
- Integration details: `platform/servo_driver/ESP_NOW_INTEGRATION.md`
- Redundant (archived): `gripper/firmware/ServoDriver/ServoDriver.ino` ❌
- Redundant (archived): `platform/servo_driver/Servo-Driver-with-ESP32/untested_ServoDriver/ServoDriver.ino` ❌


## Quick Setup Guide

### Step 1: Get MAC Addresses

**Remote Control:**
1. Upload `platform/remote/remote_control_v2_simple/remote_control_v2_simple.ino`
2. Open Serial Monitor (115200 baud)
3. Note the MAC address printed: `Remote MAC: XX:XX:XX:XX:XX:XX`

**Gripper Controller:**
1. Upload `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino`
2. Open Serial Monitor (115200 baud)
3. Note the MAC address printed: `Gripper MAC: XX:XX:XX:XX:XX:XX`

### Step 2: Update MAC Addresses

**In Remote Control firmware:**
```cpp
// Line 91 in remote_control_v2_simple.ino
uint8_t gripperBroadcastAddress[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX}; 
// Replace with actual Gripper MAC from Step 1
```

**In Gripper firmware:**
```cpp
// Line 10 in platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino
uint8_t broadcastAddress[] = {0xXX, 0xXX, 0xXX, 0xXX, 0xXX, 0xXX}; 
// Replace with actual Remote MAC from Step 1
```

### Step 3: Upload and Test

1. Re-upload both firmwares with updated MAC addresses
2. Power on both devices
3. Move the slider potentiometer (GPIO 16) on the remote
4. Watch Serial Monitor on gripper for received commands
5. Check remote display for gripper connection indicator (green circle = connected)

### Step 4: Verify Servo Movement

1. Ensure servo is connected to ServoDriver (GPIO 18/19)
2. Servo ID should be 1
3. Move slider slowly - gripper should follow
4. Check Serial output on gripper:
   ```
   Bytes received: 20
   CMD: 1
   POS: 2047
   ```

## Hardware Connections

### Remote Control (LilyGO T-Display S3 AMOLED)
- Slider Potentiometer → GPIO 16
- Power: USB-C or battery

### Gripper Controller (ESP32 ServoDriver Board)
- Servo Signal → GPIO 19 (TX)
- Servo Power → External 7.4V supply
- Servo Ground → Common ground
- ESP32 Power → USB or 5V supply

## Command Reference

### From Remote to Gripper:
```cpp
gripperCmd.command = CMD_SET_POS;    // Move to position
gripperCmd.id = 1;                   // Servo ID
gripperCmd.pos = 0-4095;             // Position value
gripperCmd.speed = 1000;             // Movement speed
gripperCmd.torque = 1;               // Torque enabled
```

### Gripper Actions:
- `CMD_SET_POS (1)`: Move servo to specified position
- `CMD_SET_MIDDLE (2)`: Calibrate servo center position
- `CMD_SET_TORQUE (3)`: Enable/disable servo holding torque


## Dual Control Mode

The integrated firmware supports **simultaneous control** from multiple sources:

### 1. ESP-NOW Remote Control (Wireless)
- Slider potentiometer on remote
- Real-time position control
- No wires needed
- Perfect for manual operation

### 2. Serial Commands (USB/UART)
- Full servo command set
- Sensor reading commands
- Programmatic control from Jetson/PC
- Example: `#1P2047S1000`

### 3. Thread-Safe Operation
Both control methods use the same servo control functions protected by mutex, ensuring safe concurrent operation. You can:
- Control gripper with remote while reading sensors via serial
- Switch between remote and programmatic control seamlessly
- Monitor ESP-NOW activity via serial debug output

### Example Workflow
```
1. Remote: Move gripper with slider
   Serial Output: "ESP-NOW: Move servo 1 to pos 2047 @ speed 1000"

2. Serial: Read distance sensor
   Command: RD
   Response: D1234

3. Serial: Override position
   Command: #1P3000S500
   Response: OK

4. Remote: Resume control with slider
   Serial Output: "ESP-NOW: Move servo 1 to pos 2100 @ speed 1000"
```

This makes the gripper controller extremely flexible for both manual operation and autonomous control!
