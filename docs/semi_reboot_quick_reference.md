# Semi-Reboot Quick Reference

## Button Location
**Remote Control**: Left Button (GPIO 42)

## Data Flow
```
┌─────────────────────────────────────────────────────────────┐
│                    SEMI-REBOOT DATA FLOW                     │
└─────────────────────────────────────────────────────────────┘

1. USER PRESSES BUTTON
   ↓
   Remote Control (ESP32)
   - GPIO 42 goes LOW
   - handleSemiRebootButton() called
   - Sets controlData.trigger_reboot = true
   - Display shows "REBOOTING TEENSY..."

2. ESP-NOW TRANSMISSION
   ↓
   Platform Controller (ESP32)
   - Receives RemoteControlData via ESP-NOW
   - Packs trigger_reboot into byte 17 of binary packet
   - Sends 23-byte packet via USB Serial

3. ROS2 PROCESSING
   ↓
   Platform Serial Bridge (Python)
   - Receives binary packet
   - Extracts trigger_reboot from byte 17
   - If trigger_reboot == 1:
     * Publishes "SR" to /arm/teensy_raw_cmd

4. COMMAND FORWARDING
   ↓
   Teensy Serial Bridge (Python)
   - Receives "SR" from /arm/teensy_raw_cmd
   - Forwards "SR\n" to Teensy via USB Serial

5. TEENSY EXECUTION
   ↓
   Teensy 4.1 Firmware
   - Receives "SR" command
   - Calls savePositionsAndReset()
     * Saves J1-J9 positions to EEPROM
     * Saves calibration flag
     * Writes magic number (0xAB12CD34)
     * Sends "POSITIONS_SAVED_RESETTING"
     * Triggers reset: SCB_AIRCR = 0x05FA0004

6. REBOOT & RESTORE
   ↓
   Teensy 4.1 Firmware (after reset)
   - setup() calls restorePositionsFromEEPROM()
   - Checks for magic number
   - If found:
     * Restores J1-J9 positions
     * Restores calibration flag
     * Clears magic number
     * Sends "POSITIONS_RESTORED_FROM_EEPROM"
   - Continues normal operation
```

## Command Summary

| Component | Action | Data Format |
|-----------|--------|-------------|
| Remote | Button press | `trigger_reboot = true` |
| Platform Controller | Binary packet | Byte 17 = 0x01 |
| Platform Bridge | ROS2 message | `String: "SR"` |
| Teensy Bridge | Serial command | `"SR\n"` |
| Teensy | EEPROM + Reset | Magic + 9 joints + flag |

## Serial Messages

### Success Flow
```
Remote:   "SEMI-REBOOT TRIGGERED!"
Platform: (binary packet sent)
Bridge:   "⚠️ SEMI-REBOOT TRIGGERED FROM REMOTE ⚠️"
Teensy:   "POSITIONS_SAVED_RESETTING"
          (2 second pause)
Teensy:   "POSITIONS_RESTORED_FROM_EEPROM"
```

### Error Conditions
```
No magic number:
  - Teensy boots normally
  - No restore message
  - Requires calibration

EEPROM corruption:
  - Invalid joint values
  - Perform full recalibration
  - Check EEPROM health

Communication failure:
  - Check ESP-NOW connection
  - Verify USB serial cables
  - Monitor ROS2 topics
```

## Timing

| Phase | Duration | Notes |
|-------|----------|-------|
| Button debounce | 200ms | Prevents multiple triggers |
| ESP-NOW transmission | <10ms | Wireless latency |
| ROS2 processing | <50ms | Topic publish/subscribe |
| EEPROM write | ~10ms | Teensy 4.1 EEPROM |
| System reset | ~1.5s | Teensy bootloader + init |
| EEPROM read | ~5ms | Restore positions |
| **Total** | **~2s** | Complete reboot cycle |

## Pin Assignments

### Remote Control (T-Display S3 AMOLED)
```
GPIO 42 (leftButtonPin)
  ├─ Internal pullup enabled
  ├─ Active LOW
  └─ Debounce: 200ms
```

### Platform Controller
```
USB Serial → Jetson
  ├─ Baud: 115200
  ├─ Protocol: Binary (23 bytes)
  └─ Byte 17: trigger_reboot
```

### Teensy 4.1
```
USB Serial ← Jetson
  ├─ Baud: 115200
  ├─ Protocol: Text commands
  └─ Command: "SR\n"
```

## EEPROM Map

```
Address  Size  Content              Value
───────────────────────────────────────────
0x00     4     Magic Number         0xAB12CD34
0x04     4     J1StepM             (int32)
0x08     4     J2StepM             (int32)
0x0C     4     J3StepM             (int32)
0x10     4     J4StepM             (int32)
0x14     4     J5StepM             (int32)
0x18     4     J6StepM             (int32)
0x1C     4     J7StepM             (int32)
0x20     4     J8StepM             (int32)
0x24     4     J9StepM             (int32)
0x28     1     Calibration Flag    0x01
───────────────────────────────────────────
Total: 41 bytes
```

## ROS2 Topics

```
/arm/teensy_raw_cmd (std_msgs/String)
  ├─ Publisher: platform_serial_bridge
  ├─ Subscriber: teensy_serial_bridge
  └─ Message: "SR"

/arm/teensy_raw_rx (std_msgs/String)
  ├─ Publisher: teensy_serial_bridge
  └─ Messages: 
      - "POSITIONS_SAVED_RESETTING"
      - "POSITIONS_RESTORED_FROM_EEPROM"
```

## Testing Commands

```bash
# Monitor command flow
ros2 topic echo /arm/teensy_raw_cmd

# Monitor Teensy responses
ros2 topic echo /arm/teensy_raw_rx

# Manual trigger (for testing)
ros2 topic pub --once /arm/teensy_raw_cmd std_msgs/msg/String "data: 'SR'"

# Check bridge status
ros2 topic echo /platform_bridge/status
ros2 topic echo /teensy_bridge/status
```

## Troubleshooting Checklist

- [ ] Remote button connected to GPIO 42
- [ ] ESP-NOW connection active (check remote display)
- [ ] Platform controller powered and connected
- [ ] USB serial cables connected (Platform ↔ Jetson, Teensy ↔ Jetson)
- [ ] ROS2 bridges running (`ros2 node list`)
- [ ] Teensy firmware includes EEPROM library
- [ ] Robot is stationary before triggering
- [ ] Calibration completed before first use

## Quick Diagnostic

```bash
# 1. Check nodes are running
ros2 node list | grep bridge

# 2. Check topics exist
ros2 topic list | grep teensy

# 3. Test manual trigger
ros2 topic pub --once /arm/teensy_raw_cmd std_msgs/msg/String "data: 'SR'"

# 4. Monitor response (should see restore message within 2s)
ros2 topic echo /arm/teensy_raw_rx --once

# 5. Verify joint states restored
ros2 topic echo /joint_states --once
```
