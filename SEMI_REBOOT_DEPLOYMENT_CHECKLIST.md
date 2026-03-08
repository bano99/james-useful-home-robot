# Semi-Reboot Deployment Checklist

## Pre-Deployment Verification

### Code Compilation
- [ ] Teensy firmware compiles without errors
- [ ] Remote control firmware compiles without errors
- [ ] Platform controller firmware compiles without errors
- [ ] Python files have no syntax errors

### Hardware Preparation
- [ ] Remote control button (GPIO 42) is accessible
- [ ] All USB serial cables are connected
- [ ] ESP-NOW communication is working
- [ ] Robot is calibrated and in known position

## Deployment Steps

### 1. Upload Teensy Firmware
```bash
# Using Arduino IDE or PlatformIO
# File: platform/teensy/AR4_teensy41_sketch_v6.3/AR4_teensy41_sketch_v6.3.ino
```
- [x] Upload firmware to Teensy 4.1
- [ ] Verify serial output shows startup messages
- [ ] Check for EEPROM library inclusion
- [ ] Confirm "SR" command is recognized

### 2. Upload Remote Control Firmware
```bash
# Using Arduino IDE
# File: platform/remote/remote_control_v2_simple/remote_control_v2_simple.ino
# Board: LilyGo T-Display-S3
```
- [x] Upload firmware to T-Display S3 AMOLED
- [ ] Verify display shows UI
- [ ] Test left button (GPIO 42) responds
- [ ] Check ESP-NOW connection indicator

### 3. Upload Platform Controller Firmware
```bash
# Using Arduino IDE
# File: platform/controller/plattform_controller/plattform_controller.ino
```
- [x] Upload firmware to platform controller ESP32
- [ ] Verify ESP-NOW reception from remote
- [ ] Check USB serial output to Jetson
- [ ] Confirm binary packet size is 23 bytes

### 4. Update ROS2 Nodes
```bash
cd ~/ros2_ws
colcon build --packages-select james_manipulation
source install/setup.bash
```
- [ ] Build ROS2 workspace
- [ ] No build errors
- [ ] Source the workspace
- [ ] Restart ROS2 nodes

### 5. Restart ROS2 Bridges
```bash
# Terminal 1: Platform Bridge
ros2 run james_manipulation platform_serial_bridge

# Terminal 2: Teensy Bridge
ros2 run james_manipulation teensy_serial_bridge
```
- [ ] Platform bridge starts successfully
- [ ] Teensy bridge starts successfully
- [ ] Both bridges show "CONNECTED" status
- [ ] No error messages in logs

## Functional Testing

### Test 1: Manual ROS2 Command
```bash
# Send SR command directly
ros2 topic pub --once /arm/teensy_raw_cmd std_msgs/msg/String "data: 'SR'"

# Monitor response
ros2 topic echo /arm/teensy_raw_rx
```
**Expected Output**:
```
data: 'POSITIONS_SAVED_RESETTING'
(2 second pause)
data: 'POSITIONS_RESTORED_FROM_EEPROM'
```
- [ ] Command received by Teensy
- [ ] Positions saved message appears
- [ ] System resets
- [ ] Positions restored message appears
- [ ] Joint states match pre-reboot values

### Test 2: Remote Button Press
```bash
# Monitor topics while pressing button
ros2 topic echo /arm/teensy_raw_cmd &
ros2 topic echo /arm/teensy_raw_rx &
```
**Steps**:
1. Press left button (GPIO 42) on remote
2. Observe display shows "REBOOTING TEENSY..."
3. Wait for reboot completion

**Verify**:
- [ ] Button press detected
- [ ] Display shows feedback
- [ ] SR command published to topic
- [ ] Teensy receives command
- [ ] Positions saved and restored
- [ ] Robot resumes normal operation

### Test 3: Position Preservation
**Steps**:
1. Move robot to specific position
2. Record joint angles: `ros2 topic echo /joint_states --once`
3. Trigger semi-reboot
4. Compare joint angles after reboot

**Verify**:
- [ ] Joint 1 position matches (±0.1°)
- [ ] Joint 2 position matches (±0.1°)
- [ ] Joint 3 position matches (±0.1°)
- [ ] Joint 4 position matches (±0.1°)
- [ ] Joint 5 position matches (±0.1°)
- [ ] Joint 6 position matches (±0.1°)
- [ ] Calibration flag preserved
- [ ] No recalibration needed

### Test 4: Multiple Reboot Cycles
**Steps**:
1. Perform 5 consecutive reboots
2. Monitor for any degradation

**Verify**:
- [ ] All 5 reboots successful
- [ ] No EEPROM errors
- [ ] Positions remain accurate
- [ ] No memory leaks
- [ ] Consistent timing (~2s per reboot)

### Test 5: Error Conditions
**Test 5a: No Saved Data**
1. Clear EEPROM (upload fresh firmware)
2. Trigger reboot without saving
3. Verify normal boot (no restore)

**Test 5b: Communication Failure**
1. Disconnect USB cable during reboot
2. Verify graceful handling
3. Reconnect and verify recovery

**Test 5c: Invalid EEPROM Data**
1. Corrupt magic number
2. Trigger reboot
3. Verify normal boot (no restore)

## Integration Testing

### Test 6: Full System Operation
**Scenario**: Normal robot operation with periodic reboots

**Steps**:
1. Calibrate robot
2. Perform several movements
3. Trigger semi-reboot
4. Continue movements
5. Repeat 3-4 multiple times

**Verify**:
- [ ] No position drift accumulation
- [ ] Smooth operation after each reboot
- [ ] No communication errors
- [ ] All subsystems remain functional

### Test 7: Concurrent Operations
**Scenario**: Reboot during active ROS2 operations

**Steps**:
1. Start MoveIt planning
2. Trigger semi-reboot mid-operation
3. Verify system recovery

**Verify**:
- [ ] Reboot interrupts operation safely
- [ ] No crashes or hangs
- [ ] System recovers after reboot
- [ ] Can resume operations

## Performance Verification

### Timing Measurements
- [ ] Button press to ESP-NOW: < 10ms
- [ ] ESP-NOW to USB Serial: < 20ms
- [ ] USB Serial to ROS2: < 50ms
- [ ] ROS2 to Teensy: < 50ms
- [ ] EEPROM write: < 15ms
- [ ] System reset: ~1.5s
- [ ] EEPROM read: < 10ms
- [ ] Total reboot time: < 2s

### Resource Usage
- [ ] EEPROM usage: 41 bytes
- [ ] RAM usage: Minimal increase
- [ ] CPU usage: No significant impact
- [ ] Network bandwidth: Negligible

## Safety Verification

### Safety Checks
- [ ] Robot stops during reboot
- [ ] No unexpected movements
- [ ] Emergency stop still functional
- [ ] Arming state preserved correctly
- [ ] Collision detection active after reboot

### Edge Cases
- [ ] Reboot during motion (should stop safely)
- [ ] Reboot while disarmed (should preserve state)
- [ ] Reboot with gripper active (should handle gracefully)
- [ ] Multiple rapid button presses (debouncing works)
- [ ] Reboot during calibration (should handle safely)

## Documentation Review

### User Documentation
- [ ] SEMI_REBOOT_GUIDE.md is complete
- [ ] teensy_semi_reboot_integration.md is accurate
- [ ] semi_reboot_quick_reference.md is helpful
- [ ] Architecture diagram is clear
- [ ] All examples work as documented

### Code Documentation
- [ ] Functions have clear comments
- [ ] EEPROM layout documented
- [ ] Packet format documented
- [ ] Error handling explained
- [ ] Magic number purpose clear

## Monitoring Setup

### Log Files
- [ ] Teensy serial logs enabled
- [ ] ROS2 bridge logs enabled
- [ ] Remote control logs enabled
- [ ] Platform controller logs enabled

### ROS2 Topics
- [ ] /arm/teensy_raw_cmd monitored
- [ ] /arm/teensy_raw_rx monitored
- [ ] /joint_states monitored
- [ ] /platform_bridge/status monitored
- [ ] /teensy_bridge/status monitored

### Diagnostic Tools
```bash
# Create monitoring script
cat > monitor_reboot.sh << 'EOF'
#!/bin/bash
echo "Monitoring Semi-Reboot System..."
ros2 topic echo /arm/teensy_raw_cmd &
ros2 topic echo /arm/teensy_raw_rx &
ros2 topic echo /joint_states --field position &
wait
EOF
chmod +x monitor_reboot.sh
```
- [ ] Monitoring script created
- [ ] Script tested and working

## Rollback Plan

### If Issues Occur
1. **Revert Teensy Firmware**
   - Upload previous version without SR command
   - Verify basic operation

2. **Revert Remote Firmware**
   - Upload previous version without button handler
   - Verify ESP-NOW communication

3. **Revert Platform Controller**
   - Upload previous version with 22-byte packets
   - Verify USB serial communication

4. **Revert ROS2 Nodes**
   ```bash
   cd ~/ros2_ws
   git checkout HEAD~1 src/james_manipulation/james_manipulation/platform_serial_bridge.py
   colcon build --packages-select james_manipulation
   ```

### Backup Files
- [ ] Previous Teensy firmware backed up
- [ ] Previous remote firmware backed up
- [ ] Previous platform firmware backed up
- [ ] Previous ROS2 code backed up

## Sign-Off

### Deployment Approval
- [ ] All tests passed
- [ ] Documentation complete
- [ ] Team trained on feature
- [ ] Rollback plan ready
- [ ] Monitoring in place

**Deployed By**: ___________________  
**Date**: ___________________  
**Version**: v6.3 + Semi-Reboot  

### Post-Deployment
- [ ] Monitor for 24 hours
- [ ] Collect user feedback
- [ ] Document any issues
- [ ] Update documentation as needed
- [ ] Plan future enhancements

## Support Contacts

**For Issues**:
1. Check troubleshooting guide
2. Review ROS2 topic outputs
3. Check serial monitor logs
4. Verify EEPROM state
5. Test with manual command first

**Emergency Rollback**:
- Use backup firmware versions
- Revert ROS2 code changes
- Restart all nodes
- Verify basic operation

---

**Notes**:
- Keep this checklist for future deployments
- Update based on lessons learned
- Share feedback with team
- Document any modifications
