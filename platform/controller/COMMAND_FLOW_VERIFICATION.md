# Platform Controller Command Flow Verification

## Issue Summary
Platform wheels were not moving when switching from arm mode to platform mode, even though the controller was receiving commands.

## Root Causes Identified

### 1. Command Routing Issue (FIXED)
**Problem**: All joystick data (including right joystick) was being sent to Jetson regardless of mode, causing right joystick to control both platform AND arm simultaneously.

**Solution**: Filter right joystick data before sending to Jetson in platform mode:
```cpp
// In platform mode: Zero out right joystick before sending to Jetson
RemoteControlData armData = controlData;
armData.right_x = 0;
armData.right_y = 0;
armData.right_rot = 0;
sendArmDataToJetson();  // Send filtered data
```

### 2. Motor State Logic Bug (FIXED)
**Problem**: Contradictory motor state logic:
```cpp
// First set state to 1 when velocity is 0
motorFL.state = motorFL.velocity != 0 ? 8 : 1;
// Then immediately overwrite to 8 when velocity is 0 (BUG!)
if (motorFL.velocity == 0) motorFL.state = 8;
```

**Solution**: Always use state 8 (closed-loop velocity control):
```cpp
// State 8 handles both moving (velocity != 0) and holding (velocity = 0)
motorFL.state = 8;
motorFR.state = 8;
motorBL.state = 8;
motorBR.state = 8;
```

### 3. Late ODrive Initialization (FIXED)
**Problem**: If ODrive motors are powered on AFTER the controller boots:
- Controller sends state commands during boot
- ODrive is off, commands are lost
- When ODrive powers on later, controller thinks state is already set
- No new state commands are sent, motors don't respond

**Solution**: Periodic state re-initialization every 5 seconds:
```cpp
// Force state re-send every 5 seconds to catch late ODrive power-on
if ((currentTime - lastMotorStateInit) > 5000) {
  motorFL_state = -1;  // Reset tracking
  motorFR_state = -1;
  motorBL_state = -1;
  motorBR_state = -1;
  // Next sendMotorCommands() will re-send all state commands
}
```

## Complete Command Flow in Platform Mode

### When Remote Control Sends Data:
```
1. onDataReceive() receives RemoteControlData
   ├─ Updates lastManualCommandTime
   ├─ Updates remoteArmed state
   └─ Calls updateSafetyState()

2. Safety Check: Is system ARMED?
   ├─ NO → emergencyStop() + send data to Jetson for monitoring
   └─ YES → Continue to step 3

3. Mode Check: Platform or Arm mode?
   ├─ Platform Mode (switch_platform_mode = true):
   │  ├─ Create filtered data (zero right joystick)
   │  ├─ Send filtered data to Jetson
   │  ├─ Extract right joystick: joystickData.x/y/rot = controlData.right_x/y/rot
   │  ├─ Calculate movement: joystickValues = calculateMovement(x, y, rot)
   │  └─ Control wheels: controlMecanumWheels(joystickValues, motors...)
   │
   └─ Arm Mode (switch_platform_mode = false):
      ├─ Send FULL data to Jetson (all joysticks)
      ├─ Zero platform joystick: joystickData.x/y/rot = 0
      ├─ Calculate zero movement: joystickValues = calculateMovement(0, 0, 0)
      └─ Stop wheels: controlMecanumWheels(joystickValues, motors...)

4. controlMecanumWheels():
   ├─ Calculate mecanum kinematics (vx, vy, omega)
   ├─ Compute wheel velocities (FL, FR, BL, BR)
   ├─ Apply exponential mapping (0-255 → 0-3.00 m/s)
   ├─ Set all motor states to 8 (closed-loop velocity control)
   └─ Call sendMotorCommands()

5. sendMotorCommands():
   ├─ Check if 5 seconds elapsed → force state re-init
   ├─ For each motor:
   │  ├─ If state changed → send "w axisX.requested_state 8"
   │  └─ Always send "v X velocity 0" command
   └─ Print debug info to Serial
```

## Verification Checklist

✅ **Platform Mode Commands Reach Wheels**
- Line 810: `controlMecanumWheels(joystickValues, motorFL, motorFR, motorBL, motorBR);`
- This is called when `switch_platform_mode = true` and system is ARMED

✅ **Right Joystick Data Used for Platform**
- Line 803-805: `joystickData.x/y/rot = controlData.right_x/y/rot`

✅ **Right Joystick NOT Sent to Jetson in Platform Mode**
- Line 791-796: Right joystick zeroed before `sendArmDataToJetson()`

✅ **Motor Commands Actually Sent**
- Line 750: `sendMotorCommands(motorFL, motorFR, motorBL, motorBR);`
- Called at end of `controlMecanumWheels()`

✅ **ODrive State Commands Sent**
- Line 869-892: State commands sent via `iicSerial1/2.println()`
- Includes periodic re-initialization every 5 seconds

✅ **ODrive Velocity Commands Sent**
- Line 900-907: Velocity commands sent via `iicSerial1/2.println()`

## Debug Output to Monitor

When testing, watch Serial output for:
```
State: ARMED (armed by user)
FL State: w axis0.requested_state 8
FR State: w axis1.requested_state 8
BL State: w axis1.requested_state 8
BR State: w axis0.requested_state 8
FL:v 0 1.234 0 FR:v 1 1.234 0 BL:v 1 1.234 0 BR:v 0 1.234 0
```

Every 5 seconds you should see:
```
Forcing motor state re-initialization...
```

## Expected Behavior

1. **Boot Sequence**:
   - Controller boots → sends state 8 to all motors
   - If ODrive is off, commands are lost (OK, will retry)
   - Every 5 seconds, state commands are re-sent

2. **Platform Mode Operation**:
   - Move right joystick → platform wheels move
   - Left joystick data sent to Jetson for arm control
   - Right joystick data NOT sent to Jetson

3. **Arm Mode Operation**:
   - Move right joystick → arm moves (via Jetson)
   - Platform wheels stopped (velocity = 0)
   - All joystick data sent to Jetson

## Testing Steps

1. Power on controller (ODrive can be off)
2. Power on remote, arm the system
3. Switch to platform mode
4. Move right joystick → wheels should move
5. If not, wait 5 seconds for state re-init
6. Check Serial output for state commands
7. Switch to arm mode
8. Move right joystick → arm should move, wheels should stop
