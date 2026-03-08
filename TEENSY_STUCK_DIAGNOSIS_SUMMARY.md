# Teensy "Stuck" Issue - Root Cause Analysis

## Problem Statement
After ~10 minutes of operation, the Teensy stops responding to joystick commands from the remote control. The arm stops moving completely, but the system appears to be running normally.

## Diagnostic Process

### Initial Hypothesis (WRONG)
- Thought the remote control was failing
- Thought the serial bridge was failing
- Thought the Teensy was hung/crashed

### Key Findings

1. **Remote IS sending commands** ✅
   - Logs show remote sending data continuously
   - Platform controller receiving and forwarding commands
   
2. **Bridge IS forwarding commands** ✅
   - Bridge receives commands from controller
   - Bridge sends XJ commands to Teensy
   - Bridge can send RP and DS commands successfully
   
3. **Teensy IS responding** ✅
   - DS (diagnostic status) command works
   - RP (request position) command works
   - RJ (direct joint angle) commands work perfectly
   
4. **XJ commands FAIL** ❌
   - XJ commands get no response (rejected silently)
   - MJ commands return "ER" (kinematic error) - but this is a DIFFERENT issue

## Critical Discovery: XJ Commands Do NOT Use IK

**IMPORTANT:** After detailed code analysis, we discovered that XJ commands (joystick) do NOT use inverse kinematics at all!

### XJ Command Flow (line 5202 in firmware)

```cpp
if (function == "XJ") {
  // 1. Parse joint angles directly (A, B, C, D, E, F)
  // 2. Calculate step differences
  // 3. Check axis limits
  // 4. Call driveMotorsXJ() directly
  // 5. NEVER calls SolveInverseKinematics()
}
```

**XJ commands send JOINT ANGLES, not cartesian coordinates!**

This means:
- XJ commands bypass IK completely (like RJ commands)
- The IK solver is NOT causing the joystick to stop working
- The issue must be something else in the XJ command handler

## Real Root Cause: XJ Commands Being Silently Rejected

### Evidence from Code Analysis

**Line 5218 - Silent Rejection:**
```cpp
if (RmStart == -1) { inData = ""; return; }  // Silently reject if Rm missing!
```

If the XJ command is missing the `Rm` parameter, it's silently rejected with NO error message!

**Lines 5258-5263 - Axis Limit Checks:**
```cpp
if ((J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault) == 0) {
  // Execute movement
}
// If ANY axis fault, command is silently rejected!
```

### Possible Causes of Silent Rejection

1. **Missing Rm parameter** - XJ command truncated or malformed
2. **Axis limit exceeded** - Arm near joint limits, movement would exceed limits
3. **Command buffer corruption** - Serial buffer overflow truncating commands
4. **State machine issue** - Teensy in wrong state (though no explicit checks in XJ handler)

## Test Results Summary

### Working Commands
- **RJ (direct joint angles)** ✅ - Bypasses IK, works perfectly
- **DS (diagnostic status)** ✅ - Returns Teensy state
- **RP (request position)** ✅ - Returns current position

### Failing Commands
- **XJ (joystick blend)** ❌ - Silently rejected (no response)
- **MJ (move joint cartesian)** ❌ - Returns "ER" (different issue, uses IK)

### Test Case
**Current position:**
```
A-53.527 B23.874 C-37.704 D5.904 E5.887 F107.261
G264.529 H-357.120 I507.576 J84.677 K10.422 L49.148
```

**Failing XJ command example:**
```
XJA-53.53B23.88C-37.70D5.90E5.89F87.25Ss20Ac10Dc10Rm20
```
Result: No response (silently rejected)

**Working RJ command:**
```
RJA-53.53B23.88C-37.70D5.90E5.89F107.25J70.00J80.00J90.00Sp20Ac10Dc10Rm20W0Lm111111
```
Result: Movement executed successfully

## IK Solver Bug (Separate Issue)

During investigation, we discovered a potential bug in `SolveInverseKinematics()` (line ~1070):

```cpp
for (int i = -3; i <= 3; i++) {
  int success = inverse_kinematics_robot_xyzuvw<float>(target, joints, joints_estimate);
  if (success) {
    // Store valid solution
  } else {
    KinematicError = 1;  // ❌ Sets error even if other attempts succeed
  }
}
```

**The bug:** `KinematicError = 1` is set when ANY of the 7 IK attempts fails, even if other attempts succeed and produce valid solutions.

**Impact:** This causes MJ/ML commands (cartesian movements) to be rejected even when valid IK solutions exist.

**However:** This bug does NOT affect XJ commands because XJ commands don't use IK at all!

**Status:** Bug confirmed but NOT the cause of joystick failure. Attempted fix caused calibration to fail (reason unknown), so fix was reverted. Needs further investigation before applying fix.

## Diagnostic Tools Created

### 1. Teensy Firmware Changes
**File:** `platform/teensy/AR4_teensy41_sketch_v6.3/AR4_teensy41_sketch_v6.3.ino`

**Added:**
- DS command (Diagnostic Status) - reports Teensy internal state
- `freeMemory()` function - estimates available RAM
- ZERO changes to hot path (no performance impact)

**DS Command Output:**
```
--- TEENSY DIAGNOSTIC STATUS ---
CmdBuffer1: empty
CmdBuffer2: empty
CmdBuffer3: empty
SerialAvailable: 0
EstopActive: false
SplineTrue: false
BlendingEnabled: false
J1StepM: 5019
J2StepM: 4893
J3StepM: 5719
J4StepM: 15681
J5StepM: 3921
J6StepM: 11530
Uptime: 808139
FreeRAM: 65328
--- END STATUS ---
```

### 2. Python Diagnostic Script
**File:** `diagnose_teensy_state.py`

**Features:**
- Connects directly to Teensy (bridge must be stopped)
- Tests DS command (status query)
- Tests RP command (position query)
- Tests RJ command (direct joint movement)
- Saves timestamped JSON results
- Compares two states to identify differences
- Manual joint movement testing

**Usage:**
```bash
# Stop bridge first
ros2 lifecycle set /teensy_serial_bridge shutdown

# Test manual movement
python3 diagnose_teensy_state.py /dev/ttyACM1 --move-joint 6 --delta 10

# Full diagnostic
python3 diagnose_teensy_state.py /dev/ttyACM1

# Compare states
python3 diagnose_teensy_state.py --compare state1.json state2.json
```

### 3. Bridge Command Tester
**File:** `test_bridge_command.py`

**Usage:**
```bash
python3 test_bridge_command.py RP
python3 test_bridge_command.py DS
```

### 4. Connectivity Checker
**File:** `check_teensy_connection.py`

**Usage:**
```bash
python3 check_teensy_connection.py /dev/ttyACM1
```

## Solution Approaches

### Option 1: Add Debug Logging to XJ Handler (RECOMMENDED)
**Location:** Teensy firmware, XJ command handler (line 5202)

**Actions:**
1. Add logging when XJ command is received
2. Log if command is rejected due to missing Rm parameter
3. Log if command is rejected due to axis limits
4. Log actual axis positions and limits when rejection occurs
5. Send error response instead of silent rejection

**Example:**
```cpp
if (RmStart == -1) { 
  Serial.println("XJ_REJECTED:MISSING_RM");
  inData = ""; 
  return; 
}

if ((J1axisFault + J2axisFault + ... ) > 0) {
  Serial.println("XJ_REJECTED:AXIS_LIMIT");
  inData = "";
  return;
}
```

### Option 2: Investigate Command Format
**Location:** Bridge or controller code

**Actions:**
1. Capture exact XJ commands being sent when joystick stops working
2. Verify all required parameters are present (especially Rm)
3. Check for command truncation or corruption
4. Verify command format matches expected format

### Option 3: Check Axis Limits
**Location:** Teensy firmware or ROS2 controller

**Actions:**
1. When joystick stops working, check current joint positions
2. Compare against axis limits (J1StepLim, J2StepLim, etc.)
3. Check if arm is near limits where small movements would exceed limits
4. Add soft limits before hard limits to prevent getting stuck

### Option 4: Fix IK Solver Bug (For MJ/ML commands)
**Location:** Teensy firmware, `SolveInverseKinematics()` function (line ~1070)

**Actions:**
1. Remove `KinematicError = 1` from inside the loop
2. Only set error when `NumberOfSol == 0` (no solutions found)
3. Test thoroughly with calibration before deploying

**Note:** This fix does NOT affect XJ commands but will improve MJ/ML reliability.

**WARNING:** Initial fix attempt caused calibration to fail. Root cause unknown. Needs investigation before re-applying.

## Testing Procedure

### When Arm Gets "Stuck":

1. **Stop the bridge:**
   ```bash
   ros2 lifecycle set /teensy_serial_bridge shutdown
   ```

2. **Test connectivity:**
   ```bash
   python3 check_teensy_connection.py /dev/ttyACM1
   ```

3. **Get diagnostic status:**
   ```bash
   python3 test_bridge_command.py DS
   ```

4. **Test joint movement:**
   ```bash
   python3 diagnose_teensy_state.py /dev/ttyACM1 --move-joint 6 --delta 10
   ```

5. **If RJ works but MJ fails:**
   - Confirms IK failure
   - Arm is in problematic position
   - Need to move arm using RJ commands

## Key Takeaways

1. **XJ commands do NOT use IK** - They send joint angles directly, bypassing inverse kinematics
2. **XJ commands are being silently rejected** - No error message, just ignored
3. **Most likely cause: Missing Rm parameter or axis limit exceeded**
4. **RJ commands always work** - Same as XJ, they bypass IK
5. **IK solver has a separate bug** - Affects MJ/ML commands, but NOT XJ commands
6. **Issue is position-dependent** - Happens after arm moves to certain configurations
7. **Teensy is NOT crashed** - It responds to DS, RP, and RJ commands perfectly

## Next Steps (Priority Order)

### Immediate Actions

1. **Add debug logging to XJ handler** - Identify why commands are rejected
   - Log when XJ command received
   - Log rejection reasons (missing Rm, axis limits)
   - Send error responses instead of silent rejection

2. **Capture XJ commands when failure occurs** - Verify command format
   - Stop bridge when joystick stops working
   - Capture exact XJ command being sent
   - Check for missing parameters or truncation

3. **Check axis positions when failure occurs** - Verify not at limits
   - Use DS command to get current positions
   - Compare against axis limits
   - Check if small movements would exceed limits

### Secondary Actions

4. **Investigate IK solver bug** - For MJ/ML commands (not XJ)
   - Understand why fix caused calibration to fail
   - Test fix in isolation without affecting XJ commands
   - Consider alternative fix approaches

5. **Add soft limits** - Prevent getting stuck at hard limits
   - Add buffer zone before hard limits
   - Reject movements that would enter danger zone
   - Provide clear error messages

6. **Improve error reporting** - Replace silent rejections
   - All command rejections should send error response
   - Include reason for rejection
   - Help diagnose issues faster

## Files Modified

1. `platform/teensy/AR4_teensy41_sketch_v6.3/AR4_teensy41_sketch_v6.3.ino` - Added DS command
2. `diagnose_teensy_state.py` - Created diagnostic script
3. `test_bridge_command.py` - Created bridge command tester
4. `check_teensy_connection.py` - Created connectivity checker
5. `docs/teensy_diagnostic_guide.md` - Created usage documentation

## Important Notes

- **XJ commands use joint angles, NOT cartesian coordinates** - This was a critical misunderstanding
- **XJ commands do NOT use inverse kinematics** - They bypass IK completely like RJ commands
- **Silent rejection is the problem** - XJ handler rejects commands without error messages
- **Calibration works fine** - Uses LL command, not affected by IK or XJ handler
- **Direct joint commands work** - RJ bypasses IK completely (same as XJ)
- **Issue is position-dependent** - Only happens in certain configurations
- **Not a timing issue** - Happens consistently at same positions
- **Not a memory issue** - FreeRAM stable at 65328 bytes
- **Not a communication issue** - DS and RP commands work perfectly
- **IK solver bug is real but unrelated** - Affects MJ/ML commands, not XJ commands

## Code Locations Reference

### XJ Command Handler
- **File:** `AR4_teensy41_sketch_v6.3.ino`
- **Line:** 5202
- **Function:** Handles joystick blend commands
- **Key checks:**
  - Line 5218: Silent rejection if Rm parameter missing
  - Lines 5258-5263: Silent rejection if axis limits exceeded

### IK Solver (Separate Issue)
- **File:** `AR4_teensy41_sketch_v6.3.ino`
- **Line:** 1032 (function start), 1070 (bug location)
- **Function:** `SolveInverseKinematics()`
- **Bug:** Sets `KinematicError = 1` when ANY attempt fails, even if others succeed
- **Affects:** MJ, ML commands (NOT XJ commands)

### Diagnostic Commands
- **DS:** Line 2877 - Returns Teensy diagnostic status
- **RP:** Returns current position (uses `sendRobotPos()`)
- **RJ:** Direct joint movement (bypasses IK)

## Investigation History

### Phase 1: Initial Diagnosis
- Confirmed remote, controller, and bridge are working
- Confirmed Teensy is responding to some commands
- Identified XJ commands are failing

### Phase 2: IK Solver Investigation (WRONG PATH)
- Analyzed `SolveInverseKinematics()` function
- Found bug in IK solver (line 1070)
- Assumed XJ commands use IK (INCORRECT)
- Attempted fix caused calibration to fail
- Reverted fix

### Phase 3: XJ Command Analysis (CORRECT PATH)
- Discovered XJ commands do NOT use IK
- Found silent rejection points in XJ handler
- Identified missing Rm parameter check (line 5218)
- Identified axis limit checks (lines 5258-5263)
- Concluded: XJ commands are being silently rejected, not failing due to IK

## Conclusion

The joystick stops working because **XJ commands are being silently rejected** by the Teensy firmware, most likely due to:
1. Missing or truncated Rm parameter in the command
2. Axis limit checks failing when arm is near joint limits

The IK solver bug we found is real but unrelated to the joystick issue. It affects MJ/ML commands (cartesian movements) but NOT XJ commands (which use joint angles directly).

**Next agent should focus on:**
1. Adding debug logging to XJ handler to identify rejection reason
2. Capturing actual XJ commands when failure occurs
3. Checking axis positions and limits when failure occurs
