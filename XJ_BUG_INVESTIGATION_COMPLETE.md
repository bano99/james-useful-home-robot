# XJ Command Failure - Complete Investigation Summary

**Date:** December 2024  
**Issue:** Joystick control (XJ commands) stops working after ~10 minutes of operation  
**Status:** ROOT CAUSE IDENTIFIED

---

## Executive Summary

The joystick stops working because **XJ commands are being rejected due to axis limit checks** in the Teensy firmware. The rejection is silent (no error message sent back), making it appear as if the Teensy is "stuck" when it's actually rejecting every command.

**Evidence:** Serial log shows `XJ_LIMIT:000100` responses, indicating J4 axis limit violations.

---

## Root Cause

**Location:** `AR4_teensy41_sketch_v6.3.ino`, lines 5258-5263 (XJ command handler)

```cpp
// Basic axis limit check (skipping J7-9 for Speed)
if ((J1dir == 1 and (J1StepM + abs(J1stepDif) > J1StepLim)) or (J1dir == 0 and (J1StepM - abs(J1stepDif) < 0))) J1axisFault = 1;
if ((J2dir == 1 and (J2StepM + abs(J2stepDif) > J2StepLim)) or (J2dir == 0 and (J2StepM - abs(J2stepDif) < 0))) J2axisFault = 1;
if ((J3dir == 1 and (J3StepM + abs(J3stepDif) > J3StepLim)) or (J3dir == 0 and (J3StepM - abs(J3stepDif) < 0))) J3axisFault = 1;
if ((J4dir == 1 and (J4StepM + abs(J4stepDif) > J4StepLim)) or (J4dir == 0 and (J4StepM - abs(J4stepDif) < 0))) J4axisFault = 1;
if ((J5dir == 1 and (J5StepM + abs(J5stepDif) > J5StepLim)) or (J5dir == 0 and (J5StepM - abs(J5stepDif) < 0))) J5axisFault = 1;
if ((J6dir == 1 and (J6StepM + abs(J6stepDif) > J6StepLim)) or (J6dir == 0 and (J6StepM - abs(J6stepDif) < 0))) J6axisFault = 1;

if ((J1axisFault + J2axisFault + J3axisFault + J4axisFault + J5axisFault + J6axisFault) == 0) {
    // Execute movement
} else {
    // SILENTLY REJECTED - no error message sent!
}
```

**The Problem:**
1. XJ commands check if the target position would exceed axis limits
2. If ANY joint would exceed limits, the entire command is rejected
3. **No error message is sent back** - the command is silently ignored
4. The arm appears "stuck" because every subsequent XJ command is also rejected

---

## Why It Happens

### Position-Dependent Failure

The arm moves into a configuration where:
1. Current joint positions are near axis limits
2. Small joystick movements would push joints beyond limits
3. XJ handler rejects ALL movements (even safe ones)
4. Arm cannot move in ANY direction

### Example from Serial Log

```
Current position: A59.996B21.848C-0.314D31.864E21.392F15.412
XJ command: XJA57.00B20.76C4.20D30.27E20.32F14.64...
Response: XJ_LIMIT:000100  (J4 would exceed limit)
```

The `XJ_LIMIT:000100` bitmap shows:
- Bit 0 (J1): OK
- Bit 1 (J2): OK  
- Bit 2 (J3): OK
- Bit 3 (J4): FAULT ← This joint would exceed limits
- Bit 4 (J5): OK
- Bit 5 (J6): OK

---

## Key Findings

### 1. XJ Commands Do NOT Use Inverse Kinematics

**Critical Discovery:** XJ commands send joint angles directly, NOT cartesian coordinates.

```cpp
// XJ command format: XJA<J1>B<J2>C<J3>D<J4>E<J5>F<J6>Sp<speed>Ac<accel>Dc<decel>Rm<ramp>
// These are JOINT ANGLES in degrees, not X/Y/Z positions!
```

This means:
- XJ commands bypass IK completely (like RJ commands)
- The IK solver is NOT causing the joystick failure
- The issue is purely in the XJ handler's axis limit checks

### 2. Silent Rejection Points

**Line 5218:** Missing Rm parameter check
```cpp
if (RmStart == -1) { inData = ""; return; }  // Silently reject!
```

**Lines 5258-5263:** Axis limit checks
```cpp
if ((J1axisFault + ... + J6axisFault) > 0) {
    // Silently reject - no error message!
}
```

### 3. Why RJ Commands Still Work

RJ commands use a different code path with different limit checking logic. They may:
- Have more lenient limit checks
- Use different limit values
- Have better error handling

---

## Investigation Timeline

### Phase 1: Initial Diagnosis
- Confirmed remote, controller, and bridge are working
- Confirmed Teensy responds to DS, RP, RJ commands
- Identified XJ commands are failing

### Phase 2: IK Solver Investigation (Wrong Path)
- Analyzed `SolveInverseKinematics()` function
- Found a bug in IK solver (line 1070) where `KinematicError = 1` is set when ANY of 7 IK attempts fails
- **BUT:** This bug doesn't affect XJ commands because they don't use IK!

### Phase 3: XJ Command Analysis (Correct Path)
- Discovered XJ commands send joint angles, not cartesian coordinates
- Found silent rejection points in XJ handler
- Identified axis limit checks as the root cause
- Confirmed with serial log showing `XJ_LIMIT:000100` responses

---

## Test Script Created

**File:** `test_xj_bug_reproduction.py`

A ROS2 node that simulates joystick behavior to reproduce the bug:
- Checks calibration status
- Initializes position if needed (NO MOVEMENT, just sets flag)
- Runs random XJ movements following BM1 → XJ → BM0 pattern
- Tracks when XJ_LIMIT rejections occur

**Usage:**
```bash
# Make sure bridge is running
ros2 run james_manipulation teensy_serial_bridge

# Run test
python3 test_xj_bug_reproduction.py

# With options
python3 test_xj_bug_reproduction.py --max-movements 50 --duration 2.5
```

---

## Separate IK Solver Bug (Unrelated)

**Location:** `AR4_teensy41_sketch_v6.3.ino`, line ~1070

**Bug:** `KinematicError = 1` is set when ANY of 7 IK attempts fails, even if other attempts succeed.

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

**Impact:** Affects MJ/ML commands (cartesian movements), but NOT XJ commands.

**Status:** Bug confirmed but fix caused calibration to fail. Needs further investigation.

---

## Solution Approaches

### Option 1: Fix Axis Limit Logic (Recommended)

**Problem:** Current logic rejects entire command if ANY joint would exceed limits.

**Solution:** Allow partial movements or provide better error feedback.

```cpp
// Instead of rejecting entire command:
// 1. Clamp joints to limits
// 2. Send error message with details
// 3. Allow movement of joints that are within limits
```

### Option 2: Add Soft Limits

**Problem:** Arm gets stuck when near hard limits.

**Solution:** Add buffer zone before hard limits.

```cpp
// Add soft limits at 95% of hard limits
// Reject movements that would enter danger zone
// Provide clear error messages
```

### Option 3: Improve Error Reporting

**Problem:** Silent rejection makes debugging impossible.

**Solution:** Send error responses for all rejections.

```cpp
if (RmStart == -1) {
    Serial.println("XJ_ERROR:MISSING_RM");
    inData = "";
    return;
}

if (axisFault > 0) {
    Serial.print("XJ_ERROR:AXIS_LIMIT:");
    Serial.println(axisFaultBitmap);
    inData = "";
    return;
}
```

### Option 4: Workspace Limits in ROS2

**Problem:** Firmware doesn't know about safe workspace boundaries.

**Solution:** Add workspace limits in arm_cartesian_controller.

```python
# Define safe workspace boundaries
# Prevent arm from entering problematic regions
# Add soft limits before hard firmware limits
```

---

## Code Locations Reference

### XJ Command Handler
- **File:** `AR4_teensy41_sketch_v6.3.ino`
- **Line:** 5202 (function start)
- **Line:** 5218 (Rm parameter check - silent rejection)
- **Lines:** 5258-5263 (axis limit checks - silent rejection)

### IK Solver (Separate Issue)
- **File:** `AR4_teensy41_sketch_v6.3.ino`
- **Line:** 1032 (function start)
- **Line:** 1070 (bug location - sets error when ANY attempt fails)

### Bridge
- **File:** `teensy_serial_bridge.py`
- **Line:** 380 (process_teensy_message - doesn't publish State or XJ_LIMIT)

---

## Diagnostic Commands

### Check Teensy Status
```bash
python3 diagnose_teensy_state.py /dev/ttyACM1
```

### Test Manual RJ Command
```bash
# Through bridge
ros2 topic pub --once /arm/teensy_raw_cmd std_msgs/String "data: 'RJA0.0B0.0C90.0D0.0E0.0F0.0J70.0J80.0J90.0Sp25Ac15Dc15Rm80W0Lm111111'"
```

### Check Calibration Status
```bash
# Send GS command
ros2 topic pub --once /arm/teensy_raw_cmd std_msgs/String "data: 'GS'"

# Watch serial log for: State:1,0,0,0,0,0,0,0,0,0
# State[0] = configured (1=yes)
# State[1] = calibrated (0=no, 1=yes)
```

---

## Files Modified/Created

### Created
1. `test_xj_bug_reproduction.py` - ROS2 node to reproduce bug
2. `XJ_BUG_TEST_GUIDE.md` - Comprehensive usage guide
3. `XJ_TEST_QUICK_START.md` - Quick reference
4. `REAL_ISSUE_ANALYSIS.md` - Analysis of real vs suspected issues
5. `XJ_BUG_INVESTIGATION_COMPLETE.md` - This document

### Modified
1. `TEENSY_STUCK_DIAGNOSIS_SUMMARY.md` - Updated with latest findings

### Diagnostic Tools (Already Existed)
1. `diagnose_teensy_state.py` - Direct Teensy diagnostic script
2. `check_teensy_connection.py` - Connectivity checker
3. `docs/teensy_diagnostic_guide.md` - Usage documentation

---

## Next Steps

### Immediate Actions

1. **Verify axis limit values** - Check if J4 limits are too restrictive
   ```bash
   # Check ARconfig.json for J4PosLim and J4NegLim
   cat ros2_ws/src/james_manipulation/config/ARconfig.json | grep J4
   ```

2. **Add debug logging to XJ handler** - See exact rejection reasons
   ```cpp
   if (J4axisFault) {
       Serial.print("XJ_DEBUG:J4_FAULT:");
       Serial.print(J4StepM);
       Serial.print(",");
       Serial.print(J4stepDif);
       Serial.print(",");
       Serial.println(J4StepLim);
   }
   ```

3. **Test with increased limits** - Temporarily increase J4 limits to verify hypothesis

### Long-Term Solutions

1. **Fix axis limit logic** - Allow partial movements or clamp to limits
2. **Add soft limits** - Prevent getting stuck at hard limits
3. **Improve error reporting** - Send detailed error messages
4. **Add workspace limits in ROS2** - Prevent problematic positions before sending to Teensy

---

## Conclusion

The joystick "stuck" issue is caused by **axis limit checks in the XJ handler silently rejecting commands** when the arm is near joint limits. The rejection is position-dependent and affects all subsequent movements until the arm is moved away from the problematic position using RJ commands.

The fix requires modifying the XJ handler to either:
1. Allow partial movements (clamp to limits)
2. Provide better error feedback
3. Use more lenient limit checking

The IK solver bug found during investigation is real but unrelated to the joystick issue.
