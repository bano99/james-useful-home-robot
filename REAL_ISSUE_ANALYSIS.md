# Real Issue Analysis - XJ Commands Stop Working

## Problem Statement
After ~10 minutes of joystick operation, XJ commands stop working. The arm stops moving.

## Key Finding from Code Analysis

**XJ commands do NOT use inverse kinematics!**

Looking at the XJ command handler (line 5202), it:
1. Parses joint angles directly (A, B, C, D, E, F)
2. Calculates step differences
3. Checks axis limits
4. Calls `driveMotorsXJ()` directly
5. **NEVER calls `SolveInverseKinematics()`**

This means the IK solver bug I found is NOT causing your joystick to stop working.

## What's Really Happening?

From your diagnosis summary:
- XJ commands get "no response (rejected silently)"
- The Teensy IS responding to other commands (DS, RP, RJ)
- RJ commands work fine

This suggests the XJ commands are being rejected BEFORE they reach the motor driver, likely due to:

1. **Buffer issues** - Command buffer full or corrupted
2. **State machine issue** - Teensy in wrong state (splineTrue, blendingEnabled, flag, etc.)
3. **Axis limit check failing** - One of the axis fault checks is triggering
4. **Command parsing issue** - XJ command format is malformed or truncated

## Investigation Steps

### 1. Check XJ Command Format

The XJ handler expects:
```
XJA<angle>B<angle>C<angle>D<angle>E<angle>F<angle>S<speed>Ac<acc>Dc<dec>Rm<ramp>
```

**Critical check at line 5218:**
```cpp
if (RmStart == -1) { inData = ""; return; }  // Silently reject if Rm missing!
```

If the XJ command is missing the `Rm` parameter, it's silently rejected!

### 2. Check Axis Limits

Lines 5258-5263 check axis limits. If ANY joint would exceed limits, the command is rejected.

### 3. Check State Flags

The XJ command has no explicit state checks, but `driveMotorsXJ()` might check:
- `estopActive`
- `splineTrue`
- `blendingEnabled`
- `flag`

## Recommended Diagnostic Approach

### Test 1: Check XJ Command Format
When joystick stops working, capture the exact XJ command being sent. Verify it has all required parameters, especially `Rm`.

### Test 2: Check Axis Positions
When joystick stops working, check if the arm is near axis limits. The XJ command might be rejected due to limit checks.

### Test 3: Check State Flags
Add logging to the XJ handler to see:
- Is the command reaching the handler?
- Is it being rejected by the `RmStart == -1` check?
- Is it being rejected by axis limit checks?
- Is `driveMotorsXJ()` being called?

### Test 4: Check Command Buffers
When joystick stops working, check:
- `cmdBuffer1`, `cmdBuffer2`, `cmdBuffer3` contents
- Are they full or corrupted?

## Hypothesis

The most likely cause is:

**XJ commands are missing the `Rm` parameter or are truncated, causing silent rejection at line 5218.**

This could happen if:
1. The bridge is not sending complete XJ commands
2. Serial buffer overflow is truncating commands
3. The controller is sending malformed commands

## Next Steps

1. Add debug logging to the XJ handler to see why commands are rejected
2. Capture the exact XJ command when joystick stops working
3. Check if the command has all required parameters
4. Check if axis limits are being exceeded

## The IK Bug

The IK bug I found IS real, but it's NOT causing your joystick issue because XJ commands don't use IK. The IK bug would affect:
- MJ commands (Move Joint with cartesian coordinates)
- ML commands (Move Linear)
- Any command that calls `SolveInverseKinematics()`

But NOT:
- XJ commands (use joint angles directly)
- RJ commands (use joint angles directly)
