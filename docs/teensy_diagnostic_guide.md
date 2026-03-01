# Teensy Diagnostic Guide

## Overview

This guide explains how to diagnose Teensy responsiveness issues using the diagnostic tools added to the firmware and the Python diagnostic script.

## Problem Description

Sometimes the Teensy stops executing movement commands from the remote control, even though:
- Serial bridge output continues to show pose updates
- No obvious errors are reported
- The system appears to be running

## Diagnostic Tools

### 1. Teensy STATUS Command (DS)

A new `DS` (Diagnostic Status) command has been added to the Teensy firmware that reports internal state without modifying the system.

**Command:** `DS\n`

**Response Format:**
```
--- TEENSY DIAGNOSTIC STATUS ---
LoopCounter: 123456
LastCommand: XJA0B0C0D0E0F10Ss20
LastCommandTime: 45678
TimeSinceLastCmd: 1234
XJCommandsReceived: 89
XJCommandsExecuted: 89
CmdBuffer1: empty
CmdBuffer2: empty
CmdBuffer3: empty
SerialAvailable: 0
EstopActive: false
SplineTrue: false
BlendingEnabled: true
J1StepM: 15000
J2StepM: 4667
J3StepM: 9111
J4StepM: 17911
J5StepM: 4589
J6StepM: 8000
FreeRAM: 245678
--- END STATUS ---
```

**Key Fields:**
- `LoopCounter`: Total loop iterations (should always increase)
- `LastCommand`: Last command received (first 20 chars)
- `LastCommandTime`: Timestamp when last command was received
- `TimeSinceLastCmd`: Milliseconds since last command
- `XJCommandsReceived`: Count of XJ (blending) commands received
- `XJCommandsExecuted`: Count of XJ commands executed (should match received)
- `CmdBuffer1/2/3`: Command buffer states
- `SerialAvailable`: Bytes waiting in serial buffer
- `EstopActive`: Emergency stop state
- `SplineTrue`: Spline mode active
- `BlendingEnabled`: Blending mode enabled
- `J1-J6StepM`: Current joint step positions
- `FreeRAM`: Available RAM in bytes

### 2. Python Diagnostic Script

**Location:** `diagnose_teensy_state.py`

The script performs automated diagnostics:
1. Queries STATUS command
2. Tests simple movement (J6 +10 degrees via MJ command)
3. Tests blending movement (J6 +10 degrees via XJ command)
4. Saves results to timestamped JSON file

## Usage Workflow

### Step 1: Baseline Capture (When Working)

After calibration, when the system is working correctly:

```bash
python diagnose_teensy_state.py /dev/ttyACM0
```

This creates a baseline file: `teensy_state_YYYYMMDD_HHMMSS.json`

### Step 2: Problem Capture (When Stuck)

When the Teensy stops responding to movement commands:

```bash
python diagnose_teensy_state.py /dev/ttyACM0
```

This creates a problem state file: `teensy_state_YYYYMMDD_HHMMSS.json`

### Step 3: Compare States

```bash
python diagnose_teensy_state.py --compare teensy_state_baseline.json teensy_state_problem.json
```

The comparison will show:
- STATUS field differences
- Movement test results
- Key findings and potential root causes

## Interpreting Results

### Scenario 1: XJCommandsReceived > XJCommandsExecuted

**Symptom:** Commands are being received but not executed

**Possible Causes:**
- Loop is stuck in driveMotorsXJ waiting for next command
- Buffer management issue
- Timing/synchronization problem

**Next Steps:**
- Check if `CmdBuffer1` has pending XJ command
- Check `BlendingEnabled` state
- Review driveMotorsXJ loop logic

### Scenario 2: LoopCounter Not Increasing

**Symptom:** Loop counter stays the same between captures

**Possible Causes:**
- Main loop is blocked/hung
- Infinite loop in movement function
- Deadlock condition

**Next Steps:**
- Check for blocking operations in loop
- Review interrupt handlers
- Check for infinite while loops

### Scenario 3: SerialAvailable > 0 But No Processing

**Symptom:** Data in serial buffer but not being processed

**Possible Causes:**
- Serial processing disabled
- Buffer overflow
- Command parsing failure

**Next Steps:**
- Check `processSerial()` function
- Review buffer size limits
- Check for malformed commands

### Scenario 4: FreeRAM Significantly Lower

**Symptom:** Available RAM decreased substantially

**Possible Causes:**
- Memory leak
- String concatenation issues
- Buffer growth

**Next Steps:**
- Review String usage in hot paths
- Check for memory allocation in loops
- Consider using fixed buffers

### Scenario 5: Movement Tests Fail

**Symptom:** Simple or blending movement tests don't execute

**Possible Causes:**
- Command format issue
- Position limits reached
- Motor driver problem

**Next Steps:**
- Check joint position limits
- Verify command format
- Test with manual serial commands

## Manual Testing

You can also send commands manually via serial terminal:

```bash
# Connect to Teensy
screen /dev/ttyACM0 115200

# Get status
DS

# Request position
RP

# Enable blending
BM1

# Send XJ command (J6 +10 degrees, adjust based on current position)
XJA0B0C0D0E0F10Ss20Ac10Dc10Rm20

# Disable blending
BM0
```

## Common Issues and Solutions

### Issue: Commands Stop After ~1 Minute

**Check:**
- `TimeSinceLastCmd` - if very large, commands aren't arriving
- `XJCommandsReceived` vs `XJCommandsExecuted` - if mismatch, execution is stuck
- `CmdBuffer1/2/3` - if full, buffer management issue

### Issue: Intermittent Response

**Check:**
- `LoopCounter` - should increase consistently
- `SerialAvailable` - if fluctuating wildly, serial timing issue
- `FreeRAM` - if decreasing over time, memory leak

### Issue: Blending Mode Specific

**Check:**
- `BlendingEnabled` state
- `XJCommandsReceived` vs `XJCommandsExecuted` mismatch
- `CmdBuffer1` for pending XJ command

## Firmware Changes Summary

**Added Global Variables:**
- `loopCounter` - tracks loop iterations
- `lastCommandTime` - timestamp of last command
- `lastCommand` - last command string (first 20 chars)
- `xjCommandsReceived` - count of XJ commands received
- `xjCommandsExecuted` - count of XJ commands executed

**Added Functions:**
- `freeMemory()` - estimates available RAM
- DS command handler - outputs diagnostic status

**Modified Functions:**
- `loop()` - increments counter, tracks commands
- `driveMotorsXJ()` - increments execution counter

All changes are non-intrusive and don't affect normal operation.
