# XJ Bug Reproduction Test Guide

## Overview

This script simulates joystick behavior to reproduce the "stuck" bug where XJ commands stop working after some time.

## Setup

### 1. Position the Arm

Manually position the arm approximately at:
- J1 = 0° (base centered)
- J2 = 0° (shoulder at reference)
- J3 = 90° (elbow at 90°)
- J4 = 0° (wrist roll centered)
- J5 = 0° (wrist pitch centered)
- J6 = 0° (wrist yaw centered)

**This doesn't need to be exact** - the UP command will set the Teensy's internal position to these values.

### 2. Stop the Bridge

If the ROS2 bridge is running, stop it:
```bash
ros2 lifecycle set /teensy_serial_bridge shutdown
```

### 3. Find the Teensy Port

```bash
ls /dev/ttyACM*
```

Usually `/dev/ttyACM0` or `/dev/ttyACM1`

## Running the Test

### Basic Usage

```bash
python3 test_xj_bug_reproduction.py /dev/ttyACM1
```

### With Options

```bash
# Run for 50 movements then stop
python3 test_xj_bug_reproduction.py /dev/ttyACM1 --max-movements 50

# Use 3-second movements instead of 2 seconds
python3 test_xj_bug_reproduction.py /dev/ttyACM1 --duration 3.0

# Combine options
python3 test_xj_bug_reproduction.py /dev/ttyACM1 --max-movements 100 --duration 2.5
```

## What the Script Does

### Initialization Phase

1. Connects to Teensy via serial
2. Sends UP command to set position to J1=0°, J2=0°, J3=90°, J4-6=0°
3. Verifies connection with RP (request position) command

### Movement Phase

Each movement follows this pattern (matching real joystick behavior):

```
1. BM1 (Enable Blending Mode)
   ↓
2. XJ commands (every 100ms for 2 seconds)
   - Interpolates from current to target position
   - Simulates continuous joystick input
   ↓
3. BM0 (Disable Blending Mode)
   ↓
4. ST (Stop command)
   ↓
5. RP (Verify position)
```

### Movement Types

The script randomly chooses one of four movement types:

- **Left**: J1-, J2+, J3- (primary: J1 ±10°±2°)
- **Right**: J1+, J2-, J3+ (primary: J1 ±10°±2°)
- **Forward**: J2+, J3- (primary: J2 ±10°±2°)
- **Backward**: J2-, J3+ (primary: J2 ±10°±2°)

### Safety Limits

- Maximum deviation from init position: ±15° for all joints
- If movement would exceed limits, returns to home position instead

## Expected Behavior

### If Bug Occurs

The script will show:
```
→ XJA-5.23B3.45C88.12D0.00E0.00F0.00Sp3.00Ac10.00Dc10.00Rm10.00
⚠ No response (timeout after 2s)
```

Or XJ commands will be silently ignored (no movement).

### Normal Operation

```
→ XJA-5.23B3.45C88.12D0.00E0.00F0.00Sp3.00Ac10.00Dc10.00Rm10.00
→ BM0
→ ST
→ RP
← A-5.230B3.450C88.120D0.000E0.000F0.000G...
```

## Monitoring

### Watch for These Signs

1. **Silent rejection**: XJ commands sent but no movement
2. **Timeout**: RP command doesn't respond
3. **Position mismatch**: Actual position doesn't match target
4. **Error responses**: "ER" or other error messages

### Statistics

The script tracks:
- Total movements completed
- Total time elapsed
- Average time per movement
- Last response received

## Stopping the Test

Press `Ctrl+C` to stop. The script will:
1. Send BM0 to disable blending
2. Send ST to stop the arm
3. Print statistics
4. Close serial connection

## Troubleshooting

### "Failed to connect"

- Check port name: `ls /dev/ttyACM*`
- Check permissions: `sudo chmod 666 /dev/ttyACM1`
- Check if bridge is running: `ros2 node list`

### "No response" on UP command

- Teensy might not be ready
- Try power cycling the Teensy
- Check serial connection

### Arm doesn't move

- Check if arm is in E-stop
- Verify calibration is complete
- Check motor power supply

### "Movement would exceed limits"

This is normal - the script will return to home position instead.

## Analyzing Results

### If Bug Reproduces

Note:
1. How many movements before failure?
2. What was the last successful command?
3. What was the arm position when it failed?
4. Does RP still work? (Teensy still responding)
5. Does RJ work? (Direct joint commands)

### If Bug Doesn't Reproduce

Try:
- Increasing movement count: `--max-movements 200`
- Faster movements: `--duration 1.0`
- Running for longer time

## Example Session

```bash
$ python3 test_xj_bug_reproduction.py /dev/ttyACM1

============================================================
XJ BUG REPRODUCTION TEST
============================================================
Port: /dev/ttyACM1
Movement duration: 2.0s
Max deviation: ±15.0°
Max movements: unlimited
============================================================

✓ Connected to Teensy on /dev/ttyACM1

============================================================
INITIALIZING TEENSY TO FIXED POSITION
============================================================
Target: J1=0° J2=0° J3=90° J4=0° J5=0° J6=0°
Please ensure arm is approximately in this position!

→ UPA0.00B0.00C90.00D0.00E0.00F0.00
← Done
✓ Position initialized

✓ Initialization complete!

Starting movement test...
Press Ctrl+C to stop

------------------------------------------------------------
MOVEMENT #1 - 14:23:45
------------------------------------------------------------
Movement type: forward
Current: J1=0.0° J2=0.0° J3=90.0°
Target:  J1=0.0° J2=10.5° J3=84.8°

[1/3] Starting Move -> Initial Priming (BM1)
→ BM1
[2/3] Joystick Active -> Pushing Target (XJ)
→ XJA0.00B0.53C89.74D0.00E0.00F0.00Sp3.00Ac10.00Dc10.00Rm10.00
→ XJA0.00B1.05C89.48D0.00E0.00F0.00Sp3.00Ac10.00Dc10.00Rm10.00
...
[3/3] Joystick Idle -> Sending STOP (BM0)
→ BM0
→ ST

Verifying position...
→ RP
← A0.000B10.500C84.800D0.000E0.000F0.000G...
Actual:  J1=0.0° J2=10.5° J3=84.8°

Elapsed time: 2.5s (1 movements)

------------------------------------------------------------
MOVEMENT #2 - 14:23:48
------------------------------------------------------------
...
```

## Next Steps

If the bug reproduces:
1. Note the exact movement number when it failed
2. Check Teensy diagnostic status: `python3 diagnose_teensy_state.py /dev/ttyACM1`
3. Try manual RJ command to verify Teensy is still responsive
4. Capture the exact XJ command that failed
5. Check if it's missing Rm parameter or hitting axis limits
