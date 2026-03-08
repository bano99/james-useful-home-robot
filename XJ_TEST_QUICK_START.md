# XJ Bug Test - Quick Start

## 1. Setup (30 seconds)

```bash
# Stop the bridge if running
ros2 lifecycle set /teensy_serial_bridge shutdown

# Position arm approximately at: J1=0°, J2=0°, J3=90°, others=0°
# (doesn't need to be exact)
```

## 2. Run Test

```bash
# Basic test (runs until bug occurs or Ctrl+C)
python3 test_xj_bug_reproduction.py /dev/ttyACM1

# Run for 50 movements then stop
python3 test_xj_bug_reproduction.py /dev/ttyACM1 --max-movements 50
```

## 3. Watch For

- **Silent rejection**: XJ commands sent but no movement
- **Timeout**: Commands don't get responses
- **"ER" errors**: Kinematic errors

## 4. When Bug Occurs

```bash
# Check if Teensy still responds
python3 diagnose_teensy_state.py /dev/ttyACM1

# Try manual RJ command
# (through the diagnostic script or manually)
```

## What the Script Does

Each movement cycle (every ~2 seconds):
1. **BM1** - Enable blending mode (like joystick start)
2. **XJ commands** - Send 20 XJ commands over 2 seconds (like continuous joystick input)
3. **BM0** - Disable blending mode (like joystick release)
4. **ST** - Stop command
5. **RP** - Verify position

Movements are random: forward, backward, left, right (±10°±2° primary joint)

## Expected Output

```
MOVEMENT #1 - 14:23:45
Movement type: forward
Current: J1=0.0° J2=0.0° J3=90.0°
Target:  J1=0.0° J2=10.5° J3=84.8°

[1/3] Starting Move -> Initial Priming (BM1)
[2/3] Joystick Active -> Pushing Target (XJ)
[3/3] Joystick Idle -> Sending STOP (BM0)

Verifying position...
Actual:  J1=0.0° J2=10.5° J3=84.8°

Elapsed time: 2.5s (1 movements)
```

## If Bug Reproduces

You'll see:
- XJ commands sent but no movement
- Timeouts on RP commands
- Or "ER" responses

**This confirms the bug and helps identify the exact conditions!**
