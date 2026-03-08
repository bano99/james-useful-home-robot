#!/usr/bin/env python3
"""
Diagnose J4 Step Calculation

This script simulates the exact calculation the Teensy does for J4
to understand why XJ_LIMIT is being triggered.

Based on the Teensy code:
    int J4stepDif = J4StepM - (int)((J4Angle + J4axisLimNeg) * J4StepDeg);
    J4dir = (J4stepDif <= 0) ? 1 : 0;
    
    if ((J4dir == 1 and (J4StepM + abs(J4stepDif) > J4StepLim)) or 
        (J4dir == 0 and (J4StepM - abs(J4stepDif) < 0))) 
        J4axisFault = 1;
"""

import json
import math

# Load config
with open('ros2_ws/src/james_manipulation/config/ARconfig.json', 'r') as f:
    config = json.load(f)

# Teensy parameters from config
J4StepDeg = float(config['J4StepDeg'])  # 86.21 steps/degree
J4PosLim = float(config['J4PosLim'])    # 130°
J4NegLim = float(config['J4NegLim'])    # 176°

# Teensy hardcoded defaults (if LL command not sent)
J4axisLimPos_default = 180.0
J4axisLimNeg_default = 180.0

# Calculated values (if using defaults)
J4axisLim_default = J4axisLimPos_default + J4axisLimNeg_default  # 360°
J4StepLim_default = int(J4axisLim_default * J4StepDeg)  # 31,035 steps
J4zeroStep_default = int(J4axisLimNeg_default * J4StepDeg)  # 15,517 steps

# Calculated values (if using config)
J4axisLim_config = J4PosLim + J4NegLim  # 306°
J4StepLim_config = int(J4axisLim_config * J4StepDeg)  # 26,380 steps
J4zeroStep_config = int(J4NegLim * J4StepDeg)  # 15,173 steps

print("="*80)
print("J4 CALCULATION DIAGNOSIS")
print("="*80)
print()
print("TEENSY PARAMETERS:")
print(f"  J4StepDeg: {J4StepDeg} steps/degree")
print()
print("CONFIG VALUES:")
print(f"  J4PosLim: {J4PosLim}°")
print(f"  J4NegLim: {J4NegLim}°")
print(f"  J4axisLim: {J4axisLim_config}° (total range)")
print(f"  J4StepLim: {J4StepLim_config} steps")
print(f"  J4zeroStep: {J4zeroStep_config} steps")
print()
print("TEENSY DEFAULTS (if LL not sent):")
print(f"  J4axisLimPos: {J4axisLimPos_default}°")
print(f"  J4axisLimNeg: {J4axisLimNeg_default}°")
print(f"  J4axisLim: {J4axisLim_default}° (total range)")
print(f"  J4StepLim: {J4StepLim_default} steps")
print(f"  J4zeroStep: {J4zeroStep_default} steps")
print()
print("="*80)
print()

# Scenario from your log
print("SCENARIO FROM LOG:")
print("-"*80)
print("Before failure:")
print("  Current J4: 4.535°")
print("  Received feedback: D4.535")
print()
print("Failed command:")
print("  Target J4: 4.5363°")
print("  XJ command: D4.5363")
print("  Result: XJ_LIMIT:000100 (J4 fault)")
print()
print("-"*80)
print()

# Simulate Teensy calculation with DEFAULTS
print("SIMULATION WITH TEENSY DEFAULTS:")
print("-"*80)

# Current position: J4 = 4.535°
current_angle = 4.535
J4StepM = int((current_angle + J4axisLimNeg_default) * J4StepDeg)
print(f"Current angle: {current_angle}°")
print(f"J4StepM = (angle + J4axisLimNeg) * J4StepDeg")
print(f"J4StepM = ({current_angle} + {J4axisLimNeg_default}) * {J4StepDeg}")
print(f"J4StepM = {J4StepM} steps")
print()

# Target position: J4 = 4.5363°
target_angle = 4.5363
target_steps = int((target_angle + J4axisLimNeg_default) * J4StepDeg)
print(f"Target angle: {target_angle}°")
print(f"Target steps = ({target_angle} + {J4axisLimNeg_default}) * {J4StepDeg}")
print(f"Target steps = {target_steps} steps")
print()

# Calculate step difference
J4stepDif = J4StepM - target_steps
print(f"J4stepDif = J4StepM - target_steps")
print(f"J4stepDif = {J4StepM} - {target_steps}")
print(f"J4stepDif = {J4stepDif} steps")
print()

# Determine direction
J4dir = 1 if J4stepDif <= 0 else 0
print(f"J4dir = {'1 (forward)' if J4dir == 1 else '0 (backward)'}")
print()

# Check limits
print("LIMIT CHECK:")
if J4dir == 1:
    new_pos = J4StepM + abs(J4stepDif)
    print(f"  Direction: FORWARD (J4dir=1)")
    print(f"  New position: J4StepM + |J4stepDif| = {J4StepM} + {abs(J4stepDif)} = {new_pos}")
    print(f"  Limit: J4StepLim = {J4StepLim_default}")
    print(f"  Check: {new_pos} > {J4StepLim_default}?")
    if new_pos > J4StepLim_default:
        print(f"  Result: ❌ FAULT! ({new_pos} exceeds {J4StepLim_default})")
    else:
        print(f"  Result: ✓ OK ({new_pos} within {J4StepLim_default})")
else:
    new_pos = J4StepM - abs(J4stepDif)
    print(f"  Direction: BACKWARD (J4dir=0)")
    print(f"  New position: J4StepM - |J4stepDif| = {J4StepM} - {abs(J4stepDif)} = {new_pos}")
    print(f"  Limit: 0")
    print(f"  Check: {new_pos} < 0?")
    if new_pos < 0:
        print(f"  Result: ❌ FAULT! ({new_pos} below 0)")
    else:
        print(f"  Result: ✓ OK ({new_pos} >= 0)")

print()
print("-"*80)
print()

# Now check what happens if we received the position feedback WRONG
print("ALTERNATIVE HYPOTHESIS: Position Feedback Mismatch")
print("-"*80)
print()
print("What if the Teensy thinks J4 is at a DIFFERENT angle than reported?")
print()

# The feedback says D4.535, but what if internally it's different?
# Let's work backwards from the failure
print("Working backwards from the failure:")
print(f"  Target that failed: {target_angle}°")
print(f"  Target steps: {target_steps}")
print()

# For the limit check to fail with J4dir=1 (forward):
# J4StepM + |J4stepDif| > J4StepLim
# Since J4stepDif = J4StepM - target_steps (and it's negative for forward)
# J4StepM + (target_steps - J4StepM) > J4StepLim
# target_steps > J4StepLim

print("For forward movement to fail:")
print(f"  target_steps > J4StepLim")
print(f"  {target_steps} > {J4StepLim_default}?")
print(f"  {target_steps > J4StepLim_default}")
print()

if target_steps > J4StepLim_default:
    print("❌ TARGET EXCEEDS LIMIT!")
    print()
    print("This means the target angle itself is beyond the limit.")
    print(f"  Target angle: {target_angle}°")
    print(f"  Max angle: (J4StepLim / J4StepDeg) - J4axisLimNeg")
    max_angle = (J4StepLim_default / J4StepDeg) - J4axisLimNeg_default
    print(f"  Max angle: ({J4StepLim_default} / {J4StepDeg}) - {J4axisLimNeg_default}")
    print(f"  Max angle: {max_angle}°")
    print()
    print(f"  Target {target_angle}° > Max {max_angle}°? {target_angle > max_angle}")

print()
print("="*80)


# Check encoder configuration
print()
print("ENCODER CONFIGURATION:")
print("-"*80)
J4OpenLoop = int(config.get('J4OpenLoopVal', 1))
J4CalStat = int(config.get('J4CalStatVal', 0))
J4CalStat2 = int(config.get('J4CalStatVal2', 0))
J4calOff = float(config.get('J4calOff', 0.0))

print(f"  J4OpenLoopVal: {J4OpenLoop} ({'OPEN LOOP' if J4OpenLoop == 1 else 'CLOSED LOOP (encoder)'})")
print(f"  J4CalStatVal: {J4CalStat}")
print(f"  J4CalStatVal2: {J4CalStat2}")
print(f"  J4calOff: {J4calOff}°")
print()

if J4OpenLoop == 0:
    print("⚠ J4 IS IN CLOSED LOOP MODE (uses encoder feedback)")
    print()
    print("This means J4StepM is updated from ENCODER readings, not commanded position!")
    print("If the encoder has an offset or calibration error, J4StepM could be wrong.")
    print()
    print("HYPOTHESIS: Encoder Offset Issue")
    print("-"*80)
    print()
    print("The encoder might be reporting J4 at a position that causes incorrect")
    print("step calculations, triggering false limit violations.")
    print()
    print("POSSIBLE CAUSES:")
    print("  1. Encoder wiring issue or noise")
    print("  2. Encoder not calibrated (J4CalStatVal=0)")
    print("  3. Wrong encoder CPR in config")
    print("  4. Mechanical slippage between encoder and joint")
    print()
    print("SOLUTION:")
    print("  1. Calibrate J4 properly")
    print("  2. Check encoder connections")
    print("  3. Verify J4EncCPR matches actual encoder")
    print(f"     Current: {config.get('J4EncCPR', 'NOT SET')}")
else:
    print("✓ J4 is in open loop mode (no encoder)")

print()
print("="*80)
print()
print("CONCLUSION:")
print("-"*80)
print("The XJ_LIMIT error for J4 at 4.5363° doesn't make sense with the configured")
print("limits. The most likely cause is:")
print()
if J4OpenLoop == 0:
    print("  ❌ ENCODER POSITION MISMATCH")
    print("     J4 uses closed-loop control, and the encoder reading doesn't match")
    print("     the actual joint angle. This causes J4StepM to have an incorrect value,")
    print("     triggering false limit violations.")
    print()
    print("  IMMEDIATE FIX: Recalibrate J4 or switch to open-loop mode")
else:
    print("  ❓ UNKNOWN - J4 is in open loop, so encoder shouldn't be the issue")
    print("     The Teensy might not have received the LL configuration command")
    print("     and is using incorrect default limits.")

print("="*80)
