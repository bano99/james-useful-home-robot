#!/usr/bin/env python3
"""
Diagnose J4 Real Failure Case

Analyzing the actual failure from the log:
  Current: D2.332°
  Target:  D2.3321°
  Result:  XJ_LIMIT:000100
"""

import json

# Load config
with open('ros2_ws/src/james_manipulation/config/ARconfig.json', 'r') as f:
    config = json.load(f)

# Teensy parameters
J4StepDeg = float(config['J4StepDeg'])  # 86.21 steps/degree
J4PosLim = float(config['J4PosLim'])    # 130°
J4NegLim = float(config['J4NegLim'])    # 176°

# Check if open loop now
J4OpenLoop = int(config.get('J4OpenLoopVal', 0))

print("="*80)
print("J4 REAL FAILURE CASE ANALYSIS")
print("="*80)
print()
print("FROM YOUR LOG:")
print("  Before: D2.332°")
print("  Target: D2.3321°")
print("  Result: XJ_LIMIT:000100 (J4 fault)")
print()
print("="*80)
print()

# Teensy uses DEFAULTS if LL command not sent
# The key question: what are J4axisLimNeg and J4StepLim?

print("CRITICAL QUESTION: What limits is the Teensy using?")
print("-"*80)
print()
print("Option 1: TEENSY DEFAULTS (if LL command never sent)")
print("  J4axisLimPos: 180°")
print("  J4axisLimNeg: 180°")
print("  J4axisLim: 360°")
print("  J4StepLim: 31,035 steps")
print()
print("Option 2: CONFIG VALUES (if LL command was sent)")
print(f"  J4axisLimPos: {J4PosLim}°")
print(f"  J4axisLimNeg: {J4NegLim}°")
print(f"  J4axisLim: {J4PosLim + J4NegLim}°")
print(f"  J4StepLim: {int((J4PosLim + J4NegLim) * J4StepDeg)} steps")
print()
print("="*80)
print()

# Let's calculate with BOTH scenarios
for scenario, (J4axisLimNeg, J4StepLim) in [
    ("DEFAULTS", (180.0, int(360.0 * J4StepDeg))),
    ("CONFIG", (J4NegLim, int((J4PosLim + J4NegLim) * J4StepDeg)))
]:
    print(f"SCENARIO: {scenario}")
    print("-"*80)
    print(f"  J4axisLimNeg: {J4axisLimNeg}°")
    print(f"  J4StepLim: {J4StepLim} steps")
    print()
    
    # Current position
    current_angle = 2.332
    J4StepM = int((current_angle + J4axisLimNeg) * J4StepDeg)
    print(f"  Current angle: {current_angle}°")
    print(f"  J4StepM = ({current_angle} + {J4axisLimNeg}) * {J4StepDeg}")
    print(f"  J4StepM = {J4StepM} steps")
    print()
    
    # Target position
    target_angle = 2.3321
    target_steps = int((target_angle + J4axisLimNeg) * J4StepDeg)
    print(f"  Target angle: {target_angle}°")
    print(f"  Target steps = ({target_angle} + {J4axisLimNeg}) * {J4StepDeg}")
    print(f"  Target steps = {target_steps} steps")
    print()
    
    # Step difference
    J4stepDif = J4StepM - target_steps
    print(f"  J4stepDif = {J4StepM} - {target_steps} = {J4stepDif} steps")
    print()
    
    # Direction
    J4dir = 1 if J4stepDif <= 0 else 0
    print(f"  J4dir = {J4dir} ({'FORWARD' if J4dir == 1 else 'BACKWARD'})")
    print()
    
    # Limit check
    print("  LIMIT CHECK:")
    if J4dir == 1:
        new_pos = J4StepM + abs(J4stepDif)
        print(f"    Forward: J4StepM + |J4stepDif| = {J4StepM} + {abs(J4stepDif)} = {new_pos}")
        print(f"    Check: {new_pos} > {J4StepLim}?")
        fault = new_pos > J4StepLim
        if fault:
            print(f"    Result: ❌ FAULT! ({new_pos} exceeds {J4StepLim})")
            print(f"    Overshoot: {new_pos - J4StepLim} steps = {(new_pos - J4StepLim) / J4StepDeg:.2f}°")
        else:
            print(f"    Result: ✓ OK")
    else:
        new_pos = J4StepM - abs(J4stepDif)
        print(f"    Backward: J4StepM - |J4stepDif| = {J4StepM} - {abs(J4stepDif)} = {new_pos}")
        print(f"    Check: {new_pos} < 0?")
        fault = new_pos < 0
        if fault:
            print(f"    Result: ❌ FAULT! ({new_pos} below 0)")
            print(f"    Undershoot: {new_pos} steps = {new_pos / J4StepDeg:.2f}°")
        else:
            print(f"    Result: ✓ OK")
    
    print()
    print("="*80)
    print()

# Now check what the ACTUAL angle limits are
print("ANGLE LIMIT ANALYSIS:")
print("-"*80)
print()

for scenario, (J4axisLimPos, J4axisLimNeg) in [
    ("DEFAULTS", (180.0, 180.0)),
    ("CONFIG", (J4PosLim, J4NegLim))
]:
    print(f"{scenario}:")
    print(f"  Range: -{J4axisLimNeg}° to +{J4axisLimPos}°")
    print(f"  Is 2.332° within range? {-J4axisLimNeg <= 2.332 <= J4axisLimPos}")
    print()

print("="*80)
print()

# Check if the issue is with the UP command
print("CONFIGURATION STATUS:")
print("-"*80)
print(f"  J4OpenLoopVal: {J4OpenLoop} ({'OPEN LOOP' if J4OpenLoop == 1 else 'CLOSED LOOP'})")
print()
if J4OpenLoop == 0:
    print("  ⚠ WARNING: J4 is still in CLOSED LOOP mode!")
    print("  The config file was updated, but the Teensy hasn't received it yet.")
    print("  You need to restart the serial bridge to send the UP command.")
else:
    print("  ✓ J4 is in OPEN LOOP mode")
print()
print("="*80)
print()

print("CONCLUSION:")
print("-"*80)
print()
print("The failure happens because:")
print()
print("1. The Teensy is using HARDCODED DEFAULT limits (±180°)")
print("   OR your configured limits (±176°/+130°)")
print()
print("2. The calculation shows the target SHOULD be within limits")
print()
print("3. BUT if J4 is still in CLOSED LOOP mode, the encoder might be")
print("   reporting a DIFFERENT position than 2.332°, causing J4StepM")
print("   to have an incorrect value.")
print()
print("NEXT STEPS:")
print("  1. Restart the serial bridge to send updated config (J4OpenLoopVal=1)")
print("  2. If still failing, add debug output to Teensy to see actual J4StepM value")
print("  3. Consider sending LL command explicitly to ensure limits are correct")
print()
print("="*80)
