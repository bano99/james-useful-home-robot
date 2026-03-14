#!/usr/bin/env python3
"""
Reverse Engineer J4StepM

Work backwards from the XJ_LIMIT failure to determine what J4StepM
value would cause the limit check to fail.

Given:
  - Target: D2.3321°
  - Result: XJ_LIMIT:000100 (J4 fault)

The Teensy code:
  int J4stepDif = J4StepM - (int)((J4Angle + J4axisLimNeg) * J4StepDeg);
  J4dir = (J4stepDif <= 0) ? 1 : 0;
  
  if ((J4dir == 1 and (J4StepM + abs(J4stepDif) > J4StepLim)) or 
      (J4dir == 0 and (J4StepM - abs(J4stepDif) < 0))) 
      J4axisFault = 1;
"""

import json

# Load config
with open('ros2_ws/src/james_manipulation/config/ARconfig.json', 'r') as f:
    config = json.load(f)

J4StepDeg = float(config['J4StepDeg'])  # 86.21

print("="*80)
print("REVERSE ENGINEERING J4StepM")
print("="*80)
print()
print("Given: Target D2.3321° causes XJ_LIMIT:000100")
print()
print("Question: What value of J4StepM would cause this failure?")
print()
print("="*80)
print()

target_angle = 2.3321

# Try both limit scenarios
for scenario, (J4axisLimNeg, J4StepLim) in [
    ("DEFAULTS (180°)", (180.0, int(360.0 * J4StepDeg))),
    ("CONFIG (176°)", (176.0, int(306.0 * J4StepDeg)))
]:
    print(f"SCENARIO: {scenario}")
    print("-"*80)
    print(f"  J4axisLimNeg: {J4axisLimNeg}°")
    print(f"  J4StepLim: {J4StepLim} steps")
    print()
    
    target_steps = int((target_angle + J4axisLimNeg) * J4StepDeg)
    print(f"  Target steps = ({target_angle} + {J4axisLimNeg}) * {J4StepDeg}")
    print(f"  Target steps = {target_steps}")
    print()
    
    # For FORWARD movement (J4dir=1) to fail:
    # J4StepM + abs(J4stepDif) > J4StepLim
    # J4StepM + abs(J4StepM - target_steps) > J4StepLim
    # If J4StepM < target_steps (forward): J4StepM + (target_steps - J4StepM) > J4StepLim
    #                                      target_steps > J4StepLim
    
    print("  FORWARD movement (J4dir=1) fails if:")
    print(f"    target_steps > J4StepLim")
    print(f"    {target_steps} > {J4StepLim}?")
    print(f"    {target_steps > J4StepLim}")
    
    if target_steps > J4StepLim:
        print(f"    ❌ TARGET ITSELF EXCEEDS LIMIT!")
        print(f"    The target angle {target_angle}° is beyond the limit.")
        max_angle = (J4StepLim / J4StepDeg) - J4axisLimNeg
        print(f"    Max allowed angle: {max_angle:.2f}°")
    print()
    
    # For BACKWARD movement (J4dir=0) to fail:
    # J4StepM - abs(J4stepDif) < 0
    # J4StepM - abs(J4StepM - target_steps) < 0
    # If J4StepM > target_steps (backward): J4StepM - (J4StepM - target_steps) < 0
    #                                       target_steps < 0
    
    print("  BACKWARD movement (J4dir=0) fails if:")
    print(f"    target_steps < 0")
    print(f"    {target_steps} < 0?")
    print(f"    {target_steps < 0}")
    
    if target_steps < 0:
        print(f"    ❌ TARGET IS NEGATIVE!")
        print(f"    The target angle {target_angle}° is below the negative limit.")
        min_angle = -J4axisLimNeg
        print(f"    Min allowed angle: {min_angle:.2f}°")
    print()
    
    # The only way for the check to fail is if J4StepM is VERY different
    print("  For the check to fail with this target:")
    print()
    
    # Case 1: If J4StepM is huge (way beyond limit)
    print("    Case 1: J4StepM is beyond J4StepLim")
    print(f"      If J4StepM > {J4StepLim}, then even moving backward might fail")
    print(f"      because J4StepM - |J4stepDif| could still be > J4StepLim")
    print()
    
    # Case 2: If J4StepM is negative
    print("    Case 2: J4StepM is negative")
    print(f"      If J4StepM < 0, then moving forward would fail")
    print(f"      because J4StepM + |J4stepDif| would be calculated incorrectly")
    print()
    
    print("="*80)
    print()

# The real question: what does the feedback "D2.332" mean?
print("CRITICAL INSIGHT:")
print("-"*80)
print()
print("The feedback shows 'D2.332°' but this is calculated by the BRIDGE,")
print("not directly from the Teensy!")
print()
print("The bridge calculates angle from J4StepM:")
print("  angle = (J4StepM / J4StepDeg) - J4axisLimNeg")
print()
print("But if the bridge is using DIFFERENT values of J4axisLimNeg than")
print("the Teensy, the reported angle will be WRONG!")
print()
print("="*80)
print()

# Check what the bridge uses
print("BRIDGE vs TEENSY MISMATCH:")
print("-"*80)
print()
print("If the bridge thinks J4axisLimNeg = 176° (from config)")
print("but the Teensy is using J4axisLimNeg = 180° (default)")
print()
print("Then when Teensy has J4StepM = 15718 steps:")
print()
print("  Teensy thinks: angle = (15718 / 86.21) - 180 = 2.332°")
print("  Bridge thinks: angle = (15718 / 86.21) - 176 = 6.332°")
print()
print("But both report the same J4StepM value, so they should agree...")
print()
print("UNLESS the Teensy is using a DIFFERENT J4StepM value internally!")
print()
print("="*80)
print()

print("HYPOTHESIS:")
print("-"*80)
print()
print("The Teensy's J4StepM is NOT what we think it is.")
print()
print("Possible causes:")
print("  1. Encoder was still active when the failure occurred")
print("  2. J4StepM was never properly initialized after calibration")
print("  3. There's a bug in how J4StepM is updated")
print("  4. The UP command didn't properly update J4 to open-loop mode")
print()
print("SOLUTION:")
print("  Add Serial.print debug output to the Teensy XJ handler to see:")
print("    - Actual J4StepM value")
print("    - Calculated target_steps")
print("    - J4stepDif")
print("    - J4dir")
print("    - Result of limit check")
print()
print("="*80)
