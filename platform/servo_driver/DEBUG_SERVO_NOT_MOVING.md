# Debug: Servo Not Moving

## Status
✅ Command parsing works: `OK: Servo 1 -> Position 2047 @ Speed 1000`  
❌ Servo doesn't physically move

## Diagnostic Steps

### Step 1: Check if servo responds to PING
```
#1PING
```

**Expected responses:**
- ✅ "Servo ID 1 responded to PING" → Servo is connected and communicating
- ❌ "Servo ID 1 did not respond" → Connection/power/ID issue

### Step 2: Check servo power
- Servo power supply voltage: 6-12V DC
- Check power LED on servo (if present)
- Measure voltage at servo connector
- Ensure power supply can provide enough current (2-3A per servo)

### Step 3: Check UART wiring
ESP32 to Servo Bus:
- GPIO 18 (S_RXD) → Servo TX (yellow/white wire)
- GPIO 19 (S_TXD) → Servo RX (green/white wire)
- GND → Servo GND (black wire)
- Power supply → Servo VCC (red wire)

### Step 4: Read current position
```
#1P?
```

**Expected:**
- Should return current position value (0-4095)
- If it returns a value, servo is communicating

### Step 5: Check torque status
Servo might have torque disabled. Try:
```
#1T1
```
Then try moving again:
```
#1P2047S1000
```

### Step 6: Try different positions
```
#1P0S500
#1P4095S500
#1P2047S500
```

Watch if servo tries to move (even slightly)

### Step 7: Check web interface
Does the servo move when controlled via web interface?
- If YES → Serial command issue (but we got OK response...)
- If NO → Hardware/power/wiring issue

### Step 8: Check servo mode
Servo might be in motor mode (continuous rotation) instead of servo mode:
```
#1M0
```
Then try:
```
#1P2047S1000
```

## Common Issues

### Issue 1: Servo ID Mismatch
Your servo might not be ID 1. Try:
```
#2PING
#3PING
#4PING
```

Or scan all IDs (we can add this feature if needed)

### Issue 2: Insufficient Power
- Servos need significant current (1-3A each)
- USB power is NOT enough
- Need dedicated power supply

### Issue 3: Torque Disabled
- Servo torque might be off
- Try: `#1T1` to enable

### Issue 4: Wrong Baud Rate on Servo Bus
- ESP32 → Servo bus is 1000000 baud (1 Mbps)
- This is set in `servoInit()` in STSCTRL.h
- If servo is configured differently, it won't respond

### Issue 5: Servo Already at Position
- If servo is already at position 2047, it won't move
- Try a different position: `#1P0S500` or `#1P4095S500`

## Diagnostic Commands to Run

Run these in order and report results:

```
1. #1PING
   → Does it respond?

2. #1P?
   → What position does it report?

3. #1T1
   → Enable torque

4. #1P0S500
   → Try moving to 0

5. #1P4095S500
   → Try moving to max

6. RD
   → Check if sensors work (confirms ESP32 is working)
```

## Next Steps Based on Results

### If PING fails:
- Check power supply
- Check wiring
- Try different servo ID
- Check servo is ST series (not SC series)

### If PING works but no movement:
- Enable torque: `#1T1`
- Set servo mode: `#1M0`
- Check mechanical obstruction
- Try lower speed: `#1P2047S100`

### If position read fails:
- Servo communication issue
- Check UART wiring
- Check baud rate match

### If web interface works but serial doesn't:
- This would be very strange since we got "OK" response
- Check if web interface actually calls same functions
- Add debug output to see if `st.WritePosEx()` is actually called

## Add Debug Output

We can add more verbose output to see what's happening. Let me know if you want me to add:
1. Debug output showing actual SCServo library call
2. Return value from `st.WritePosEx()`
3. Servo feedback after command
