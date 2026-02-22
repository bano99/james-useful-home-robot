# Boot Hang Fix - VL53L1X Sensor Initialization

## Problem
The ESP32 was hanging during boot at the VL53L1X sensor initialization. Serial output showed:
```
Scanning I2C bus...
No I2C devices found!
Checking for VL53L1X...
Only device at 0x29 found
Attempting to change VL53L1X address to 0x30...
Attempting VL53L1X init at 0x30...
[HANG - no further output]
```

## Root Cause
The Adafruit VL53L1X library's `begin()` method can block indefinitely when:
1. The sensor is not properly connected
2. I2C communication fails
3. The sensor is in a bad state
4. The address change operation didn't work as expected

The library attempts to communicate with the sensor and waits for responses that never come, causing an infinite hang.

## Solution Applied

### 1. Added I2C Ping Before begin()
Before calling the blocking `vl53.begin()` method, we now verify the device responds to I2C:
```cpp
Wire1.beginTransmission(vl53Address);
byte error = Wire1.endTransmission();

if (error == 0) {
  // Device responds, safe to call begin()
  beginResult = vl53.begin(vl53Address, &Wire1);
} else {
  // Device doesn't respond, skip begin()
  Serial.println("Device does not respond to I2C ping, skipping begin()");
}
```

### 2. Added Detailed Logging
More verbose logging helps identify exactly where the hang occurs:
- "Calling vl53.begin()..." before the blocking call
- "begin() returned true/false" after it completes
- Error codes from I2C operations

### 3. Added Kill Switch
Added `ENABLE_SENSORS` flag in `SENSOR_CTRL.h`:
```cpp
#define ENABLE_SENSORS true  // Set to false to disable sensor init
```

If sensors continue to cause problems, set this to `false` to completely skip sensor initialization and allow the system to boot normally.

## Testing the Fix

### Upload and Monitor
1. Upload the modified firmware
2. Open Serial Monitor at 115200 baud
3. Watch for these messages:

**Success case:**
```
Attempting VL53L1X init at 0x30...
Device responds to I2C, attempting begin()...
Calling vl53.begin()...
begin() returned true
VL53L1X begin() succeeded, starting ranging...
VL53L1X initialized at 0x30
```

**Sensor not present (graceful failure):**
```
Attempting VL53L1X init at 0x30...
Device does not respond to I2C ping, skipping begin()
WARNING: VL53L1X not initialized
System will operate normally without sensors.
```

### If Still Hangs
If the system still hangs at `Calling vl53.begin()...`:

1. Open `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/SENSOR_CTRL.h`
2. Change line ~20 to:
   ```cpp
   #define ENABLE_SENSORS false
   ```
3. Re-upload firmware
4. System will boot without attempting sensor initialization

## Hardware Troubleshooting

If sensors should be working but aren't:

1. **Check Wiring:**
   - SDA → GPIO16
   - SCL → GPIO17
   - VCC → 3.3V
   - GND → GND

2. **Check I2C Pull-ups:**
   - VL53L1X and BNO055 need 4.7kΩ pull-up resistors on SDA and SCL
   - Some breakout boards have these built-in

3. **Check Power:**
   - Sensors need stable 3.3V
   - Insufficient power can cause I2C hangs

4. **Test I2C Bus:**
   - After boot (with ENABLE_SENSORS=false), type `SCAN` in serial monitor
   - Should show any connected I2C devices

## Files Modified
- `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/SENSOR_CTRL.h`
  - Added ENABLE_SENSORS flag
  - Added I2C ping before begin() calls
  - Added detailed logging
  - Added error handling for address change

## Related Issues
- Previously removed USB serial wait (was hanging when USB not connected)
- This is a similar blocking I2C library call issue
