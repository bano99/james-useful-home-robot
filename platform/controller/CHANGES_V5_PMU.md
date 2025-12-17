# Display Changes - Version 5 (PMU Battery Reading)

## Root Cause Found!

The LilyGO T-Display S3 AMOLED board uses a **PMU (Power Management Unit)** chip to manage battery, not direct ADC reading!

The example code revealed that battery voltage must be read through the `LilyGo_AMOLED` library, not via GPIO pins.

## Problem
- ADC reading was only 73 (should be ~2500-3500)
- Voltage showed 0.14V instead of ~4V
- GPIO 1 is not a battery monitoring pin

## Solution
Use the LilyGo_AMOLED library to access the PMU chip for accurate battery readings.

## Changes Made

### 1. Added LilyGo_AMOLED Library

```cpp
#include <LilyGo_AMOLED.h>  // For PMU battery monitoring

// Create LilyGo AMOLED object
LilyGo_Class amoled;
```

### 2. Initialize PMU in setup()

```cpp
void setup() {
  // Initialize LilyGo AMOLED board (includes PMU)
  amoled.beginAMOLED_147();  // For 1.47" AMOLED board
  
  // Enable PMU battery monitoring features
  amoled.enableBattDetection();
  amoled.enableBattVoltageMeasure();
  amoled.enableSystemVoltageMeasure();
  amoled.enableVbusVoltageMeasure();
  
  // ... rest of setup
}
```

### 3. Updated readBatteryVoltage()

**OLD (incorrect):**
```cpp
float readBatteryVoltage() {
  int adcValue = analogRead(BATTERY_PIN);  // Wrong!
  float voltage = (adcValue / 4095.0) * 3.3;
  return voltage;
}
```

**NEW (correct):**
```cpp
float readBatteryVoltage() {
  // Use PMU to read battery voltage (returns millivolts)
  uint16_t batteryMV = amoled.getBattVoltage();
  float voltage = batteryMV / 1000.0;  // Convert mV to V
  
  // Debug output
  Serial.printf("Battery: %u mV (%.2fV), Percent: %d%%, Charging: %s\n", 
                batteryMV, voltage,
                amoled.getBatteryPercent(),
                amoled.isCharging() ? "YES" : "NO");
  
  return voltage;
}
```

## PMU Functions Available

The LilyGo_AMOLED library provides these battery-related functions:

```cpp
amoled.getBattVoltage()      // Returns battery voltage in mV
amoled.getBatteryPercent()   // Returns battery percentage (0-100)
amoled.isCharging()          // Returns true if charging
amoled.isDischarge()         // Returns true if discharging
amoled.isVbusIn()            // Returns true if USB connected
amoled.getChargerStatus()    // Returns charging state
amoled.getVbusVoltage()      // Returns USB voltage in mV
amoled.getSystemVoltage()    // Returns system voltage in mV
```

## New Debug Output

Serial monitor now shows (every 2 seconds):
```
Battery: 3850 mV (3.85V), Percent: 78%, Charging: NO
```

This provides:
- Actual battery voltage in mV and V
- Battery percentage from PMU
- Charging status

## Library Installation

The `LilyGo_AMOLED` library must be installed:

### Arduino IDE:
1. Go to Sketch → Include Library → Manage Libraries
2. Search for "LilyGo_AMOLED"
3. Install the library by LilyGo

### PlatformIO:
Add to `platformio.ini`:
```ini
lib_deps = 
    https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series
```

## Expected Results

After uploading:
- Battery voltage should show correct value (3.0V - 4.2V)
- Percentage should be accurate
- Color coding should work correctly
- Serial monitor shows detailed battery info

## Benefits of PMU Reading

✅ **Accurate voltage** - Direct from power management chip
✅ **Battery percentage** - Calculated by PMU
✅ **Charging detection** - Knows when USB is connected
✅ **No calibration needed** - PMU handles voltage divider internally
✅ **More features** - Can detect USB, charging state, etc.

## Future Enhancements

With PMU access, we can now add:
- Charging indicator on display
- USB connection indicator
- More accurate battery percentage
- Low battery warnings
- Charging animation

## Files Updated

1. `plattform_controller.ino` - Added PMU support
2. `CHANGES_V5_PMU.md` - This file

## Testing

1. Upload firmware
2. Open serial monitor (115200 baud)
3. Should see:
   ```
   Initializing LilyGo AMOLED board...
   PMU battery monitoring enabled
   Display initialized!
   ...
   Battery: 3850 mV (3.85V), Percent: 78%, Charging: NO
   ```
4. Display should show correct voltage (e.g., "3.85V")
5. Battery icon should fill correctly
6. Color should match battery level

## Troubleshooting

### If compilation fails:
- Install `LilyGo_AMOLED` library
- Check library version is compatible

### If voltage still shows 0V:
- Check serial monitor for PMU initialization messages
- Verify `amoled.beginAMOLED_147()` succeeds
- Try disconnecting USB and running on battery only

### If display doesn't work:
- The PMU initialization might conflict with display
- Check if display still initializes after PMU setup
- May need to adjust initialization order

## Notes

- The board model is 1.47" AMOLED, hence `beginAMOLED_147()`
- PMU chip handles all battery management internally
- No need for voltage divider calculations
- Battery percentage is more accurate than our calculation
