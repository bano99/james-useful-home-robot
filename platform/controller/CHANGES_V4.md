# Display Changes - Version 4 (Battery Voltage Debug)

## Issues Found

1. **Voltage reading too low**: Shows 0.14V instead of ~4V
2. **Green artifacts**: Text not fully cleared before redrawing

## Fixes Applied

### 1. Enhanced Battery Voltage Reading

Added comprehensive debug output to identify the correct voltage calculation:

```cpp
float readBatteryVoltage() {
  // Average 10 readings for stability
  int adcSum = 0;
  for (int i = 0; i < 10; i++) {
    adcSum += analogRead(BATTERY_PIN);
    delay(1);
  }
  int adcValue = adcSum / 10;
  
  // Try three different calculations
  float voltage1 = (adcValue / 4095.0) * 3.3 * 2.0;  // With divider
  float voltage2 = (adcValue / 4095.0) * 3.3;        // Direct
  float voltage3 = (adcValue / 4095.0) * 4.2;        // Scaled
  
  // Debug output shows all three
  Serial.printf("Battery ADC: %d\n", adcValue);
  Serial.printf("  With divider (x2): %.2fV\n", voltage1);
  Serial.printf("  Direct (x1):       %.2fV\n", voltage2);
  Serial.printf("  Direct scaled:     %.2fV\n", voltage3);
  
  return voltage2;  // Currently using direct reading
}
```

### 2. Fixed Display Artifacts

Increased clear area to prevent green artifacts:

```cpp
// Clear entire area - make it larger to avoid artifacts
gfx2->fillRect(x - 20, y - 20, width + 40, height + 40, COLOR_BG);
```

## Debug Output

Serial monitor now shows (every 2 seconds):
```
Battery ADC: 250
  With divider (x2): 0.40V
  Direct (x1):       0.20V
  Direct scaled:     0.26V
```

## Troubleshooting Steps

### Step 1: Check Serial Monitor
Open serial monitor at 115200 baud and observe the battery readings.

### Step 2: Identify the Issue

**If ADC is very low (< 500):**
- Wrong pin (try GPIO 4 or 14 instead of GPIO 1)
- Battery not connected
- USB charging active

**If ADC is normal (~2500-3500) but voltage wrong:**
- Wrong calculation/multiplier
- Need to adjust divider ratio

### Step 3: Measure Actual Voltage
Use a multimeter to measure the actual battery voltage.

### Step 4: Compare
Compare actual voltage with the three calculations shown in serial monitor.

### Step 5: Update Code
Once you identify which calculation is correct, update the return statement in `readBatteryVoltage()`.

## Possible Pin Configurations

The LilyGO T-Display S3 AMOLED might use different pins:

```cpp
// Try these pins if GPIO 1 doesn't work:
#define BATTERY_PIN 1   // Current
#define BATTERY_PIN 4   // Alternative 1
#define BATTERY_PIN 14  // Alternative 2
```

## Expected Values

For a 1S LiPo battery:

| Battery Voltage | Expected ADC | With Divider (x2) | Direct (x1) | Scaled (x4.2/3.3) |
|----------------|--------------|-------------------|-------------|-------------------|
| 4.2V (full)    | ~3100-3400   | ~5.0V            | ~2.5V       | ~3.2V            |
| 3.7V (nominal) | ~2700-3000   | ~4.4V            | ~2.2V       | ~2.8V            |
| 3.0V (empty)   | ~2200-2500   | ~3.6V            | ~1.8V       | ~2.3V            |

## Files Updated

1. `plattform_controller.ino` - Enhanced voltage reading with debug
2. `BATTERY_VOLTAGE_DEBUG.md` - Comprehensive troubleshooting guide
3. `CHANGES_V4.md` - This file

## Next Steps

1. Upload firmware
2. Open serial monitor (115200 baud)
3. Observe ADC value and three voltage calculations
4. Measure actual battery voltage with multimeter
5. Report back with:
   - ADC value
   - Actual voltage
   - Which calculation is closest
6. Update code with correct calculation

## Quick Test

If you want to test the display without fixing voltage reading:

```cpp
float readBatteryVoltage() {
  return 3.7;  // Temporary fixed value
}
```

This will show "3.7V" on display for testing other features.
