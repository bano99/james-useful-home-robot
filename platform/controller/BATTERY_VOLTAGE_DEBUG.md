# Battery Voltage Reading Debug Guide

## Current Issue
Display shows 0.14V instead of expected ~4V for 1S LiPo battery.

## Debug Output Added

The firmware now outputs three different voltage calculations every 2 seconds:

```
Battery ADC: 250
  With divider (x2): 0.40V
  Direct (x1):       0.20V
  Direct scaled:     0.26V
```

## What to Check

### 1. Check Serial Monitor Output

Open serial monitor at 115200 baud and look for the battery readings.

**Expected ADC values for 1S LiPo:**
- 4.2V (full) → ADC should be ~3100-3400
- 3.7V (nominal) → ADC should be ~2700-3000
- 3.0V (empty) → ADC should be ~2200-2500

**If ADC is very low (< 500):**
- Wrong pin (GPIO 1 might not be battery monitoring pin)
- Battery not connected
- USB charging active (some boards can't read during charge)

### 2. Identify Correct Calculation

Based on your serial output, determine which calculation is closest to actual battery voltage:

**Option A: With divider (x2)**
```cpp
return voltage1;  // (ADC / 4095) * 3.3 * 2.0
```

**Option B: Direct (x1)** - Currently active
```cpp
return voltage2;  // (ADC / 4095) * 3.3
```

**Option C: Direct scaled**
```cpp
return voltage3;  // (ADC / 4095) * 4.2
```

### 3. Try Different ADC Pins

The LilyGO T-Display S3 AMOLED might use a different pin for battery monitoring.

Common battery ADC pins on ESP32-S3:
- GPIO 1 (current setting)
- GPIO 4
- GPIO 14
- Check LilyGO schematic for exact pin

**To test different pins:**
```cpp
#define BATTERY_PIN 4  // Try 1, 4, or 14
```

## Quick Fix Steps

### Step 1: Upload Current Firmware
The firmware now shows all three calculations in serial monitor.

### Step 2: Check Serial Output
```
Battery ADC: 250
  With divider (x2): 0.40V
  Direct (x1):       0.20V
  Direct scaled:     0.26V
```

### Step 3: Measure Actual Battery Voltage
Use a multimeter to measure actual battery voltage (should be 3.0V - 4.2V).

### Step 4: Compare and Adjust

**If actual battery is 3.8V and serial shows:**
- ADC: 250 → Wrong pin or not connected
- ADC: 2900 → Good! Now pick the right calculation

**Example:**
```
Actual battery: 3.8V
Battery ADC: 2900
  With divider (x2): 4.76V  ← Too high
  Direct (x1):       2.33V  ← Too low
  Direct scaled:     2.97V  ← Close but low
```

In this case, you might need a custom multiplier:
```cpp
float voltage = (adcValue / 4095.0) * 3.3 * 1.63;  // Custom ratio
```

### Step 5: Update Code

Once you identify the correct calculation, update `readBatteryVoltage()`:

```cpp
float readBatteryVoltage() {
  int adcSum = 0;
  for (int i = 0; i < 10; i++) {
    adcSum += analogRead(BATTERY_PIN);
    delay(1);
  }
  int adcValue = adcSum / 10;
  
  // Use the calculation that matches your actual voltage
  float voltage = (adcValue / 4095.0) * 3.3;  // Adjust this line
  
  return voltage;
}
```

## Common Scenarios

### Scenario 1: ADC reads ~250 (very low)
**Problem**: Wrong pin or battery not connected
**Solution**: 
1. Try different pins (4, 14)
2. Check if battery is connected
3. Disconnect USB and try battery only

### Scenario 2: ADC reads ~2900 but voltage is wrong
**Problem**: Wrong multiplier/divider ratio
**Solution**: Adjust the calculation based on actual voltage

### Scenario 3: ADC reads 0
**Problem**: Pin not configured or USB charging
**Solution**:
1. Verify `analogReadResolution(12)` is called in setup
2. Try battery only (no USB)
3. Check pin number

## LilyGO T-Display S3 AMOLED Specifics

According to LilyGO documentation, the battery voltage pin might be:
- **GPIO 4** with a voltage divider
- **GPIO 14** on some versions
- Check the board schematic or examples

**To find the correct pin:**
1. Check LilyGO GitHub examples
2. Look for `BATTERY_ADC_PIN` or similar
3. Try common pins: 1, 4, 14

## Temporary Workaround

If you can't get accurate readings, you can temporarily use a fixed voltage for testing:

```cpp
float readBatteryVoltage() {
  return 3.7;  // Temporary fixed value for testing display
}
```

This will show "3.7V" on the display so you can test other features.

## Next Steps

1. **Upload firmware** with debug output
2. **Check serial monitor** for ADC value and calculations
3. **Measure actual voltage** with multimeter
4. **Identify correct calculation** or pin
5. **Update code** with correct settings
6. **Remove debug output** once working

## Report Back

When checking serial monitor, report:
- ADC value (e.g., "Battery ADC: 250")
- Actual battery voltage measured with multimeter
- Which calculation is closest

This will help determine the correct configuration for your board.
