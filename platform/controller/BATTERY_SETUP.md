# Battery Monitoring Setup Guide

## Hardware Configuration

The LilyGO T-Display S3 AMOLED V2 has built-in battery monitoring circuitry.

### Pin Configuration
- **ADC Pin**: GPIO 1 (built-in, do not change)
- **Voltage Divider**: 2:1 ratio (built-in on board)
- **Battery Type**: 1S LiPo (3.7V nominal)

## Voltage Ranges

### 1S LiPo (Current Configuration)
- **Full**: 4.2V (100%)
- **Nominal**: 3.7V (~60%)
- **Low**: 3.3V (~20%)
- **Empty**: 3.0V (0%)

### Color Coding
- **Green**: > 50% (> 3.6V)
- **Yellow**: 20-50% (3.24V - 3.6V)
- **Red**: < 20% (< 3.24V)

## Troubleshooting

### Battery Shows 0.00V or No Reading

**Possible Causes:**
1. **USB Charging Active**: The board may not read battery voltage while USB is connected and charging
2. **No Battery Connected**: Verify battery is properly connected
3. **Wrong ADC Pin**: Should be GPIO 1 (built-in)

**Solutions:**
1. Disconnect USB and run on battery only to test
2. Check serial monitor for debug output:
   ```
   Battery ADC: 2048, Voltage: 3.35V
   ```
3. If ADC value is 0, battery is not connected or charging circuit is active

### Battery Shows Wrong Voltage

**Check Voltage Divider Ratio:**
The board has a 2:1 voltage divider. If readings are off:

1. Measure actual battery voltage with multimeter
2. Compare with displayed voltage
3. Adjust `BATTERY_DIVIDER_RATIO` if needed:
   ```cpp
   // If displayed voltage is half of actual:
   #define BATTERY_DIVIDER_RATIO 2.0
   
   // If displayed voltage is double actual:
   #define BATTERY_DIVIDER_RATIO 1.0
   ```

### Battery Percentage Incorrect

**Adjust Voltage Range:**
Edit in `drawBatteryIndicator()` function:
```cpp
float minVoltage = 3.0;   // Adjust for your battery's empty voltage
float maxVoltage = 4.2;   // Adjust for your battery's full voltage
```

## Testing Battery Monitoring

### Step 1: Check Serial Output
Upload firmware and open serial monitor (115200 baud):
```
Battery ADC: 2500, Voltage: 4.09V
Battery ADC: 2480, Voltage: 4.06V
Battery ADC: 2460, Voltage: 4.03V
```

### Step 2: Verify ADC Range
- **Full battery (4.2V)**: ADC should be ~2550
- **Nominal (3.7V)**: ADC should be ~2250
- **Low (3.3V)**: ADC should be ~2000
- **Empty (3.0V)**: ADC should be ~1820

### Step 3: Check Display
- Voltage should match serial output
- Percentage should be reasonable
- Color should match battery level

## USB Charging Behavior

When USB is connected:
- Battery may be charging
- Voltage reading may be affected
- Some boards show 0V during charging
- This is normal behavior

**To test battery monitoring:**
1. Fully charge battery with USB
2. Disconnect USB
3. Power on with battery only
4. Check voltage display

## Calibration

If you need precise readings:

1. **Measure actual voltage** with multimeter
2. **Read ADC value** from serial monitor
3. **Calculate actual ratio**:
   ```
   Actual Ratio = (Measured Voltage) / ((ADC / 4095) * 3.3)
   ```
4. **Update in code**:
   ```cpp
   #define BATTERY_DIVIDER_RATIO 2.0  // Use calculated value
   ```

## Example Readings

### Healthy 1S LiPo
```
Battery ADC: 2550, Voltage: 4.18V  → 100% (Green)
Battery ADC: 2400, Voltage: 3.93V  →  77% (Green)
Battery ADC: 2200, Voltage: 3.60V  →  50% (Yellow)
Battery ADC: 2000, Voltage: 3.27V  →  23% (Yellow)
Battery ADC: 1850, Voltage: 3.03V  →   2% (Red)
```

### USB Connected (Charging)
```
Battery ADC: 0, Voltage: 0.00V  → May show 0% (Normal during charge)
```

## Safety Notes

⚠️ **LiPo Battery Safety:**
- Never discharge below 3.0V per cell
- Never charge above 4.2V per cell
- Monitor temperature during use
- Use proper LiPo charging equipment
- Store at 3.7V-3.8V for long term

⚠️ **Low Battery Warning:**
When display shows RED (<20%):
- Stop using robot immediately
- Recharge battery
- Do not continue operation
