# I2C Pin Configuration Guide

## Quick Answer

**SDA and SCL are NOT interchangeable!**

- **SDA** = Serial Data (bidirectional data line)
- **SCL** = Serial Clock (clock signal)

Always connect:
- Sensor SDA → ESP32 SDA pin
- Sensor SCL → ESP32 SCL pin

## Pin Selection for ESP32

### Current Configuration (Recommended)

```cpp
#define SENSOR_I2C_SDA 16  // GPIO16 for SDA
#define SENSOR_I2C_SCL 17  // GPIO17 for SCL
```

**Why GPIO16/17?**
- Commonly available on ESP32 boards
- No conflicts with other peripherals
- Good for I2C communication
- Consecutive numbering (conventional)

### Alternative Options

If GPIO16/17 are not available, use one of these:

**Option 2: GPIO25/26**
```cpp
#define SENSOR_I2C_SDA 25
#define SENSOR_I2C_SCL 26
```

**Option 3: GPIO32/33**
```cpp
#define SENSOR_I2C_SDA 32
#define SENSOR_I2C_SCL 33
```

**Option 4: GPIO21/22** (Standard, if available)
```cpp
#define SENSOR_I2C_SDA 21
#define SENSOR_I2C_SCL 22
```

## How to Change Pins

### Step 1: Edit SENSOR_CTRL.h

Open `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/SENSOR_CTRL.h`

Find these lines (near the top):
```cpp
#define SENSOR_I2C_SDA 16  // Change to available SDA pin
#define SENSOR_I2C_SCL 17  // Change to available SCL pin
```

Change to your available pins:
```cpp
#define SENSOR_I2C_SDA 25  // Example: using GPIO25
#define SENSOR_I2C_SCL 26  // Example: using GPIO26
```

### Step 2: Upload Firmware

That's it! The firmware will automatically use the new pins.

## Wiring

### VL53L1X ToF Sensor

```
VL53L1X          ESP32
--------         -----
VCC      →       3.3V
GND      →       GND
SDA      →       GPIO16 (or your SDA pin)
SCL      →       GPIO17 (or your SCL pin)
```

### BNO055 IMU Sensor

```
BNO055           ESP32
------           -----
VIN      →       3.3V or 5V (check your module)
GND      →       GND
SDA      →       GPIO16 (or your SDA pin)
SCL      →       GPIO17 (or your SCL pin)
```

**Both sensors share the same I2C bus** - connect them in parallel.

## Pin Restrictions

### Avoid These Pins

**Don't use for I2C:**
- GPIO0: Boot mode selection (can cause boot issues)
- GPIO1/3: USB serial (UART0) - needed for programming
- GPIO6-11: Connected to flash memory (DON'T USE!)
- GPIO12: Boot voltage selection (can cause issues)

### Safe Pins for I2C

**Good choices:**
- GPIO16, 17 ✅ (recommended)
- GPIO21, 22 ✅ (standard, if available)
- GPIO25, 26 ✅ (good alternative)
- GPIO32, 33 ✅ (acceptable, ADC pins)
- GPIO4, 5 ✅ (if available)
- GPIO13, 14, 15 ✅ (if available)

## Common Pin Conventions

### Why Use Consecutive Numbers?

It's conventional (but not required) to use consecutive GPIO numbers:
- 16/17 ✅ Common
- 21/22 ✅ Standard
- 25/26 ✅ Common
- 32/33 ✅ Common

You could use non-consecutive pins:
- 16/26 ✅ Works fine
- 25/33 ✅ Works fine

But consecutive numbers are easier to remember and document.

### SDA Usually Lower Number

By convention, SDA is often the lower GPIO number:
- SDA=16, SCL=17 ✅ Conventional
- SDA=17, SCL=16 ✅ Works but unconventional

This matches the standard I2C pins (SDA=21, SCL=22).

## Testing Pin Configuration

### Step 1: Upload Firmware

After changing pins in SENSOR_CTRL.h, upload firmware.

### Step 2: Check Serial Monitor

Open Serial Monitor (115200 baud). You should see:
```
Initializing sensors on I2C (SDA=GPIO16, SCL=GPIO17)...
```

This confirms which pins are being used.

### Step 3: Test Sensor Detection

Type `SS` in Serial Monitor:
```
VL53L1X:OK BNO055:OK
```

If sensors show ERROR, check:
1. Wiring (SDA/SCL not swapped?)
2. Power (3.3V connected?)
3. Pin numbers correct in firmware?

## I2C Scanner (Troubleshooting)

If sensors aren't detected, run an I2C scanner:

```cpp
// Add to setup() temporarily
void scanI2C() {
  Serial.println("Scanning I2C bus...");
  for(byte addr = 1; addr < 127; addr++) {
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) {
      Serial.print("Device found at 0x");
      if (addr < 16) Serial.print("0");
      Serial.println(addr, HEX);
    }
  }
}
```

Expected devices:
- 0x29: BNO055
- 0x30: VL53L1X (after address change)
- 0x40: PCA9685 (if present on board)
- 0x3C: OLED (if present on board)

## Summary

1. **SDA and SCL are NOT interchangeable** - they have different functions
2. **GPIO pin numbers ARE flexible** - use what's available on your board
3. **Recommended: GPIO16/17** - commonly available, no conflicts
4. **Easy to change** - just edit 2 lines in SENSOR_CTRL.h
5. **Test with Serial Monitor** - verify pins and sensor detection

---

**Current Configuration**: SDA=GPIO16, SCL=GPIO17
**To Change**: Edit SENSOR_CTRL.h, lines 28-29
**To Test**: Upload firmware, type `SS` in Serial Monitor
