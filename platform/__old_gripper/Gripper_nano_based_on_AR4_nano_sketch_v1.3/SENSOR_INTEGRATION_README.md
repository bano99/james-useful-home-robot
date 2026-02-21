# Gripper Nano Sketch v1.4 - Sensor Integration

This version extends the original AR4 Nano sketch with VL53L1X ToF distance sensor and BNO055 IMU support.

## Changes from Original v1.3

### Hardware Changes
- **A4 and A5 pins**: Reserved for I2C communication (sensors)
- **Servo usage**: Only servo0 on A0 is used for gripper control
- **Servos 4 and 5**: Removed (were on A4/A5, now used for I2C)

### Software Changes
- Added sensor libraries (Adafruit_VL53L1X, Adafruit_BNO055)
- Added sensor initialization in `setup()`
- Added three new serial commands for sensor data
- Sensors initialize automatically on startup
- VL53L1X address changed to 0x30 to avoid conflict with BNO055 at 0x29

## New Serial Commands

All commands follow the existing protocol: two-letter command code followed by parameters, terminated with `\n`.

### RD - Read Distance
**Command**: `RD\n`

**Response**: 
- Success: `D<distance>\n` where distance is in millimeters
- Failure: `ERROR\n`

**Example**:
```
Send: RD\n
Receive: D245\n  (245mm distance)
```

### RO - Read Orientation
**Command**: `RO\n`

**Response**:
- Success: `OX<x>Y<y>Z<z>\n` where x, y, z are Euler angles in degrees
- Failure: `ERROR\n`

**Example**:
```
Send: RO\n
Receive: OX45.23Y12.45Z180.00\n
```

### RC - Read Calibration
**Command**: `RC\n`

**Response**:
- Success: `CS<sys>G<gyro>A<accel>M<mag>\n` where values are 0-3 (0=uncalibrated, 3=fully calibrated)
- Failure: `ERROR\n`

**Example**:
```
Send: RC\n
Receive: CS3G3A3M2\n  (System=3, Gyro=3, Accel=3, Mag=2)
```

## Existing Commands (Unchanged)

All original commands remain functional:
- `SV` - Move servo (only servo0 works)
- `JF` - Jump if input
- `ON` - Set output on
- `OF` - Set output off
- `WI` - Wait for input
- `TM` - Test message echo

## Integration with Future Versions

To integrate sensor support into a new AR4 sketch version:

1. **Add includes** (after `#include <Servo.h>`):
```cpp
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>
```

2. **Add sensor objects** (after servo declarations):
```cpp
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);
bool vl53Initialized = false;
bool bnoInitialized = false;
#define VL53L1X_DEFAULT_ADDR 0x29
#define VL53L1X_NEW_ADDR 0x30
```

3. **Add helper function** (before `setup()`):
```cpp
bool changeVL53L1XAddress(uint8_t new_address) {
  Wire.beginTransmission(VL53L1X_DEFAULT_ADDR);
  Wire.write(0x00);
  Wire.write(0x01);
  Wire.write(new_address << 1);
  return (Wire.endTransmission() == 0);
}

void initializeSensors() {
  Wire.begin();
  
  bool foundAt30 = vl53.begin(VL53L1X_NEW_ADDR, &Wire);
  if (!foundAt30) {
    if (vl53.begin(VL53L1X_DEFAULT_ADDR, &Wire)) {
      changeVL53L1XAddress(VL53L1X_NEW_ADDR);
      delay(50);
    }
  }
  
  if (vl53.startRanging()) {
    vl53.setTimingBudget(50);
    vl53Initialized = true;
  }
  
  delay(100);
  if (bno.begin()) {
    delay(1000);
    bno.setExtCrystalUse(true);
    bnoInitialized = true;
  }
}
```

4. **Modify `setup()`**:
   - Remove `pinMode(A4, OUTPUT)` and `pinMode(A5, OUTPUT)`
   - Remove servo attachments for servo4 and servo5
   - Add `initializeSensors();` at the end

5. **Add command handlers in `loop()`** (add the three new command blocks for RD, RO, RC)

## Hardware Setup

### Connections
- **VL53L1X**: VCC, GND, SDA→A4, SCL→A5
- **BNO055**: VCC, GND, SDA→A4, SCL→A5
- **Gripper Servo**: Signal→A0

### I2C Addresses
- VL53L1X: 0x30 (changed from 0x29 at startup)
- BNO055: 0x29

## Testing

Use Arduino Serial Monitor (9600 baud) to test:

```
TM Test\n          → Should echo "Test"
SV0P90\n           → Move gripper servo to 90°
RD\n               → Read distance
RO\n               → Read orientation
RC\n               → Read calibration status
```

## Required Libraries

Install via Arduino Library Manager:
1. Adafruit VL53L1X
2. Adafruit BNO055
3. Adafruit Unified Sensor

## Notes

- Sensors initialize automatically on startup
- If a sensor fails to initialize, its commands will return `ERROR`
- VL53L1X address change is temporary (resets on power cycle)
- BNO055 requires calibration for best accuracy (move in figure-8 pattern)
- Sensor data is read on-demand only (no continuous polling)
