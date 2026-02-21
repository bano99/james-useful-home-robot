# Sensor Test Sketch for Gripper

This sketch tests the VL53L1X ToF distance sensor and GY-BNO055 IMU before integrating them into the main gripper control system.

## Hardware Setup

### Connections
- **VL53L1X ToF Distance Sensor**
  - VCC → 3.3V or 5V
  - GND → GND
  - SDA → A4 (Arduino Nano)
  - SCL → A5 (Arduino Nano)

- **GY-BNO055 IMU**
  - VCC → 3.3V or 5V
  - GND → GND
  - SDA → A4 (Arduino Nano)
  - SCL → A5 (Arduino Nano)

Both sensors share the I2C bus on pins A4 (SDA) and A5 (SCL).

### I2C Address Conflict Resolution
- **VL53L1X**: Starts at 0x29, changed to 0x30 at runtime
- **BNO055**: Uses 0x28 or 0x29 (depending on board configuration)

The sketch handles address conflicts automatically:
1. VL53L1X initializes first at 0x29
2. Address is changed to 0x30 using direct I2C register write
3. BNO055 initializes second (can use 0x28 or 0x29)

**No XSHUT wiring required!** Address change happens via software.

## Required Libraries

Install these libraries via Arduino IDE Library Manager:

1. **Adafruit VL53L1X** by Adafruit
2. **Adafruit BNO055** by Adafruit
3. **Adafruit Unified Sensor** by Adafruit

### Installation Steps
1. Open Arduino IDE
2. Go to Sketch → Include Library → Manage Libraries
3. Search for each library and click "Install"

## Usage

1. Connect both sensors to the Arduino Nano as described above
2. Upload the sketch to your Arduino Nano
3. Open Serial Monitor (Tools → Serial Monitor)
4. Set baud rate to 9600
5. Observe the sensor readings

## Expected Output

The sketch will display:
- Initialization status for both sensors
- BNO055 sensor details
- Calibration status
- Continuous readings every 500ms:
  - Distance from VL53L1X (in mm)
  - Orientation from BNO055 (X, Y, Z in degrees)
  - Calibration status (S=System, G=Gyro, A=Accel, M=Mag)

## Calibration

The BNO055 IMU requires calibration for best accuracy:
- **Gyroscope**: Keep the sensor still
- **Accelerometer**: Place sensor in 6 different stable positions
- **Magnetometer**: Move sensor in a figure-8 pattern

Calibration values range from 0 (uncalibrated) to 3 (fully calibrated).

## Troubleshooting

### VL53L1X Not Found
- Check wiring connections (VCC, GND, SDA=A4, SCL=A5)
- Verify 3.3V or 5V power supply
- Check I2C address (should be 0x29 at startup)
- Try adding pull-up resistors (4.7kΩ) on SDA and SCL lines if needed
- Use an I2C scanner to verify the sensor is visible

### BNO055 Not Found
- Check wiring connections
- Verify power supply (3.3V recommended)
- Check I2C address (default 0x28, or 0x29 if jumpered)
- Some BNO055 boards have address selection jumpers
- If your BNO055 is on 0x29, change `BNO055_ADDR` in the sketch

### I2C Conflicts
- Use an I2C scanner sketch to verify both devices are visible
- The sketch automatically handles address conflicts via software
- Ensure no other I2C devices conflict with 0x28, 0x29, or 0x30
- If address change fails, the sketch will report it in Serial Monitor
- If you still have issues, try powering sensors separately to test each one

### Distance Reading "Out of Range"
- VL53L1X has a range of ~40mm to 4000mm
- Ensure there's an object within range
- Check for proper sensor orientation

## Next Steps

Once both sensors are working correctly:
1. Note the typical readings and behavior
2. Integrate sensor code into `Gripper_nano_based_on_AR4_nano_sketch_v1.3.ino`
3. Add serial commands to read sensor data on demand
4. Implement sensor-based gripper control logic
