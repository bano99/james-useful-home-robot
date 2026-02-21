# Servo Driver Serial Commands

This document describes the serial commands for controlling the ST servo driver via USB/UART.

## Connection Settings
- Baud Rate: 115200
- Format: 8N1 (8 data bits, no parity, 1 stop bit)

## Servo Control Protocol

The servo driver uses text-based commands that are parsed by the ESP32 and converted to the Feetech STS/SMS servo protocol using the SCServo library.

### Basic Servo Commands

The driver uses the SCServo library protocol. Here are the key commands:

#### Position Control (Servo Mode)
```
#<ID>P<Position>S<Speed>T<Time>
```
- `ID`: Servo ID (0-253)
- `Position`: Target position (0-4095 for ST servos, 360° range)
- `Speed`: Movement speed (0-4000)
- `Time`: Time to reach position in milliseconds (optional)

**Examples:**
```
#1P2047S1000     // Move servo ID 1 to center position (2047) at speed 1000
#1P0S500         // Move servo ID 1 to 0° at speed 500
#1P4095S2000     // Move servo ID 1 to max position at speed 2000
```

#### Read Position
```
#<ID>P?
```
**Example:**
```
#1P?             // Read current position of servo ID 1
```

#### Read Load
```
#<ID>L?
```
Returns the current load on the servo (-1000 to +1000). Useful for detecting when gripper hits an object or mechanical stop.

**Example:**
```
#1L?             // Read current load of servo ID 1
```

#### Enable/Disable Torque
```
#<ID>T<0|1>
```
- `0`: Disable torque (servo can be moved manually)
- `1`: Enable torque (servo holds position)

**Examples:**
```
#1T0             // Disable torque on servo ID 1
#1T1             // Enable torque on servo ID 1
```

#### Set Servo Mode (Position Control)
```
#<ID>M0
```
**Example:**
```
#1M0             // Set servo ID 1 to position control mode
```

#### Set Motor Mode (Continuous Rotation)
```
#<ID>M3
```
**Example:**
```
#1M3             // Set servo ID 1 to continuous rotation mode
```

#### Ping Servo
```
#<ID>PING
```
**Example:**
```
#1PING           // Check if servo ID 1 is connected
```

#### Auto-Calibrate to Open Position
```
#<ID>CALOPEN
```
Automatically moves gripper to fully open position until hitting mechanical stop. Uses load monitoring to detect the hard stop safely.

**Example:**
```
#1CALOPEN        // Calibrate servo ID 1 to max open position
```

#### Auto-Calibrate to Close Position
```
#<ID>CALCLOSE
```
Automatically moves gripper to fully closed position until hitting mechanical stop.

**Example:**
```
#1CALCLOSE       // Calibrate servo ID 1 to min close position
```

## Sensor Commands

These commands are handled by the ESP32 firmware (not forwarded to servos):

### Distance Sensor (VL53L1X)
```
RD               // Read distance in millimeters
```
**Response:** `D<distance>` or error message

### IMU Orientation (BNO055)
```
RO               // Read orientation (Euler angles)
```
**Response:** `OX<x>Y<y>Z<z>` (degrees)

### IMU Calibration Status
```
RC               // Read calibration status
```
**Response:** `CS<sys>G<gyro>A<accel>M<mag>` (0-3 for each)

### Sensor Status
```
SS               // Check sensor initialization status
```
**Response:** `VL53L1X:OK/ERROR BNO055:OK/ERROR`

### I2C Bus Scan
```
SCAN             // Scan I2C bus for connected devices
I2C              // Alternative command for I2C scan
```

### Sensor Info
```
INFO             // Display detailed sensor information
```

### Help
```
?                // Show available commands
HELP             // Show available commands
```

## Position Values Reference

For ST servos (360° range, 4096 steps):
- 0° = 0
- 90° = 1024
- 180° = 2047 (center)
- 270° = 3071
- 360° = 4095

**Formula:** `Position = (Angle / 360) * 4095`

## Speed Values

- Range: 0-4000
- Higher values = faster movement
- Recommended: 500-2000 for smooth motion
- Default: 2000

## Common Use Cases

### Initialize Servo to Center
```
#1P2047S1000
```

### Sweep Motion
```
#1P0S1000
#1P4095S1000
#1P0S1000
```

### Check Servo Connection
```
#1PING
```

### Disable Torque for Manual Positioning
```
#1T0
```

### Enable Torque and Hold Position
```
#1T1
```

## Troubleshooting

1. **No response from servo:**
   - Check servo ID matches command
   - Verify servo power supply
   - Check UART wiring (GPIO18=RX, GPIO19=TX)
   - Try ping command: `#1PING`
   - Upload the latest firmware with text command parser

2. **"Command must start with #" error:**
   - Servo commands must begin with # character
   - Check serial monitor line ending is set to "Newline"

3. **Servo moves erratically:**
   - Reduce speed value
   - Check power supply voltage (should be stable)
   - Verify servo mode is set correctly

4. **Sensors not detected:**
   - Run `SCAN` command to check I2C bus
   - Verify wiring: GPIO16=SDA, GPIO17=SCL
   - Check 3.3V power to sensors

5. **Web interface works but serial doesn't:**
   - Make sure you uploaded the latest firmware with SERIAL_PARSER.h
   - Check baud rate is 115200
   - Verify line ending is set to Newline (\n)

## Firmware Update Required

If serial commands don't work, you need to upload the updated firmware:
1. Open `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/ServoDriver.ino` in Arduino IDE
2. Make sure `SERIAL_PARSER.h` is included
3. Upload to ESP32
4. Test with Serial Monitor or `test_serial_control.py`

## Notes

- Text commands (starting with #) are parsed by ESP32 and converted to SCServo binary protocol
- Sensor commands (RD, RO, RC, SS, SCAN, INFO, HELP) are handled by ESP32 firmware
- The display shows normal status information during operation
- Web interface and serial interface both work simultaneously
