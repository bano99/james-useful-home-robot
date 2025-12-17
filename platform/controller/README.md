# James Platform Controller

ESP32-based platform controller for the James home robot with AMOLED display interface.

## Hardware

- **Board**: LilyGO T-Display S3 AMOLED V2
- **Display**: 1.91" AMOLED touchscreen (536x240 resolution)
- **MCU**: ESP32-S3 with WiFi
- **Communication**: 
  - ESP-NOW for manual control (from remote)
  - I2C-to-UART bridge for ODrive motor controllers
  - Serial/USB for autonomous commands (from Jetson)

## Features

### AMOLED Display Interface

The display shows real-time status at 10 Hz refresh rate:

#### Status Information
- **Mode**: Current control mode (MANUAL/AUTONOMOUS/STOPPED)
- **Connection Status**: 
  - Manual control connection (ESP-NOW from remote)
  - Autonomous control connection (Serial from Jetson)
- **Motor Velocities**: Visual bars showing current velocity for each wheel
  - FL (Front Left)
  - FR (Front Right)
  - BL (Back Left)
  - BR (Back Right)
- **Command Values**: Raw joystick inputs (X, Y, Rotation)
- **Calculated Values**: Velocity and direction

#### Display Layout (Landscape Mode)

```
┌──────────────────────────────────────────────────────────────┐
│ JAMES                                            MANUAL      │
├──────────────────────────────────────────────────────────────┤
│                                                              │
│    ┌───┐          ┌─────────┐              ┌──┐            │
│    │ ● │          │    ↑    │              │██│  12.3V     │
│    │   │          │  ←─┼─→  │              │██│            │
│    └───┘          │    ↓    │              │██│  85%       │
│   MANUAL          └─────────┘              └──┘            │
│                   Movement                Battery           │
│                                                              │
├──────────────────────────────────────────────────────────────┤
│ FL:1.2 FR:1.3 BL:1.2 BR:1.3                                 │
└──────────────────────────────────────────────────────────────┘

LEFT: Connection Status (Large colored circle)
  - GREEN = Connected (MANUAL or AUTO label)
  - RED = Offline

CENTER: Movement Indicator (Large circle with arrow)
  - Arrow shows direction and magnitude
  - Rotating arc shows rotation direction

RIGHT: Battery Status (Large bar graph)
  - Voltage display
  - Percentage display
  - Color coded (Green > 50%, Yellow > 20%, Red < 20%)
```

### Control Priority

1. **Manual Control** (Highest Priority)
   - Commands received via ESP-NOW from remote control
   - Takes immediate priority over autonomous commands
   - Connection timeout: 1 second

2. **Autonomous Control**
   - Commands received via Serial/USB from Jetson
   - Only executed when no manual override is active
   - Connection timeout: 1 second

3. **Emergency Stop**
   - Triggered by watchdog timer or connection loss
   - Stops all motors immediately

## Pin Configuration

### Display (QSPI)
- CS: GPIO 6
- SCK: GPIO 47
- D0: GPIO 18
- D1: GPIO 7
- D2: GPIO 48
- D3: GPIO 5
- RST: GPIO 17
- Power: GPIO 38 (must be HIGH)

### I2C (ODrive Communication)
- SDA: GPIO 3
- SCL: GPIO 2

### Serial/USB
- TX: GPIO 43
- RX: GPIO 44
- Baud: 115200

## Dependencies

Required Arduino libraries:
- `esp_now.h` - ESP-NOW communication
- `WiFi.h` - WiFi stack for ESP-NOW
- `DFRobot_IICSerial.h` - I2C to UART bridge
- `Arduino_GFX_Library.h` - Display graphics

Install via Arduino Library Manager:
```
Arduino_GFX by @moononournation
DFRobot_IICSerial by DFRobot
```

## Compilation

### Arduino IDE Settings
- Board: "ESP32S3 Dev Module" or "LilyGo T-Display-S3"
- USB CDC On Boot: "Enabled"
- Flash Size: "16MB (128Mb)"
- Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)"
- PSRAM: "OPI PSRAM"

### PlatformIO (Alternative)
```ini
[env:lilygo-t-display-s3-amoled]
platform = espressif32
board = lilygo-t-display-s3
framework = arduino
lib_deps = 
    moononournation/GFX Library for Arduino@^1.4.7
    dfrobot/DFRobot_IICSerial@^1.0.1
```

## Usage

1. **Upload Firmware**: Flash the `plattform_controller.ino` to the ESP32
2. **Power On**: The display will initialize and show the status screen
3. **Connect Remote**: Pair the remote control via ESP-NOW
4. **Connect Jetson**: Connect via USB for autonomous control

## Display Update Rate

The display updates at 10 Hz (100ms interval) to balance:
- Smooth visual feedback
- Low CPU overhead
- Minimal impact on motor control loop

## Troubleshooting

### Display Not Working
- Check GPIO 38 is HIGH (display power)
- Verify QSPI pin connections
- Check serial output for initialization errors

### Connection Issues
- Manual: Check ESP-NOW MAC address matches remote
- Autonomous: Verify USB serial connection at 115200 baud
- Check connection status on display

### Motor Control Issues
- Verify I2C connections to ODrive controllers
- Check ODrive UART configuration (115200 baud)
- Monitor serial output for command echoes

## Future Enhancements

- [ ] Touchscreen emergency stop button
- [ ] Touchscreen mode switching
- [ ] Battery level display
- [ ] Temperature monitoring
- [ ] Error message display
- [ ] Configuration menu

## References

- [LilyGO T-Display S3 AMOLED](https://lilygo.cc/products/t-display-s3-amoled)
- [Arduino_GFX Examples](https://github.com/moononournation/Arduino_GFX)
- [ODrive ASCII Protocol](https://docs.odriverobotics.com/v/latest/ascii-protocol.html)
