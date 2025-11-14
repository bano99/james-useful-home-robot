# James Robot - Remote Control Firmware

This directory contains the firmware for the James robot remote control, built on the LilyGO T-Display S3 AMOLED V2 board.

## Hardware

- **Board**: LilyGO T-Display S3 AMOLED V2
- **Display**: 1.91" AMOLED touchscreen (536x240 native resolution)
  - **Remote Control**: Portrait mode (240x536) - held vertically
  - **Platform Controller**: Landscape mode (536x240) - mounted horizontally at front of robot
- **MCU**: ESP32-S3 with WiFi
- **Joystick**: 3-axis analog joystick (pins 13, 14, 15)
- **Communication**: ESP-NOW protocol

## Firmware Versions

### remote_control_v1.ino
Basic remote control firmware without display support.
- Reads 3-axis joystick input
- Sends commands via ESP-NOW
- Serial output for debugging

### remote_control_v2_simple.ino (Recommended)
Enhanced version with AMOLED display using TFT_eSPI library.
- **Easier to compile** - uses standard TFT_eSPI library
- Real-time joystick value display with visual bars
- Connection status and signal strength
- Battery level monitoring support
- Simpler setup and configuration

### remote_control_v2_amoled.ino (Advanced)
Enhanced version with AMOLED display using LVGL.

**Features:**
- Real-time joystick value display with visual bars
- Connection status indicator (Connected/Disconnected)
- Signal strength percentage with color coding
- Battery level monitoring (if hardware available)
- 10 Hz display refresh rate
- Color-coded status indicators:
  - Green: Good/Connected
  - Yellow: Warning/Moderate
  - Red: Error/Disconnected

**Display Layout (Portrait Mode):**
```
┌─────────────────────┐
│      JAMES          │
│      REMOTE         │
│                     │
│ Forward/Back: 127   │
│ ┌────┐              │
│ │░░░░│              │
│ │████│              │
│ │████│              │
│ │░░░░│              │
│ └────┘              │
│                     │
│ Left/Right: -64     │
│ ┌────┐              │
│ │░░░░│              │
│ │████│              │
│ │░░░░│              │
│ └────┘              │
│                     │
│ Rotation: 0         │
│ ┌────┐              │
│ │░░░░│              │
│ │█░░░│              │
│ │░░░░│              │
│ └────┘              │
│                     │
│ Status: Connected   │
│ Signal: 95%         │
│ Battery: N/A        │
└─────────────────────┘
```

## Dependencies

### Arduino Libraries Required:

**For remote_control_v2_simple.ino (Recommended):**
- **ESP32 Board Support**: Install via Arduino Board Manager
- **TFT_eSPI**: Install via Arduino Library Manager
  - After installation, copy `User_Setup.h` to the TFT_eSPI library folder
  - Or configure TFT_eSPI to use the provided User_Setup.h
- **ESP-NOW**: Built into ESP32 core

**For remote_control_v2_amoled.ino (Advanced):**
- **ESP32 Board Support**: Install via Arduino Board Manager
- **LVGL** (v8.x): Light and Versatile Graphics Library
  - Install via Arduino Library Manager or from: https://github.com/lvgl/lvgl
- **ESP-NOW**: Built into ESP32 core

### Custom Files:
- `rm67162.h` / `rm67162.cpp`: Display driver for RM67162 AMOLED (for LVGL version)
- `lv_conf.h`: LVGL configuration (for LVGL version)
- `User_Setup.h`: TFT_eSPI configuration (for simple version)

## Installation

1. **Install Arduino IDE** (1.8.x or 2.x)

2. **Install ESP32 Board Support**:
   - Open Arduino IDE
   - Go to File → Preferences
   - Add to "Additional Board Manager URLs":
     ```
     https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
     ```
   - Go to Tools → Board → Board Manager
   - Search for "ESP32" and install "esp32 by Espressif Systems"

3. **Install LVGL Library**:
   - Go to Sketch → Include Library → Manage Libraries
   - Search for "lvgl" and install version 8.x
   - Copy `lv_conf.h` to your Arduino libraries folder

4. **Configure Board**:
   - Select Board: "ESP32S3 Dev Module"
   - USB CDC On Boot: "Enabled"
   - Flash Size: "16MB (128Mb)"
   - Partition Scheme: "16M Flash (3MB APP/9.9MB FATFS)"
   - PSRAM: "OPI PSRAM"

5. **Install TFT_eSPI Library** (for simple version):
   - Go to Sketch → Include Library → Manage Libraries
   - Search for "TFT_eSPI" and install
   - **Important**: Copy `remote_control_v2_simple/User_Setup.h` to `Arduino/libraries/TFT_eSPI/` folder (overwrite existing)
   - This configures TFT_eSPI for the RM67162 AMOLED display

6. **Upload Firmware**:
   - Open `remote_control_v2_simple.ino` (recommended) or `remote_control_v2_amoled.ino`
   - Update `broadcastAddress` with your platform controller MAC address
   - Click Upload

## Configuration

### MAC Address Setup
The remote control needs to know the MAC address of the platform controller.

1. Find the platform controller MAC address:
   - Upload firmware to platform controller
   - Open Serial Monitor
   - Note the MAC address printed at startup

2. Update remote control firmware:
   ```cpp
   uint8_t broadcastAddress[] = {0x24, 0x58, 0x7C, 0xD3, 0x6B, 0x30};
   ```
   Replace with your platform controller's MAC address.

### Joystick Calibration
If joystick values seem incorrect, adjust the dead zone:
```cpp
const int adcDeadZoneMin = 1900;  // Lower threshold
const int adcDeadZoneMax = 2250;  // Upper threshold
```

### Battery Monitoring (Optional)
To enable battery monitoring:
1. Connect battery voltage divider to an available ADC pin
2. Update `getBatteryVoltage()` function:
   ```cpp
   float getBatteryVoltage() {
     const int BATTERY_PIN = 4;  // Your ADC pin
     const float VOLTAGE_DIVIDER = 2.0;  // Your divider ratio
     float voltage = analogRead(BATTERY_PIN) * (V_REF / ADC_MAX) * VOLTAGE_DIVIDER;
     return voltage;
   }
   ```

## Troubleshooting

### Compilation Errors
**See [TROUBLESHOOTING_COMPILATION.md](TROUBLESHOOTING_COMPILATION.md) for detailed solutions to:**
- TFT_CASET, TFT_RAMWR not declared errors
- RM67162_DRIVER not recognized
- TFT_eSPI configuration issues
- User_Setup.h problems

### Display Not Working
- Check SPI connections
- Verify TFT_eSPI library is installed correctly
- Ensure `User_Setup.h` is copied to TFT_eSPI library folder
- Restart Arduino IDE after copying User_Setup.h
- Check serial output for initialization messages

### ESP-NOW Connection Failed
- Verify MAC address is correct
- Check both devices are on the same WiFi channel
- Ensure ESP-NOW is initialized before adding peer
- Check serial output for error messages

### Joystick Values Incorrect
- Verify joystick is connected to pins 13, 14, 15
- Check joystick power supply (3.3V)
- Adjust dead zone values if needed
- Test with Serial Monitor to see raw ADC values

### Display Flickering
- Reduce display update rate (increase `DISPLAY_UPDATE_MS`)
- Check SPI frequency (try reducing from 80MHz)
- Ensure adequate power supply

## Pin Mapping

### Joystick Pins:
- Pin 13: Forward/Backward (Y-axis)
- Pin 14: Left/Right (X-axis)
- Pin 15: Rotation (Z-axis)

### Display Pins (Internal):
- Pin 6: CS (Chip Select)
- Pin 47: SCLK (SPI Clock)
- Pin 18: MOSI (SPI Data)
- Pin 7: DC (Data/Command)
- Pin 17: RST (Reset)
- Pin 38: BL (Backlight)
- Pin 15: POWER (Display Power)

## Performance

- **Transmission Rate**: ~6.7 Hz (150ms interval)
- **Display Update Rate**: 10 Hz (100ms interval)
- **Latency**: < 200ms end-to-end
- **Range**: ~50-100m (depending on environment)

## Future Enhancements

- [ ] Touchscreen button support for emergency stop
- [ ] Configuration menu via touchscreen
- [ ] Joystick sensitivity adjustment
- [ ] Dead zone configuration
- [ ] Settings persistence in EEPROM
- [ ] Multiple control profiles
- [ ] Haptic feedback
- [ ] Low battery warning

## References

- LilyGO AMOLED Series: https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series
- LVGL Documentation: https://docs.lvgl.io/
- ESP-NOW Documentation: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/api-reference/network/esp_now.html

## License

This firmware is part of the James Robot project and is released under the same license as the main project.
