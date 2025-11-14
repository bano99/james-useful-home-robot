# Quick Start Guide - Remote Control with Display

Get your James robot remote control up and running in 15 minutes!

## What You Need

### Hardware:
- LilyGO T-Display S3 AMOLED V2 board
- 3-axis analog joystick (connected to pins 13, 14, 15)
- USB-C cable for programming
- Computer with Arduino IDE

**Note**: The remote control display is configured for **portrait mode** (240x536 pixels) - the remote is held vertically. The platform controller uses the same display in landscape mode (536x240) as it's mounted horizontally on the robot.

### Software:
- Arduino IDE (1.8.x or 2.x)
- ESP32 board support
- TFT_eSPI library

## Step-by-Step Installation

### 1. Install Arduino IDE
Download from: https://www.arduino.cc/en/software

### 2. Add ESP32 Board Support

1. Open Arduino IDE
2. Go to **File → Preferences**
3. In "Additional Board Manager URLs", add:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
4. Click OK
5. Go to **Tools → Board → Board Manager**
6. Search for "ESP32"
7. Install "esp32 by Espressif Systems"
8. Wait for installation to complete

### 3. Install TFT_eSPI Library

1. Go to **Sketch → Include Library → Manage Libraries**
2. Search for "TFT_eSPI"
3. Install "TFT_eSPI by Bodmer"
4. Note the installation location (usually `Documents/Arduino/libraries/TFT_eSPI`)

### 4. Configure TFT_eSPI for LilyGO Board

**Option A - Copy User_Setup.h (Recommended):**
1. Navigate to your Arduino libraries folder:
   - Windows: `Documents\Arduino\libraries\TFT_eSPI\`
   - Mac: `~/Documents/Arduino/libraries/TFT_eSPI/`
   - Linux: `~/Arduino/libraries/TFT_eSPI/`
2. Backup the existing `User_Setup.h` file (rename to `User_Setup.h.backup`)
3. Copy `platform/remote/remote_control_v2_simple/User_Setup.h` to this location
4. Overwrite the existing file

**Important**: The User_Setup.h file configures TFT_eSPI to work with the RM67162 AMOLED display using ST7789 compatibility mode.

**Option B - Manual Configuration:**
1. Open `TFT_eSPI/User_Setup.h` in a text editor
2. Comment out all existing driver definitions
3. Add these lines at the top:
   ```cpp
   #define ST7789_DRIVER      // Use ST7789 (compatible with RM67162)
   #define TFT_WIDTH  240
   #define TFT_HEIGHT 536
   #define TFT_MOSI 18
   #define TFT_SCLK 47
   #define TFT_CS   6
   #define TFT_DC   7
   #define TFT_RST  17
   #define TFT_BL   38
   #define TFT_POWER 15
   #define SPI_FREQUENCY 80000000
   
   // Required command definitions
   #define TFT_CASET   0x2A
   #define TFT_PASET   0x2B
   #define TFT_RAMWR   0x2C
   #define TFT_RAMRD   0x2E
   #define TFT_INVON   0x21
   #define TFT_INVOFF  0x20
   #define TFT_SWRST   0x01
   #define TFT_INIT_DELAY 0x80
   ```

### 5. Configure Board Settings

1. Connect your LilyGO board via USB-C
2. In Arduino IDE, go to **Tools** and set:
   - **Board**: "ESP32S3 Dev Module"
   - **USB CDC On Boot**: "Enabled"
   - **Flash Size**: "16MB (128Mb)"
   - **Partition Scheme**: "16M Flash (3MB APP/9.9MB FATFS)"
   - **PSRAM**: "OPI PSRAM"
   - **Upload Speed**: "921600"
   - **Port**: Select your COM port (e.g., COM3, /dev/ttyUSB0)

### 6. Get Platform Controller MAC Address

Before uploading the remote firmware, you need the MAC address of your platform controller.

**If you have the platform controller:**
1. Upload any firmware to the platform controller
2. Open Serial Monitor (115200 baud)
3. Note the MAC address printed (format: `24:58:7c:d3:6b:30`)

**If you don't have it yet:**
- Use the default address in the code for now
- Update it later when you have the platform controller

### 7. Update MAC Address in Firmware

1. Open `platform/remote/remote_control_v2_simple/remote_control_v2_simple.ino`
2. Find this line:
   ```cpp
   uint8_t broadcastAddress[] = {0x24, 0x58, 0x7C, 0xD3, 0x6B, 0x30};
   ```
3. Replace with your platform controller's MAC address
4. Example: If MAC is `AA:BB:CC:DD:EE:FF`, change to:
   ```cpp
   uint8_t broadcastAddress[] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
   ```

### 8. Upload Firmware

1. Click the **Upload** button (→) in Arduino IDE
2. Wait for compilation (may take 30-60 seconds)
3. Wait for upload to complete
4. You should see "Done uploading" message

### 9. Test the Remote

1. Open **Tools → Serial Monitor** (set to 115200 baud)
2. You should see:
   ```
   James Remote Control - Simple Display Version
   MAC Address: xx:xx:xx:xx:xx:xx
   Display initialized
   Setup complete
   ```
3. The display should show:
   - "JAMES REMOTE" title
   - Connection status
   - Joystick value displays with bars

4. Move the joystick - you should see:
   - Values update on display
   - Bars move left/right
   - Serial output showing sent values

## Troubleshooting

### "Board not found" or "Port not available"
- Check USB cable is connected
- Try a different USB port
- Install CH340 or CP2102 drivers if needed
- Press and hold BOOT button while connecting USB

### "Compilation error" with TFT_eSPI
- **Error about TFT_CASET, TFT_RAMWR, etc.**: User_Setup.h not configured correctly
  - Make sure you copied the User_Setup.h from `remote_control_v2_simple/` folder
  - Verify it contains `#define ST7789_DRIVER` and command definitions
  - Restart Arduino IDE after copying User_Setup.h
- Verify User_Setup.h is in correct location: `Arduino/libraries/TFT_eSPI/User_Setup.h`
- Check TFT_eSPI library is installed (version 2.4.0 or later recommended)
- Try reinstalling TFT_eSPI library if issues persist

### Display shows nothing
- Check board is powered (LED should be on)
- Verify User_Setup.h configuration
- Try uploading again
- Check Serial Monitor for error messages

### Display shows garbage/wrong colors
- Verify TFT_RGB_ORDER in User_Setup.h
- Try changing rotation: `tft.setRotation(1)` to `tft.setRotation(3)`
- Check SPI frequency (try reducing to 40MHz)

### "Status: Disconnected" always shown
- Verify platform controller MAC address is correct
- Check platform controller is powered on
- Ensure both devices are on same WiFi channel
- Check ESP-NOW is initialized on platform controller

### Joystick values don't change
- Verify joystick is connected to pins 13, 14, 15
- Check joystick power (should be 3.3V)
- Test with Serial Monitor to see raw ADC values
- Adjust dead zone if needed

## Next Steps

1. **Test with Platform Controller**: 
   - Upload firmware to platform controller
   - Verify remote can control the robot

2. **Calibrate Joystick**:
   - Adjust dead zone values if needed
   - Test sensitivity and response

3. **Add Battery Monitoring** (Optional):
   - Connect battery voltage divider to ADC pin
   - Update `getBatteryVoltage()` function
   - Test battery percentage display

4. **Customize Display** (Optional):
   - Change colors in the code
   - Adjust bar sizes and positions
   - Add additional status information

## Support

If you encounter issues:
1. Check the main README.md for detailed documentation
2. Review FIRMWARE_COMPARISON.md for version differences
3. Check Serial Monitor output for error messages
4. Verify all connections and settings

## Success Checklist

- [ ] Arduino IDE installed
- [ ] ESP32 board support installed
- [ ] TFT_eSPI library installed
- [ ] User_Setup.h configured
- [ ] Board settings correct
- [ ] MAC address updated
- [ ] Firmware uploaded successfully
- [ ] Display shows title and status
- [ ] Joystick values update on display
- [ ] Serial Monitor shows "Sent" messages

Congratulations! Your remote control is ready to use! 🎉
