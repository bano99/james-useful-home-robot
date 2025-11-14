# Compilation Troubleshooting Guide

## Common TFT_eSPI Compilation Errors

### Error: TFT_CASET, TFT_PASET, TFT_RAMWR not declared

**Full Error Message:**
```
error: 'TFT_CASET' was not declared in this scope
error: 'TFT_PASET' was not declared in this scope  
error: 'TFT_RAMWR' was not declared in this scope
error: 'TFT_RAMRD' was not declared in this scope
error: 'TFT_INVON' was not declared in this scope
error: 'TFT_INVOFF' was not declared in this scope
error: 'TFT_SWRST' was not declared in this scope
error: 'TFT_INIT_DELAY' was not declared in this scope
```

**Cause:**
The TFT_eSPI library doesn't have native support for the RM67162 AMOLED display. The User_Setup.h file needs to be properly configured with ST7789 driver compatibility and command definitions.

**Solution:**

1. **Locate your TFT_eSPI library folder:**
   - Windows: `C:\Users\[YourName]\Documents\Arduino\libraries\TFT_eSPI\`
   - Mac: `~/Documents/Arduino/libraries/TFT_eSPI/`
   - Linux: `~/Arduino/libraries/TFT_eSPI/`

2. **Backup the original User_Setup.h:**
   ```
   Rename: User_Setup.h → User_Setup.h.backup
   ```

3. **Copy the correct User_Setup.h:**
   - Source: `platform/remote/remote_control_v2_simple/User_Setup.h`
   - Destination: `Arduino/libraries/TFT_eSPI/User_Setup.h`

4. **Verify the file contains these key lines:**
   ```cpp
   #define ST7789_DRIVER      // Use ST7789 (compatible with RM67162)
   
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

5. **Restart Arduino IDE** (important!)

6. **Try compiling again**

### Error: RM67162_DRIVER not recognized

**Error Message:**
```
warning: RM67162_DRIVER defined but not used
```

**Cause:**
TFT_eSPI doesn't have RM67162_DRIVER defined. We use ST7789_DRIVER instead as it's compatible.

**Solution:**
Make sure your User_Setup.h uses `ST7789_DRIVER` not `RM67162_DRIVER`:
```cpp
#define ST7789_DRIVER  // Correct
// NOT: #define RM67162_DRIVER
```

### Error: TOUCH_CS pin not defined

**Warning Message:**
```
warning: TOUCH_CS pin not defined, TFT_eSPI touch functions will not be available!
```

**Cause:**
This is just a warning. The LilyGO board has a touchscreen but we're not using touch functions in the basic firmware.

**Solution:**
- **Ignore this warning** - it won't prevent compilation
- Or add to User_Setup.h to suppress warning:
  ```cpp
  #define TOUCH_CS -1  // No touch CS pin
  ```

### Error: Multiple definition of setup/loop

**Error Message:**
```
multiple definition of `setup'
multiple definition of `loop'
```

**Cause:**
Arduino IDE is trying to compile multiple .ino files in the same folder.

**Solution:**
Make sure you only have ONE .ino file in the sketch folder:
- Correct: `remote_control_v2_simple/remote_control_v2_simple.ino`
- Remove any other .ino files from the same folder

### Error: TFT_eSPI.h: No such file or directory

**Error Message:**
```
fatal error: TFT_eSPI.h: No such file or directory
```

**Cause:**
TFT_eSPI library is not installed.

**Solution:**
1. Open Arduino IDE
2. Go to **Sketch → Include Library → Manage Libraries**
3. Search for "TFT_eSPI"
4. Install "TFT_eSPI by Bodmer"
5. Restart Arduino IDE

### Error: esp_now.h: No such file or directory

**Error Message:**
```
fatal error: esp_now.h: No such file or directory
```

**Cause:**
ESP32 board support is not installed or wrong board selected.

**Solution:**
1. Install ESP32 board support (see QUICK_START.md)
2. Select correct board: **Tools → Board → ESP32S3 Dev Module**
3. Verify ESP32 core version is 2.0.0 or later

## Step-by-Step Verification Checklist

Use this checklist to verify your setup:

- [ ] TFT_eSPI library installed (version 2.4.0+)
- [ ] ESP32 board support installed (version 2.0.0+)
- [ ] User_Setup.h copied to TFT_eSPI library folder
- [ ] User_Setup.h contains `#define ST7789_DRIVER`
- [ ] User_Setup.h contains all command definitions (TFT_CASET, etc.)
- [ ] Arduino IDE restarted after copying User_Setup.h
- [ ] Correct board selected: ESP32S3 Dev Module
- [ ] Only one .ino file in sketch folder
- [ ] Sketch folder name matches .ino filename

## Manual User_Setup.h Configuration

If copying the file doesn't work, manually edit `Arduino/libraries/TFT_eSPI/User_Setup.h`:

1. **Comment out all existing driver definitions:**
   ```cpp
   // #define ILI9341_DRIVER
   // #define ST7735_DRIVER
   // etc... comment them all out
   ```

2. **Add at the top of the file:**
   ```cpp
   // LilyGO T-Display S3 AMOLED Configuration
   #define USER_SETUP_INFO "LilyGO_T_Display_S3_AMOLED"
   
   // Use ST7789 driver (compatible with RM67162)
   #define ST7789_DRIVER
   
   // Display resolution (Portrait mode)
   #define TFT_WIDTH  240
   #define TFT_HEIGHT 536
   
   // Pin definitions
   #define TFT_MOSI 18
   #define TFT_SCLK 47
   #define TFT_CS   6
   #define TFT_DC   7
   #define TFT_RST  17
   #define TFT_BL   38
   #define TFT_POWER 15
   
   // SPI frequency
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
   
   // Color order
   #define TFT_RGB_ORDER TFT_RGB
   
   // Font loading
   #define LOAD_GLCD
   #define LOAD_FONT2
   #define LOAD_FONT4
   #define LOAD_FONT6
   #define LOAD_FONT7
   #define LOAD_FONT8
   #define SMOOTH_FONT
   ```

3. **Save the file**

4. **Restart Arduino IDE**

5. **Try compiling again**

## Still Having Issues?

### Check Library Versions

Open Arduino IDE and go to **Sketch → Include Library → Manage Libraries**:
- TFT_eSPI: Should be 2.4.0 or later
- ESP32: Should be 2.0.0 or later (check in Board Manager)

### Verify File Locations

1. **Check TFT_eSPI is installed:**
   ```
   Arduino/libraries/TFT_eSPI/TFT_eSPI.h should exist
   ```

2. **Check User_Setup.h location:**
   ```
   Arduino/libraries/TFT_eSPI/User_Setup.h should exist
   ```

3. **Check sketch location:**
   ```
   platform/remote/remote_control_v2_simple/remote_control_v2_simple.ino
   ```

### Enable Verbose Compilation

To see more detailed error messages:
1. Go to **File → Preferences**
2. Check "Show verbose output during: compilation"
3. Try compiling again
4. Look for the first error in the output

### Clean Build

Sometimes Arduino IDE caches old files:
1. Close Arduino IDE
2. Delete the build cache:
   - Windows: `C:\Users\[YourName]\AppData\Local\Temp\arduino_build_*`
   - Mac: `~/Library/Arduino15/`
   - Linux: `~/.arduino15/`
3. Restart Arduino IDE
4. Try compiling again

### Alternative: Use Arduino CLI

If Arduino IDE continues to have issues:
```bash
# Install Arduino CLI
# Then compile with:
arduino-cli compile --fqbn esp32:esp32:esp32s3 remote_control_v2_simple
```

## Getting Help

If you're still stuck:

1. **Check the error message carefully** - the first error is usually the real problem
2. **Verify all steps in QUICK_START.md** were followed
3. **Try the v1 firmware** (no display) to verify basic ESP32 functionality
4. **Post in Arduino forums** with:
   - Full error message
   - Arduino IDE version
   - TFT_eSPI version
   - ESP32 core version
   - Board selected

## Success Indicators

You'll know it's working when:
- ✅ Compilation completes with "Done compiling"
- ✅ Only warnings (not errors) about TOUCH_CS
- ✅ Upload succeeds with "Done uploading"
- ✅ Serial Monitor shows initialization messages
- ✅ Display lights up and shows UI

Good luck! 🚀
