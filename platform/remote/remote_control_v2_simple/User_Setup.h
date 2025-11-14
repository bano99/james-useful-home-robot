/*
 * TFT_eSPI User Setup for LilyGO T-Display S3 AMOLED V2
 * 
 * This file should be copied to the TFT_eSPI library folder:
 * Arduino/libraries/TFT_eSPI/User_Setup.h
 * 
 * Or use User_Setup_Select.h to include this file
 */

// Driver selection
#define USER_SETUP_INFO "LilyGO_T_Display_S3_AMOLED"

// Use ST7789 driver (compatible with RM67162)
#define ST7789_DRIVER

// Display resolution (Portrait mode - remote control)
#define TFT_WIDTH  240
#define TFT_HEIGHT 536

// Color depth
#define TFT_RGB_ORDER TFT_RGB  // RGB or BGR

// RM67162 specific commands (compatible with ST7789)
#define TFT_CASET   0x2A
#define TFT_PASET   0x2B
#define TFT_RAMWR   0x2C
#define TFT_RAMRD   0x2E
#define TFT_INVON   0x21
#define TFT_INVOFF  0x20
#define TFT_SWRST   0x01
#define TFT_INIT_DELAY 0x80

// Pin definitions for LilyGO T-Display S3 AMOLED
#define TFT_MOSI 18
#define TFT_SCLK 47
#define TFT_CS   6
#define TFT_DC   7
#define TFT_RST  17
#define TFT_BL   38

// Display power pin
#define TFT_POWER 15

// SPI frequency
#define SPI_FREQUENCY  80000000  // 80 MHz

// Optional: Use HSPI port
#define USE_HSPI_PORT

// Font settings
#define LOAD_GLCD   // Font 1. Original Adafruit 8 pixel font needs ~1820 bytes in FLASH
#define LOAD_FONT2  // Font 2. Small 16 pixel high font, needs ~3534 bytes in FLASH
#define LOAD_FONT4  // Font 4. Medium 26 pixel high font, needs ~5848 bytes in FLASH
#define LOAD_FONT6  // Font 6. Large 48 pixel font, needs ~2666 bytes in FLASH
#define LOAD_FONT7  // Font 7. 7 segment 48 pixel font, needs ~2438 bytes in FLASH
#define LOAD_FONT8  // Font 8. Large 75 pixel font needs ~3256 bytes in FLASH

#define SMOOTH_FONT  // Enable anti-aliased fonts

// PSRAM usage
#define USE_PSRAM

// Touch screen (if using touchscreen features)
// #define TOUCH_CS 21  // Uncomment if touchscreen is used
