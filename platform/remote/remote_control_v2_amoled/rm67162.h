/*
 * RM67162 AMOLED Display Driver
 * For LilyGO T-Display S3 AMOLED V2
 * 
 * Display specifications:
 * - Resolution: 536x240 pixels
 * - Interface: QSPI
 * - Driver IC: RM67162
 */

#ifndef _RM67162_H_
#define _RM67162_H_

#include <Arduino.h>
#include <SPI.h>

// Display dimensions
#define LCD_WIDTH  536
#define LCD_HEIGHT 240

// QSPI pins for LilyGO T-Display S3 AMOLED
#define LCD_CS     6
#define LCD_SCLK   47
#define LCD_MOSI   18
#define LCD_MISO   -1  // Not used
#define LCD_DC     7
#define LCD_RST    17
#define LCD_BL     38
#define LCD_POWER  15

// Color definitions (RGB565 format)
#define BLACK       0x0000
#define BLUE        0x001F
#define RED         0xF800
#define GREEN       0x07E0
#define CYAN        0x07FF
#define MAGENTA     0xF81F
#define YELLOW      0xFFE0
#define WHITE       0xFFFF

// Function declarations
void rm67162_init(void);
void lcd_setRotation(uint8_t rotation);
void lcd_fill(uint16_t color);
void lcd_PushColors(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *data);
void lcd_brightness(uint8_t brightness);
void lcd_sleep(void);
void lcd_wakeup(void);

// Internal functions
void lcd_send_cmd(uint8_t cmd);
void lcd_send_data(uint8_t data);
void lcd_send_data16(uint16_t data);
void lcd_send_data32(uint32_t data);
void lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2);

#endif // _RM67162_H_
