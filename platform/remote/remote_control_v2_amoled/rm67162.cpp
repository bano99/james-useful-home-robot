/*
 * RM67162 AMOLED Display Driver Implementation
 * For LilyGO T-Display S3 AMOLED V2
 */

#include "rm67162.h"

static SPIClass *spi = NULL;
static uint8_t current_rotation = 0;
static uint16_t lcd_width = LCD_WIDTH;
static uint16_t lcd_height = LCD_HEIGHT;

// Send command to display
void lcd_send_cmd(uint8_t cmd) {
  digitalWrite(LCD_DC, LOW);
  digitalWrite(LCD_CS, LOW);
  spi->transfer(cmd);
  digitalWrite(LCD_CS, HIGH);
}

// Send single byte data
void lcd_send_data(uint8_t data) {
  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_CS, LOW);
  spi->transfer(data);
  digitalWrite(LCD_CS, HIGH);
}

// Send 16-bit data
void lcd_send_data16(uint16_t data) {
  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_CS, LOW);
  spi->transfer16(data);
  digitalWrite(LCD_CS, HIGH);
}

// Send 32-bit data
void lcd_send_data32(uint32_t data) {
  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_CS, LOW);
  spi->transfer32(data);
  digitalWrite(LCD_CS, HIGH);
}

// Set address window for drawing
void lcd_address_set(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
  lcd_send_cmd(0x2A);  // Column address set
  lcd_send_data16(x1);
  lcd_send_data16(x2);
  
  lcd_send_cmd(0x2B);  // Row address set
  lcd_send_data16(y1);
  lcd_send_data16(y2);
  
  lcd_send_cmd(0x2C);  // Memory write
}

// Initialize the display
void rm67162_init(void) {
  // Initialize pins
  pinMode(LCD_CS, OUTPUT);
  pinMode(LCD_DC, OUTPUT);
  pinMode(LCD_RST, OUTPUT);
  pinMode(LCD_BL, OUTPUT);
  pinMode(LCD_POWER, OUTPUT);
  
  // Power on display
  digitalWrite(LCD_POWER, HIGH);
  delay(10);
  
  // Initialize SPI
  spi = new SPIClass(HSPI);
  spi->begin(LCD_SCLK, LCD_MISO, LCD_MOSI, LCD_CS);
  spi->setFrequency(80000000);  // 80 MHz
  spi->setDataMode(SPI_MODE0);
  
  // Hardware reset
  digitalWrite(LCD_RST, HIGH);
  delay(10);
  digitalWrite(LCD_RST, LOW);
  delay(10);
  digitalWrite(LCD_RST, HIGH);
  delay(120);
  
  // RM67162 initialization sequence
  lcd_send_cmd(0x11);  // Sleep out
  delay(120);
  
  lcd_send_cmd(0x3A);  // Interface pixel format
  lcd_send_data(0x55); // 16-bit/pixel (RGB565)
  
  lcd_send_cmd(0x51);  // Write display brightness
  lcd_send_data(0xFF); // Maximum brightness
  
  lcd_send_cmd(0x29);  // Display on
  delay(20);
  
  // Turn on backlight
  digitalWrite(LCD_BL, HIGH);
  
  // Clear screen to black
  lcd_fill(BLACK);
  
  Serial.println("RM67162 display initialized");
}

// Set display rotation
void lcd_setRotation(uint8_t rotation) {
  current_rotation = rotation % 4;
  
  lcd_send_cmd(0x36);  // Memory access control
  
  switch (current_rotation) {
    case 0:  // Portrait
      lcd_send_data(0x00);
      lcd_width = LCD_HEIGHT;
      lcd_height = LCD_WIDTH;
      break;
    case 1:  // Landscape
      lcd_send_data(0x60);
      lcd_width = LCD_WIDTH;
      lcd_height = LCD_HEIGHT;
      break;
    case 2:  // Portrait inverted
      lcd_send_data(0xC0);
      lcd_width = LCD_HEIGHT;
      lcd_height = LCD_WIDTH;
      break;
    case 3:  // Landscape inverted
      lcd_send_data(0xA0);
      lcd_width = LCD_WIDTH;
      lcd_height = LCD_HEIGHT;
      break;
  }
}

// Fill entire screen with color
void lcd_fill(uint16_t color) {
  lcd_address_set(0, 0, lcd_width - 1, lcd_height - 1);
  
  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_CS, LOW);
  
  for (uint32_t i = 0; i < lcd_width * lcd_height; i++) {
    spi->transfer16(color);
  }
  
  digitalWrite(LCD_CS, HIGH);
}

// Push color data to specified area
void lcd_PushColors(uint16_t x, uint16_t y, uint16_t width, uint16_t height, uint16_t *data) {
  lcd_address_set(x, y, x + width - 1, y + height - 1);
  
  digitalWrite(LCD_DC, HIGH);
  digitalWrite(LCD_CS, LOW);
  
  uint32_t len = width * height;
  for (uint32_t i = 0; i < len; i++) {
    spi->transfer16(data[i]);
  }
  
  digitalWrite(LCD_CS, HIGH);
}

// Set backlight brightness (0-255)
void lcd_brightness(uint8_t brightness) {
  lcd_send_cmd(0x51);
  lcd_send_data(brightness);
}

// Put display to sleep
void lcd_sleep(void) {
  lcd_send_cmd(0x28);  // Display off
  delay(20);
  lcd_send_cmd(0x10);  // Sleep in
  delay(120);
  digitalWrite(LCD_BL, LOW);  // Turn off backlight
}

// Wake up display
void lcd_wakeup(void) {
  digitalWrite(LCD_BL, HIGH);  // Turn on backlight
  lcd_send_cmd(0x11);  // Sleep out
  delay(120);
  lcd_send_cmd(0x29);  // Display on
  delay(20);
}
