/*
 * James Robot - Remote Control with AMOLED Display
 * Hardware: LilyGO T-Display S3 AMOLED V2
 * Board: LilyGo T-Display-S3
 * 
 * Based on working Arduino_GFX_HelloWorld example
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino_GFX_Library.h>

// Display configuration (Portrait mode)
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 536
#define DISPLAY_UPDATE_MS 100

// Joystick pins
const int rightJoystickUpDownPin = 13;
const int rightJoystickLeftRightPin = 14;
const int rightJoystickRotationPin = 15;

// ADC configuration
const int adcDeadZoneMin = 1900;
const int adcDeadZoneMax = 2250;

// Data structure
typedef struct TransformedValues {
  int value13;
  int value14;
  int value15;
} TransformedValues;

TransformedValues transformedValues;

// Receiver MAC address
uint8_t broadcastAddress[] = {0x24, 0x58, 0x7C, 0xD3, 0x6B, 0x30};

// Connection status
bool lastSendSuccess = false;
unsigned long lastSuccessTime = 0;
unsigned long lastDisplayUpdate = 0;
int sendSuccessCount = 0;
int sendFailCount = 0;

// Display setup - EXACTLY like working HelloWorld example
Arduino_DataBus *bus = new Arduino_ESP32QSPI(6 /* cs */, 47 /* sck */, 18 /* d0 */, 7 /* d1 */, 48 /* d2 */, 5 /* d3 */);
Arduino_GFX *gfx = new Arduino_RM67162(bus, 17 /* RST */, 0 /* rotation */);
Arduino_GFX *gfx2;  // Canvas for double buffering

// Colors
#define COLOR_BG       BLACK
#define COLOR_TEXT     WHITE
#define COLOR_GOOD     GREEN
#define COLOR_WARNING  YELLOW
#define COLOR_ERROR    RED
#define COLOR_BAR_BG   DARKGREY
#define COLOR_BAR_X    GREEN
#define COLOR_BAR_Y    BLUE
#define COLOR_BAR_ROT  ORANGE

void initDisplay() {
  Serial.println("Initializing display...");
  
  // CRITICAL: Power on display (pin 38)
  pinMode(38, OUTPUT);
  digitalWrite(38, HIGH);
  
  // Initialize GFX - same as HelloWorld
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  }
  
  // Create canvas for double buffering
  gfx2 = new Arduino_Canvas(240, 536, gfx, 0, 0);
  gfx2->begin(GFX_SKIP_OUTPUT_BEGIN);
  
  Serial.println("Display initialized!");
  
  // Draw initial UI directly (no test pattern)
  Serial.println("Drawing UI...");
  drawUI();
  
  Serial.println("UI drawn!");
}

void drawUI() {
  gfx2->fillScreen(BLACK);
  
  // Connection status banner at top (will be updated in loop)
  gfx2->fillRect(0, 0, 240, 30, DARKGREY);
  gfx2->setTextColor(WHITE);
  gfx2->setTextSize(2);
  gfx2->setCursor(20, 8);
  gfx2->println("JAMES REMOTE");
  
  // Draw joystick visualization area
  gfx2->setTextSize(2);  // Larger font for labels
  gfx2->setTextColor(WHITE);
  
  // Position indicator (X/Y)
  gfx2->setCursor(10, 45);
  gfx2->println("Position:");
  gfx2->drawCircle(120, 150, 70, DARKGREY);  // Outer circle
  gfx2->drawCircle(120, 150, 3, WHITE);      // Center dot
  gfx2->drawFastHLine(50, 150, 140, DARKGREY);  // Crosshair
  gfx2->drawFastVLine(120, 80, 140, DARKGREY);
  
  // Rotation indicator
  gfx2->setCursor(10, 270);
  gfx2->println("Rotation:");
  gfx2->drawCircle(120, 350, 50, DARKGREY);  // Rotation circle
  gfx2->drawFastVLine(120, 300, 100, DARKGREY);  // Center line
  
  // Value displays
  gfx2->setCursor(10, 450);
  gfx2->println("X:");
  gfx2->setCursor(10, 470);
  gfx2->println("Y:");
  gfx2->setCursor(10, 490);
  gfx2->println("R:");
  
  gfx2->flush();
}

void drawJoystickPosition(int16_t x, int16_t y, int16_t valueX, int16_t valueY) {
  // Clear entire area with rectangle (more reliable than circle)
  gfx2->fillRect(x - 85, y - 85, 170, 170, BLACK);
  
  // Redraw circle and crosshair with thicker lines
  gfx2->drawCircle(x, y, 70, DARKGREY);
  gfx2->drawCircle(x, y, 69, DARKGREY);
  gfx2->drawCircle(x, y, 3, WHITE);
  
  // Thicker crosshair
  for (int i = -1; i <= 1; i++) {
    gfx2->drawFastHLine(x - 70, y + i, 140, DARKGREY);
    gfx2->drawFastVLine(x + i, y - 70, 140, DARKGREY);
  }
  
  // Calculate position (scale -255..255 to -60..60 pixels)
  // FLIP both axes: negative valueY should go UP, negative valueX should go LEFT
  int16_t posX = -(valueX * 60) / 255;  // Inverted!
  int16_t posY = -(valueY * 60) / 255;  // Inverted!
  
  // Draw position indicator with thicker outline
  gfx2->fillCircle(x + posX, y + posY, 10, GREEN);
  gfx2->drawCircle(x + posX, y + posY, 11, WHITE);
  gfx2->drawCircle(x + posX, y + posY, 10, WHITE);
}

void drawRotationIndicator(int16_t x, int16_t y, int16_t rotation) {
  // Clear previous rotation indicator
  gfx2->fillCircle(x, y, 52, BLACK);
  
  // Redraw circle with thicker line
  gfx2->drawCircle(x, y, 50, DARKGREY);
  gfx2->drawCircle(x, y, 49, DARKGREY);
  
  // Thicker center line
  for (int i = -1; i <= 1; i++) {
    gfx2->drawFastVLine(x + i, y - 50, 100, DARKGREY);
  }
  
  // Calculate angle (rotation -255..255 to -180..180 degrees)
  float angle = (rotation * 180.0) / 255.0;
  float radians = (angle - 90) * PI / 180.0;  // -90 to start from top
  
  // Draw rotation line with thickness
  int16_t endX = x + (int16_t)(cos(radians) * 45);
  int16_t endY = y + (int16_t)(sin(radians) * 45);
  
  // Thicker line (draw multiple lines)
  for (int i = -2; i <= 2; i++) {
    for (int j = -2; j <= 2; j++) {
      gfx2->drawLine(x + i, y + j, endX + i, endY + j, ORANGE);
    }
  }
  
  gfx2->fillCircle(endX, endY, 6, ORANGE);
  gfx2->drawCircle(endX, endY, 6, WHITE);
  
  // Draw arc to show rotation direction with thicker arc
  // Only draw if rotation is significant
  if (abs(rotation) > 20) {
    uint16_t color = rotation > 0 ? YELLOW : CYAN;
    // Draw thicker arc indicator
    for (int i = -8; i <= 8; i++) {
      float arcAngle = (angle + i - 90) * PI / 180.0;
      int16_t arcX = x + (int16_t)(cos(arcAngle) * 40);
      int16_t arcY = y + (int16_t)(sin(arcAngle) * 40);
      gfx2->fillCircle(arcX, arcY, 2, color);
    }
  }
}

void updateDisplay() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastDisplayUpdate < DISPLAY_UPDATE_MS) {
    return;
  }
  lastDisplayUpdate = currentTime;
  
  // Update connection status banner (prominent at top)
  bool connected = lastSendSuccess && (currentTime - lastSuccessTime < 1000);
  uint16_t bannerColor = connected ? GREEN : RED;
  gfx2->fillRect(0, 0, 240, 30, bannerColor);
  gfx2->setTextColor(BLACK);
  gfx2->setTextSize(2);
  gfx2->setCursor(20, 8);
  gfx2->print("JAMES REMOTE");
  
  // Draw joystick position (X/Y combined)
  drawJoystickPosition(120, 150, transformedValues.value14, transformedValues.value13);
  
  // Draw rotation indicator
  drawRotationIndicator(120, 350, transformedValues.value15);
  
  // Update value displays with larger font
  char buf[20];
  gfx2->setTextSize(2);  // Larger font
  gfx2->setTextColor(WHITE);
  
  // X value (Left/Right)
  snprintf(buf, sizeof(buf), "X:%4d", transformedValues.value14);
  gfx2->fillRect(10, 440, 110, 20, BLACK);
  gfx2->setCursor(10, 440);
  gfx2->print(buf);
  
  // Y value (Forward/Back)
  snprintf(buf, sizeof(buf), "Y:%4d", transformedValues.value13);
  gfx2->fillRect(10, 465, 110, 20, BLACK);
  gfx2->setCursor(10, 465);
  gfx2->print(buf);
  
  // Rotation value
  snprintf(buf, sizeof(buf), "R:%4d", transformedValues.value15);
  gfx2->fillRect(10, 490, 110, 20, BLACK);
  gfx2->setCursor(10, 490);
  gfx2->print(buf);
  
  // Signal quality (success rate)
  int totalSends = sendSuccessCount + sendFailCount;
  if (totalSends > 0) {
    int signalPercent = (sendSuccessCount * 100) / totalSends;
    snprintf(buf, sizeof(buf), "Q:%3d%%", signalPercent);  // Q for Quality
    gfx2->fillRect(130, 490, 100, 20, BLACK);
    gfx2->setCursor(130, 490);
    
    if (signalPercent > 80) {
      gfx2->setTextColor(GREEN);
    } else if (signalPercent > 50) {
      gfx2->setTextColor(YELLOW);
    } else {
      gfx2->setTextColor(RED);
    }
    gfx2->print(buf);
  }
  
  // CRITICAL: Flush canvas to display!
  gfx2->flush();
}

// Joystick functions
int mapExp(int value, int inMin, int inMax, int outMin, int outMax) {
  float normalizedValue = (value - inMin) / static_cast<float>(inMax - inMin);
  float mappedValue = exp(normalizedValue) - 1;
  mappedValue = mappedValue / (exp(1) - 1);
  return outMin + mappedValue * (outMax - outMin);
}

int mapJoystickToSpeed(int adcValue) {
  if (adcValue < adcDeadZoneMin) {
    return mapExp(adcValue, 0, adcDeadZoneMin, -255, 0);
  } else if (adcValue > adcDeadZoneMax) {
    return mapExp(adcValue, adcDeadZoneMax, 4095, 0, 255);
  } else {
    return 0;
  }
}

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSendSuccess = true;
    lastSuccessTime = millis();
    sendSuccessCount++;
    
    if (sendSuccessCount + sendFailCount > 1000) {
      sendSuccessCount /= 2;
      sendFailCount /= 2;
    }
  } else {
    lastSendSuccess = false;
    sendFailCount++;
  }
}

void sendTransformedValues() {
  transformedValues.value13 = mapJoystickToSpeed(analogRead(rightJoystickUpDownPin));
  transformedValues.value14 = mapJoystickToSpeed(analogRead(rightJoystickLeftRightPin));
  transformedValues.value15 = mapJoystickToSpeed(analogRead(rightJoystickRotationPin));
  
  esp_err_t result = esp_now_send(NULL, (uint8_t *)&transformedValues, sizeof(transformedValues));
  
  if (result == ESP_OK) {
    Serial.printf("Sent: X=%d Y=%d Rot=%d\n", 
                  transformedValues.value14, 
                  transformedValues.value13, 
                  transformedValues.value15);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nJames Remote Control - QSPI Version");

  // Initialize display FIRST
  initDisplay();

  // Setup joystick pins
  pinMode(rightJoystickUpDownPin, INPUT);
  pinMode(rightJoystickLeftRightPin, INPUT);
  pinMode(rightJoystickRotationPin, INPUT);

  // Initialize WiFi
  Serial.println("Initializing ESP-NOW...");
  WiFi.mode(WIFI_STA);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    gfx2->setCursor(10, 450);
    gfx2->setTextColor(RED);
    gfx2->println("ESP-NOW FAILED!");
    gfx2->flush();
    return;
  }
  
  esp_now_register_send_cb(onDataSent);

  // Register peer
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    gfx2->setCursor(10, 450);
    gfx2->setTextColor(RED);
    gfx2->println("PEER ADD FAILED!");
    gfx2->flush();
    return;
  }

  Serial.println("Setup complete!");
  gfx2->setCursor(10, 450);
  gfx2->setTextColor(GREEN);
  gfx2->println("Ready!");
  gfx2->flush();
}

void loop() {
  sendTransformedValues();
  updateDisplay();
  delay(150);
}
