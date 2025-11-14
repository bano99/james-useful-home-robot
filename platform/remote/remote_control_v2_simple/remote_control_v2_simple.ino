/*
 * James Robot - Remote Control with AMOLED Display
 * Hardware: LilyGO T-Display S3 AMOLED V2
 * 
 * Uses Arduino_GFX library (install from Library Manager)
 * Search for "GFX Library for Arduino" by moononournation
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Arduino_GFX_Library.h>

// Display pins for LilyGO T-Display S3 AMOLED
#define TFT_BL     38
#define TFT_POWER  15

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

// Create display using Arduino_GFX (same as working example)
Arduino_DataBus *bus = new Arduino_HWSPI(7 /* DC */, 6 /* CS */, 47 /* SCK */, 18 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_RM67162(bus, 17 /* RST */, 0 /* rotation */, true /* IPS */);

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
  
  // Power on display
  pinMode(TFT_POWER, OUTPUT);
  digitalWrite(TFT_POWER, HIGH);
  delay(10);
  
  // Turn on backlight
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
  
  // Initialize GFX
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
  } else {
    Serial.println("Display initialized!");
  }
  
  // Test pattern - you should see these colors!
  Serial.println("Drawing test pattern...");
  gfx->fillScreen(RED);
  delay(500);
  gfx->fillScreen(GREEN);
  delay(500);
  gfx->fillScreen(BLUE);
  delay(500);
  gfx->fillScreen(BLACK);
  
  // Draw UI
  gfx->setTextColor(GREEN);
  gfx->setTextSize(2);
  gfx->setCursor(60, 10);
  gfx->println("JAMES");
  gfx->setCursor(50, 30);
  gfx->println("REMOTE");
  
  gfx->setTextSize(1);
  gfx->setTextColor(WHITE);
  gfx->setCursor(10, 100);
  gfx->println("Fwd/Back:");
  gfx->setCursor(10, 200);
  gfx->println("Left/Right:");
  gfx->setCursor(10, 300);
  gfx->println("Rotation:");
  gfx->setCursor(10, 400);
  gfx->println("Status:");
  
  Serial.println("UI drawn!");
}

void drawBar(int x, int y, int width, int height, int value, uint16_t color) {
  // Draw background
  gfx->fillRect(x, y, width, height, COLOR_BAR_BG);
  
  // Calculate bar position
  int centerY = y + height / 2;
  int barHeight = (abs(value) * height / 2) / 255;
  
  if (value > 0) {
    gfx->fillRect(x, centerY, width, barHeight, color);
  } else if (value < 0) {
    gfx->fillRect(x, centerY - barHeight, width, barHeight, color);
  }
  
  // Draw center line
  gfx->drawFastHLine(x, centerY, width, WHITE);
}

void updateDisplay() {
  unsigned long currentTime = millis();
  
  if (currentTime - lastDisplayUpdate < DISPLAY_UPDATE_MS) {
    return;
  }
  lastDisplayUpdate = currentTime;
  
  char buf[20];
  
  // Forward/Back
  snprintf(buf, sizeof(buf), "%4d", transformedValues.value13);
  gfx->fillRect(100, 100, 50, 10, BLACK);
  gfx->setCursor(100, 100);
  gfx->setTextColor(WHITE);
  gfx->setTextSize(1);
  gfx->print(buf);
  drawBar(10, 120, 60, 60, transformedValues.value13, COLOR_BAR_X);
  
  // Left/Right
  snprintf(buf, sizeof(buf), "%4d", transformedValues.value14);
  gfx->fillRect(100, 200, 50, 10, BLACK);
  gfx->setCursor(100, 200);
  gfx->print(buf);
  drawBar(10, 220, 60, 60, transformedValues.value14, COLOR_BAR_Y);
  
  // Rotation
  snprintf(buf, sizeof(buf), "%4d", transformedValues.value15);
  gfx->fillRect(100, 300, 50, 10, BLACK);
  gfx->setCursor(100, 300);
  gfx->print(buf);
  drawBar(10, 320, 60, 60, transformedValues.value15, COLOR_BAR_ROT);
  
  // Connection status
  gfx->fillRect(10, 415, 100, 10, BLACK);
  gfx->setCursor(10, 415);
  if (lastSendSuccess && (currentTime - lastSuccessTime < 1000)) {
    gfx->setTextColor(GREEN);
    gfx->print("Connected");
  } else {
    gfx->setTextColor(RED);
    gfx->print("Disconnected");
  }
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
  Serial.println("\n\nJames Remote Control - Arduino_GFX Version");

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
    gfx->setCursor(10, 450);
    gfx->setTextColor(RED);
    gfx->println("ESP-NOW FAILED!");
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
    gfx->setCursor(10, 450);
    gfx->setTextColor(RED);
    gfx->println("PEER ADD FAILED!");
    return;
  }

  Serial.println("Setup complete!");
  gfx->setCursor(10, 450);
  gfx->setTextColor(GREEN);
  gfx->println("Ready!");
}

void loop() {
  sendTransformedValues();
  updateDisplay();
  delay(150);
}
