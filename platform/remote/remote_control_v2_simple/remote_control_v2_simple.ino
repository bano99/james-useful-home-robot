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
#include <esp_mac.h>
#include <Arduino_GFX_Library.h>

// Display configuration (Portrait mode)
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 536
#define DISPLAY_UPDATE_MS 100

// Right joystick pins (platform control)
const int rightJoystickUpDownPin = 13;      // Forward/backward
const int rightJoystickLeftRightPin = 14;   // Left/right
const int rightJoystickRotationPin = 15;    // Rotation

// Left joystick pins (arm control) - from remote_control_v1
const int leftJoystickUpDownPin = 1;        // Arm forward/back (Cartesian X)
const int leftJoystickLeftRightPin = 11;    // Arm left/right (Cartesian Y)
const int leftJoystickRotationPin = 12;     // Arm up/down or rotation (Cartesian Z)

// Switch pin - from remote_control_v1
const int leftSwitchPin = 45;               // Mode switch

// Left button pin for semi-reboot
const int leftButtonPin = 42;               // Semi-reboot trigger button

// ADC configuration
const int adcDeadZoneMin = 1900;
const int adcDeadZoneMax = 2250;

// Updated data structure for arm control
// Gripper Control Data
#define CMD_SET_POS    1
#define CMD_SET_MIDDLE 2
#define CMD_SET_TORQUE 3

typedef struct GripperCommand {
  int command;
  int id;
  int pos;
  int speed;
  int torque;
} GripperCommand;

typedef struct GripperStatus {
  int id;
  int pos;
  int load;
  float voltage;
  bool connected;
} GripperStatus;

GripperCommand gripperCmd;
GripperStatus gripperStatus;

// POT filtering
const int gripperPotPin = 16;
int lastGripperPotValue = -1;
const int potThreshold = 10;
bool gripperConnected = false;
unsigned long lastGripperMessageTime = 0;

// Center button pin for arming/disarming
const int centerButtonPin = 39;

// Arming state
bool systemArmed = false;
bool lastButtonState = false;
unsigned long lastButtonPressTime = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 200;  // Debounce time

// Semi-reboot button state
bool lastLeftButtonState = false;
unsigned long lastLeftButtonPressTime = 0;

// Joystick calibration offsets
int rightJoystickYOffset = 0;
int rightJoystickXOffset = 0;
int rightJoystickRotOffset = 0;
int leftJoystickYOffset = 0;
int leftJoystickXOffset = 0;
int leftJoystickZOffset = 0;

bool calibrationComplete = false;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DURATION_MS = 1000;  // 1 second calibration

typedef struct RemoteControlData {
  // Right joystick (platform control)
  int right_y;      // Pin 13 - forward/back
  int right_x;      // Pin 14 - left/right
  int right_rot;    // Pin 15 - rotation
  
  // Left joystick (arm control)
  int left_y;       // Pin 1 - arm forward/back (Cartesian X)
  int left_x;       // Pin 11 - arm left/right (Cartesian Y)
  int left_z;       // Pin 12 - arm up/down or rotation (Cartesian Z)
  
  // Switch state
  bool switch_platform_mode;  // Pin 45 - true = platform mode, false = vertical arm mode
  
  // Gripper (retained in struct but we use separate ESP-NOW for Phase 1)
  int gripper_pot;  // Pin 16
  
  // Safety arming state
  bool armed;  // true = armed (motion allowed), false = disarmed (no motion)
  
  // Semi-reboot trigger
  bool trigger_reboot;  // true = trigger Teensy semi-reboot
} RemoteControlData;

RemoteControlData controlData;

// Receiver MAC addresses
uint8_t robotBroadcastAddress[] = {0x24, 0x58, 0x7C, 0xD3, 0x6B, 0x30}; // Robot receiver (Teensy/ROS2)
uint8_t gripperBroadcastAddress[] = {0x14, 0x33, 0x5C, 0x25, 0x05, 0x78}; // Gripper controller MAC

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

void calibrateJoysticks() {
  Serial.println("Calibrating joysticks - keep hands off!");
  
  // Take multiple samples and average
  const int numSamples = 50;
  long sumRightY = 0, sumRightX = 0, sumRightRot = 0;
  long sumLeftY = 0, sumLeftX = 0, sumLeftZ = 0;
  
  for (int i = 0; i < numSamples; i++) {
    sumRightY += analogRead(rightJoystickUpDownPin);
    sumRightX += analogRead(rightJoystickLeftRightPin);
    sumRightRot += analogRead(rightJoystickRotationPin);
    sumLeftY += analogRead(leftJoystickUpDownPin);
    sumLeftX += analogRead(leftJoystickLeftRightPin);
    sumLeftZ += analogRead(leftJoystickRotationPin);
    delay(20);  // 20ms between samples = 1 second total
  }
  
  // Calculate center positions
  int centerRightY = sumRightY / numSamples;
  int centerRightX = sumRightX / numSamples;
  int centerRightRot = sumRightRot / numSamples;
  int centerLeftY = sumLeftY / numSamples;
  int centerLeftX = sumLeftX / numSamples;
  int centerLeftZ = sumLeftZ / numSamples;
  
  // Calculate offsets from expected center (2047)
  rightJoystickYOffset = 2047 - centerRightY;
  rightJoystickXOffset = 2047 - centerRightX;
  rightJoystickRotOffset = 2047 - centerRightRot;
  leftJoystickYOffset = 2047 - centerLeftY;
  leftJoystickXOffset = 2047 - centerLeftX;
  leftJoystickZOffset = 2047 - centerLeftZ;
  
  calibrationComplete = true;
  
  Serial.println("Calibration complete!");
  Serial.printf("Right offsets: Y=%d X=%d Rot=%d\n", 
                rightJoystickYOffset, rightJoystickXOffset, rightJoystickRotOffset);
  Serial.printf("Left offsets: Y=%d X=%d Z=%d\n", 
                leftJoystickYOffset, leftJoystickXOffset, leftJoystickZOffset);
}

int readCalibratedJoystick(int pin, int offset) {
  int rawValue = analogRead(pin);
  int calibratedValue = rawValue + offset;
  
  // Clamp to valid ADC range
  if (calibratedValue < 0) calibratedValue = 0;
  if (calibratedValue > 4095) calibratedValue = 4095;
  
  return calibratedValue;
}

void handleArmingButton() {
  bool currentButtonState = (digitalRead(centerButtonPin) == LOW);  // Active low
  unsigned long currentTime = millis();
  
  // Debounce: only process if enough time has passed since last press
  if (currentButtonState && !lastButtonState && 
      (currentTime - lastButtonPressTime > BUTTON_DEBOUNCE_MS)) {
    
    // Toggle armed state
    systemArmed = !systemArmed;
    lastButtonPressTime = currentTime;
    
    Serial.printf("System %s\n", systemArmed ? "ARMED" : "DISARMED");
  }
  
  lastButtonState = currentButtonState;
}

void handleSemiRebootButton() {
  bool currentButtonState = (digitalRead(leftButtonPin) == LOW);  // Active low
  unsigned long currentTime = millis();
  
  // Debounce: only process if enough time has passed since last press
  if (currentButtonState && !lastLeftButtonState && 
      (currentTime - lastLeftButtonPressTime > BUTTON_DEBOUNCE_MS)) {
    
    // Trigger semi-reboot
    controlData.trigger_reboot = true;
    lastLeftButtonPressTime = currentTime;
    
    Serial.println("SEMI-REBOOT TRIGGERED!");
    
    // Show visual feedback on display
    gfx2->fillRect(0, 0, 240, 60, RED);
    gfx2->setTextColor(WHITE);
    gfx2->setTextSize(2);
    gfx2->setCursor(20, 15);
    gfx2->println("REBOOTING");
    gfx2->setCursor(20, 35);
    gfx2->println("TEENSY...");
    gfx2->flush();
  } else {
    // Reset trigger flag when button is released
    controlData.trigger_reboot = false;
  }
  
  lastLeftButtonState = currentButtonState;
}

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
  gfx2->setCursor(10, 75);
  gfx2->println("Position:");
  gfx2->drawCircle(120, 180, 70, DARKGREY);  // Outer circle
  gfx2->drawCircle(120, 180, 3, WHITE);      // Center dot
  gfx2->drawFastHLine(50, 180, 140, DARKGREY);  // Crosshair
  gfx2->drawFastVLine(120, 110, 140, DARKGREY);
  
  // Rotation indicator
  gfx2->setCursor(10, 300);
  gfx2->println("Rotation:");
  gfx2->drawCircle(120, 380, 50, DARKGREY);  // Rotation circle
  gfx2->drawFastVLine(120, 330, 100, DARKGREY);  // Center line
  
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
  
  // Update connection status banner with arming state
  bool connected = lastSendSuccess && (currentTime - lastSuccessTime < 1000);
  
  // Banner color based on armed state
  uint16_t bannerColor;
  const char* statusText;
  
  if (!connected) {
    bannerColor = RED;
    statusText = "DISCONNECTED";
  } else if (!calibrationComplete) {
    bannerColor = YELLOW;
    statusText = "CALIBRATING...";
  } else if (systemArmed) {
    bannerColor = GREEN;
    statusText = "ARMED";
  } else {
    bannerColor = ORANGE;
    statusText = "DISARMED";
  }
  
  gfx2->fillRect(0, 0, 240, 30, bannerColor);
  
  // Gripper connection indicator (small circle in top right)
  uint16_t gripperColor = (millis() - lastGripperMessageTime < 2000) ? GREEN : RED;
  gfx2->fillCircle(220, 15, 8, gripperColor);
  gfx2->drawCircle(220, 15, 8, WHITE);

  gfx2->setTextColor(BLACK);
  gfx2->setTextSize(2);
  gfx2->setCursor(10, 8);
  gfx2->print(statusText);
  
  // Second line: Control mode (Platform vs Arm)
  gfx2->fillRect(0, 30, 240, 30, BLACK);
  gfx2->setTextSize(2);
  gfx2->setTextColor(WHITE);
  gfx2->setCursor(10, 38);
  gfx2->print("Mode: ");
  
  if (controlData.switch_platform_mode) {
    gfx2->setTextColor(CYAN);
    gfx2->print("PLATFORM");
  } else {
    gfx2->setTextColor(MAGENTA);
    gfx2->print("ARM");
  }
  
  // Draw joystick position (X/Y combined) - showing RIGHT joystick (platform control)
  drawJoystickPosition(120, 180, controlData.right_x, controlData.right_y);
  
  // Draw rotation indicator - showing RIGHT joystick rotation
  drawRotationIndicator(120, 380, controlData.right_rot);
  
  // BOTTOM: Signal quality only
  gfx2->fillRect(0, 490, 240, 46, BLACK);
  gfx2->setTextSize(2);
  gfx2->setTextColor(WHITE);
  
  // Signal quality (success rate)
  int totalSends = sendSuccessCount + sendFailCount;
  if (totalSends > 0) {
    int signalPercent = (sendSuccessCount * 100) / totalSends;
    char buf[20];
    snprintf(buf, sizeof(buf), "Signal: %3d%%", signalPercent);
    gfx2->setCursor(10, 505);
    
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

void onDataRecv(const esp_now_recv_info_t *esp_now_info, const uint8_t *incomingData, int len) {
  if (len == sizeof(gripperStatus)) {
    memcpy(&gripperStatus, incomingData, sizeof(gripperStatus));
    lastGripperMessageTime = millis();
    gripperConnected = true;
  }
}

void sendControlData() {
  // Handle arming button
  handleArmingButton();
  
  // Handle semi-reboot button
  handleSemiRebootButton();
  
  // Read right joystick with calibration (platform control)
  int rightYRaw = readCalibratedJoystick(rightJoystickUpDownPin, rightJoystickYOffset);
  int rightXRaw = readCalibratedJoystick(rightJoystickLeftRightPin, rightJoystickXOffset);
  int rightRotRaw = readCalibratedJoystick(rightJoystickRotationPin, rightJoystickRotOffset);
  
  controlData.right_y = mapJoystickToSpeed(rightYRaw);
  controlData.right_x = mapJoystickToSpeed(rightXRaw);
  controlData.right_rot = mapJoystickToSpeed(rightRotRaw);
  
  // Read left joystick with calibration (arm control)
  int leftYRaw = readCalibratedJoystick(leftJoystickUpDownPin, leftJoystickYOffset);
  int leftXRaw = readCalibratedJoystick(leftJoystickLeftRightPin, leftJoystickXOffset);
  int leftZRaw = readCalibratedJoystick(leftJoystickRotationPin, leftJoystickZOffset);
  
  controlData.left_y = mapJoystickToSpeed(leftYRaw);
  controlData.left_x = mapJoystickToSpeed(leftXRaw);
  controlData.left_z = mapJoystickToSpeed(leftZRaw);
  
  // Read switch state
  controlData.switch_platform_mode = digitalRead(leftSwitchPin) == HIGH;
  
  // Set armed state
  controlData.armed = systemArmed;
  
  // Gripper pot control with filtering and midpoint calibration
  int currentPot = analogRead(gripperPotPin);
  controlData.gripper_pot = currentPot;
  
  // Midpoint calibration: POT 2060 -> Servo 2047
  // Using piece-wise linear mapping to ensure the center is exact
  int servoPos;
  if (abs(currentPot - 2060) <= 15) {
    servoPos = 2047; // Deadband around midpoint
  } else if (currentPot < 2060) {
    servoPos = map(currentPot, 0, 2045, 0, 2046);
  } else {
    servoPos = map(currentPot, 2075, 4095, 2048, 4095);
  }
  
  // ONLY send gripper commands when ARMED
  if (systemArmed && abs(currentPot - lastGripperPotValue) > potThreshold) {
    lastGripperPotValue = currentPot;
    
    gripperCmd.command = CMD_SET_POS;
    gripperCmd.id = 1; 
    gripperCmd.pos = servoPos; 
    gripperCmd.speed = 1000;
    gripperCmd.torque = 1;

    esp_err_t result = esp_now_send(gripperBroadcastAddress, (uint8_t *)&gripperCmd, sizeof(gripperCmd));
    if (result == ESP_OK) {
      Serial.printf("Gripper POT: %d -> Servo POS: %d\n", currentPot, servoPos);
    } else {
      Serial.printf("Gripper send FAILED: %d\n", result);
    }
  }
  
  // Send platform/arm data via ESP-NOW to robot receiver
  esp_err_t result = esp_now_send(robotBroadcastAddress, (uint8_t *)&controlData, sizeof(controlData));
  
  if (result == ESP_OK) {
    // Serial.printf("Sent: R[%d,%d,%d] L[%d,%d,%d] SW:%d\n", 
    //               controlData.right_y, controlData.right_x, controlData.right_rot,
    //               controlData.left_y, controlData.left_x, controlData.left_z,
    //               controlData.switch_platform_mode);
  } else {
    Serial.println("Send failed");
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nJames Remote Control - QSPI Version");

  // Initialize display FIRST
  initDisplay();

  // Setup right joystick pins
  pinMode(rightJoystickUpDownPin, INPUT);
  pinMode(rightJoystickLeftRightPin, INPUT);
  pinMode(rightJoystickRotationPin, INPUT);
  
  // Setup left joystick pins
  pinMode(leftJoystickUpDownPin, INPUT);
  pinMode(leftJoystickLeftRightPin, INPUT);
  pinMode(leftJoystickRotationPin, INPUT);
  
  // Setup switch pin
  pinMode(leftSwitchPin, INPUT_PULLUP);

  Serial.println("Initializing ESP-NOW...");
  WiFi.mode(WIFI_STA);
  delay(100); // Give radio a moment to initialize
  
  uint8_t baseMac[6];
  esp_read_mac(baseMac, ESP_MAC_WIFI_STA);
  Serial.printf("Remote MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", 
                baseMac[0], baseMac[1], baseMac[2], baseMac[3], baseMac[4], baseMac[5]);
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init failed");
    gfx2->setCursor(10, 450);
    gfx2->setTextColor(RED);
    gfx2->println("ESP-NOW FAILED!");
    gfx2->flush();
    return;
  }
  
  esp_now_register_send_cb(onDataSent);
  esp_now_register_recv_cb(onDataRecv);

  // Register Peer 1: Robot Receiver
  esp_now_peer_info_t peerInfoRobot = {};
  memcpy(peerInfoRobot.peer_addr, robotBroadcastAddress, 6);
  peerInfoRobot.channel = 0;  
  peerInfoRobot.encrypt = false;
  if (esp_now_add_peer(&peerInfoRobot) != ESP_OK) {
    Serial.println("Failed to add robot peer");
  }

  // Register Peer 2: Gripper Controller
  esp_now_peer_info_t peerInfoGripper = {};
  memcpy(peerInfoGripper.peer_addr, gripperBroadcastAddress, 6);
  peerInfoGripper.channel = 0;  
  peerInfoGripper.encrypt = false;
  if (esp_now_add_peer(&peerInfoGripper) != ESP_OK) {
    Serial.println("Failed to add gripper peer");
  }

  // Setup center button pin for arming/disarming
  pinMode(centerButtonPin, INPUT_PULLUP);
  
  // Setup left button pin for semi-reboot
  pinMode(leftButtonPin, INPUT_PULLUP);
  
  // Initialize button state to current reading to prevent false trigger on first read
  delay(50);  // Let pin stabilize
  lastButtonState = (digitalRead(centerButtonPin) == LOW);
  lastLeftButtonState = (digitalRead(leftButtonPin) == LOW);
  lastButtonPressTime = millis();  // Initialize to current time to prevent immediate trigger
  lastLeftButtonPressTime = millis();
  
  Serial.println("Setup complete!");
  
  // Display calibration message
  gfx2->fillRect(0, 0, 240, 30, YELLOW);
  gfx2->setTextColor(BLACK);
  gfx2->setTextSize(2);
  gfx2->setCursor(10, 8);
  gfx2->println("CALIBRATING...");
  gfx2->setCursor(10, 480);
  gfx2->setTextColor(YELLOW);
  gfx2->println("Hands off joysticks!");
  gfx2->flush();
  
  // Perform calibration
  delay(500);  // Give user time to see message
  calibrateJoysticks();
  
  // Show ready message
  gfx2->fillRect(0, 0, 240, 536, BLACK);
  drawUI();
  gfx2->fillRect(0, 0, 240, 30, ORANGE);
  gfx2->setTextColor(BLACK);
  gfx2->setCursor(10, 8);
  gfx2->println("DISARMED");
  gfx2->setCursor(10, 480);
  gfx2->setTextColor(GREEN);
  gfx2->println("Ready! Press center");
  gfx2->setCursor(10, 500);
  gfx2->println("button (Pin 39) to ARM");
  gfx2->flush();
  
  delay(1000);  // Show message briefly
}

void loop() {
  sendControlData();
  updateDisplay();
  delay(150);
}
