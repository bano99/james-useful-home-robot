#include <Wire.h> 
#define I2C_SDA 3 // Define the SDA pin 
#define I2C_SCL 2 // Define the SCL pin

#include <esp_now.h>
#include <WiFi.h>
#include "esp_timer.h"

#include <DFRobot_IICSerial.h>
#include <DNSServer.h>
#include <DFRobot_IICSerial.h>
#include <Arduino_GFX_Library.h>

// Display configuration (Landscape mode - rotated 90 degrees)
#define SCREEN_WIDTH 536
#define SCREEN_HEIGHT 240
#define DISPLAY_UPDATE_MS 100  // 10 Hz refresh rate

DFRobot_IICSerial iicSerial1(Wire, /*subUartChannel =*/SUBUART_CHANNEL_1,/*IA1 = */1,/*IA0 = */1);//Construct UART1
DFRobot_IICSerial iicSerial2(Wire, /*subUartChannel =*/SUBUART_CHANNEL_2, /*IA1 = */1,/*IA0 = */1);//Construct UART2

// Display setup - QSPI configuration for LilyGO T-Display S3 AMOLED V2
Arduino_DataBus *bus = new Arduino_ESP32QSPI(6 /* cs */, 47 /* sck */, 18 /* d0 */, 7 /* d1 */, 48 /* d2 */, 5 /* d3 */);
Arduino_GFX *gfx = new Arduino_RM67162(bus, 17 /* RST */, 3 /* rotation - 3=landscape flipped */);
Arduino_GFX *gfx2;  // Canvas for double buffering

// Display colors
#define COLOR_BG       BLACK
#define COLOR_TEXT     WHITE
#define COLOR_GOOD     GREEN
#define COLOR_WARNING  YELLOW
#define COLOR_ERROR    RED
#define COLOR_BAR_BG   DARKGREY
#define COLOR_MANUAL   CYAN
#define COLOR_AUTO     MAGENTA

// Display state
unsigned long lastDisplayUpdate = 0;
bool displayInitialized = false;
 
 
struct JoystickValues {
  float direction;    // 0-360 degrees
  int velocity;       // 0-255
  int rotationalVelocity; // -127 to 127
};

struct MotorCommand {
  int state;
  float velocity;
};


// Updated structure to match remote control (RemoteControlData)
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
  
  // Gripper (placeholder for future)
  int gripper_pot;  // Pin 16 (not yet implemented)
  
  // Safety arming state
  bool armed;  // true = armed (motion allowed), false = disarmed (no motion)
  
  // Semi-reboot trigger
  bool trigger_reboot;  // true = trigger Teensy semi-reboot
} RemoteControlData;

RemoteControlData controlData;

// Legacy joystick data structure for existing platform control
typedef struct JoystickData {
  int y;
  int x;
  int rot;
} JoystickData;

JoystickData joystickData;

// Motor command structures
MotorCommand motorFL, motorFR, motorBL, motorBR;
JoystickValues joystickValues;
int motorFL_state, motorFR_state, motorBL_state, motorBR_state;

// Motor state re-initialization tracking
unsigned long lastMotorStateInit = 0;
const unsigned long MOTOR_STATE_REINIT_INTERVAL_MS = 5000;  // Re-send state every 5 seconds

// Control mode
enum ControlMode {
  MODE_MANUAL,
  MODE_AUTONOMOUS,
  MODE_STOPPED
};
ControlMode currentMode = MODE_STOPPED;

// Safety state machine
enum SafetyState {
  STATE_DISCONNECTED,
  STATE_CONNECTED_DISARMED,
  STATE_CALIBRATING,
  STATE_ARMED
};
SafetyState safetyState = STATE_DISCONNECTED;

// Connection status
unsigned long lastManualCommandTime = 0;
unsigned long lastAutonomousCommandTime = 0;
unsigned long lastArmCommandTime = 0;
const unsigned long CONNECTION_TIMEOUT_MS = 1000;

// Safety state tracking
unsigned long lastStateChangeTime = 0;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DISPLAY_DURATION_MS = 1500;  // Show calibration message
bool remoteArmed = false;

// USB Serial communication to Jetson
unsigned long lastJetsonSendTime = 0;
const unsigned long JETSON_SEND_INTERVAL_MS = 20;  // 50 Hz
bool jetsonConnected = false;

// Timer variables
esp_timer_handle_t watchdog_timer;
const int64_t timeout_us = 150000; // 0.15 seconds in microseconds

// ISR for the watchdog timer
void IRAM_ATTR onWatchdogTimeout(void* arg) {
  // Stop all movements
  motorFL.velocity = 0;
  motorFR.velocity = 0;
  motorBL.velocity = 0;
  motorBR.velocity = 0;

  motorFL.state = 8;
  motorFR.state = 8;
  motorBL.state = 8;
  motorBR.state = 8;

  // Call controlMecanumWheels with zero velocities to stop the robot
  controlMecanumWheels(joystickValues, motorFL, motorFR, motorBL, motorBR);
}



// #############################################################
// Safety Functions
// #############################################################

void emergencyStop() {
  // Set all motor velocities to zero
  motorFL.velocity = 0;
  motorFR.velocity = 0;
  motorBL.velocity = 0;
  motorBR.velocity = 0;
  
  // Set all motors to closed-loop control (state 8) with zero velocity
  motorFL.state = 8;
  motorFR.state = 8;
  motorBL.state = 8;
  motorBR.state = 8;
  
  // Send stop commands immediately
  sendMotorCommands(motorFL, motorFR, motorBL, motorBR);
  
  Serial.println("EMERGENCY STOP - All motors stopped");
}

void updateSafetyState() {
  unsigned long currentTime = millis();
  SafetyState previousState = safetyState;
  
  // Check connection status
  bool connected = (currentTime - lastManualCommandTime) < CONNECTION_TIMEOUT_MS;
  
  // State machine logic
  switch (safetyState) {
    case STATE_DISCONNECTED:
      if (connected) {
        safetyState = STATE_CONNECTED_DISARMED;
        calibrationStartTime = currentTime;
        lastStateChangeTime = currentTime;
        Serial.println("State: CONNECTED_DISARMED");
      }
      break;
      
    case STATE_CONNECTED_DISARMED:
      if (!connected) {
        safetyState = STATE_DISCONNECTED;
        emergencyStop();
        lastStateChangeTime = currentTime;
        Serial.println("State: DISCONNECTED (emergency stop)");
      } else if (currentTime - calibrationStartTime < CALIBRATION_DISPLAY_DURATION_MS) {
        // Show calibration message for a bit (remote is calibrating)
        safetyState = STATE_CALIBRATING;
        lastStateChangeTime = currentTime;
        Serial.println("State: CALIBRATING");
      }
      break;
      
    case STATE_CALIBRATING:
      if (!connected) {
        safetyState = STATE_DISCONNECTED;
        emergencyStop();
        lastStateChangeTime = currentTime;
        Serial.println("State: DISCONNECTED (emergency stop)");
      } else if (currentTime - calibrationStartTime >= CALIBRATION_DISPLAY_DURATION_MS) {
        // Calibration display period over, back to disarmed
        safetyState = STATE_CONNECTED_DISARMED;
        lastStateChangeTime = currentTime;
        Serial.println("State: CONNECTED_DISARMED (calibration complete)");
      }
      break;
      
    case STATE_ARMED:
      if (!connected) {
        safetyState = STATE_DISCONNECTED;
        emergencyStop();
        lastStateChangeTime = currentTime;
        Serial.println("State: DISCONNECTED (from armed - emergency stop)");
      } else if (!remoteArmed) {
        safetyState = STATE_CONNECTED_DISARMED;
        emergencyStop();
        lastStateChangeTime = currentTime;
        Serial.println("State: CONNECTED_DISARMED (disarmed by user)");
      }
      break;
  }
  
  // Log state changes
  if (previousState != safetyState) {
    Serial.printf("Safety state changed: %d -> %d\n", previousState, safetyState);
  }
}

// #############################################################
// Jetson Communication Functions
// #############################################################

void sendArmDataToJetson() {
  unsigned long currentTime = millis();
  
  // Send at 50 Hz rate
  if (currentTime - lastJetsonSendTime < JETSON_SEND_INTERVAL_MS) {
    return;
  }
  lastJetsonSendTime = currentTime;
  
  // Binary packet: 23 bytes total (added trigger_reboot field)
  // [0xAA][type][left_x][left_y][left_z][right_x][right_y][right_rot][mode][gripper][armed][trigger_reboot][timestamp][checksum]
  uint8_t packet[23];
  
  packet[0] = 0xAA;  // Start marker
  packet[1] = 1;     // Message type: manual_control
  
  // Pack 16-bit signed values (little endian)
  *((int16_t*)&packet[2]) = controlData.left_x;
  *((int16_t*)&packet[4]) = controlData.left_y;
  *((int16_t*)&packet[6]) = controlData.left_z;
  *((int16_t*)&packet[8]) = controlData.right_x;
  *((int16_t*)&packet[10]) = controlData.right_y;
  *((int16_t*)&packet[12]) = controlData.right_rot;
  
  packet[14] = controlData.switch_platform_mode ? 0 : 1;  // 0=platform, 1=vertical
  packet[15] = (uint8_t)controlData.gripper_pot;
  packet[16] = controlData.armed ? 1 : 0;  // Armed state
  packet[17] = controlData.trigger_reboot ? 1 : 0;  // Semi-reboot trigger
  
  // Pack 32-bit timestamp (little endian)
  *((uint32_t*)&packet[18]) = currentTime;
  
  // Calculate checksum (sum of all bytes except checksum)
  uint8_t checksum = 0;
  for (int i = 0; i < 22; i++) {
    checksum += packet[i];
  }
  packet[22] = checksum;
  
  // Send binary packet
  Serial.write(packet, 23);
}

void processJetsonResponse() {
  // Check for incoming status messages from Jetson
  if (Serial.available()) {
    String response = Serial.readStringUntil('\n');
    response.trim();
    
    if (response.length() > 0) {
      // Simple parsing - look for "arm_status" type
      if (response.indexOf("\"type\":\"arm_status\"") >= 0) {
        jetsonConnected = true;
        lastAutonomousCommandTime = millis();
        
        // Could parse more detailed status here if needed
        // For now, just acknowledge connection
      }
      
      if (response.indexOf("PING") >= 0) {
        Serial.println("PONG_PLATFORM");
      }
    }
  }
}

// #############################################################
// Display Functions
// #############################################################

void initDisplay() {
  Serial.println("Initializing display...");
  
  // CRITICAL: Power on display (pin 38)
  pinMode(38, OUTPUT);
  digitalWrite(38, HIGH);
  
  // Initialize GFX
  if (!gfx->begin()) {
    Serial.println("gfx->begin() failed!");
    return;
  }
  
  // Create canvas for double buffering
  gfx2 = new Arduino_Canvas(SCREEN_WIDTH, SCREEN_HEIGHT, gfx, 0, 0);
  gfx2->begin(GFX_SKIP_OUTPUT_BEGIN);
  
  displayInitialized = true;
  Serial.println("Display initialized!");
  
  // Draw initial UI
  drawUI();
}

void drawUI() {
  if (!displayInitialized) return;
  
  gfx2->fillScreen(COLOR_BG);
  
  // Draw static layout elements (moved down by 30px for second header line)
  // Left: Connection status circle placeholder
  gfx2->drawCircle(120, 150, 80, COLOR_BAR_BG);
  
  // Right: Movement direction circle
  gfx2->drawCircle(400, 150, 80, COLOR_BAR_BG);
  gfx2->drawCircle(400, 150, 3, COLOR_TEXT);  // Center dot
  
  gfx2->flush();
}

// Battery monitoring removed - requires PSRAM and complex library setup

void drawConnectionStatus(int x, int y, int radius, bool connected, const char* label) {
  if (!displayInitialized) return;
  
  // Clear area
  gfx2->fillCircle(x, y, radius + 5, COLOR_BG);
  
  // Draw status circle
  uint16_t color = connected ? COLOR_GOOD : COLOR_ERROR;
  gfx2->fillCircle(x, y, radius, color);
  gfx2->drawCircle(x, y, radius, COLOR_TEXT);
  
  // Draw label below
  gfx2->setTextSize(2);
  gfx2->setTextColor(COLOR_TEXT);
  int textWidth = strlen(label) * 12;  // Approximate width
  gfx2->setCursor(x - textWidth/2, y + radius + 10);
  gfx2->print(label);
}

void drawMovementIndicator(int centerX, int centerY, int radius) {
  if (!displayInitialized) return;
  
  // Clear area
  gfx2->fillCircle(centerX, centerY, radius + 10, COLOR_BG);
  
  // Redraw circle
  gfx2->drawCircle(centerX, centerY, radius, COLOR_BAR_BG);
  gfx2->drawCircle(centerX, centerY, 3, COLOR_TEXT);
  
  // Draw crosshair
  gfx2->drawFastHLine(centerX - radius, centerY, radius * 2, COLOR_BAR_BG);
  gfx2->drawFastVLine(centerX, centerY - radius, radius * 2, COLOR_BAR_BG);
  
  // Calculate movement vector
  // Use joystick values directly for visualization
  int vecX = map(joystickData.x, -255, 255, -radius + 10, radius - 10);
  int vecY = map(joystickData.y, -255, 255, -radius + 10, radius - 10);
  
  // Draw movement arrow if there's movement
  if (abs(joystickData.x) > 10 || abs(joystickData.y) > 10) {
    int endX = centerX + vecX;
    int endY = centerY - vecY;  // Invert Y for screen coordinates
    
    // Draw thick line
    for (int i = -2; i <= 2; i++) {
      for (int j = -2; j <= 2; j++) {
        gfx2->drawLine(centerX + i, centerY + j, endX + i, endY + j, COLOR_GOOD);
      }
    }
    
    // Draw arrowhead
    gfx2->fillCircle(endX, endY, 8, COLOR_GOOD);
    gfx2->drawCircle(endX, endY, 8, COLOR_TEXT);
  }
  
  // Draw rotation indicator
  if (abs(joystickData.rot) > 10) {
    uint16_t rotColor = joystickData.rot > 0 ? CYAN : MAGENTA;
    // Draw rotation arc
    for (int angle = 0; angle < 360; angle += 10) {
      float rad = angle * PI / 180.0;
      int arcX = centerX + (int)((radius - 15) * cos(rad));
      int arcY = centerY + (int)((radius - 15) * sin(rad));
      gfx2->fillCircle(arcX, arcY, 3, rotColor);
    }
  }
}

// Battery indicator removed - requires complex PMU library with PSRAM

void updateDisplay() {
  if (!displayInitialized) return;
  
  unsigned long currentTime = millis();
  
  if (currentTime - lastDisplayUpdate < DISPLAY_UPDATE_MS) {
    return;
  }
  lastDisplayUpdate = currentTime;
  
  // Check connection status
  bool manualConnected = (currentTime - lastManualCommandTime) < CONNECTION_TIMEOUT_MS;
  bool autoConnected = (currentTime - lastAutonomousCommandTime) < CONNECTION_TIMEOUT_MS;
  bool armConnected = (currentTime - lastArmCommandTime) < CONNECTION_TIMEOUT_MS;
  
  // LEFT: Connection Status with Safety State (moved down)
  bool connected = manualConnected || autoConnected;
  const char* statusLabel;
  uint16_t statusColor;
  
  if (!connected) {
    statusLabel = "OFFLINE";
    statusColor = COLOR_ERROR;
  } else {
    switch (safetyState) {
      case STATE_DISCONNECTED:
        statusLabel = "OFFLINE";
        statusColor = COLOR_ERROR;
        break;
      case STATE_CONNECTED_DISARMED:
        statusLabel = "DISARMED";
        statusColor = ORANGE;
        break;
      case STATE_CALIBRATING:
        statusLabel = "CALIBRATE";
        statusColor = COLOR_WARNING;
        break;
      case STATE_ARMED:
        statusLabel = "ARMED";
        statusColor = COLOR_GOOD;
        break;
    }
  }
  
  // Draw connection status with color-coded circle (moved down by 30px)
  gfx2->fillCircle(120, 150, 80, COLOR_BG);
  gfx2->fillCircle(120, 150, 75, statusColor);
  gfx2->drawCircle(120, 150, 80, COLOR_TEXT);
  
  // Draw label below
  gfx2->setTextSize(2);
  gfx2->setTextColor(COLOR_TEXT);
  int textWidth = strlen(statusLabel) * 12;
  gfx2->setCursor(120 - textWidth/2, 150 + 85);
  gfx2->print(statusLabel);
  
  // RIGHT: Movement Direction Indicator (moved down by 30px)
  drawMovementIndicator(400, 150, 80);
  
  // TOP LINE 1: Title
  gfx2->fillRect(0, 0, SCREEN_WIDTH, 60, COLOR_BG);
  gfx2->setTextSize(2);
  gfx2->setTextColor(COLOR_TEXT);
  gfx2->setCursor(10, 8);
  gfx2->print("JAMES");
  
  // Show current mode with color
  gfx2->setCursor(SCREEN_WIDTH - 150, 8);
  switch (currentMode) {
    case MODE_MANUAL:
      gfx2->setTextColor(COLOR_MANUAL);
      gfx2->print("MANUAL");
      break;
    case MODE_AUTONOMOUS:
      gfx2->setTextColor(COLOR_AUTO);
      gfx2->print("AUTO");
      break;
    case MODE_STOPPED:
      gfx2->setTextColor(COLOR_ERROR);
      gfx2->print("STOP");
      break;
  }
  
  // TOP LINE 2: Control Mode (Platform vs Arm)
  gfx2->setTextSize(2);
  gfx2->setCursor(10, 35);
  gfx2->setTextColor(COLOR_TEXT);
  gfx2->print("Mode: ");
  
  if (controlData.switch_platform_mode) {
    gfx2->setTextColor(CYAN);
    gfx2->print("PLATFORM");
  } else {
    gfx2->setTextColor(MAGENTA);
    gfx2->print("ARM");
  }
  
  // BOTTOM: Velocity values and Jetson status (small text)
  gfx2->fillRect(0, SCREEN_HEIGHT - 25, SCREEN_WIDTH, 25, COLOR_BG);
  gfx2->setTextSize(1);
  gfx2->setTextColor(COLOR_TEXT);
  
  char buf[100];
  snprintf(buf, sizeof(buf), "FL:%.1f FR:%.1f BL:%.1f BR:%.1f", 
           motorFL.velocity, motorFR.velocity, motorBL.velocity, motorBR.velocity);
  gfx2->setCursor(10, SCREEN_HEIGHT - 18);
  gfx2->print(buf);
  
  // Jetson connection status (right side)
  gfx2->setCursor(SCREEN_WIDTH - 80, SCREEN_HEIGHT - 18);
  if (armConnected) {
    gfx2->setTextColor(jetsonConnected ? COLOR_GOOD : COLOR_WARNING);
    gfx2->print("JETSON");
  } else {
    gfx2->setTextColor(COLOR_BAR_BG);
    gfx2->print("NO ARM");
  }
  
  // Flush to display
  gfx2->flush();
}

// #############################################################

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n\nJames Platform Controller - AMOLED Version (Landscape)");
  
  // Initialize display FIRST
  initDisplay();
  
  Wire.begin(I2C_SDA, I2C_SCL); // Initialize I2C communication with the defined SDA and SCL pins
  
  // Initialize ESP-NOW
  Serial.println("Initializing ESP-NOW...");
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onDataReceive);
  Serial.println("ESP-NOW initialized");

   // Configure the timer
  esp_timer_create_args_t timer_args;
  timer_args.callback = &onWatchdogTimeout;
  timer_args.name = "watchdog_timer";
  
  esp_timer_create(&timer_args, &watchdog_timer);

  delay(1000);

  Serial.println("Initializing ODrive communication...");
  iicSerial1.begin(115200, IICSerial_8N1);/*UART1 init*/
  iicSerial2.begin(115200, IICSerial_8N1);/*UART2 init*/
  
  delay(1000);

  motorFL_state = -1;
  motorFR_state = -1;
  motorBL_state = -1;
  motorBR_state = -1;

  Serial.println("Setup complete!");
}


void loop() {
  // Update safety state machine
  updateSafetyState();
  
  // Check for connection timeout and stop motors if needed
  unsigned long currentTime = millis();
  if (safetyState == STATE_ARMED && 
      (currentTime - lastManualCommandTime) >= CONNECTION_TIMEOUT_MS) {
    emergencyStop();
    safetyState = STATE_DISCONNECTED;
    Serial.println("Connection timeout - emergency stop");
  }
  
  // Process Jetson communication
  processJetsonResponse();
  
  // Update display
  updateDisplay();
  
  delay(1);
}

// ###################################################################################################



// Function to calculate the direction, velocity, and rotational velocity
JoystickValues calculateMovement(int x, int y, int rot) {
  JoystickValues result;

  // Invert joystick inputs to correct movement directions
  x = -x; // Invert forward/backward
  y = -y; // Invert left/right
  rot = -rot; // Invert rotation

  // Calculate direction in degrees (0-360)
  result.direction = atan2(y, x) * (180 / PI);
  if (result.direction < 0) {
    result.direction += 360;
  }

  // Calculate velocity (0-255)
  result.velocity = sqrt(x * x + y * y);
  if (result.velocity > 255) {
    result.velocity = 255;
  }

  // Set rotational velocity (-127 to 127)
  result.rotationalVelocity = rot;
  if (result.rotationalVelocity < -127) {
    result.rotationalVelocity = -127;
  } else if (result.rotationalVelocity > 127) {
    result.rotationalVelocity = 127;
  }

  return result;
}





void controlMecanumWheels(JoystickValues joystick, MotorCommand& motorFL, MotorCommand& motorFR, MotorCommand& motorBL, MotorCommand& motorBR) {
  // Calculate wheel velocities based on joystick values
  float rad = joystick.direction * PI / 180.0; // Convert direction to radians
  float cosD = cos(rad);
  float sinD = sin(rad);

  float vx = joystick.velocity * cosD;  // Velocity component in x direction
  float vy = joystick.velocity * sinD;  // Velocity component in y direction
  float omega = joystick.rotationalVelocity; // Rotational velocity

  // Calculate velocities for each wheel based on joystick input
  float velocityFL = (vy - vx - omega); // Front-left
  float velocityFR = (vy + vx + omega); // Front-right
  float velocityBL = (vy + vx - omega); // Back-left
  float velocityBR = (vy - vx + omega); // Back-right

  // Exponential mapping function for more natural motor control
  auto mapExp = [](float value, float inMin, float inMax, float outMin, float outMax) {
    float normalizedValue = (value - inMin) / (inMax - inMin); // Normalize value to [0, 1]
    float mappedValue = exp(normalizedValue) - 1; // Apply exponential mapping. Adjust base as needed.
    mappedValue = mappedValue / (exp(1) - 1); // Normalize back to [0, 1]
    return outMin + mappedValue * (outMax - outMin); // Map to output range
  };
  
  // Apply mirroring for left side motors
  velocityFL = -velocityFL;
  velocityBL = -velocityBL;

  // Apply exponential mapping to each wheel velocity to map it to -3.00 to 3.00 range
  if (velocityFL != 0) {
    motorFL.velocity = (velocityFL / abs(velocityFL)) * mapExp(abs(velocityFL), 0, 255, 0.00, 3.00);
  } else {
    motorFL.velocity = 0;
  }
  if (velocityFR != 0) {
    motorFR.velocity = (velocityFR / abs(velocityFR)) * mapExp(abs(velocityFR), 0, 255, 0.00, 3.00);
  } else {
    motorFR.velocity = 0;
  }

  if (velocityBL != 0) {
    motorBL.velocity = (velocityBL / abs(velocityBL)) * mapExp(abs(velocityBL), 0, 255, 0.00, 3.00);
  } else {
    motorBL.velocity = 0;
  }
  if (velocityBR != 0) {
    motorBR.velocity = (velocityBR / abs(velocityBR)) * mapExp(abs(velocityBR), 0, 255, 0.00, 3.00);
  } else {
    motorBR.velocity = 0;
  }
/*  
  // Calculate wheel velocities based on joystick values
  float rad = joystick.direction * PI / 180.0; // Convert direction to radians
  float cosD = cos(rad);
  float sinD = sin(rad);

  float vx = joystick.velocity * cosD;  // Velocity component in x direction
  float vy = joystick.velocity * sinD;  // Velocity component in y direction
  float omega = joystick.rotationalVelocity; // Rotational velocity

  // Calculate velocities for each wheel
  float velocityFL = (vy - vx - omega)/50; // Front-left
  float velocityFR = (vy + vx + omega)/50; // Front-right
  float velocityBL = (vy + vx - omega)/50; // Back-left
  float velocityBR = (vy - vx + omega)/50; // Back-right

  // Normalize velocities to the range -10 to 10
  float maxSpeed = max(max(abs(velocityFL), abs(velocityFR)), max(abs(velocityBL), abs(velocityBR)));
  if (maxSpeed > 10) {
    velocityFL /= maxSpeed / 10.0;
    velocityFR /= maxSpeed / 10.0;
    velocityBL /= maxSpeed / 10.0;
    velocityBR /= maxSpeed / 10.0;
  }





  // Create motor commands
  motorFL.velocity = velocityFL;
  motorFR.velocity = velocityFR;
  motorBL.velocity = velocityBL;
  motorBR.velocity = velocityBR;
*/
  // Set motor states - always use state 8 (closed-loop velocity control)
  // State 8 handles both moving and holding position (velocity = 0)
  motorFL.state = 8;
  motorFR.state = 8;
  motorBL.state = 8;
  motorBR.state = 8;

//Serial.println("FL:" + String(motorFL.velocity) + " FR:" + String(motorFR.velocity) + " BL:" + String(motorBL.velocity) + "BR:" + String(motorBR.velocity));
  sendMotorCommands(motorFL, motorFR, motorBL, motorBR);

}
 





  
// Simplified callback - always expects RemoteControlData structure
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  // Always expect RemoteControlData structure
  if (len == sizeof(RemoteControlData)) {
    memcpy(&controlData, incomingData, sizeof(controlData));
    
    // Update timestamps
    lastManualCommandTime = millis();
    lastArmCommandTime = millis();
    
    // Update remote armed state
    remoteArmed = controlData.armed;
    
    // Update safety state machine
    updateSafetyState();
    
    // Check if remote armed and transition to ARMED state if needed
    if (remoteArmed && safetyState == STATE_CONNECTED_DISARMED) {
      safetyState = STATE_ARMED;
      lastStateChangeTime = millis();
      Serial.println("State: ARMED (armed by user)");
    }
    
    // SAFETY CHECK: Only process motion commands if ARMED
    if (safetyState == STATE_ARMED && remoteArmed) {
      // Platform control logic based on switch state
      if (controlData.switch_platform_mode) {
        // Platform mode: Use right joystick for mecanum control
        // ZERO OUT right joystick data before sending to Jetson (platform controls it)
        RemoteControlData armData = controlData;
        armData.right_x = 0;
        armData.right_y = 0;
        armData.right_rot = 0;
        
        // Send filtered data to Jetson (no right joystick in platform mode)
        RemoteControlData originalData = controlData;
        controlData = armData;
        sendArmDataToJetson();
        controlData = originalData;
        
        joystickData.x = controlData.right_x;
        joystickData.y = controlData.right_y;
        joystickData.rot = controlData.right_rot;
        
        // Manual control takes priority - set mode
        currentMode = MODE_MANUAL;
        
        // Calculate movement values and control mecanum wheels
        joystickValues = calculateMovement(joystickData.x, joystickData.y, joystickData.rot);
        controlMecanumWheels(joystickValues, motorFL, motorFR, motorBL, motorBR);
      } else {
        // Vertical arm mode: Right joystick controls arm, stop platform
        // Send full data to Jetson (arm mode - all joysticks active)
        sendArmDataToJetson();
        
        // Stop platform motors
        joystickData.x = 0;
        joystickData.y = 0;
        joystickData.rot = 0;
        
        joystickValues = calculateMovement(0, 0, 0);
        controlMecanumWheels(joystickValues, motorFL, motorFR, motorBL, motorBR);
        
        currentMode = MODE_MANUAL;
      }
      
    } else {
      // NOT ARMED: Stop all motors immediately
      // Still send data to Jetson for monitoring (but with zero velocities)
      sendArmDataToJetson();
      emergencyStop();
      currentMode = MODE_STOPPED;
    }
    
    // Acknowledge receipt
    esp_now_send(info->des_addr, (uint8_t *)"ACK", sizeof("ACK"));
    
  } else {
    // Unexpected packet size - log error
    Serial.printf("Unexpected packet size: %d bytes (expected %d)\n", len, sizeof(RemoteControlData));
  }
}



void sendMotorCommands(MotorCommand motorFL, MotorCommand motorFR, MotorCommand motorBL, MotorCommand motorBR) {
  // Periodically force state re-initialization to handle late ODrive power-on
  unsigned long currentTime = millis();
  bool forceStateUpdate = (currentTime - lastMotorStateInit) > MOTOR_STATE_REINIT_INTERVAL_MS;
  
  if (forceStateUpdate) {
    lastMotorStateInit = currentTime;
    // Reset state tracking to force re-send
    motorFL_state = -1;
    motorFR_state = -1;
    motorBL_state = -1;
    motorBR_state = -1;
    Serial.println("Forcing motor state re-initialization...");
  }
  
  // Force state update if motors were previously uninitialized or if state changed
  // This handles the case where ODrive is powered on after the controller
  
  if(motorFL_state != motorFL.state){
    motorFL_state = motorFL.state;
    String state_commandFL = "w axis0.requested_state " + String(motorFL.state); // Front left motor (0)
    iicSerial2.println(state_commandFL); // Send to left motors
    Serial.println("FL State: " + state_commandFL);
  }
  if(motorFR_state != motorFR.state){
    motorFR_state = motorFR.state;
    String state_commandFR = "w axis1.requested_state " + String(motorFR.state); // Front right motor (1)
    iicSerial1.println(state_commandFR); // Send to right motors
    Serial.println("FR State: " + state_commandFR);
  }
  if(motorBL_state != motorBL.state){
    motorBL_state = motorBL.state;
    String state_commandBL = "w axis1.requested_state " + String(motorBL.state); // Back left motor (1)
    iicSerial2.println(state_commandBL); // Send to left motors
    Serial.println("BL State: " + state_commandBL);
  }
  if(motorBR_state != motorBR.state){
    motorBR_state = motorBR.state;
    String state_commandBR = "w axis0.requested_state " + String(motorBR.state); // Back right motor (0)
    iicSerial1.println(state_commandBR); // Send to right motors
    Serial.println("BR State: " + state_commandBR);
  }

  // Create command strings for motor states
  



  // Create command strings for velociy
  String commandFL = "v 0 " + String(motorFL.velocity) + " 0"; // Front left motor (0)
  String commandFR = "v 1 " + String(motorFR.velocity) + " 0"; // Front right motor (1)
  String commandBL = "v 1 " + String(motorBL.velocity) + " 0"; // Back left motor (1)
  String commandBR = "v 0 " + String(motorBR.velocity) + " 0"; // Back right motor (0)


  // Send commands to motors
  iicSerial2.println(commandFL); // Send to left motors
  iicSerial1.println(commandFR); // Send to right motors
  iicSerial2.println(commandBL); // Send to left motors
  iicSerial1.println(commandBR); // Send to right motors


  //int motorFL_state, motorFR_state, motorBL_state, motorBR_state;


// w axis0.requested_state 1
  Serial.println("FL:" + commandFL + " FR:" + commandFR + " BL:" + commandBL + "BR:" + commandBR);

}


