# Safety Arming System - Implementation Plan

## Overview

This document describes the implementation of a safety arming system to prevent unintended robot motion on remote connection. The system uses the center button (Pin 39) for arming/disarming and automatic joystick calibration.

## Safety Flow

```
1. Remote connects → Display shows "CONNECTED - DISARMED"
2. Auto-calibrate joystick centers (1 second, hands off)
3. User presses center button (Pin 39) → "ARMED"
4. Motion only occurs when: armed AND joystick_moved
5. User presses center button again → "DISARMED"
```

## Key Safety Features

- Center button (Pin 39) for explicit arming/disarming toggle
- Automatic joystick center calibration on connection
- Immediate motor stop when disarmed
- Clear visual feedback on both displays
- Auto-disarm on connection loss

---

## REMOTE CONTROLLER IMPLEMENTATION

### File: `platform/remote/remote_control_v2_simple/remote_control_v2_simple.ino`

### 1. Data Structure Changes

**Add to `RemoteControlData` struct:**
```cpp
typedef struct RemoteControlData {
  // ... existing fields ...
  
  // Safety arming state
  bool armed;  // true = armed (motion allowed), false = disarmed (no motion)
} RemoteControlData;
```

### 2. Global Variables

**Add these variables:**
```cpp
// Center button pin for arming/disarming
const int centerButtonPin = 39;

// Arming state
bool systemArmed = false;
bool lastButtonState = false;
unsigned long lastButtonPressTime = 0;
const unsigned long BUTTON_DEBOUNCE_MS = 200;  // Debounce time

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
```

### 3. New Function: Joystick Calibration

**Add this function before `setup()`:**
```cpp
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
```

### 4. New Function: Read Calibrated Joystick

**Add this function:**
```cpp
int readCalibratedJoystick(int pin, int offset) {
  int rawValue = analogRead(pin);
  int calibratedValue = rawValue + offset;
  
  // Clamp to valid ADC range
  if (calibratedValue < 0) calibratedValue = 0;
  if (calibratedValue > 4095) calibratedValue = 4095;
  
  return calibratedValue;
}
```

### 5. New Function: Handle Arming Button

**Add this function:**
```cpp
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
```

### 6. Modify `sendControlData()` Function

**Replace joystick reading section with calibrated reads:**
```cpp
void sendControlData() {
  // Handle arming button
  handleArmingButton();
  
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
  
  // Read switch state (platform mode vs vertical arm mode)
  controlData.switch_platform_mode = digitalRead(leftSwitchPin) == HIGH;
  
  // Set armed state
  controlData.armed = systemArmed;
  
  // ... rest of function (gripper pot, esp_now_send) remains the same ...
}
```

### 7. Modify `updateDisplay()` Function

**Add arming status display at top:**
```cpp
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
  gfx2->setTextColor(BLACK);
  gfx2->setTextSize(2);
  gfx2->setCursor(10, 8);
  gfx2->print(statusText);
  
  // ... rest of display update code remains the same ...
}
```

### 8. Modify `setup()` Function

**Add center button pin setup and calibration after ESP-NOW initialization:**
```cpp
void setup() {
  // ... existing setup code ...
  
  // Setup center button pin for arming/disarming
  pinMode(centerButtonPin, INPUT_PULLUP);
  
  Serial.println("Setup complete!");
  
  // Display calibration message
  gfx2->fillRect(0, 0, 240, 30, YELLOW);
  gfx2->setTextColor(BLACK);
  gfx2->setTextSize(2);
  gfx2->setCursor(10, 8);
  gfx2->println("CALIBRATING...");
  gfx2->setCursor(10, 450);
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
  gfx2->setCursor(10, 450);
  gfx2->setTextColor(GREEN);
  gfx2->println("Ready! Press center button");
  gfx2->setCursor(10, 470);
  gfx2->println("(Pin 39) to ARM");
  gfx2->flush();
  
  delay(1000);  // Show message briefly
}
```

---

## PLATFORM CONTROLLER IMPLEMENTATION

### File: `platform/controller/plattform_controller/plattform_controller.ino`

### 1. State Machine Enumeration

**Add after existing enums:**
```cpp
// Safety state machine
enum SafetyState {
  STATE_DISCONNECTED,
  STATE_CONNECTED_DISARMED,
  STATE_CALIBRATING,
  STATE_ARMED
};
SafetyState safetyState = STATE_DISCONNECTED;
```

### 2. Global Variables

**Add these variables:**
```cpp
// Safety state tracking
unsigned long lastStateChangeTime = 0;
unsigned long calibrationStartTime = 0;
const unsigned long CALIBRATION_DISPLAY_DURATION_MS = 1500;  // Show calibration message
bool remoteArmed = false;
```

### 3. New Function: Stop All Motors Immediately

**Add this function before `setup()`:**
```cpp
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
```

### 4. New Function: Update Safety State Machine

**Add this function:**
```cpp
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
        lastStateChangeTime = currentTime;
        Serial.println("State: DISCONNECTED");
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
        lastStateChangeTime = currentTime;
        Serial.println("State: DISCONNECTED");
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
        Serial.println("State: DISCONNECTED (from armed)");
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
```

### 5. Modify `onDataReceive()` Callback

**Replace the function with safety checks:**
```cpp
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
    
    // Always send arm data to Jetson (let Jetson decide what to do with it)
    sendArmDataToJetson();
    
    // SAFETY CHECK: Only process motion commands if ARMED
    if (safetyState == STATE_ARMED && remoteArmed) {
      // Platform control logic based on switch state
      if (controlData.switch_platform_mode) {
        // Platform mode: Use right joystick for mecanum control
        joystickData.x = controlData.right_x;
        joystickData.y = controlData.right_y;
        joystickData.rot = controlData.right_rot;
      } else {
        // Vertical arm mode: Use right joystick for vertical arm control
        // For now, still control platform but could be modified later
        joystickData.x = controlData.right_x;
        joystickData.y = controlData.right_y;
        joystickData.rot = controlData.right_rot;
      }
      
      // Manual control takes priority - set mode
      currentMode = MODE_MANUAL;
      
      // Calculate movement values and control mecanum wheels
      joystickValues = calculateMovement(joystickData.x, joystickData.y, joystickData.rot);
      controlMecanumWheels(joystickValues, motorFL, motorFR, motorBL, motorBR);
      
    } else {
      // NOT ARMED: Stop all motors immediately
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
```

### 6. Modify `updateDisplay()` Function

**Update to show safety state:**
```cpp
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
  
  // LEFT: Connection Status with Safety State
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
  
  // Draw connection status with color-coded circle
  gfx2->fillCircle(120, 120, 80, COLOR_BG);
  gfx2->fillCircle(120, 120, 75, statusColor);
  gfx2->drawCircle(120, 120, 80, COLOR_TEXT);
  
  // Draw label below
  gfx2->setTextSize(2);
  gfx2->setTextColor(COLOR_TEXT);
  int textWidth = strlen(statusLabel) * 12;
  gfx2->setCursor(120 - textWidth/2, 120 + 85);
  gfx2->print(statusLabel);
  
  // ... rest of display code remains the same ...
}
```

### 7. Modify `loop()` Function

**Add safety state update:**
```cpp
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
```

---

## Testing Procedure

### 1. Initial Connection Test
- Power on platform controller
- Power on remote controller
- Verify display shows "CALIBRATING..." then "DISARMED"
- Verify platform display shows "DISARMED"
- Try moving joysticks - robot should NOT move

### 2. Arming Test
- Press center button (Pin 39) on remote
- Verify remote display shows "ARMED" in green
- Verify platform display shows "ARMED" in green
- Move joysticks - robot SHOULD move

### 3. Disarming Test
- Press center button again
- Verify remote display shows "DISARMED" in orange
- Verify platform display shows "DISARMED" in orange
- Verify robot stops immediately
- Try moving joysticks - robot should NOT move

### 4. Connection Loss Test
- Arm the system
- Move joysticks to verify motion
- Power off remote
- Verify platform stops immediately
- Verify platform display shows "OFFLINE"

### 5. Calibration Test
- Power cycle remote
- Keep hands off joysticks during calibration
- Verify "CALIBRATING..." message appears
- After calibration, verify joysticks are centered (no drift)

---

## Safety Notes

1. **Always test in a safe environment** - clear area, robot on blocks initially
2. **Emergency stop** - Keep power switch accessible at all times
3. **Calibration** - Must be performed with hands off joysticks
4. **Connection loss** - System automatically disarms and stops
5. **Visual feedback** - Always check display before operating

---

## Future Enhancements

- Add audible beep on arming/disarming
- Add velocity limiting in first 2 seconds after arming
- Add "soft start" ramping for smoother motion
- Log arming events to SD card for safety audit
- Add second confirmation button for arming (two-hand operation)

---

## Hardware Pin Assignment

- **Center Button (Arming/Disarming)**: Pin 39 (ESP32 on remote controller)
  - Active LOW (pressed = LOW, released = HIGH)
  - Internal pull-up enabled
  - Used for toggle arming state

---

## Revision History

- v1.0 - Initial implementation plan (2026-02-21)
- v1.1 - Updated to use center button on Pin 39 instead of left switch (2026-02-22)
