/*
 * James Robot - Remote Control with AMOLED Display
 * Hardware: LilyGO T-Display S3 AMOLED V2
 * Features:
 * - 3-axis joystick control (forward/back, left/right, rotation)
 * - AMOLED display showing joystick values in real-time
 * - Connection status and signal strength display
 * - Battery level monitoring (if available)
 * - ESPNOW communication to platform controller
 */

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "rm67162.h"  // LilyGO AMOLED display driver
#include "lv_conf.h"
#include "lvgl.h"

// Display configuration (Portrait mode)
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 536
#define DISPLAY_UPDATE_MS 100  // Update display every 100ms (10 Hz)

// ADC configuration
#define V_REF 3.3
#define ADC_MAX 4095
#define ZERO_LOW 1.5
#define ZERO_HIGH 1.8

const int adcDeadZoneMin = 1900;
const int adcDeadZoneMax = 2250;

// Joystick pins
const int rightJoystickUpDownPin = 13;      // Forward/backward
const int rightJoystickLeftRightPin = 14;   // Left/right
const int rightJoystickRotationPin = 15;    // Rotation

// Define the structure for sending data
typedef struct TransformedValues {
  int value13; // forward / backward
  int value14; // left / right
  int value15; // rotation
} TransformedValues;

TransformedValues transformedValues;

// Receiver MAC address (platform controller)
uint8_t broadcastAddress[] = {0x24, 0x58, 0x7C, 0xD3, 0x6B, 0x30};

// Connection status tracking
bool lastSendSuccess = false;
unsigned long lastSuccessTime = 0;
unsigned long lastDisplayUpdate = 0;
int sendSuccessCount = 0;
int sendFailCount = 0;

// LVGL display buffer
static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[SCREEN_WIDTH * 10];

// UI elements
lv_obj_t *label_title;
lv_obj_t *label_joystick_x;
lv_obj_t *label_joystick_y;
lv_obj_t *label_joystick_rot;
lv_obj_t *label_connection;
lv_obj_t *label_signal;
lv_obj_t *label_battery;
lv_obj_t *bar_x;
lv_obj_t *bar_y;
lv_obj_t *bar_rot;

// Display flush callback for LVGL
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p) {
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

  lcd_PushColors(area->x1, area->y1, w, h, (uint16_t *)&color_p->full);
  lv_disp_flush_ready(disp);
}

// Initialize LVGL and display
void initDisplay() {
  // Initialize AMOLED display
  rm67162_init();
  lcd_setRotation(0);  // Portrait orientation (240x536)
  
  // Initialize LVGL
  lv_init();
  lv_disp_draw_buf_init(&draw_buf, buf, NULL, SCREEN_WIDTH * 10);

  // Initialize display driver
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  disp_drv.hor_res = SCREEN_WIDTH;
  disp_drv.ver_res = SCREEN_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  Serial.println("Display initialized (Portrait mode)");
}

// Create UI elements (Portrait layout)
void createUI() {
  // Set dark background
  lv_obj_set_style_bg_color(lv_scr_act(), lv_color_hex(0x000000), 0);

  // Title
  label_title = lv_label_create(lv_scr_act());
  lv_label_set_text(label_title, "JAMES\nREMOTE");
  lv_obj_set_style_text_color(label_title, lv_color_hex(0x00FF00), 0);
  lv_obj_set_style_text_font(label_title, &lv_font_montserrat_16, 0);
  lv_obj_set_style_text_align(label_title, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 10);

  // Joystick X (Forward/Backward)
  label_joystick_x = lv_label_create(lv_scr_act());
  lv_label_set_text(label_joystick_x, "Fwd/Back: 0");
  lv_obj_set_style_text_color(label_joystick_x, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_joystick_x, LV_ALIGN_TOP_LEFT, 10, 80);

  bar_x = lv_bar_create(lv_scr_act());
  lv_obj_set_size(bar_x, 60, 80);  // Vertical bar
  lv_obj_align(bar_x, LV_ALIGN_TOP_LEFT, 10, 100);
  lv_bar_set_range(bar_x, -255, 255);
  lv_bar_set_value(bar_x, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_x, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_bg_color(bar_x, lv_color_hex(0x00FF00), LV_PART_INDICATOR);

  // Joystick Y (Left/Right)
  label_joystick_y = lv_label_create(lv_scr_act());
  lv_label_set_text(label_joystick_y, "Left/Right: 0");
  lv_obj_set_style_text_color(label_joystick_y, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_joystick_y, LV_ALIGN_TOP_LEFT, 10, 190);

  bar_y = lv_bar_create(lv_scr_act());
  lv_obj_set_size(bar_y, 60, 80);  // Vertical bar
  lv_obj_align(bar_y, LV_ALIGN_TOP_LEFT, 10, 210);
  lv_bar_set_range(bar_y, -255, 255);
  lv_bar_set_value(bar_y, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_y, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_bg_color(bar_y, lv_color_hex(0x0080FF), LV_PART_INDICATOR);

  // Joystick Rotation
  label_joystick_rot = lv_label_create(lv_scr_act());
  lv_label_set_text(label_joystick_rot, "Rotation: 0");
  lv_obj_set_style_text_color(label_joystick_rot, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_joystick_rot, LV_ALIGN_TOP_LEFT, 10, 300);

  bar_rot = lv_bar_create(lv_scr_act());
  lv_obj_set_size(bar_rot, 60, 80);  // Vertical bar
  lv_obj_align(bar_rot, LV_ALIGN_TOP_LEFT, 10, 320);
  lv_bar_set_range(bar_rot, -255, 255);
  lv_bar_set_value(bar_rot, 0, LV_ANIM_OFF);
  lv_obj_set_style_bg_color(bar_rot, lv_color_hex(0x404040), LV_PART_MAIN);
  lv_obj_set_style_bg_color(bar_rot, lv_color_hex(0xFF8000), LV_PART_INDICATOR);

  // Connection status
  label_connection = lv_label_create(lv_scr_act());
  lv_label_set_text(label_connection, "Status:\nConnecting...");
  lv_obj_set_style_text_color(label_connection, lv_color_hex(0xFFFF00), 0);
  lv_obj_align(label_connection, LV_ALIGN_TOP_LEFT, 10, 410);

  // Signal strength
  label_signal = lv_label_create(lv_scr_act());
  lv_label_set_text(label_signal, "Signal: --");
  lv_obj_set_style_text_color(label_signal, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_signal, LV_ALIGN_TOP_LEFT, 10, 460);

  // Battery level
  label_battery = lv_label_create(lv_scr_act());
  lv_label_set_text(label_battery, "Battery: N/A");
  lv_obj_set_style_text_color(label_battery, lv_color_hex(0xFFFFFF), 0);
  lv_obj_align(label_battery, LV_ALIGN_TOP_LEFT, 10, 490);

  Serial.println("UI created (Portrait layout)");
}

// Update display with current values
void updateDisplay() {
  unsigned long currentTime = millis();
  
  // Update display at 10 Hz
  if (currentTime - lastDisplayUpdate < DISPLAY_UPDATE_MS) {
    return;
  }
  lastDisplayUpdate = currentTime;

  // Update joystick values
  char buf[32];
  
  snprintf(buf, sizeof(buf), "Fwd/Back: %d", transformedValues.value13);
  lv_label_set_text(label_joystick_x, buf);
  lv_bar_set_value(bar_x, transformedValues.value13, LV_ANIM_OFF);

  snprintf(buf, sizeof(buf), "Left/Right: %d", transformedValues.value14);
  lv_label_set_text(label_joystick_y, buf);
  lv_bar_set_value(bar_y, transformedValues.value14, LV_ANIM_OFF);

  snprintf(buf, sizeof(buf), "Rotation: %d", transformedValues.value15);
  lv_label_set_text(label_joystick_rot, buf);
  lv_bar_set_value(bar_rot, transformedValues.value15, LV_ANIM_OFF);

  // Update connection status
  if (lastSendSuccess && (currentTime - lastSuccessTime < 1000)) {
    lv_label_set_text(label_connection, "Status: Connected");
    lv_obj_set_style_text_color(label_connection, lv_color_hex(0x00FF00), 0);
  } else {
    lv_label_set_text(label_connection, "Status: Disconnected");
    lv_obj_set_style_text_color(label_connection, lv_color_hex(0xFF0000), 0);
  }

  // Calculate signal quality (success rate)
  int totalSends = sendSuccessCount + sendFailCount;
  if (totalSends > 0) {
    int signalPercent = (sendSuccessCount * 100) / totalSends;
    snprintf(buf, sizeof(buf), "Signal: %d%%", signalPercent);
    lv_label_set_text(label_signal, buf);
    
    // Color code signal strength
    if (signalPercent > 80) {
      lv_obj_set_style_text_color(label_signal, lv_color_hex(0x00FF00), 0);
    } else if (signalPercent > 50) {
      lv_obj_set_style_text_color(label_signal, lv_color_hex(0xFFFF00), 0);
    } else {
      lv_obj_set_style_text_color(label_signal, lv_color_hex(0xFF0000), 0);
    }
  }

  // Update battery level (if available)
  // Note: Battery monitoring requires additional hardware/ADC pin
  // For now, display N/A
  float batteryVoltage = getBatteryVoltage();
  if (batteryVoltage > 0) {
    int batteryPercent = calculateBatteryPercent(batteryVoltage);
    snprintf(buf, sizeof(buf), "Battery: %d%%", batteryPercent);
    lv_label_set_text(label_battery, buf);
    
    // Color code battery level
    if (batteryPercent > 50) {
      lv_obj_set_style_text_color(label_battery, lv_color_hex(0x00FF00), 0);
    } else if (batteryPercent > 20) {
      lv_obj_set_style_text_color(label_battery, lv_color_hex(0xFFFF00), 0);
    } else {
      lv_obj_set_style_text_color(label_battery, lv_color_hex(0xFF0000), 0);
    }
  }

  // Handle LVGL tasks
  lv_timer_handler();
}

// Get battery voltage (placeholder - requires hardware implementation)
float getBatteryVoltage() {
  // TODO: Implement battery voltage reading if ADC pin is connected
  // For LiPo battery: typically 3.0V (empty) to 4.2V (full)
  // Example: return analogRead(BATTERY_PIN) * (V_REF / ADC_MAX) * voltage_divider_ratio;
  return 0.0;  // Return 0 if not available
}

// Calculate battery percentage from voltage
int calculateBatteryPercent(float voltage) {
  // LiPo battery voltage curve (approximate)
  const float V_MIN = 3.0;  // Empty
  const float V_MAX = 4.2;  // Full
  
  if (voltage <= V_MIN) return 0;
  if (voltage >= V_MAX) return 100;
  
  return (int)((voltage - V_MIN) / (V_MAX - V_MIN) * 100);
}

// Read joystick value
int readJoystick(int pin) {
  return analogRead(pin);
}

// Exponential mapping function for more natural control
int mapExp(int value, int inMin, int inMax, int outMin, int outMax) {
  float normalizedValue = (value - inMin) / static_cast<float>(inMax - inMin);
  float mappedValue = exp(normalizedValue) - 1;
  mappedValue = mappedValue / (exp(1) - 1);
  return outMin + mappedValue * (outMax - outMin);
}

// Map joystick ADC value to speed (-255 to 255)
int mapJoystickToSpeed(int adcValue) {
  if (adcValue < adcDeadZoneMin) {
    return mapExp(adcValue, 0, adcDeadZoneMin, -255, 0);
  } else if (adcValue > adcDeadZoneMax) {
    return mapExp(adcValue, adcDeadZoneMax, 4095, 0, 255);
  } else {
    return 0; // Dead zone
  }
}

// Callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  if (status == ESP_NOW_SEND_SUCCESS) {
    lastSendSuccess = true;
    lastSuccessTime = millis();
    sendSuccessCount++;
    
    // Reset counters periodically to avoid overflow
    if (sendSuccessCount + sendFailCount > 1000) {
      sendSuccessCount /= 2;
      sendFailCount /= 2;
    }
  } else {
    lastSendSuccess = false;
    sendFailCount++;
  }
}

// Read joystick values, transform them, and send via ESP-NOW
void sendTransformedValues() {
  // Read and transform joystick values
  transformedValues.value13 = mapJoystickToSpeed(readJoystick(rightJoystickUpDownPin));
  transformedValues.value14 = mapJoystickToSpeed(readJoystick(rightJoystickLeftRightPin));
  transformedValues.value15 = mapJoystickToSpeed(readJoystick(rightJoystickRotationPin));
  
  // Send via ESP-NOW
  esp_err_t result = esp_now_send(NULL, (uint8_t *)&transformedValues, sizeof(transformedValues));
  
  // Log for debugging
  if (result == ESP_OK) {
    Serial.printf("Sent: X=%d Y=%d Rot=%d\n", 
                  transformedValues.value14, 
                  transformedValues.value13, 
                  transformedValues.value15);
  } else {
    Serial.println("Send failed");
  }
}

// Read and print MAC address
void readMacAddress() {
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("MAC Address: %02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}

void setup() {
  // Initialize serial communication
  Serial.begin(115200);
  Serial.println("James Remote Control - AMOLED Version");

  // Initialize display
  initDisplay();
  createUI();

  // Set pin modes for joystick
  pinMode(rightJoystickUpDownPin, INPUT);
  pinMode(rightJoystickLeftRightPin, INPUT);
  pinMode(rightJoystickRotationPin, INPUT);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  readMacAddress();
  
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    lv_label_set_text(label_connection, "Status: ESP-NOW Error");
    lv_obj_set_style_text_color(label_connection, lv_color_hex(0xFF0000), 0);
    return;
  }
  
  // Register send callback
  esp_now_register_send_cb(onDataSent);

  // Register peer (platform controller)
  esp_now_peer_info_t peerInfo;
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;  
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    lv_label_set_text(label_connection, "Status: Peer Add Failed");
    lv_obj_set_style_text_color(label_connection, lv_color_hex(0xFF0000), 0);
    return;
  }

  Serial.println("Setup complete");
}

void loop() {
  // Read and send joystick values
  sendTransformedValues();
  
  // Update display
  updateDisplay();
  
  // Small delay for ~6.7 Hz transmission rate (150ms)
  delay(150);
}
