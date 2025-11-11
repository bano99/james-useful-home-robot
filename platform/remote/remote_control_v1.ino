#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

#define V_REF 3.3
#define ADC_MAX 4095
#define ZERO_LOW 1.5
#define ZERO_HIGH 1.8

const int adcDeadZoneMin = 1900;
const int adcDeadZoneMax = 2250;

// all pins available on the Remote control
const int leftJoystickUpDownPin = 1;
const int leftJoystickLeftRightPin = 11;
const int leftJoystickRotationPin = 12;
const int leftSliderPin = 10;
const int leftLEDPin = 41;
const int leftSwitchPin = 45;
const int leftButtonPin = 42;

const int rightJoystickUpDownPin = 13;
const int rightJoystickLeftRightPin = 14;
const int rightJoystickRotationPin = 15;
const int rightSliderPin = 16;
const int rightLEDPin = 46;
const int rightSwitchPin = 40;

const int centerButtonPin = 39;


// Define the structure for sending data
typedef struct TransformedValues {
  int value13; // vorwärts / rückwärts
  int value14; // link / rechts
  int value15; // rotation
} TransformedValues;

TransformedValues transformedValues;

// Define the peer MAC address (replace with the receiver's MAC address)
uint8_t broadcastAddress[] = {0x24, 0x58, 0x7C, 0xD3, 0x6B, 0x30};
// 24:58:7c:d1:b2:7c - sender
// 24:58:7c:d3:6b:30 - receiver

// Function to scale and transform voltage to -255 to 255
int transformVoltage(float voltage) {
  // Interpret values between 1.5V and 1.8V as zero
  if (voltage >= ZERO_LOW && voltage <= ZERO_HIGH) {
    return 0;
  }

  // Scale voltage to -255 to 255
  float scaledVoltage = (voltage - V_REF / 2) * (510 / V_REF);

  // Apply a sigmoid-like function to make values less sensitive around the middle and more sensitive towards the edges
  float sensitivityFactor = 2 / (1 + exp(-abs(scaledVoltage) / 50)) - 1;
  int transformedValue = (int)(scaledVoltage * sensitivityFactor);

  // Ensure values are within the -255 to 255 range
  if (transformedValue < -255) transformedValue = -255;
  if (transformedValue > 255) transformedValue = 255;

  return transformedValue;
}

int readJoystick(int pin) {
    return analogRead(pin);
}
// Callback when data is sent
void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("Delivery Status: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Success" : "Fail");
}

// Exponential mapping function for more natural motor control
int mapExp(int value, int inMin, int inMax, int outMin, int outMax) {
    float normalizedValue = (value - inMin) / static_cast<float>(inMax - inMin); // Normalize value to [0, 1]
    float mappedValue = exp(normalizedValue) - 1; // Apply exponential mapping. Adjust base as needed.
    mappedValue = mappedValue / (exp(1) - 1); // Normalize back to [0, 1]
    return outMin + mappedValue * (outMax - outMin); // Map to output range
}

int mapJoystickToSpeed(int adcValue) {
    if (adcValue < adcDeadZoneMin) {
        return mapExp(adcValue, 0, adcDeadZoneMin, -255, 0);
    } else if (adcValue > adcDeadZoneMax) {
        return mapExp(adcValue, adcDeadZoneMax, 4095, 0, 255);
    } else {
        return 0; // Dead zone
    }
}

// Function to read analog values, transform them, and send them via ESP-NOW
void sendTransformedValues() {
  

  transformedValues.value13 = mapJoystickToSpeed(readJoystick(13));
  transformedValues.value14 = mapJoystickToSpeed(readJoystick(14));
  transformedValues.value15 = mapJoystickToSpeed(readJoystick(15));
  unsigned long startTime = millis(); 
  esp_err_t result = esp_now_send(NULL, (uint8_t *)&transformedValues, sizeof(transformedValues));
  if (result == ESP_OK) {
    unsigned long elapsedTime = millis() - startTime;
    //Serial.println("Sent successfully");
    Serial.println("Sent successfully in " + String(elapsedTime) + "ms " + String(transformedValues.value14) + " , " + String(transformedValues.value13) + " , " + String(transformedValues.value15));
  } else {
    Serial.println("Send failed");
  }
}

void readMacAddress(){
  uint8_t baseMac[6];
  esp_err_t ret = esp_wifi_get_mac(WIFI_IF_STA, baseMac);
  if (ret == ESP_OK) {
    Serial.printf("%02x:%02x:%02x:%02x:%02x:%02x\n",
                  baseMac[0], baseMac[1], baseMac[2],
                  baseMac[3], baseMac[4], baseMac[5]);
  } else {
    Serial.println("Failed to read MAC address");
  }
}


void setup() {
  // Initialize serial communication
  Serial.begin(115200);

  // Set pin modes
  pinMode(13, INPUT);
  pinMode(14, INPUT);
  pinMode(15, INPUT);

  // Initialize WiFi
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_send_cb(onDataSent);

    // Register peer
    esp_now_peer_info_t peerInfo;
    memcpy(peerInfo.peer_addr, broadcastAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;

    // Add peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        Serial.println("Failed to add peer");
        return;
    }
}

void loop() {
  //readMacAddress();
  // Read and send transformed values via ESP-NOW
  sendTransformedValues();
  
  delay(150); // Delay for demonstration purposes
}
