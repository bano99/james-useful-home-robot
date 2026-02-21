/*  Sensor Test Sketch for VL53L1X and GY-BNO055
    Tests the ToF distance sensor and IMU connected to Arduino Nano
    
    Hardware Setup:
    - VL53L1X ToF Distance Sensor: I2C (A4=SDA, A5=SCL)
    - GY-BNO055 IMU: I2C (A4=SDA, A5=SCL)
    - Both sensors share the I2C bus
    
    I2C Address Conflict Resolution:
    - VL53L1X initializes first at 0x29
    - Address changed to 0x30 using direct I2C write
    - Then BNO055 initializes (can use 0x28 or 0x29)
    
    Required Libraries:
    - Adafruit_VL53L1X
    - Adafruit_BNO055
    - Adafruit_Sensor
    
    Install via Arduino Library Manager
*/

#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

// I2C Addresses
#define VL53L1X_DEFAULT_ADDR 0x29
#define VL53L1X_NEW_ADDR 0x30

// Create sensor objects
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);  // Use 0x29 address

// Function to change VL53L1X I2C address using direct register write
bool changeVL53L1XAddress(uint8_t new_address) {
  Wire.beginTransmission(VL53L1X_DEFAULT_ADDR);
  Wire.write(0x00);  // Register for I2C address
  Wire.write(0x01);  // Sub-register
  Wire.write(new_address << 1);  // New address (7-bit shifted to 8-bit)
  return (Wire.endTransmission() == 0);
}

// Function to scan I2C bus for devices
void scanI2C() {
  byte error, address;
  int nDevices = 0;
  
  Serial.println("Scanning I2C bus...");
  for(address = 1; address < 127; address++) {
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
    
    if (error == 0) {
      Serial.print("Device found at address 0x");
      if (address < 16) Serial.print("0");
      Serial.print(address, HEX);
      Serial.println();
      nDevices++;
    }
  }
  
  if (nDevices == 0) {
    Serial.println("No I2C devices found!");
  } else {
    Serial.print("Found ");
    Serial.print(nDevices);
    Serial.println(" device(s)");
  }
  Serial.println();
}

// Timing variables
unsigned long lastPrintTime = 0;
const unsigned long printInterval = 500; // Print every 500ms

void setup() {
  Serial.begin(9600);
  while (!Serial) delay(10); // Wait for serial port
  
  Serial.println("=================================");
  Serial.println("VL53L1X + BNO055 Sensor Test");
  Serial.println("=================================");
  Serial.println();
  
  // Initialize I2C
  Wire.begin();
  
  // Check if VL53L1X is already at 0x30 (from previous run/reset)
  Serial.println("Checking for VL53L1X...");
  bool foundAt30 = vl53.begin(VL53L1X_NEW_ADDR, &Wire);
  
  if (foundAt30) {
    Serial.print("VL53L1X already at address 0x");
    Serial.print(VL53L1X_NEW_ADDR, HEX);
    Serial.println(" (from previous run)");
  } else {
    // Try default address 0x29
    Serial.println("Trying default address 0x29...");
    if (!vl53.begin(VL53L1X_DEFAULT_ADDR, &Wire)) {
      Serial.println("ERROR: Failed to find VL53L1X sensor!");
      Serial.println("Check wiring (SDA=A4, SCL=A5, VCC, GND)");
      Serial.println();
      Serial.println("Scanning I2C bus...");
      scanI2C();
      while (1) delay(10);
    }
    Serial.println("VL53L1X found at 0x29!");
    
    // Change VL53L1X address to free up 0x29 for BNO055
    Serial.print("Changing address to 0x");
    Serial.print(VL53L1X_NEW_ADDR, HEX);
    Serial.println("...");
    
    if (changeVL53L1XAddress(VL53L1X_NEW_ADDR)) {
      Serial.println("Address change command sent successfully!");
    } else {
      Serial.println("WARNING: Address change command may have failed");
    }
    delay(50);  // Give sensor time to respond at new address
  }
  
  // Configure VL53L1X
  Serial.println("Configuring VL53L1X...");
  if (!vl53.startRanging()) {
    Serial.println("ERROR: Couldn't start VL53L1X ranging!");
    while (1) delay(10);
  }
  
  // Set timing budget (higher = more accurate but slower)
  vl53.setTimingBudget(50); // 50ms timing budget
  Serial.println("VL53L1X configured and ready");
  Serial.println();
  
  // Now initialize BNO055 at 0x29
  Serial.println("Initializing BNO055 IMU at address 0x29...");
  
  // BNO055 needs time to boot up
  delay(100);
  
  if (!bno.begin()) {
    Serial.println("ERROR: Failed to find BNO055 sensor!");
    Serial.println("Check wiring and I2C address");
    Serial.println();
    Serial.println("Scanning I2C bus for devices...");
    scanI2C();
    while (1) delay(10);
  }
  Serial.println("BNO055 found at 0x29!");
  
  delay(1000); // Give BNO055 time to initialize
  bno.setExtCrystalUse(true);
  Serial.println("BNO055 configured");
  Serial.println();
  
  // Display sensor details
  displaySensorDetails();
  displayCalibrationStatus();
  
  Serial.println();
  Serial.println("Starting continuous sensor readings...");
  Serial.println("=================================");
  Serial.println();
}

void loop() {
  unsigned long currentTime = millis();
  
  // Print sensor data at regular intervals
  if (currentTime - lastPrintTime >= printInterval) {
    lastPrintTime = currentTime;
    
    // Read VL53L1X distance
    if (vl53.dataReady()) {
      int16_t distance = vl53.distance();
      
      Serial.print("Distance: ");
      if (distance == -1) {
        Serial.print("Out of range");
      } else {
        Serial.print(distance);
        Serial.print(" mm");
      }
      Serial.print(" | ");
      
      vl53.clearInterrupt();
    }
    
    // Read BNO055 orientation
    sensors_event_t orientationData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    
    Serial.print("Orientation - X: ");
    Serial.print(orientationData.orientation.x, 2);
    Serial.print("° Y: ");
    Serial.print(orientationData.orientation.y, 2);
    Serial.print("° Z: ");
    Serial.print(orientationData.orientation.z, 2);
    Serial.print("°");
    
    // Get calibration status
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);
    Serial.print(" | Cal: S:");
    Serial.print(system);
    Serial.print(" G:");
    Serial.print(gyro);
    Serial.print(" A:");
    Serial.print(accel);
    Serial.print(" M:");
    Serial.println(mag);
  }
}

void displaySensorDetails() {
  Serial.println("--- BNO055 Sensor Details ---");
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.print("Sensor:       "); Serial.println(sensor.name);
  Serial.print("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println();
}

void displayCalibrationStatus() {
  Serial.println("--- BNO055 Calibration Status ---");
  Serial.println("Calibration values: 0=uncalibrated, 3=fully calibrated");
  uint8_t system, gyro, accel, mag;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("System: "); Serial.print(system);
  Serial.print(" Gyro: "); Serial.print(gyro);
  Serial.print(" Accel: "); Serial.print(accel);
  Serial.print(" Mag: "); Serial.println(mag);
  Serial.println("Note: Move the sensor in a figure-8 pattern to calibrate magnetometer");
  Serial.println();
}
