/*  AR4 Annin Robot Control Software Arduino Nano sketch - Modified with Sensors
    Copyright (c) 2024, Chris Annin
    All rights reserved.

    Modified to include VL53L1X ToF sensor and BNO055 IMU support
    
    Modifications:
    - Added VL53L1X distance sensor on I2C (A4/A5)
    - Added BNO055 IMU on I2C (A4/A5)
    - Only servo0 (A0) is used for gripper control
    - A4 and A5 are reserved for I2C communication
    - New commands: RD (read distance), RO (read orientation), RC (read calibration)

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.
          Selling AR2 software, robots, robot parts, or any versions of robots or software based on this
          work is strictly prohibited.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

    Log:
    v1.4 - Added VL53L1X and BNO055 sensor support
*/

#include <Servo.h>
#include <Wire.h>
#include <Adafruit_VL53L1X.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_Sensor.h>

String inData;

// Servo objects - only servo0 is used
Servo servo0;

// Sensor objects
Adafruit_VL53L1X vl53 = Adafruit_VL53L1X();
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x29);

// Sensor initialization flags
bool vl53Initialized = false;
bool bnoInitialized = false;

// I2C Addresses
#define VL53L1X_DEFAULT_ADDR 0x29
#define VL53L1X_NEW_ADDR 0x30

// Pin definitions
const int Input2 = 2;
const int Input3 = 3;
const int Input4 = 4;
const int Input5 = 5;
const int Input6 = 6;
const int Input7 = 7;

const int Output8 = 8;
const int Output9 = 9;
const int Output10 = 10;
const int Output11 = 11;
const int Output12 = 12;
const int Output13 = 13;

// Function to change VL53L1X I2C address
bool changeVL53L1XAddress(uint8_t new_address) {
  Wire.beginTransmission(VL53L1X_DEFAULT_ADDR);
  Wire.write(0x00);
  Wire.write(0x01);
  Wire.write(new_address << 1);
  return (Wire.endTransmission() == 0);
}

// Initialize sensors
void initializeSensors() {
  Wire.begin();
  
  // Initialize VL53L1X
  bool foundAt30 = vl53.begin(VL53L1X_NEW_ADDR, &Wire);
  
  if (!foundAt30) {
    if (vl53.begin(VL53L1X_DEFAULT_ADDR, &Wire)) {
      changeVL53L1XAddress(VL53L1X_NEW_ADDR);
      delay(50);
    }
  }
  
  if (vl53.startRanging()) {
    vl53.setTimingBudget(50);
    vl53Initialized = true;
  }
  
  // Initialize BNO055
  delay(100);
  if (bno.begin()) {
    delay(1000);
    bno.setExtCrystalUse(true);
    bnoInitialized = true;
  }
}

void setup() {
  Serial.begin(9600);

  // Setup pins - A4 and A5 are reserved for I2C
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  pinMode(Input2, INPUT_PULLUP);
  pinMode(Input3, INPUT_PULLUP);
  pinMode(Input4, INPUT_PULLUP);
  pinMode(Input5, INPUT_PULLUP);
  pinMode(Input6, INPUT_PULLUP);
  pinMode(Input7, INPUT_PULLUP);

  pinMode(Output8, OUTPUT);
  pinMode(Output9, OUTPUT);
  pinMode(Output10, OUTPUT);
  pinMode(Output11, OUTPUT);
  pinMode(Output12, OUTPUT);
  pinMode(Output13, OUTPUT);

  // Only attach servo0 for gripper
  // Attach with extended range: 500-2500 microseconds
  servo0.attach(A0, 500, 2500);
  // Initialize to position 0 (fully open)
  servo0.writeMicroseconds(500);  // 0 degrees = fully open

  // Initialize sensors
  initializeSensors();
}

void loop() {
  while (Serial.available() > 0) {
    char recieved = Serial.read();
    inData += recieved;
    
    if (recieved == '\n') {
      String function = inData.substring(0, 2);

      //-----COMMAND TO MOVE SERVO---------------------------------------------------
      // SV command accepts 0-360 degrees
      // Servo has 2:1 ratio: commanded 180° = actual 90° movement
      // So we need wider pulse range to get full movement
      if (function == "SV") {
        int SVstart = inData.indexOf('V');
        int POSstart = inData.indexOf('P');
        int servoNum = inData.substring(SVstart + 1, POSstart).toInt();
        int servoPOS = inData.substring(POSstart + 1).toInt();
        
        // Only servo0 is active
        if (servoNum == 0) {
          // Map 0-180 degrees to full pulse range (500-2500µs)
          // This compensates for 2:1 gear ratio
          int pulseWidth = map(servoPOS, 0, 180, 500, 2500);
          servo0.writeMicroseconds(pulseWidth);
        }
        Serial.print("Servo Done");
      }

      //-----COMMAND READ DISTANCE FROM VL53L1X--------------------------------------
      // Format: RD\n
      // Response: D<distance_in_mm>\n or ERROR\n
      if (function == "RD") {
        if (vl53Initialized && vl53.dataReady()) {
          int16_t distance = vl53.distance();
          Serial.print("D");
          Serial.println(distance);
          vl53.clearInterrupt();
        } else {
          Serial.println("ERROR");
        }
      }

      //-----COMMAND READ ORIENTATION FROM BNO055------------------------------------
      // Format: RO\n
      // Response: OX<x>Y<y>Z<z>\n or ERROR\n
      if (function == "RO") {
        if (bnoInitialized) {
          sensors_event_t orientationData;
          bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
          
          Serial.print("OX");
          Serial.print(orientationData.orientation.x, 2);
          Serial.print("Y");
          Serial.print(orientationData.orientation.y, 2);
          Serial.print("Z");
          Serial.println(orientationData.orientation.z, 2);
        } else {
          Serial.println("ERROR");
        }
      }

      //-----COMMAND READ CALIBRATION STATUS FROM BNO055-----------------------------
      // Format: RC\n
      // Response: CS<sys>G<gyro>A<accel>M<mag>\n or ERROR\n
      if (function == "RC") {
        if (bnoInitialized) {
          uint8_t system, gyro, accel, mag;
          bno.getCalibration(&system, &gyro, &accel, &mag);
          
          Serial.print("CS");
          Serial.print(system);
          Serial.print("G");
          Serial.print(gyro);
          Serial.print("A");
          Serial.print(accel);
          Serial.print("M");
          Serial.println(mag);
        } else {
          Serial.println("ERROR");
        }
      }

      //-----COMMAND IF INPUT THEN JUMP----------------------------------------------
      if (function == "JF") {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH) {
          Serial.println("T");
        }
        if (digitalRead(IJInputNum) == LOW) {
          Serial.println("F");
        }
      }

      //-----COMMAND SET OUTPUT ON---------------------------------------------------
      if (function == "ON") {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        Serial.print("Done");
      }

      //-----COMMAND SET OUTPUT OFF--------------------------------------------------
      if (function == "OF") {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        Serial.print("Done");
      }

      //-----COMMAND TO WAIT 5v INPUT------------------------------------------------
      if (function == "WI") {
        int inputVal = -1;
        int inputIndex = inData.indexOf('A');
        int valueIndex = inData.indexOf('B');
        int timoutIndex = inData.indexOf('C');
        int input = inData.substring(inputIndex + 1, valueIndex).toInt();
        int value = inData.substring(valueIndex + 1, timoutIndex).toInt();
        int timeout = inData.substring(timoutIndex + 1).toInt();
        unsigned long timeoutMillis = timeout * 1000;
        unsigned long startTime = millis();
        while ((millis() - startTime < timeoutMillis) && (inputVal != value)) {
          inputVal = digitalRead(input);
          delay(100);
        }
        delay(5);
        Serial.print("Done");
      }

      //-----COMMAND ECHO TEST MESSAGE-----------------------------------------------
      if (function == "TM") {
        String echo = inData.substring(2);
        Serial.println(echo);
      }

      else {
        inData = "";  // Clear received buffer
      }
    }
  }
}
