#include <Wire.h> 
#define I2C_SDA 3 // Define the SDA pin 
#define I2C_SCL 2 // Define the SCL pin

#include <esp_now.h>
#include <WiFi.h>
#include "esp_timer.h"

#include <DFRobot_IICSerial.h>
#include <DNSServer.h>
#include <DFRobot_IICSerial.h>

DFRobot_IICSerial iicSerial1(Wire, /*subUartChannel =*/SUBUART_CHANNEL_1,/*IA1 = */1,/*IA0 = */1);//Construct UART1
DFRobot_IICSerial iicSerial2(Wire, /*subUartChannel =*/SUBUART_CHANNEL_2, /*IA1 = */1,/*IA0 = */1);//Construct UART2


 
 
struct JoystickValues {
  float direction;    // 0-360 degrees
  int velocity;       // 0-255
  int rotationalVelocity; // -127 to 127
};

struct MotorCommand {
  int state;
  float velocity;
};


// Define the structure for receiving data
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

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA, I2C_SCL); // Initialize I2C communication with the defined SDA and SCL pins
  
  // Initialize ESP-NOW
  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }
  esp_now_register_recv_cb(onDataReceive);

   // Configure the timer
  esp_timer_create_args_t timer_args;
  timer_args.callback = &onWatchdogTimeout;
  timer_args.name = "watchdog_timer";
  
  esp_timer_create(&timer_args, &watchdog_timer);


  delay(1000);

  iicSerial1.begin(115200, IICSerial_8N1);/*UART1 init*/
  iicSerial2.begin(115200, IICSerial_8N1);/*UART2 init*/
  
  delay(1000);

  motorFL_state = -1;
  motorFR_state = -1;
  motorBL_state = -1;
  motorBR_state = -1;


}


void loop() {

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
  // Set motor states based on velocity
  motorFL.state = motorFL.velocity != 0 ? 8 : 1; // State 8 for moving, 1 for stopped
  motorFR.state = motorFR.velocity != 0 ? 8 : 1;
  motorBL.state = motorBL.velocity != 0 ? 8 : 1;
  motorBR.state = motorBR.velocity != 0 ? 8 : 1;

  // Hold position if velocity is 0
  if (motorFL.velocity == 0) motorFL.state = 8;
  if (motorFR.velocity == 0) motorFR.state = 8;
  if (motorBL.velocity == 0) motorBL.state = 8;
  if (motorBR.velocity == 0) motorBR.state = 8;

//Serial.println("FL:" + String(motorFL.velocity) + " FR:" + String(motorFR.velocity) + " BL:" + String(motorBL.velocity) + "BR:" + String(motorBR.velocity));
  sendMotorCommands(motorFL, motorFR, motorBL, motorBR);

}
 





  
// Correct signature for the onDataReceive callback
void onDataReceive(const esp_now_recv_info *info, const uint8_t *incomingData, int len) {
  memcpy(&joystickData, incomingData, sizeof(joystickData));

//Serial.println("Data received: x" + String(joystickData.x) + " y"+ String(joystickData.y) + " rot" + String(joystickData.rot));
/*
  
  

  // Reset the watchdog timer
  esp_timer_stop(watchdog_timer);
  esp_timer_start_once(watchdog_timer, timeout_us);

*/
  // Commit back to the remote control to acknowledge receipt
  esp_now_send(info->des_addr, (uint8_t *)"ACK", sizeof("ACK"));

  // Calculate movement values based on the received joystick data
  joystickValues = calculateMovement(joystickData.x, joystickData.y, joystickData.rot);

  // Control the mecanum wheels based on the calculated values
  controlMecanumWheels(joystickValues, motorFL, motorFR, motorBL, motorBR);

  
}



void sendMotorCommands(MotorCommand motorFL, MotorCommand motorFR, MotorCommand motorBL, MotorCommand motorBR) {

if(motorFL_state !=motorFL.state){
  motorFL_state =motorFL.state;
  String state_commandFL = "w axis0.requested_state " + String(motorFL.state); // Front left motor (0)
  iicSerial2.println(state_commandFL); // Send to left motors
}
if(motorFR_state !=motorFR.state){
  motorFR_state =motorFR.state;
  String state_commandFR = "w axis1.requested_state " + String(motorFR.state); // Front right motor (1)
  iicSerial1.println(state_commandFR); // Send to left motors
}
if(motorBL_state !=motorBL.state){
  motorBL_state =motorBL.state;
  String state_commandBL = "w axis1.requested_state " + String(motorBL.state); // Front left motor (0)
  iicSerial2.println(state_commandBL); // Send to left motors
}
if(motorBR_state !=motorBR.state){
  motorBR_state =motorBR.state;
  String state_commandBR = "w axis0.requested_state " + String(motorBR.state); // Front left motor (0)
  iicSerial1.println(state_commandBR); // Send to left motors
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


