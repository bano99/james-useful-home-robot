# ST3020 Gripper Integration - TODO List

## Project Overview

Integrate ST3020 servo with Waveshare ESP32 Servo Driver board for gripper control, including:
- VL53L1X ToF distance sensor
- BNO055 IMU orientation sensor
- 5-level torque control
- Remote control integration (LilyGO T-Display S3 AMOLED)

**Note**: NO FSR force sensors - relying on servo load feedback and visual feedback only.

---

## Phase 1: PREPARE NOW (Before Hardware Arrives)

### 1.1 Firmware Preparation ✅ CAN DO NOW

**Location**: `platform/servo_driver/Servo-Driver-with-ESP32/ServoDriver/`

#### Task 1.1.1: Create Modified Firmware Branch
- [x] ~~Copy `ServoDriver/` to `ServoDriver_Gripper/` (working copy)~~ - Working directly on original
- [x] Document changes in README
- [x] Set up version control for modifications

#### Task 1.1.2: Add Sensor Support to Firmware ✅ COMPLETED
- [x] Add sensor library includes to `ServoDriver.ino`
- [x] Create `SENSOR_CTRL.h` header file
- [x] Implement `initSensors()` function
- [x] Implement sensor reading functions
- [x] Add sensor command handlers (RD, RO, RC, SS)
- [x] Test compilation (ready to test)

#### Task 1.1.3: Add 5-Level Torque Control ✅ COMPLETED
- [x] Create `TORQUE_CTRL.h` header file
- [x] Implement torque level mapping (1-5 → 200-1000)
- [x] Add torque command handlers (TL, GT, TI, T+, T-, GL)
- [x] Integrate with existing servo control
- [x] Test compilation (ready to test)

#### Task 1.1.4: Test on Available ESP32 ⏳ READY TO TEST
- [ ] Upload modified firmware to spare ESP32
- [ ] Test USB serial communication
- [ ] Test sensor commands (will return ERROR without sensors - OK)
- [ ] Test torque commands (will work without servo - OK)
- [ ] Verify no compilation errors
- [ ] Verify no runtime crashes

**Deliverable**: Modified firmware ready to test with ST3020 when it arrives

---

### 1.2 Remote Control Integration ✅ CAN DO NOW

**Location**: `platform/controller/` (existing remote control code)

#### Task 1.2.1: Analyze Existing Remote Control
- [ ] Locate current remote control sketch
- [ ] Document current ESPNOW message structure
- [ ] Identify available buttons/inputs for gripper control
- [ ] Check display space for gripper status

#### Task 1.2.2: Extend Remote Control Firmware
- [ ] Add gripper control variables (position, torque level)
- [ ] Add button mapping for gripper open/close
- [ ] Add button mapping for torque level adjustment
- [ ] Update ESPNOW message structure to include gripper commands
- [ ] Add gripper status to AMOLED display
- [ ] Test compilation

#### Task 1.2.3: Update Platform Controller to Receive Gripper Commands
- [ ] Modify platform controller ESPNOW handler
- [ ] Add gripper command forwarding to servo driver ESP32
- [ ] Add serial communication to servo driver
- [ ] Test compilation

**Deliverable**: Remote control ready to send gripper commands

---

### 1.3 Communication Protocol Design ✅ CAN DO NOW

#### Task 1.3.1: Define Command Protocol
- [ ] Document all commands (servo, sensors, torque)
- [ ] Define message format for ESPNOW (remote → platform → gripper)
- [ ] Define message format for serial (platform → gripper)
- [ ] Define response format (gripper → platform → remote)
- [ ] Create protocol specification document

#### Task 1.3.2: Create Test Scripts ✅ COMPLETED
- [x] Python script to test USB serial commands
- [x] Python script to simulate Jetson communication
- [x] Test command parsing without hardware

**Deliverable**: Complete protocol documentation and test tools

---

### 1.4 Documentation ✅ CAN DO NOW

#### Task 1.4.1: Create Integration Guide
- [ ] Hardware connection diagram
- [ ] Pin assignment table
- [ ] I2C address map
- [ ] Power supply requirements
- [ ] Wiring checklist

#### Task 1.4.2: Create Testing Procedures
- [ ] Sensor testing procedure
- [ ] Servo testing procedure
- [ ] Torque calibration procedure
- [ ] Remote control testing procedure
- [ ] Integration testing checklist

**Deliverable**: Complete documentation for hardware integration

---

## Phase 2: HARDWARE TESTING (When Hardware Arrives)

### 2.1 Initial Hardware Setup

#### Task 2.1.1: Waveshare Board Setup
- [ ] Install CH340 driver (if Windows)
- [ ] Connect board via USB
- [ ] Upload test sketch (blink)
- [ ] Verify programming works
- [ ] Test USB serial communication

#### Task 2.1.2: Power Supply Setup
- [ ] Connect 12V power supply for servo
- [ ] Connect 5V power for ESP32 logic
- [ ] Verify voltage levels
- [ ] Test under load (servo moving)

---

### 2.2 ST3020 Servo Testing

#### Task 2.2.1: Basic Servo Connection
- [ ] Connect ST3020 to GPIO 18/19 (UART)
- [ ] Connect 12V power to servo
- [ ] Upload Waveshare original firmware
- [ ] Test basic servo commands

#### Task 2.2.2: Verify ST3020 Compatibility
- [ ] Test position control: `st.WritePosEx(1, 500, 1000, 0)`
- [ ] Test position feedback: `st.ReadPos(1)`
- [ ] Test load feedback: `st.ReadLoad(1)`
- [ ] Test torque enable/disable: `st.EnableTorque(1, 0/1)`
- [ ] Document any protocol differences

#### Task 2.2.3: Upload Modified Firmware
- [ ] Upload `ServoDriver_Gripper` firmware
- [ ] Test all servo commands still work
- [ ] Test new torque level commands
- [ ] Verify no regressions

---

### 2.3 Sensor Integration Testing

#### Task 2.3.1: VL53L1X ToF Sensor
- [ ] Connect VL53L1X to I2C bus (GPIO 21/22)
- [ ] Test sensor detection (I2C scan)
- [ ] Test address change (0x29 → 0x30)
- [ ] Test distance readings
- [ ] Test RD command via serial

#### Task 2.3.2: BNO055 IMU Sensor
- [ ] Connect BNO055 to I2C bus (GPIO 21/22)
- [ ] Test sensor detection (I2C scan)
- [ ] Verify address 0x29 (no conflict with VL53L1X at 0x30)
- [ ] Test orientation readings
- [ ] Test calibration status
- [ ] Test RO and RC commands via serial

#### Task 2.3.3: Multi-Sensor I2C Bus Test
- [ ] Connect both sensors simultaneously
- [ ] I2C scan to verify all devices present
- [ ] Test VL53L1X readings
- [ ] Test BNO055 readings
- [ ] Test concurrent sensor updates
- [ ] Verify no I2C bus conflicts

---

### 2.4 Torque Control Calibration

#### Task 2.4.1: Torque Level Testing
- [ ] Test Level 1 (200) - very light grip
- [ ] Test Level 2 (400) - light grip
- [ ] Test Level 3 (600) - medium grip
- [ ] Test Level 4 (800) - firm grip
- [ ] Test Level 5 (1000) - maximum grip

#### Task 2.4.2: Grip Force Measurement
- [ ] Test with delicate objects (paper, plastic cup)
- [ ] Test with medium objects (bottle, book)
- [ ] Test with heavy objects (tools, metal parts)
- [ ] Adjust torque levels if needed
- [ ] Document recommended levels per object type

#### Task 2.4.3: Load Feedback Calibration
- [ ] Measure load values at each torque level
- [ ] Correlate load readings with actual grip force
- [ ] Define thresholds for grip success detection
- [ ] Implement grip verification logic

---

### 2.5 Remote Control Integration Testing

#### Task 2.5.1: Remote to Platform Communication
- [ ] Test ESPNOW connection
- [ ] Test gripper command transmission
- [ ] Verify message reception on platform controller
- [ ] Test display update on remote

#### Task 2.5.2: Platform to Gripper Communication
- [ ] Connect platform controller to gripper ESP32 (serial)
- [ ] Test command forwarding
- [ ] Test response reception
- [ ] Verify end-to-end communication

#### Task 2.5.3: Complete Remote Control Test
- [ ] Test gripper open/close from remote
- [ ] Test torque level adjustment from remote
- [ ] Test status display on remote screen
- [ ] Test manual override priority

---

### 2.6 System Integration Testing

#### Task 2.6.1: Mechanical Integration
- [ ] Mount ST3020 servo to gripper mechanism
- [ ] Mount sensors on gripper (VL53L1X, BNO055)
- [ ] Route cables cleanly
- [ ] Secure ESP32 board
- [ ] Test mechanical range of motion

#### Task 2.6.2: Functional Testing
- [ ] Test complete grip cycle (open → approach → close → lift)
- [ ] Test distance sensor during approach
- [ ] Test orientation sensor during manipulation
- [ ] Test torque control during grip
- [ ] Test load feedback for grip verification

#### Task 2.6.3: Jetson Integration
- [ ] Connect gripper ESP32 to Jetson Nano 2 (USB)
- [ ] Test serial communication from Jetson
- [ ] Create Python control library
- [ ] Test ROS2 integration (if applicable)
- [ ] Test autonomous gripper control

---

## Phase 3: OPTIMIZATION & DOCUMENTATION

### 3.1 Performance Optimization
- [ ] Optimize sensor update rate
- [ ] Optimize I2C bus speed
- [ ] Reduce communication latency
- [ ] Implement command queuing if needed

### 3.2 Safety Features
- [ ] Implement grip timeout (prevent overheating)
- [ ] Implement overload detection
- [ ] Implement emergency release command
- [ ] Test safety features

### 3.3 Final Documentation
- [ ] Update wiring diagrams with actual setup
- [ ] Document final torque calibration values
- [ ] Create user manual for gripper control
- [ ] Document troubleshooting procedures
- [ ] Create video demonstration

---

## File Structure

```
platform/
├── servo_driver/
│   ├── Servo-Driver-with-ESP32/          # Original Waveshare firmware
│   └── ServoDriver_Gripper/              # Modified firmware (NEW)
│       ├── ServoDriver_Gripper.ino       # Main sketch
│       ├── STSCTRL.h                     # Servo control (existing)
│       ├── SENSOR_CTRL.h                 # Sensor control (NEW)
│       ├── TORQUE_CTRL.h                 # Torque control (NEW)
│       ├── CONNECT.h                     # Communication (existing)
│       ├── BOARD_DEV.h                   # Board peripherals (existing)
│       ├── WEBPAGE.h                     # Web interface (existing)
│       ├── RGB_CTRL.h                    # RGB LED (existing)
│       └── README.md                     # Documentation (NEW)
├── controller/
│   └── [remote_control_sketch]/          # Existing remote control
│       └── [modifications for gripper]   # Extended for gripper (NEW)
└── gripper/
    ├── docs/
    │   ├── ST3020_gripper_integration_analysis.md  # Analysis (DONE)
    │   ├── ST3020_gripper_integration_TODO.md      # This file
    │   ├── gripper_wiring_diagram.md               # Wiring (TODO)
    │   ├── gripper_command_protocol.md             # Protocol (TODO)
    │   └── gripper_testing_procedures.md           # Testing (TODO)
    └── test_scripts/
        ├── test_serial_commands.py                 # Serial test (TODO)
        └── test_gripper_control.py                 # Control test (TODO)
```

---

## Priority Order for "DO NOW" Tasks

### HIGH PRIORITY (Start Immediately)

1. **Task 1.1.2**: Add sensor support to firmware
2. **Task 1.1.3**: Add 5-level torque control
3. **Task 1.1.4**: Test on available ESP32
4. **Task 1.3.1**: Define command protocol

### MEDIUM PRIORITY (After firmware basics)

5. **Task 1.2.1**: Analyze existing remote control
6. **Task 1.2.2**: Extend remote control firmware
7. **Task 1.3.2**: Create test scripts

### LOW PRIORITY (Nice to have before hardware)

8. **Task 1.4.1**: Create integration guide
9. **Task 1.4.2**: Create testing procedures

---

## Estimated Timeline

### Before Hardware Arrives (Can Do Now)
- **Week 1**: Firmware modifications (Tasks 1.1.x) - 3-4 days
- **Week 2**: Remote control integration (Tasks 1.2.x) - 2-3 days
- **Week 3**: Documentation and testing (Tasks 1.3.x, 1.4.x) - 2-3 days

**Total: 7-10 days of preparation work**

### After Hardware Arrives
- **Day 1**: Hardware setup and basic testing (Tasks 2.1.x, 2.2.1)
- **Day 2**: ST3020 compatibility verification (Tasks 2.2.2, 2.2.3)
- **Day 3**: Sensor integration (Tasks 2.3.x)
- **Day 4**: Torque calibration (Tasks 2.4.x)
- **Day 5**: Remote control testing (Tasks 2.5.x)
- **Day 6-7**: System integration (Tasks 2.6.x)
- **Day 8**: Optimization and final documentation (Phase 3)

**Total: 8 days after hardware arrives**

---

## Success Criteria

### Phase 1 (Preparation)
- ✅ Modified firmware compiles without errors
- ✅ Firmware runs on spare ESP32 without crashes
- ✅ Remote control firmware compiles and runs
- ✅ Protocol documentation complete
- ✅ Test scripts ready

### Phase 2 (Hardware Testing)
- ✅ ST3020 servo responds to commands
- ✅ Both sensors work on shared I2C bus
- ✅ 5 torque levels calibrated and tested
- ✅ Remote control can operate gripper
- ✅ Jetson can control gripper via serial

### Phase 3 (Integration)
- ✅ Complete grip cycle works reliably
- ✅ Sensor feedback integrated with control
- ✅ Safety features functional
- ✅ Documentation complete
- ✅ System ready for robot arm integration

---

## Notes

- **No FSR sensors**: Relying on servo load feedback and visual feedback only
- **Spare ESP32**: Use for firmware testing before hardware arrives
- **Remote control**: Extend existing LilyGO T-Display S3 AMOLED remote
- **Communication chain**: Remote → Platform Controller → Gripper ESP32
- **I2C sharing**: VL53L1X (0x30) + BNO055 (0x29) + OLED (0x3C) - no conflicts

---

## Questions to Resolve

- [ ] Which GPIO pins are used on platform controller for serial to gripper?
- [ ] What is the exact remote control sketch location?
- [ ] What buttons/inputs are available on remote for gripper control?
- [ ] Should gripper status be shown on platform controller AMOLED or remote AMOLED?
- [ ] What is the preferred communication method: USB serial or UART pins?

---

**Last Updated**: December 6, 2025
**Status**: Ready to start Phase 1 tasks
**Next Action**: Begin Task 1.1.2 - Add sensor support to firmware
