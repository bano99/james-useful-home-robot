# Arm Cartesian Control Implementation Tasks

## Overview
This document contains the step-by-step implementation tasks for adding manual Cartesian control of the AR4-MK3 robot arm via the remote control's left joystick. The control flow is:

```
Remote Control (Left Joystick) 
  → ESP-NOW → Platform Controller 
  → USB Serial → Jetson Orin Nano (ROS2 Nodes) 
  → Inverse Kinematics 
  → USB Serial → Teensy 4.1 
  → AR4-MK3 Arm
```

## Pin Assignments (from remote_control_v1)
- **Left Joystick Up/Down**: Pin 1
- **Left Joystick Left/Right**: Pin 11
- **Left Joystick Rotation**: Pin 12
- **Left Switch**: Pin 45 (controls right joystick mode)
- **Right Joystick Up/Down**: Pin 13
- **Right Joystick Left/Right**: Pin 14
- **Right Joystick Rotation**: Pin 15

## Control Modes
- **Left Joystick**: Always controls robot arm (Cartesian X/Y/Z)
- **Right Joystick**: 
  - When switch ON: Controls platform (mecanum drive)
  - When switch OFF: Controls arm vertical (Z) and rotation
- **Right joystick data**: Forwarded to Jetson when switch is set (for potential autonomous override)

---

## Task List

### **Task 1: Update Remote Control Firmware**
**File**: `platform/remote/remote_control_v2_amoled/remote_control_v2_amoled.ino`

**Objectives**:
- Add left joystick reading (pins 1, 11, 12)
- Add left switch reading (pin 45)
- Update ESP-NOW data structure to include left joystick and switch state
- Apply dead zone filtering to left joystick
- Update AMOLED display to show left joystick values

**Acceptance Criteria**:
- Left joystick values are read correctly from pins 1, 11, 12
- Switch state is read from pin 45
- Dead zone filtering (±10-15) prevents drift
- ESP-NOW packet includes all joystick data and switch state
- Display shows left joystick values in real-time
- Transmission rate remains at ~6.7 Hz (150ms interval)

**Technical Details**:
```cpp
// Updated data structure
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
  
  // Gripper (existing)
  int gripper_pot;  // Pin 16
} RemoteControlData;
```

---

### **Task 2: Update Platform Controller Firmware**
**File**: `platform/controller/plattform_controller/plattform_controller.ino`

**Objectives**:
- Update ESP-NOW receive structure to match new remote control data
- Implement USB Serial communication to Jetson
- Forward left joystick data to Jetson as JSON
- Forward right joystick data to Jetson when switch is set
- Implement bidirectional serial protocol (send commands, receive status)
- Update AMOLED display to show arm control status

**Acceptance Criteria**:
- ESP-NOW receives all joystick data and switch state
- Serial communication established with Jetson at 115200 baud
- JSON messages sent to Jetson at control rate (~50 Hz)
- Status messages received from Jetson and displayed
- Display shows arm control mode and connection status
- Platform control continues to work normally

**Technical Details**:
```cpp
// Serial protocol - Platform Controller → Jetson
{
  "type": "manual_control",
  "left_x": -127 to 127,
  "left_y": -127 to 127,
  "left_z": -127 to 127,
  "right_x": -127 to 127,      // Only if switch is set
  "right_y": -127 to 127,      // Only if switch is set
  "right_rot": -127 to 127,    // Only if switch is set
  "switch_mode": "platform" or "vertical",
  "timestamp": millis()
}

// Serial protocol - Jetson → Platform Controller
{
  "type": "arm_status",
  "ik_success": true/false,
  "current_pose": [x, y, z, rx, ry, rz],
  "timestamp": millis()
}
```

---

### **Task 3: Create ROS2 Package Structure**
**Package**: `james_manipulation`

**Objectives**:
- Create ROS2 package for arm manipulation
- Set up directory structure for nodes, config, and launch files
- Create package.xml and setup.py with dependencies
- Add to workspace and verify build

**Acceptance Criteria**:
- Package `james_manipulation` created in `ros2_ws/src/`
- Directory structure includes: nodes/, config/, launch/, msg/
- Dependencies listed: rclpy, geometry_msgs, sensor_msgs, moveit_msgs, tf2_ros
- Package builds successfully with `colcon build`

**Directory Structure**:
```
ros2_ws/src/james_manipulation/
├── james_manipulation/
│   ├── __init__.py
│   ├── platform_serial_bridge.py
│   ├── arm_cartesian_controller.py
│   └── teensy_serial_bridge.py
├── config/
│   └── arm_cartesian_params.yaml
├── launch/
│   └── arm_cartesian_control.launch.py
├── package.xml
└── setup.py
```

---

### **Task 4: Implement Platform Serial Bridge Node**
**File**: `ros2_ws/src/james_manipulation/james_manipulation/platform_serial_bridge.py`

**Objectives**:
- Create ROS2 node to interface with Platform Controller via USB Serial
- Parse incoming JSON messages from Platform Controller
- Publish arm control commands to ROS2 topic
- Subscribe to arm status and forward to Platform Controller
- Handle serial connection errors and reconnection

**Acceptance Criteria**:
- Serial port opens successfully (/dev/ttyUSB0 at 115200 baud)
- JSON messages parsed correctly
- Publishes to `/arm/manual_cartesian_cmd` topic
- Subscribes to `/arm/status` topic
- Handles disconnection and reconnection gracefully
- Logs errors appropriately

**ROS2 Topics**:
- Publish: `/arm/manual_cartesian_cmd` (std_msgs/String - JSON)
- Subscribe: `/arm/status` (std_msgs/String - JSON)

---

### **Task 5: Implement Arm Cartesian Controller Node**
**File**: `ros2_ws/src/james_manipulation/james_manipulation/arm_cartesian_controller.py`

**Objectives**:
- Create ROS2 node for Cartesian arm control
- Subscribe to manual control commands from Platform Serial Bridge
- Get current end-effector pose from TF tree
- Calculate target Cartesian pose based on joystick input
- Call MoveIt2 IK service to compute joint angles
- Publish joint commands to Teensy Serial Bridge
- Publish status back to Platform Serial Bridge
- Implement velocity scaling and control rate parameters

**Acceptance Criteria**:
- Subscribes to `/arm/manual_cartesian_cmd` successfully
- TF lookup for end-effector pose works correctly
- Target pose calculated based on joystick input and mode
- IK service calls succeed with <50ms timeout
- Joint commands published to `/arm/joint_commands`
- Status published to `/arm/status`
- Control loop runs at 50 Hz
- Velocity scaling parameter tunable (default: 0.001 m/s per joystick unit)

**ROS2 Topics**:
- Subscribe: `/arm/manual_cartesian_cmd` (std_msgs/String)
- Subscribe: `/joint_states` (sensor_msgs/JointState)
- Publish: `/arm/joint_commands` (sensor_msgs/JointState)
- Publish: `/arm/status` (std_msgs/String)

**ROS2 Services**:
- Client: `/compute_ik` (moveit_msgs/srv/GetPositionIK)

**Parameters**:
- `velocity_scale`: 0.001 (m/s per joystick unit)
- `rotation_scale`: 0.01 (rad/s per joystick unit)
- `control_rate`: 50.0 (Hz)
- `command_timeout`: 0.5 (seconds)

---

### **Task 6: Implement Teensy Serial Bridge Node**
**File**: `ros2_ws/src/james_manipulation/james_manipulation/teensy_serial_bridge.py`

**Objectives**:
- Create ROS2 node to interface with Teensy 4.1 via USB Serial
- Subscribe to joint commands from Cartesian Controller
- Format commands for AR4 Teensy protocol
- Send commands to Teensy via serial
- Read joint state feedback from Teensy
- Publish joint states to ROS2

**Acceptance Criteria**:
- Serial port opens successfully (/dev/ttyACM0 at 115200 baud)
- Joint commands formatted correctly for AR4 protocol
- Commands sent to Teensy successfully
- Joint state feedback parsed correctly
- Publishes to `/joint_states` at 50 Hz
- Handles serial errors gracefully

**ROS2 Topics**:
- Subscribe: `/arm/joint_commands` (sensor_msgs/JointState)
- Publish: `/joint_states` (sensor_msgs/JointState)

**AR4 Protocol Example**:
```
Command: MJ J1:45.5 J2:30.2 J3:60.1 J4:0.0 J5:45.0 J6:0.0\n
Response: JS J1:45.5 J2:30.2 J3:60.1 J4:0.0 J5:45.0 J6:0.0\n
```

---

### **Task 7: Create Configuration File**
**File**: `ros2_ws/src/james_manipulation/config/arm_cartesian_params.yaml`

**Objectives**:
- Create YAML configuration file for all parameters
- Document each parameter with comments
- Set safe default values for initial testing

**Acceptance Criteria**:
- All node parameters defined in YAML
- Comments explain each parameter
- Default values are conservative and safe
- File loaded correctly by launch file

**Parameters**:
```yaml
platform_serial_bridge:
  serial_port: "/dev/ttyUSB0"
  baud_rate: 115200
  timeout: 0.1

teensy_serial_bridge:
  serial_port: "/dev/ttyACM0"
  baud_rate: 115200
  timeout: 0.1

arm_cartesian_controller:
  velocity_scale: 0.001      # m/s per joystick unit (1mm/s)
  rotation_scale: 0.01       # rad/s per joystick unit
  control_rate: 50.0         # Hz
  command_timeout: 0.5       # seconds
  ik_timeout: 0.05          # seconds (50ms)
  workspace_limits:
    x_min: 0.2
    x_max: 0.6
    y_min: -0.3
    y_max: 0.3
    z_min: 0.1
    z_max: 0.8
```

---

### **Task 8: Create Launch File**
**File**: `ros2_ws/src/james_manipulation/launch/arm_cartesian_control.launch.py`

**Objectives**:
- Create launch file to start all nodes
- Load parameters from YAML config
- Set up proper node dependencies
- Add remapping if needed

**Acceptance Criteria**:
- All three nodes launch successfully
- Parameters loaded from config file
- Nodes start in correct order
- Launch file can be called from command line
- Proper error handling if nodes fail

---

### **Task 9: Component Testing - Remote Control**
**Objectives**:
- Test left joystick reading on all three axes
- Test switch reading
- Verify ESP-NOW transmission with new data structure
- Test display updates with left joystick values
- Verify dead zone filtering

**Acceptance Criteria**:
- Left joystick values read correctly and displayed
- Switch state toggles correctly
- ESP-NOW packets received by Platform Controller
- Dead zone prevents drift (values stay at 0 when centered)
- Display updates at 10 Hz

**Test Procedure**:
1. Upload firmware to remote control
2. Open serial monitor
3. Move left joystick and verify values
4. Toggle switch and verify state change
5. Check display shows correct values
6. Center joystick and verify values return to 0

---

### **Task 10: Component Testing - Platform Controller**
**Objectives**:
- Test ESP-NOW reception of new data structure
- Test USB Serial communication with Jetson
- Verify JSON message formatting
- Test bidirectional communication
- Verify display updates

**Acceptance Criteria**:
- ESP-NOW receives all data correctly
- Serial port opens and communicates with Jetson
- JSON messages formatted correctly
- Status messages received and parsed
- Display shows arm control status
- Platform control still works normally

**Test Procedure**:
1. Upload firmware to Platform Controller
2. Connect USB to computer (simulating Jetson)
3. Use serial monitor to verify JSON output
4. Send test status messages and verify reception
5. Test with remote control sending data
6. Verify display updates correctly

---

### **Task 11: Component Testing - ROS2 Nodes**
**Objectives**:
- Test each ROS2 node independently
- Verify topic communication
- Test IK service calls
- Verify serial communication
- Test parameter loading

**Acceptance Criteria**:
- Platform Serial Bridge receives and publishes data
- Cartesian Controller computes IK successfully
- Teensy Serial Bridge formats commands correctly
- All topics publish at expected rates
- Parameters load from config file

**Test Procedure**:
1. Launch each node individually
2. Use `ros2 topic echo` to verify publications
3. Use `ros2 topic pub` to send test data
4. Verify IK service with test poses
5. Check logs for errors
6. Verify parameter values with `ros2 param list`

---

### **Task 12: Integration Testing - End-to-End Data Flow**
**Objectives**:
- Test complete data flow from joystick to Teensy
- Verify latency is acceptable (<100ms)
- Test mode switching
- Verify coordinate transformations
- Test with MoveIt2 in simulation

**Acceptance Criteria**:
- Joystick input reaches Teensy successfully
- End-to-end latency < 100ms
- Mode switching works correctly
- Cartesian movements match joystick input
- IK solutions are valid
- No data loss or corruption

**Test Procedure**:
1. Launch all nodes with launch file
2. Move joystick and verify arm moves in simulation
3. Measure latency with timestamps
4. Toggle mode switch and verify behavior change
5. Test all joystick directions
6. Verify coordinate frame transformations

---

### **Task 13: Hardware Testing - Arm Movement**
**Objectives**:
- Test with real AR4-MK3 arm
- Tune velocity scaling parameters
- Verify safety limits
- Test emergency stop
- Measure actual latency

**Acceptance Criteria**:
- Arm moves smoothly in response to joystick
- Velocity scaling feels natural
- Workspace limits prevent collisions
- Emergency stop halts arm immediately
- No sudden or jerky movements
- Latency acceptable for manual control

**Test Procedure**:
1. Set up arm in safe workspace
2. Start with very low velocity scale (0.0001)
3. Gradually increase until comfortable
4. Test all movement directions
5. Test mode switching
6. Test emergency stop
7. Verify workspace limits
8. Measure end-to-end latency

**Safety Checklist**:
- [ ] Workspace clear of obstacles
- [ ] Emergency stop tested and working
- [ ] Velocity limits set conservatively
- [ ] Collision detection enabled
- [ ] Observer ready to hit emergency stop
- [ ] Start with simulation first

---

### **Task 14: Parameter Tuning**
**Objectives**:
- Tune velocity scaling for comfortable control
- Adjust dead zones to prevent drift
- Optimize control rate for responsiveness
- Set appropriate workspace limits
- Configure IK solver parameters

**Acceptance Criteria**:
- Velocity scaling provides smooth, controllable movement
- Dead zones eliminate drift without reducing responsiveness
- Control rate provides real-time feel
- Workspace limits prevent unsafe movements
- IK solver succeeds >95% of the time

**Parameters to Tune**:
- `velocity_scale`: Start at 0.0001, increase to 0.001-0.002
- `rotation_scale`: Start at 0.001, increase to 0.01-0.02
- Dead zone: 10-20 ADC units
- `control_rate`: 30-50 Hz
- `ik_timeout`: 30-100ms
- Workspace limits based on physical setup

---

### **Task 15: Documentation**
**Objectives**:
- Document pin assignments
- Document serial protocols
- Document ROS2 topics and services
- Create user guide for operation
- Document troubleshooting steps

**Acceptance Criteria**:
- Pin assignments documented in hardware_interconnection.md
- Serial protocols documented with examples
- ROS2 interface documented
- User guide explains how to operate arm control
- Troubleshooting guide covers common issues

**Documentation Files**:
- Update `docs/hardware_interconnection.md` with final pin assignments
- Create `docs/arm_cartesian_control_user_guide.md`
- Create `docs/arm_cartesian_control_troubleshooting.md`
- Update `README.md` with arm control features

---

## Testing Checklist

### Pre-Hardware Testing
- [ ] All firmware compiles without errors
- [ ] All ROS2 nodes build successfully
- [ ] Component tests pass for remote control
- [ ] Component tests pass for platform controller
- [ ] Component tests pass for ROS2 nodes
- [ ] Integration test passes in simulation
- [ ] Data flow verified end-to-end
- [ ] Latency measured and acceptable

### Hardware Testing
- [ ] Workspace prepared and safe
- [ ] Emergency stop tested
- [ ] Arm moves correctly in all directions
- [ ] Mode switching works
- [ ] Velocity scaling tuned
- [ ] Workspace limits verified
- [ ] No unexpected behavior observed
- [ ] Documentation complete

## Estimated Timeline

- **Task 1**: 4-6 hours (Remote control firmware)
- **Task 2**: 6-8 hours (Platform controller firmware)
- **Task 3**: 1-2 hours (ROS2 package setup)
- **Task 4**: 4-6 hours (Platform serial bridge)
- **Task 5**: 8-10 hours (Cartesian controller - most complex)
- **Task 6**: 4-6 hours (Teensy serial bridge)
- **Task 7**: 1-2 hours (Configuration file)
- **Task 8**: 2-3 hours (Launch file)
- **Task 9**: 2-3 hours (Remote control testing)
- **Task 10**: 2-3 hours (Platform controller testing)
- **Task 11**: 3-4 hours (ROS2 node testing)
- **Task 12**: 4-6 hours (Integration testing)
- **Task 13**: 6-8 hours (Hardware testing)
- **Task 14**: 4-6 hours (Parameter tuning)
- **Task 15**: 3-4 hours (Documentation)

**Total Estimated Time**: 54-76 hours (approximately 2-3 weeks)

## Dependencies

- **Hardware**: LilyGO T-Display S3 AMOLED (x2), Teensy 4.1, AR4-MK3 arm
- **Software**: Arduino IDE, ROS2 Jazzy, MoveIt2, Python 3, pyserial
- **Existing**: Platform controller firmware, AR4 ROS2 package, MoveIt2 configuration

## Notes

- Start with simulation before hardware testing
- Keep velocity scaling very low initially
- Test emergency stop thoroughly before any arm movement
- Document any deviations from the plan
- Take videos of successful tests for documentation
