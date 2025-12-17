# Hardware Interconnection Diagram

This document outlines the hardware connections between the various components of the James Useful Home Robot, including the Remote Control, Platform Controller, Robot Arm, and Gripper.

## System Overview

The system consists of a central computation unit (NVIDIA Jetson Nano Orin) which communicates with three primary microcontroller subsystems via USB/Serial. The Remote Control communicates wirelessly with the Platform Controller via ESP-NOW.

```mermaid
graph TD
    subgraph "Remote Control Unit"
        RC["Remote Control<br/>(ESP32-S3 + AMOLED)"]
        L_JOY["Left Joystick<br/>(Arm Manual Control)"]
        R_JOY["Right Joystick<br/>(Platform/Arm Vertical)"]
        SW_MODE["Mode Switch<br/>(Left/Right Control)"]
        G_POT["Gripper Pot<br/>(Pin 16)"]
    end

    subgraph "Mobile Platform"
        PC["Platform Controller<br/>(ESP32-S3 + AMOLED)"]
        OD1["ODrive 1<br/>(Front Motors)"]
        OD2["ODrive 2<br/>(Rear Motors)"]
        M_FL((M FL))
        M_FR((M FR))
        M_BL((M BL))
        M_BR((M BR))
    end

    subgraph "Distributed Compute Cluster"
        ORIN["Jetson Orin Nano<br/>(Central Hub)"]
        NANO1["Jetson Nano 1<br/>(Vision Preproc)"]
        NANO2["Jetson Nano 2<br/>(Sensor Hub)"]
    end

    subgraph "Manipulation System"
        TEENSY["Robot Arm Controller<br/>(Teensy 4.1)"]
        GRP["Gripper Controller<br/>(ESP32 Nano)"]
        
        DRIVERS["Stepper Drivers<br/>x6"]
        ENC["Encoders<br/>x6"]
        SERVO((Servo))
    end
    
    subgraph "Vision & Sensors"
        D415["D415 Camera<br/>(Manipulation)"]
        D435["D435 Camera<br/>(Long Range / SLAM)"]
        T265["T265 Camera<br/>(Tracking / Odom)"]
        TOF["ToF Sensor<br/>(VL53L1X)"]
        IMU["IMU<br/>(BNO055)"]
    end

    %% Inputs to Remote
    L_JOY --> RC
    R_JOY --> RC
    SW_MODE --> RC
    G_POT --> RC

    %% Remote Communications
    RC -.->|"ESP-NOW<br/>(Platform Control)"| PC
    RC -.->|"ESP-NOW<br/>(Direct Gripper Control)"| GRP
    
    %% Compute Cluster Connections
    NANO1 <==>|"Network<br/>(Ethernet/WiFi)"| ORIN
    NANO2 <==>|"Network<br/>(Ethernet/WiFi)"| ORIN
    
    %% Controller Connections to Orin
    PC <==>|"USB / Serial"| ORIN
    TEENSY <==>|"USB / Serial"| ORIN
    GRP <==>|"USB / Serial"| ORIN
    
    %% Platform Internal
    PC -->|"I2C-to-UART<br/>Bridge"| OD1
    PC -->|"I2C-to-UART<br/>Bridge"| OD2
    OD1 --> M_FL
    OD1 --> M_FR
    OD2 --> M_BL
    OD2 --> M_BR

    %% Arm Internal
    TEENSY --> DRIVERS
    TEENSY --> ENC

    %% Gripper Internal
    GRP -->|PWM| SERVO
    
    %% Sensor/Camera Connections
    D415 <==>|USB| ORIN
    D435 <==>|USB| NANO1
    T265 <==>|USB| NANO2
    
    ORIN -->|"I2C Bus 7<br/>(Pins 3 & 5)"| TOF
    ORIN -->|"I2C Bus 7<br/>(Pins 3 & 5)"| IMU
```

---

## 2. Component Details & Pinouts

### 2.1 Remote Control Unit (Remote)
**Hardware**: LilyGO T-Display S3 AMOLED V2  
**Role**: Master controller for Mobility (Mecanum) and Manipulation (Arm/Gripper).

#### Control Modes
The **Left Switch** toggles the precision control behavior of the **Right Joystick**.

**Mode A: Platform Control (Switch ON)**
- **Left Joystick**: *Horizontal Arm Control* (Cartesian X/Y). Moves gripper tip Forward/Back/Left/Right while maintaining height and orientation.
- **Right Joystick**: *Platform Movement*. Standard mecanum drive (Holonomic).
- **Gripper Pot**: Servo Open/Close position.

**Mode B: Vertical Arm Control (Switch OFF)**
- **Left Joystick**: *Horizontal Arm Control* (Cartesian X/Y). Same as Mode A.
- **Right Joystick**: *Vertical Arm Control*.
    - Up/Down: Gripper moves vertically (Cartesian Z) maintaining orientation.
    - Left/Right: Gripper Rotation (Yaw) or J1 rotation.
- **Gripper Pot**: Servo Open/Close position.

#### Connections
| Component | Pin (ESP32) | Function | Notes |
| :--- | :--- | :--- | :--- |
| **Right Joystick** | 13 | Analog In | Y-axis (Fwd/Back) |
| | 14 | Analog In | X-axis (Left/Right) |
| | 15 | Analog In | Rotation / Twist |
| **Gripper Pot** | 16 | Analog In | Gripper Open/Close Position |
| **Left Joystick** | *TBD* | Analog In | Y-axis (Arm Fwd/Back) |
| | *TBD* | Analog In | X-axis (Arm Left/Right) |
| | *TBD* | Analog In | Z-axis / Rotation |
| **Mode Switch** | *TBD* | Digital In | Toggle Platform/Arm Vertical Mode |

**Note**: Left Joystick manual control is calculated via Inverse Kinematics on the Orin (received via Platform Controller -> Orin link).

### 2.2 Platform Controller
**Hardware**: LilyGO T-Display S3 AMOLED V2  
**Role**: Controls Mecanum wheels, displays status, bridges remote to Orin.

#### Connections
| Component | Pin (ESP32) | Function | Notes |
| :--- | :--- | :--- | :--- |
| **Orin** | USB | Data | Connected via USB-C port (Serial over USB) |
| **I2C Bridge** | 3 | SDA | To DFRobot I2C-to-Dual-UART Bridge |
| | 2 | SCL | To DFRobot I2C-to-Dual-UART Bridge |
| **Display** | *Internal* | | Landscape orientation |

### 2.3 Robot Arm (AR4 MK3)
**Hardware**: Teensy 4.1  
**Role**: Controls the 6-axis AR4 MK3 robotic arm.  
**Connection to Orin**: USB Serial

### 2.4 Gripper System
**Hardware**: ESP32 (Nano form factor)  
**Role**: Controls end-effector servo only.  
**Connection to Remote**: Direct ESP-NOW link for low-latency gripper actuation.  
**Connection to Orin**: USB Serial (for status/configuration).

#### Connections
| Component | Pin | Protocol | Notes |
| :--- | :--- | :--- | :--- |
| **Servo** | *TBD* | PWM | Gripper Open/Close |
| **Remote** | Wireless | ESP-NOW | Direct control from RC Pin 16 |

### 2.5 Distributed Compute Cluster
**1. NVIDIA Jetson Orin Nano ("Brain Stem")**
- **Role**: Central Controller, Nav2, MoveIt, Safety.
- **Connections**:
    - **D415 Camera** (USB): Direct manipulation visuals.
    - **Sensors** (I2C): ToF and IMU.
    - **Controllers** (USB): Platform, Arm, Gripper.
    - **Network**: Links to Nano 1 & 2.

**2. Jetson Nano 1 ("The Eyes")**
- **Role**: Vision Pre-processing (SLAM, Feature Extraction).
- **Connections**:
    - **D435 Camera** (USB): Long-range RGB-D.

**3. Jetson Nano 2 ("The Senses")**
- **Role**: Odometry and Backup Sensors.
- **Connections**:
    - **T265 Camera** (USB): Visual Inertial Odometry.

---

## 3. Communication Flow

### Manual Control Path
1.  **User** manipulates Remote inputs.
2.  **Remote** reads Inputs:
    - **Right Joystick** -> Sent via **ESP-NOW** to **Platform Controller**.
    - **Left Joystick** -> Sent via **ESP-NOW** to **Platform Controller** -> Forwarded via **USB** to **Orin**.
    - **Gripper Pot** -> Sent via **ESP-NOW** directly to **Gripper ESP32**.
3.  **Orin** (running ROS 2):
    - Receives Left Joystick (Cartesian Cmd).
    - Calculates IK for Arm Joint velocities (J1 Bottom -> J6 Top).
    - Sends Joint commands via **USB** to **Teensy**.
4.  **Platform Controller**:
    - If Mode Switch = ON: Maps Right Joystick directly to Mecanum drive (Manual).
    - If Mode Switch = OFF: Ignores Right Joystick.

### Autonomous Path
- **Nano 1 & 2** process camera feeds and send simplified odometry/map data to **Orin**.
- **Orin** fuses Sensor data (IMU, ToF, T265 Odom) and Plans path.
- **Orin** sends commands to Platform, Arm, and Gripper via USB.

