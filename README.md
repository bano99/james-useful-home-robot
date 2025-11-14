# James - Autonomous Home Cleanup Robot

James is an open-source autonomous home cleanup robot designed to perform simple tasks such as picking up objects from floors and tables and returning them to their proper locations. The system integrates a mobile mecanum-wheel platform with an AR4-MK3 robotic arm, Intel RealSense cameras, and ROS2-based software architecture.

## Features

- **Autonomous Navigation**: SLAM-based mapping and navigation using RTAB-Map and Nav2
- **Object Recognition**: YOLOv8-based object detection with 3D localization
- **Robotic Manipulation**: AR4-MK3 6-DOF arm with custom gripper for pick-and-place operations
- **Voice Control**: Natural language task instructions using LLM-based task planning
- **Safety First**: Human detection, collision avoidance, and manual override capabilities
- **Web Dashboard**: Real-time monitoring and control via web interface
- **Learning System**: Teach James where objects belong through voice commands

## Hardware Components

### Platform
- **Base**: Custom mecanum wheel platform (45cm x 56cm)
- **Wheels**: 17.5cm diameter mecanum wheels
- **Motors**: 4x brushless motors controlled by 2x ODrive controllers
- **Controller**: ESP32-S3 (LilyGO T-Display S3 AMOLED V2)
- **Display**: 1.91" AMOLED touchscreen for status and emergency stop

### Compute
- **Primary Brain**: 2x NVIDIA Jetson Nano
  - Jetson Nano 1: SLAM, Navigation, Platform Control
  - Jetson Nano 2: Object Detection, Manipulation Planning, LLM Inference

### Sensors
- **RealSense D435**: RGB-D camera for object detection and depth perception
- **RealSense T265**: Tracking camera for visual-inertial odometry

### Manipulator
- **AR4-MK3**: 6-DOF robotic arm from Annin Robotics
- **Custom Gripper**: Adaptive gripper for various object types
- **Motors**: JMC closed-loop motors on first 3 joints

### Remote Control
- **Controller**: ESP32-S3 (LilyGO T-Display S3 AMOLED V2)
- **Input**: 3-axis joystick (forward/back, left/right, rotation)
- **Communication**: ESPNOW wireless protocol
- **Display**: Real-time joystick values and connection status

## Project Structure

```
james-home-robot/
├── docs/                    # Documentation and architecture diagrams
├── platform/                # ESP32 firmware for platform and remote control
│   ├── controller/         # Platform controller firmware
│   └── remote/             # Remote control firmware
├── ros2_ws/                # ROS2 workspace
│   └── src/                # ROS2 packages
│       ├── james_bringup/          # Launch files
│       ├── james_description/      # URDF and meshes
│       ├── james_navigation/       # Nav2 configuration
│       ├── james_perception/       # Object detection
│       ├── james_manipulation/     # Arm control
│       ├── james_voice/            # Voice control
│       └── james_platform/         # Platform controller interface
├── models/                 # URDF models, meshes, and ML models
├── scripts/                # Utility scripts
├── tests/                  # Integration tests
└── README.md              # This file
```

## Quick Start Guide

### Prerequisites

**Hardware:**
- 2x NVIDIA Jetson Nano with Ubuntu 24.04
- ESP32-S3 development boards (LilyGO T-Display S3 AMOLED V2)
- Intel RealSense D435 and T265 cameras
- AR4-MK3 robotic arm
- Mecanum wheel platform with ODrive controllers

**Software:**
- ROS2 Jazzy
- Python 3.10+
- Arduino IDE or PlatformIO for ESP32 development

### Installation

#### 1. Clone the Repository

```bash
git clone https://github.com/bano99/james-useful-home-robot.git
cd james-useful-home-robot
```

#### 2. Set Up Jetson Nano

**Install ROS2 Jazzy:**
```bash
# Follow official ROS2 Jazzy installation guide for Ubuntu 24.04
# https://docs.ros.org/en/jazzy/Installation.html
```

**Install Dependencies:**
```bash
# Install RealSense SDK
sudo apt-get install ros-jazzy-realsense2-camera ros-jazzy-realsense2-description

# Install RTAB-Map
sudo apt-get install ros-jazzy-rtabmap-ros

# Install Nav2
sudo apt-get install ros-jazzy-navigation2 ros-jazzy-nav2-bringup

# Install MoveIt2
sudo apt-get install ros-jazzy-moveit

# Install additional dependencies
sudo apt-get install python3-pip
pip3 install ultralytics opencv-python numpy
```

#### 3. Build ROS2 Workspace

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

#### 4. Flash ESP32 Firmware

**Platform Controller:**
```bash
cd platform/controller
# Follow instructions in platform/controller/README.md
```

**Remote Control:**
```bash
cd platform/remote
# Follow instructions in platform/remote/README.md
```

#### 5. Calibrate Cameras

```bash
# Calibrate RealSense D435
ros2 run james_perception calibrate_camera

# Verify T265 tracking
ros2 launch james_bringup t265_test.launch.py
```

### Running James

#### 1. Start Core Systems

```bash
# Terminal 1: Launch all nodes
ros2 launch james_bringup james_full.launch.py

# Terminal 2: Start web dashboard
cd scripts
./start_dashboard.sh
```

#### 2. Build Initial Map

```bash
# Use remote control to drive James around your home
# The SLAM system will automatically build a map
# Save the map when complete:
ros2 service call /rtabmap/save_map std_srvs/srv/Empty
```

#### 3. Teach Object Locations

```bash
# Drive James to the toy box location
# Say: "James, this is where toys belong"
# James will confirm and save the location
```

#### 4. Give Voice Commands

```bash
# Example commands:
# "Pick up toys from the living room and put them in the toy box"
# "Bring dirty glasses to the kitchen"
# "Find my book and bring it to me"
```

### Web Dashboard

Access the web dashboard at: `http://<jetson-ip>:8080`

Features:
- Live camera feeds (RGB and depth)
- 3D map visualization with robot position
- Current task status and queue
- System health monitoring
- Manual control interface

## Safety Features

- **Human Detection**: Maintains 50cm minimum distance from detected persons
- **Emergency Stop**: Hardware button on remote control, touchscreen button on platform
- **Manual Override**: Remote control always takes priority over autonomous commands
- **Collision Avoidance**: Real-time obstacle detection and path replanning
- **Velocity Limits**: Conservative speed limits (max 0.3 m/s) for safe operation

## Development

### Running Tests

```bash
# Unit tests
cd ros2_ws
colcon test

# Integration tests
cd tests
python3 run_integration_tests.py
```

### Contributing

We welcome contributions! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Code of Conduct

This project follows the [Contributor Covenant Code of Conduct](CODE_OF_CONDUCT.md).

## Documentation

- [Remote Installation Guide](docs/remote_installation_guide.md) - **START HERE**: Install ROS2 on Jetson Nano remotely via SSH
- [Hardware Setup Guide](docs/hardware_setup.md) - Complete Jetson Nano setup with Ubuntu, ROS2, and dependencies
- [Jetson Nano Quick Fix](docs/jetson_nano_quick_fix.md) - Troubleshoot boot issues and flash correct image
- [MCP Server Setup](docs/mcp_setup.md) - Configure GitHub MCP server for CI/CD monitoring

Additional documentation will be added as the project develops.

## License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- [Annin Robotics](https://anninrobotics.com/) for the AR4-MK3 arm
- Intel RealSense for camera hardware and SDK
- [ROS2](https://docs.ros.org/) community for the robotics framework
- [RTAB-Map](http://introlab.github.io/rtabmap/) for SLAM capabilities

## Contact

- **Project Lead**: bano99
- **GitHub**: [@bano99](https://github.com/bano99)
- **Issues**: [GitHub Issues](https://github.com/bano99/james-useful-home-robot/issues)

## Roadmap

- [x] Platform control and navigation
- [x] SLAM and mapping
- [ ] Object detection and recognition
- [ ] Manipulation and pick-and-place
- [ ] Voice control integration
- [ ] Web dashboard
- [ ] Multi-robot coordination (future)
- [ ] Advanced learning capabilities (future)

---

**Status**: Active Development | **Version**: 0.1.0-alpha
