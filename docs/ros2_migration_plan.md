# ROS2 Migration Implementation Plan

## Overview

Migrate James robot from ROS2 Foxy to a hybrid architecture leveraging the new Jetson Orin Nano's computational power while keeping original Jetson Nanos useful via Docker containers.

## Hardware Architecture

### New Setup
```
┌─────────────────────────────────────────────────────────────┐
│                    James Robot Network                      │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  Jetson Orin Nano (Primary Brain)                         │
│  ├── Ubuntu 22.04 + ROS2 Humble (native)                 │
│  ├── SLAM & Navigation (RTAB-Map, Nav2)                   │
│  ├── Manipulation Planning (MoveIt2)                      │
│  ├── LLM Inference (Local language model)                 │
│  ├── Task Coordination & Decision Making                  │
│  ├── RealSense D415 (0.35m height - ground-level vision) │
│  └── Object Detection Fusion (YOLOv8 + TensorRT)         │
│                                                             │
│  Original Jetson Nano 1 (High-Level Vision Specialist)    │
│  ├── Ubuntu 20.04 + Docker                                │
│  ├── ROS2 Humble Container (jetson-containers)            │
│  ├── RealSense D435 (1.3m height - SLAM & navigation)     │
│  ├── High-level object detection processing               │
│  └── SLAM feature extraction and preprocessing            │
│                                                             │
│  Original Jetson Nano 2 (Odometry Specialist - Optional)  │
│  ├── Ubuntu 20.04 + Docker                                │
│  ├── ROS2 Humble Container (jetson-containers)            │
│  ├── RealSense T265 (1.2m height - odometry, optional)    │
│  ├── Sensor fusion and pose estimation                    │
│  └── Support processing (if T265 removed)                 │
│                                                             │
│  Powerful PC (LLM Server)                                  │
│  ├── Ubuntu 22.04 + ROS2 Humble                           │
│  ├── Large Language Model inference                       │
│  ├── Advanced task planning                               │
│  ├── Natural language processing                          │
│  └── Knowledge base management                            │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## Migration Phases

### Phase 1: Research & Testing (Week 1-2)
**Goal**: Validate Docker approach and cross-version compatibility

#### 1.1 Install jetson-containers on Original Jetson Nano
```bash
# On original Jetson Nano
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh

# Test basic container
jetson-containers run $(autotag l4t-pytorch)
```

#### 1.2 Test ROS2 Humble Container
```bash
# Build ROS2 Humble container with camera support
jetson-containers build --name=james_ros2_humble \
    ros:humble-desktop \
    opencv:cuda \
    realsense

# Test ROS2 functionality
jetson-containers run james_ros2_humble
# Inside container: ros2 --version
```

#### 1.3 Test Camera Access from Docker
```bash
# Run container with USB device access
jetson-containers run --device=/dev/bus/usb james_ros2_humble

# Inside container: Test RealSense
ros2 launch realsense2_camera rs_launch.py device_type:=d435
```

#### 1.4 Test Cross-Version Communication
```bash
# Terminal 1: Native Foxy (existing setup)
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp talker

# Terminal 2: Docker Humble
jetson-containers run james_ros2_humble
ros2 run demo_nodes_cpp listener
```

### Phase 2: Jetson Orin Setup (Week 3)
**Goal**: Set up primary brain with native ROS2 Humble

#### 2.1 Install Ubuntu 22.04 on Jetson Orin Nano
```bash
# Flash JetPack 5.x with Ubuntu 22.04
# Follow NVIDIA's official flashing guide
```

#### 2.2 Install ROS2 Humble (Native)
```bash
# Add ROS2 repository
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop -y

# Install key packages
sudo apt install -y \
    python3-colcon-common-extensions \
    ros-humble-rtabmap-ros \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-moveit \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description
```

#### 2.3 Install AI/ML Stack
```bash
# Install PyTorch for Jetson Orin
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118

# Install YOLOv8
pip3 install ultralytics

# Install TensorRT for optimization
sudo apt install tensorrt
```

### Phase 3: Container Migration (Week 4)
**Goal**: Migrate original Jetson Nanos to Docker containers

#### 3.1 Create Custom James Containers

**Camera Node Container:**
```bash
# Create Dockerfile for camera processing
jetson-containers build --name=james_camera_node \
    ros:humble-desktop \
    opencv:cuda \
    realsense \
    pytorch
```

**Platform Control Container:**
```bash
# Create Dockerfile for platform control
jetson-containers build --name=james_platform_control \
    ros:humble-desktop \
    --package=custom_platform_drivers
```

#### 3.2 Migrate ROS2 Packages
```bash
# Update package.xml files for Humble compatibility
# Update CMakeLists.txt for newer dependencies
# Test build in containers

# Build workspace in container
jetson-containers run james_camera_node
cd /workspace/ros2_ws
colcon build --packages-select james_perception
```

### Phase 4: Network Integration (Week 5)
**Goal**: Integrate all devices into unified ROS2 network

#### 4.1 Configure DDS Discovery
```bash
# Set up multicast discovery across devices
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Configure network interfaces for discovery
# Ensure all devices can see each other
```

#### 4.2 Test Distributed System
```bash
# Jetson Orin: Start SLAM
ros2 launch james_slam rtabmap.launch.py

# Jetson Nano 1: Start cameras
jetson-containers run james_camera_node
ros2 launch james_perception cameras.launch.py

# Jetson Nano 2: Start platform
jetson-containers run james_platform_control
ros2 launch james_platform platform.launch.py

# Verify communication
ros2 topic list
ros2 node list
```

### Phase 5: LLM Integration (Week 6)
**Goal**: Add powerful PC for LLM processing

#### 5.1 Set Up LLM Server
```bash
# On powerful PC
# Install ROS2 Humble
# Install local LLM (Ollama, llama.cpp, or similar)
# Create ROS2 service for LLM queries
```

#### 5.2 Create LLM ROS2 Interface
```python
# james_llm/llm_service.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from james_interfaces.srv import ProcessCommand

class LLMService(Node):
    def __init__(self):
        super().__init__('llm_service')
        self.service = self.create_service(
            ProcessCommand, 
            'process_voice_command', 
            self.process_command_callback
        )
    
    def process_command_callback(self, request, response):
        # Process natural language command
        # Return structured task plan
        pass
```

## Performance Expectations

### Computational Power Distribution

| Device | Task | CPU Usage | GPU Usage | Memory |
|--------|------|-----------|-----------|---------|
| Jetson Orin Nano | SLAM, Navigation, Planning | 60-80% | 70-90% | 6-7GB |
| Jetson Nano 1 | Camera Processing | 70-90% | 50-70% | 3-3.5GB |
| Jetson Nano 2 | Platform Control | 30-50% | 10-20% | 2-2.5GB |
| Powerful PC | LLM Inference | 40-60% | 80-95% | 8-16GB |

### Expected Improvements

1. **40x AI Performance**: Orin Nano vs original Nano (40 TOPS vs 0.5 TOPS)
2. **Better SLAM**: More computational power for complex environments
3. **Faster Object Detection**: TensorRT optimization on Orin
4. **Local LLM**: No cloud dependency for voice commands
5. **Improved Manipulation**: Better planning with more compute power

## Risk Mitigation

### Rollback Plan
- Keep original Foxy setup intact during migration
- Test each phase thoroughly before proceeding
- Maintain backup images of working configurations

### Performance Monitoring
```bash
# Monitor container resource usage
docker stats

# Monitor ROS2 communication
ros2 topic hz /camera/image_raw
ros2 topic bw /camera/image_raw

# Monitor system resources
htop
nvidia-smi
```

### Compatibility Testing
- Test all ROS2 message types between versions
- Verify camera drivers work in containers
- Ensure real-time performance for safety systems

## Success Criteria

### Phase 1 Success
- [ ] jetson-containers runs on original Jetson Nano
- [ ] ROS2 Humble container communicates with Foxy native
- [ ] RealSense cameras accessible from Docker
- [ ] Performance overhead < 15%

### Phase 2 Success
- [ ] Jetson Orin Nano runs ROS2 Humble natively
- [ ] SLAM performance improved vs original setup
- [ ] All required packages installed and working

### Phase 3 Success
- [ ] Original Jetson Nanos run containerized ROS2 Humble
- [ ] Camera processing maintains real-time performance
- [ ] Platform control maintains safety requirements

### Phase 4 Success
- [ ] All devices communicate in unified ROS2 network
- [ ] End-to-end system functionality maintained
- [ ] No message loss or significant latency

### Phase 5 Success
- [ ] LLM processes voice commands locally
- [ ] Task planning improved with language understanding
- [ ] System responds to natural language instructions

## Timeline Summary

| Week | Phase | Key Deliverables |
|------|-------|------------------|
| 1-2 | Research & Testing | Docker validation, cross-version communication |
| 3 | Jetson Orin Setup | Primary brain with ROS2 Humble |
| 4 | Container Migration | Original Nanos running Docker containers |
| 5 | Network Integration | Unified distributed ROS2 system |
| 6 | LLM Integration | Local language model processing |

## Next Immediate Steps

1. **Install jetson-containers** on one original Jetson Nano
2. **Test ROS2 Humble container** with basic functionality
3. **Verify RealSense camera access** from Docker
4. **Benchmark performance** vs native Foxy setup
5. **Document findings** and adjust plan as needed

This migration will significantly enhance James's capabilities while maintaining the investment in existing hardware!