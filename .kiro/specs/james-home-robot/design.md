# James Home Robot - Design Document

## Overview

James is built on a layered ROS2 architecture running on dual Jetson Nano computers. The system integrates low-level motor control (ESP32), perception (Intel RealSense cameras), manipulation (AR4-MK3 arm), and high-level decision-making (LLM-based task planning). The design prioritizes modularity, safety, and independent testability of components.

### Physical Dimensions

**Platform:**
- Footprint (including wheels): 45cm x 56cm
- Wheel diameter: 17.5cm
- Wheel width: 6.5cm
- Platform construction: Rigid aluminum extrusions

**Robot Arm:**
- AR4-MK3 mounted at: 36cm above ground
- J3 maximum height: 83cm above ground
- Arm reach: ~60cm (typical for AR4-MK3)

**Cameras:**
- Mounted height: 120cm above ground
- Position: 23cm behind J1 (robot arm base)
- Distance from platform front: ~35cm behind front edge
- Both RealSense D435 and T265 at same height and position

**Total System:**
- Maximum height: ~120cm (camera mount)
- Weight: 50-60kg (platform + arm + electronics)
- Center of gravity: Low due to heavy platform base

### Key Design Principles

1. **Safety First**: All autonomous operations can be overridden by manual control; collision avoidance is mandatory
2. **Modular Architecture**: Each subsystem (SLAM, perception, manipulation, navigation) operates as independent ROS2 nodes
3. **Visual Feedback Loop**: Compensate for mechanical limitations (wheel slip, arm backlash) using camera feedback
4. **Local-First Processing**: Minimize cloud dependencies for real-time operations; offload only non-critical tasks
5. **Incremental Development**: SLAM → Object Recognition → Object Learning → Manipulation

## Architecture

### System Layers

```
┌─────────────────────────────────────────────────────────────┐
│                    Application Layer                         │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ Voice Control│  │ Task Planner │  │  Web UI      │      │
│  │   + LLM      │  │              │  │  Dashboard   │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                  Decision & Planning Layer                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Behavior   │  │ Manipulation │  │  Navigation  │      │
│  │    Tree      │  │   Planner    │  │   Planner    │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    Perception Layer                          │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │     SLAM     │  │    Object    │  │   Human      │      │
│  │   (RTAB-Map) │  │  Detection   │  │  Detection   │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                     Control Layer                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │   Platform   │  │     Arm      │  │   Gripper    │      │
│  │  Controller  │  │  Controller  │  │  Controller  │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
                            ▼
┌─────────────────────────────────────────────────────────────┐
│                    Hardware Layer                            │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐      │
│  │ ESP32 + 2x   │  │  AR4-MK3     │  │  RealSense   │      │
│  │   ODrive     │  │   Robot Arm  │  │  D435 + T265 │      │
│  └──────────────┘  └──────────────┘  └──────────────┘      │
└─────────────────────────────────────────────────────────────┘
```

### Hardware Compute Distribution

**Jetson Nano 1 (Primary):**
- SLAM (RTAB-Map)
- Navigation planning (Nav2)
- Platform control coordination
- Safety monitoring

**Jetson Nano 2 (Secondary):**
- Object detection/recognition
- Manipulation planning (MoveIt2)
- Arm control
- LLM inference (quantized model) or API gateway

**ESP32 Platform Controller:**
- ODrive UART communication
- Remote control ESPNOW receiver
- Manual override arbitration
- Emergency stop handling

**ESP32 Remote Control (LilyGO T-Display S3 AMOLED V2):**
- Hardware: Same board as platform controller
- 3-axis joystick input reading (forward/back, left/right, rotation)
- ESPNOW transmitter to platform controller
- AMOLED display showing joystick values and connection status
- Touchscreen interface for mode selection and settings
- Current implementation: Reads analog pins 13, 14, 15 for joystick axes

## Components and Interfaces

### 1. Platform Control System

#### ESP32 Platform Controller (LilyGO T-Display S3 AMOLED V2)

**Hardware:**
- Board: LilyGO T-Display S3 AMOLED V2
- Display: 1.91" AMOLED touchscreen (536x240 resolution)
- MCU: ESP32-S3 with WiFi
- I2C-to-UART Bridge: DFRobot_IICSerial (2 channels for 2x ODrive)
- Reference: https://lilygo.cc/products/t-display-s3-amoled
- Examples: https://github.com/Xinyuan-LilyGO/LilyGo-AMOLED-Series

**Responsibilities:**
- Receive commands from both Remote Control (ESPNOW) and Main Brain (Serial/USB)
- Implement priority system: Manual > Autonomous
- Translate high-level velocity commands (vx, vy, omega) to individual wheel velocities
- Send wheel velocity commands to ODrive controllers via I2C-to-UART bridge
- Display real-time status on AMOLED screen
- Provide touchscreen interface for emergency stop and mode selection

**Interfaces:**
- **Input (ESPNOW)**: Manual control commands from Remote Control
  - Current format: `{x: int, y: int, rot: int}` (raw joystick values -255 to 255)
  - Received at ~6.7 Hz (150ms interval)
- **Input (Serial/USB)**: Autonomous commands from Jetson
  - Format: `{x: int, y: int, rot: int}` (same as ESPNOW remote control format)
  - Range: -255 to 255 for each axis
  - Reuses existing `calculateMovement()` and `controlMecanumWheels()` functions
  - ROS2 node will translate cmd_vel (vx, vy, omega) to joystick-equivalent values
  - Advantage: No firmware changes needed, tested and working code
- **Output (I2C-to-UART)**: Wheel velocity commands to 2x ODrive
  - ODrive 1 (iicSerial1): Front-right (axis1), Back-right (axis0)
  - ODrive 2 (iicSerial2): Front-left (axis0), Back-left (axis1)
  - Command format: `v <axis> <velocity> 0` (ODrive ASCII protocol)
  - Velocity range: -3.0 to 3.0 (exponentially mapped from joystick)
- **Output (Serial/USB)**: Status and command echo to Jetson
  - Proposed format: `{commanded_speeds: [fl, fr, rl, rr], mode: str, timestamp: uint64}`
  - Needs to be implemented in firmware
- **Display Output**: AMOLED touchscreen
  - Show current velocities, mode (manual/autonomous), connection status
  - Touch buttons for emergency stop, mode switch

**Mecanum Kinematics:**
```
For mecanum wheels arranged in X-configuration:
FL = vx - vy - omega * (L + W)
FR = vx + vy + omega * (L + W)
RL = vx + vy - omega * (L + W)
RR = vx - vy + omega * (L + W)

Where:
- vx: forward velocity
- vy: lateral velocity
- omega: rotational velocity
- L: half of wheelbase
- W: half of track width
```

#### ROS2 Platform Controller Node

**Responsibilities:**
- Bridge between ROS2 and ESP32 Platform Controller
- Subscribe to cmd_vel commands from navigation
- Publish odometry and TF transforms
- Monitor connection health

**ROS2 Topics:**
- Subscribe: `/cmd_vel` (geometry_msgs/Twist)
- Publish: `/odom` (nav_msgs/Odometry)
- Publish: `/platform/status` (custom status message)
- Publish: `/tf` (platform base_link transform)

### 2. SLAM and Localization

#### RTAB-Map Configuration

**Why RTAB-Map:**
- Excellent ROS2 support
- Handles both visual and depth data
- Works well with RealSense cameras
- Supports loop closure for long-term mapping
- Can integrate T265 odometry

**Sensor Fusion:**
- **Primary**: RealSense D435 (RGB-D SLAM)
- **Secondary**: RealSense T265 (Visual-Inertial Odometry)
- **Experimental**: Platform wheel odometry (interpolated from commands)
  - Note: Current ESP32 controller does not provide actual wheel measurements
  - Hall sensors exist but are not currently read by firmware
  - May be useful for command verification but not reliable for localization
  - Visual odometry from T265 is significantly more accurate on slippery surfaces

**Configuration:**
```yaml
rtabmap:
  frame_id: base_link
  odom_frame_id: odom
  subscribe_depth: true
  subscribe_rgb: true
  subscribe_odom_info: true
  
  # Use T265 odometry as prior
  odom_sensor_sync: true
  
  # Memory management for Jetson Nano
  Mem/IncrementalMemory: true
  Mem/InitWMWithAllNodes: false
  
  # Loop closure
  Mem/LoopClosureDetection: true
  
  # Optimize for indoor environments
  RGBD/OptimizeMaxError: 0.1
```

**ROS2 Topics:**
- Subscribe: `/camera/depth/image_rect_raw` (D435)
- Subscribe: `/camera/color/image_raw` (D435)
- Subscribe: `/camera/aligned_depth_to_color/camera_info`
- Subscribe: `/t265/odom/sample` (T265 odometry)
- Publish: `/rtabmap/map` (OccupancyGrid)
- Publish: `/rtabmap/cloud_map` (PointCloud2)

### 3. Navigation System

#### Nav2 Stack Configuration

**Components:**
- **Global Planner**: NavFn or Smac Planner (A* based)
- **Local Planner**: DWB Controller (Dynamic Window Approach)
- **Recovery Behaviors**: Spin, backup, wait
- **Costmap Layers**: Static map, obstacle layer, inflation layer

**Safety Configuration:**
```yaml
local_costmap:
  robot_radius: 0.35  # Based on 45x56cm footprint (diagonal ~72cm, radius ~36cm + margin)
  inflation_radius: 0.40  # Reduced to allow navigation through 55-60cm doors
  obstacle_layer:
    observation_sources: depth_camera
    depth_camera:
      topic: /camera/depth/points
      max_obstacle_height: 2.0
      min_obstacle_height: 0.10  # Ignore low obstacles (yoga mats, carpets, cables)
      # Objects < 10cm height are considered traversable
      
controller_server:
  max_vel_x: 0.3  # Conservative for 50-60kg robot
  max_vel_y: 0.3
  max_vel_theta: 0.5
  min_vel_x: -0.1
  min_vel_y: -0.3
  
  # Emergency stop distance - reduced for tight spaces
  min_obstacle_distance: 0.15  # 15cm minimum clearance
  # Note: Robot must navigate through 55-60cm doors with 45x56cm footprint
```

**Traversable Obstacle Handling:**
- Objects below 10cm height (yoga mats, carpets, cables) are ignored by obstacle detection
- This allows James to drive over common floor items safely
- Wheel diameter (17.5cm) provides sufficient ground clearance
- Training data should include examples of:
  - Yoga mats (typically 3-6mm thick)
  - Carpets and rugs (typically 5-20mm thick)
  - Cables and cords (typically < 2cm diameter)
  - Thresholds between rooms (typically < 2cm)
- Objects above 10cm are treated as obstacles to avoid

**Visual Odometry as Primary Source:**
- T265 visual-inertial odometry is the primary source for localization (not wheel odometry)
- Wheel odometry is unreliable due to:
  - Slippery floor surfaces causing significant slip
  - Hall sensors not currently integrated in ESP32 firmware
  - No closed-loop feedback from motors to controller
- Command verification: Compare commanded velocities with T265 measured displacement
- Detect systematic slip patterns and adjust velocity limits accordingly
- Future enhancement: Integrate hall sensor readings if needed for diagnostics

### 4. Object Detection and Recognition

#### Local Detection (Jetson Nano 2)

**Model Selection:**
- **Primary**: YOLOv8n (nano) or YOLOv5s optimized for Jetson
- **Fallback**: MobileNet-SSD for lower latency
- **TensorRT Optimization**: Convert models to TensorRT for 3-5x speedup

**Detection Pipeline:**
1. Capture RGB frame from D435
2. Run inference on Jetson Nano 2
3. For each detection, extract depth from aligned depth image
4. Project 2D bounding box + depth to 3D point cloud
5. Publish detected objects with 3D poses

**ROS2 Object Detection Node:**
```python
# Pseudo-interface
class ObjectDetectionNode:
    def __init__(self):
        self.subscribe('/camera/color/image_raw')
        self.subscribe('/camera/aligned_depth_to_color/image_raw')
        self.publish('/detected_objects', DetectedObjectArray)
        
    def detect(self, rgb_image, depth_image):
        # Run YOLO inference
        detections = self.model.predict(rgb_image)
        
        # Convert to 3D
        objects_3d = []
        for det in detections:
            x, y, w, h = det.bbox
            depth = depth_image[y:y+h, x:x+w].median()
            point_3d = self.project_to_3d(x, y, depth)
            objects_3d.append({
                'class': det.class_name,
                'confidence': det.confidence,
                'position': point_3d,
                'bbox_3d': self.estimate_3d_bbox(det, depth)
            })
        
        return objects_3d
```

#### Cloud-Based Recognition (Optional Enhancement)

**Use Cases:**
- Unknown object classification
- Fine-grained object recognition (e.g., specific toy types)
- Object attribute extraction (color, material, fragility)

**API Integration:**
- **Option 1**: OpenAI Vision API
- **Option 2**: Google Cloud Vision API
- **Option 3**: Custom model on cloud GPU

**Implementation:**
- Only call cloud API when local confidence < 0.7
- Cache results to avoid repeated API calls
- Implement request queuing to avoid blocking

### 5. Manipulation System

#### AR4-MK3 ROS2 Integration

**Existing Support:**
- AR4-MK3 has official ROS2 support (https://anninrobotics.com/ros2/)
- Provides URDF model and MoveIt2 configuration
- Joint trajectory controller interface

**Custom Modifications:**
- Update URDF to reflect JMC closed-loop motors on first 3 joints
- Calibrate backlash compensation parameters
- Add custom gripper URDF and controller

#### MoveIt2 Configuration

**Motion Planning:**
- **Planner**: OMPL with RRTConnect algorithm
- **Planning Scene**: Integrate RTAB-Map point cloud as collision geometry
- **IK Solver**: KDL or TRAC-IK for better success rate

**Visual Servoing for Backlash Compensation:**
```python
class VisualServoNode:
    def __init__(self):
        self.subscribe('/camera/color/image_raw')
        self.subscribe('/detected_objects')
        self.publish('/arm/correction_twist', Twist)
        
    def servo_to_object(self, target_object):
        while not self.is_aligned(target_object):
            # Compute error in image space
            error_x, error_y = self.compute_image_error(target_object)
            
            # Convert to Cartesian correction
            correction = self.image_to_cartesian(error_x, error_y)
            
            # Send small correction to arm
            self.publish_correction(correction)
            
            # Wait for arm to settle
            time.sleep(0.1)
```

#### Grasp Planning

**Approach:**
1. Segment object point cloud from scene
2. Compute object principal axes (PCA)
3. Generate candidate grasp poses along principal axes
4. Score grasps based on:
   - Gripper reachability
   - Collision-free approach
   - Grasp stability (force closure)
5. Execute highest-scoring grasp

**Gripper Controller:**
- Custom ROS2 controller for gripper
- Force feedback (if available) to prevent crushing objects
- Configurable grip strength per object type

### 6. Human Detection and Safety

#### Human Detection

**Method 1: Depth-based (Primary)**
- Use D435 depth image to detect human-shaped blobs
- Fast and runs locally on Jetson
- Threshold: Objects 1.2m - 2.0m tall, 0.3m - 0.6m wide

**Method 2: ML-based (Secondary)**
- YOLOv8 person class detection
- Higher accuracy but more compute-intensive
- Run at lower frequency (5 Hz vs 15 Hz for depth-based)

**Safety Zone Implementation:**
```python
class SafetyMonitor:
    SAFETY_ZONES = {
        'critical': 0.3,   # Emergency stop
        'warning': 0.5,    # Slow down
        'caution': 1.0     # Reduce max velocity
    }
    
    def check_safety(self, detected_humans):
        min_distance = float('inf')
        
        for human in detected_humans:
            distance = self.compute_distance(human.position)
            min_distance = min(min_distance, distance)
        
        if min_distance < self.SAFETY_ZONES['critical']:
            self.publish_emergency_stop()
        elif min_distance < self.SAFETY_ZONES['warning']:
            self.publish_velocity_limit(0.1)  # 10 cm/s
        elif min_distance < self.SAFETY_ZONES['caution']:
            self.publish_velocity_limit(0.2)  # 20 cm/s
```

### 7. Voice Control and LLM Integration

#### Voice Recognition

**Option 1: Local (Preferred)**
- **Model**: Whisper.cpp (quantized) on Jetson Nano 2
- **Pros**: Low latency, no internet required, privacy
- **Cons**: Limited accuracy, compute overhead

**Option 2: Cloud**
- **Service**: Google Speech-to-Text or OpenAI Whisper API
- **Pros**: High accuracy
- **Cons**: Latency, requires internet, privacy concerns

#### LLM Task Planning

**Architecture:**
```
Voice Input → Speech-to-Text → LLM → Task Parameters → Behavior Tree
```

**LLM Prompt Structure:**
```
System: You are James, a home cleanup robot. Parse user commands into structured tasks.

Available actions:
- navigate_to(location: str)
- pick_object(object_type: str, location: str)
- place_object(object_type: str, location: str)
- search_object(object_type: str)

User: "Pick up the toys from the living room floor and put them in the toy box"


Response: {
  "task": "cleanup_objects",
  "steps": [
    {"action": "navigate_to", "params": {"location": "living_room"}},
    {"action": "search_object", "params": {"object_type": "toy"}},
    {"action": "pick_object", "params": {"object_type": "toy", "location": "floor"}},
    {"action": "navigate_to", "params": {"location": "toy_box"}},
    {"action": "place_object", "params": {"object_type": "toy", "location": "toy_box"}},
    {"action": "repeat_until", "params": {"condition": "no_toys_on_floor"}}
  ]
}
```

**LLM Options:**
- **Local**: Llama 3.1 8B (quantized to 4-bit) via llama.cpp
- **Cloud**: OpenAI GPT-4 or Anthropic Claude via API
- **Hybrid**: Local for simple commands, cloud for complex reasoning

#### Text-to-Speech Feedback

- **Library**: piper-tts (fast, local, good quality)
- **Use Cases**: Confirm task understanding, report status, request clarification

### 8. Object Location Learning System

#### Knowledge Base Structure

```python
# Object location database
object_locations = {
    'toy': [
        {'location': 'toy_box', 'position': [x, y, z], 'priority': 1},
        {'location': 'kids_room_shelf', 'position': [x, y, z], 'priority': 2}
    ],
    'glass': [
        {'location': 'kitchen_cabinet', 'position': [x, y, z], 'priority': 1},
        {'location': 'dishwasher', 'position': [x, y, z], 'priority': 2}
    ],
    'book': [
        {'location': 'bookshelf', 'position': [x, y, z], 'priority': 1}
    ]
}
```

#### Teaching Interface

**Voice-Based Teaching:**
```
User: "James, this is where toys belong"
James: "I see a toy box at this location. Should I remember this as the primary location for toys?"
User: "Yes"
James: "Understood. I will bring toys here."
```

**Implementation:**
```python
class LocationLearningNode:
    def teach_location(self, object_type, location_name):
        # Get current robot position from SLAM
        current_pose = self.get_current_pose()
        
        # Detect object at current location
        detected_objects = self.detect_objects_at_pose(current_pose)
        
        # Confirm with user
        confirmation = self.request_voice_confirmation(
            f"Should I remember {location_name} as the location for {object_type}?"
        )
        
        if confirmation:
            self.knowledge_base.add_location(
                object_type=object_type,
                location_name=location_name,
                position=current_pose,
                priority=1
            )
            self.save_knowledge_base()
```

#### Persistent Storage

- **Format**: SQLite database or JSON file
- **Location**: `/home/james/knowledge/object_locations.db`
- **Backup**: Automatic backup to cloud storage (optional)

## Data Models

### Core Data Structures

#### DetectedObject
```python
class DetectedObject:
    id: str                    # Unique identifier
    class_name: str            # Object type (e.g., "toy", "glass")
    confidence: float          # Detection confidence [0-1]
    position: Point3D          # 3D position in map frame
    bbox_3d: BoundingBox3D     # 3D bounding box
    timestamp: Time            # Detection timestamp
    point_cloud: PointCloud    # Segmented object point cloud
```

#### ObjectLocation
```python
class ObjectLocation:
    object_type: str           # Type of object
    location_name: str         # Human-readable name
    position: Pose             # 3D pose in map frame
    priority: int              # Priority (1=primary, 2=secondary, etc.)
    capacity: int              # Max objects (optional)
    current_count: int         # Current objects at location
```

#### Task
```python
class Task:
    id: str
    type: str                  # "pick_and_place", "search", "navigate"
    status: str                # "pending", "in_progress", "completed", "failed"
    parameters: dict           # Task-specific parameters
    steps: List[TaskStep]      # Sequence of actions
    created_at: Time
    completed_at: Time
```

#### RobotState
```python
class RobotState:
    pose: Pose                 # Current position in map
    velocity: Twist            # Current velocity
    arm_joint_states: List[float]  # Arm joint positions
    gripper_state: str         # "open", "closed", "holding"
    held_object: DetectedObject    # Object in gripper (if any)
    battery_level: float       # Battery percentage
    mode: str                  # "manual", "autonomous", "emergency_stop"
```

## Error Handling

### Platform Control Errors

**Command Effectiveness Monitoring:**
```python
class CommandEffectivenessMonitor:
    """
    Monitor how well commanded velocities translate to actual movement.
    Uses T265 visual odometry as ground truth since wheel odometry is unreliable.
    """
    def monitor_effectiveness(self, commanded_vel, visual_odom):
        # Integrate commanded velocity over time window
        expected_displacement = self.integrate_velocity(commanded_vel, dt=0.1)
        
        # Get actual displacement from T265
        actual_displacement = visual_odom.displacement
        
        # Calculate effectiveness ratio
        effectiveness = np.linalg.norm(actual_displacement) / np.linalg.norm(expected_displacement)
        
        # Detect poor traction (slip or stuck)
        if effectiveness < 0.5:  # Less than 50% of commanded motion achieved
            self.publish_traction_warning()
            self.reduce_velocity_limit(factor=0.7)
            return "poor_traction"
        elif effectiveness < 0.8:  # Moderate slip
            self.publish_slip_warning()
            return "moderate_slip"
        
        return "good"
```

**Communication Loss:**
- ESP32 → Jetson: If no heartbeat for 1 second, enter safe mode (stop platform)
- Remote Control → ESP32: If no signal for 2 seconds, disable manual override
- Jetson 1 ↔ Jetson 2: If ROS2 communication fails, continue with degraded functionality

### SLAM Failures

**Lost Localization:**
1. Stop platform movement
2. Rotate in place to gather more visual features
3. Attempt relocalization using loop closure
4. If failed after 30 seconds, request manual intervention

**Map Quality Degradation:**
- Monitor loop closure success rate
- If < 50% over 5 minutes, trigger map optimization
- Periodically save map checkpoints

### Object Detection Errors

**False Positives:**
- Require minimum confidence threshold (0.6 for local, 0.8 for critical actions)
- Implement temporal filtering (object must be detected in 3 consecutive frames)
- Use point cloud segmentation to validate object size

**Occlusion Handling:**
- If object partially occluded, navigate to better viewpoint
- Use multiple camera angles (if additional cameras installed)
- Request user confirmation for low-confidence detections

### Manipulation Failures

**Grasp Failure Detection:**
```python
class GraspMonitor:
    def verify_grasp(self):
        # Method 1: Gripper force sensor
        if self.gripper.force < MIN_GRASP_FORCE:
            return False
        
        # Method 2: Visual verification
        object_still_visible = self.detect_object_at_target()
        if object_still_visible:
            return False
        
        # Method 3: Weight sensor (if available)
        if self.gripper.weight < EXPECTED_WEIGHT * 0.5:
            return False
        
        return True
```

**Recovery Strategies:**
1. Retry grasp with adjusted pose (±5cm, ±10°)
2. Try alternative grasp pose
3. Request user assistance after 3 failed attempts

### Safety System Failures

**Emergency Stop Hierarchy:**
1. **Hardware E-Stop**: Physical button on remote control (highest priority)
2. **Software E-Stop**: Collision imminent (< 30cm to obstacle)
3. **Watchdog E-Stop**: Main control loop timeout (> 500ms)

**Failure Recovery:**
- All E-stops require manual reset via voice command or remote control
- System performs self-check before resuming operation
- Log all E-stop events for analysis

## Testing Strategy

### Unit Testing

**Platform Controller (ESP32):**
- Test mecanum kinematics calculations
- Test manual override priority logic
- Test ODrive communication protocol
- Mock ESPNOW and UART interfaces

**ROS2 Nodes:**
- Test each node in isolation using ROS2 bag files
- Mock sensor inputs and verify outputs
- Test error handling and recovery behaviors

**Object Detection:**
- Test on labeled dataset of household objects
- Measure precision, recall, and inference time
- Test with various lighting conditions

### Integration Testing

**SLAM + Navigation:**
- Record test environment with RealSense cameras
- Replay and verify map quality
- Test path planning in known environment
- Verify obstacle avoidance

**Manipulation Pipeline:**
- Test grasp planning with known object models
- Verify visual servoing convergence
- Test pick-and-place sequence end-to-end

**Voice Control:**
- Test with various command phrasings
- Verify LLM task parsing accuracy
- Test error handling for ambiguous commands

### System Testing

**Full Mission Scenarios:**
1. **Toy Cleanup**: "Pick up all toys from living room and put them in toy box"
2. **Kitchen Cleanup**: "Bring dirty glasses to the kitchen"
3. **Search and Retrieve**: "Find my book and bring it to me"

**Safety Testing:**
- Test human detection and avoidance
- Test emergency stop from various states
- Test recovery from communication loss
- Test behavior on slippery surfaces

**Stress Testing:**
- Run continuous operation for 2+ hours
- Test with multiple objects in cluttered environment
- Test with dynamic obstacles (people moving around)

### Hardware-in-the-Loop Testing

**Simulation Environment:**
- **Option 1**: Gazebo with ROS2 integration
- **Option 2**: NVIDIA Isaac Sim (if hardware available)
- **Option 3**: Custom Unity/Unreal simulation

**Simulation Scope:**
- Test navigation algorithms before deploying to real robot
- Test manipulation planning with physics simulation
- Generate synthetic training data for object detection

**Note on Isaac Sim:**
- Requires RTX GPU (not available on Jetson Nano)
- Could run on separate development machine
- Export trained models/policies to Jetson for deployment

### Continuous Integration

**Automated Testing Pipeline:**
1. Code commit triggers CI build
2. Run unit tests on all ROS2 packages
3. Run integration tests with recorded sensor data
4. Deploy to test robot (if available)
5. Run smoke tests on hardware
6. Generate test report and coverage metrics

**Test Infrastructure:**
- GitHub Actions or GitLab CI for automation
- Docker containers for reproducible builds
- ROS2 bag files for regression testing

## Development Phases

### Phase 1: SLAM and Navigation (Weeks 1-3)

**Goals:**
- Get RTAB-Map running with D435 and T265
- Integrate Nav2 for autonomous navigation
- Enhance platform controller ESP32 firmware with:
  - Serial communication with Jetson
  - AMOLED display status interface
  - Autonomous/manual mode switching
  - Command arbitration logic
- Test navigation in home environment

**Deliverables:**
- James can build a map of the home
- James can navigate to commanded positions
- Manual override works reliably (already functional via ESPNOW)
- Basic safety (obstacle avoidance) functional
- Platform controller displays status on AMOLED screen
- Jetson can send velocity commands to platform controller

### Phase 2: Object Recognition (Weeks 4-6)

**Goals:**
- Train/deploy object detection model
- Integrate with SLAM for 3D object localization
- Implement object tracking across frames
- Test detection accuracy in home environment

**Deliverables:**
- James can detect common household objects
- Objects are localized in 3D map
- Detection runs at acceptable frame rate (>10 Hz)
- False positive rate < 10%

### Phase 3: Object Location Learning (Weeks 7-8)

**Goals:**
- Implement knowledge base for object locations
- Create teaching interface (voice-based)
- Test learning and recall of object locations
- Implement location selection logic

**Deliverables:**
- James can learn where objects belong
- James can recall learned locations
- Knowledge persists across reboots
- Multiple locations per object type supported

### Phase 4: Manipulation (Weeks 9-12)

**Goals:**
- Integrate AR4-MK3 with MoveIt2
- Implement grasp planning
- Implement visual servoing for precision
- Test pick-and-place operations

**Deliverables:**
- James can grasp objects reliably (>80% success)
- Visual servoing compensates for backlash
- Pick-and-place works for target objects
- Grasp failure detection and recovery works

### Phase 5: Integration and Voice Control (Weeks 13-15)

**Goals:**
- Integrate all subsystems
- Implement voice control with LLM
- Implement behavior tree for task execution
- End-to-end testing of cleanup tasks

**Deliverables:**
- James can execute voice commands
- Full cleanup tasks work end-to-end
- System is stable for extended operation
- Documentation complete

## Additional Hardware Recommendations

### Recommended Additions

1. **IMU on Gripper**: Helps detect contact and grasp success
   - Model: BNO055 or MPU9250
   - Interface: I2C to arm controller

2. **Distance Sensors on Gripper**: Fine approach control
   - Model: VL53L1X Time-of-Flight sensors (2x)
   - Range: 4m, accuracy ±1cm

3. **Additional Front/Back Cameras**: Better situational awareness
   - Model: Raspberry Pi Camera Module 3 or similar
   - Mount: 25-35cm above ground
   - Purpose: Detect low obstacles, verify object pickup

4. **Camera on Gripper**: Visual servoing and grasp verification
   - Model: Small USB camera or Raspberry Pi Camera
   - Resolution: 640x480 sufficient
   - Frame rate: 30 Hz

5. **Human Presence Sensor**: Proactive safety
   - Model: mmWave radar (e.g., LD2410)
   - Range: 6m
   - Detects stationary humans (better than camera-only)

### Hardware Upgrade Path

**If Jetson Nano insufficient:**
- **Option 1**: Jetson Orin Nano (6-8 GB)
  - 40x AI performance vs Jetson Nano
  - Better for running larger models locally
  - Cost: ~$500

- **Option 2**: Jetson Orin NX (8-16 GB)
  - 100x AI performance vs Jetson Nano
  - Can run multiple models simultaneously
  - Cost: ~$800

**Decision Criteria:**
- If SLAM + object detection + LLM inference exceeds 80% CPU/GPU on dual Jetson Nano
- If frame rate drops below 10 Hz for critical functions
- If unable to run quantized LLM locally

## Monitoring and Debugging

### Web Dashboard

**Purpose:**
Provides real-time monitoring and visualization of James's operations, accessible from any device on the local network. This addresses Requirement 10 for web-based status monitoring.

**Features:**
- Live camera feeds (D435, T265, additional cameras)
- 3D visualization of SLAM map and detected objects
- Robot state (pose, battery, mode, held object)
- Current task description and execution progress
- Task queue with status indicators
- ROS2 system status (active nodes, topic frequencies, service availability)
- System health metrics (CPU, GPU, memory, temperature)
- Manual control interface (emergency stop, mode switching)
- Object location knowledge base viewer

**Technology Stack:**
- **Backend**: ROS2 web bridge (rosbridge_suite) running on Jetson Nano 1
- **Frontend**: React + Three.js for 3D visualization
- **Communication**: WebSocket for real-time updates
- **Port**: 8080 (HTTP) and 9090 (WebSocket)
- **Deployment**: Served by nginx on Jetson Nano 1

**Implementation Details:**
```yaml
# Web server configuration
web_server:
  host: "0.0.0.0"  # Listen on all interfaces
  http_port: 8080
  websocket_port: 9090
  update_rate: 10  # Hz for real-time data
  
  # Camera streaming
  camera_streams:
    - topic: "/camera/color/image_raw"
      compression: "jpeg"
      quality: 80
    - topic: "/camera/depth/image_rect_raw"
      compression: "png"
      
  # Data topics to expose
  status_topics:
    - "/robot_state"
    - "/current_task"
    - "/task_queue"
    - "/diagnostics"
    - "/tf"
    - "/rtabmap/map"
    - "/detected_objects"
```

**Access:**
- URL: `http://james.local:8080` or `http://<jetson-ip>:8080`
- No authentication required on local network (can be added later)
- Responsive design for mobile/tablet access

### ROS2 Introspection Tools

**Command-line Tools:**
- `ros2 topic list/echo/hz` - Monitor topics
- `ros2 node list/info` - Inspect nodes
- `ros2 service list/call` - Test services
- `ros2 bag record/play` - Record and replay data

**Graphical Tools:**
- **RViz2**: Visualize SLAM map, point clouds, TF tree, robot model
- **rqt_graph**: Visualize node connections
- **PlotJuggler**: Plot time-series data (velocities, errors, etc.)

### MCP Server Integration

**Recommended MCP Servers:**

1. **Filesystem MCP**: Access logs and configuration files
2. **Shell MCP**: Execute ROS2 commands and system diagnostics
3. **Custom James MCP**: Robot-specific operations
   - Query robot state
   - Send test commands
   - Retrieve task history
   - Access knowledge base

**Custom James MCP Tools:**
```python
# Example MCP tool definitions
tools = [
    {
        "name": "get_robot_state",
        "description": "Get current robot state including pose, mode, and held object",
        "parameters": {}
    },
    {
        "name": "send_navigation_goal",
        "description": "Send navigation goal to robot",
        "parameters": {
            "x": "float",
            "y": "float",
            "theta": "float"
        }
    },
    {
        "name": "query_object_locations",
        "description": "Query learned object locations",
        "parameters": {
            "object_type": "string"
        }
    },
    {
        "name": "get_task_status",
        "description": "Get status of current or recent tasks",
        "parameters": {
            "task_id": "string (optional)"
        }
    }
]
```

### Logging Strategy

**Log Levels:**
- **DEBUG**: Detailed diagnostic information
- **INFO**: General informational messages
- **WARN**: Warning messages (degraded performance, retries)
- **ERROR**: Error messages (operation failed)
- **FATAL**: Critical errors (system shutdown required)

**Log Storage:**
- **Location**: `/var/log/james/`
- **Rotation**: Daily rotation, keep 30 days
- **Format**: JSON for structured logging
- **Remote**: Optional upload to cloud for analysis

**Key Metrics to Log:**
- Navigation: Path planning time, execution time, replanning events
- Detection: Inference time, detection count, confidence scores
- Manipulation: Grasp attempts, success rate, execution time
- Safety: E-stop events, human detections, collision warnings

## Deployment and Updates

### System Setup

**Base System:**
- Ubuntu 24.04 LTS (Noble Numbat) - supported until 2029
- ROS2 Jazzy Jalisco (native support for Ubuntu 24.04)
- CUDA 11.4 (for Jetson Nano)
- TensorRT 8.x

**Note on OS Choice:**
- Ubuntu 24.04 LTS provides longer support (until 2029 vs 2027 for 22.04)
- ROS2 Jazzy is the recommended distribution for Ubuntu 24.04
- Alternative: ROS2 Humble can also run on 24.04 if needed for compatibility
- Jetson Nano officially supports Ubuntu 20.04, but 22.04/24.04 can be installed via community images
- Verify Jetson Nano compatibility with Ubuntu 24.04 before deployment

**Installation Script:**
```bash
#!/bin/bash
# setup_james.sh

# Install ROS2 Jazzy (for Ubuntu 24.04)
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-jazzy-desktop -y

# Install dependencies
sudo apt install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rtabmap-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-moveit \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description

# Install Python packages
pip3 install \
    ultralytics \
    whisper \
    piper-tts \
    llama-cpp-python

# Clone James repository
cd ~/
git clone https://github.com/yourusername/james-robot.git
cd james-robot

# Build ROS2 workspace
colcon build --symlink-install

# Source workspace
echo "source ~/james-robot/install/setup.bash" >> ~/.bashrc
```

### Automatic Updates

**Update Mechanism:**
1. Git pull latest code from repository
2. Run automated tests on new code
3. If tests pass, build and install
4. Restart affected ROS2 nodes
5. Verify system health
6. Rollback if health check fails

**Implementation:**
```python
class AutoUpdater:
    def check_for_updates(self):
        # Pull latest from git
        result = subprocess.run(['git', 'pull'], capture_output=True)
        
        if 'Already up to date' in result.stdout:
            return False
        
        # Run tests
        if not self.run_tests():
            self.rollback()
            return False
        
        # Build
        if not self.build_workspace():
            self.rollback()
            return False
        
        # Restart nodes
        self.restart_nodes()
        
        # Health check
        if not self.health_check():
            self.rollback()
            return False
        
        return True
```

### Configuration Management

**Configuration Files:**
- `/etc/james/config.yaml` - Main configuration
- `/etc/james/object_locations.db` - Learned object locations
- `/etc/james/slam_map/` - Saved SLAM maps
- `/etc/james/models/` - ML models

**Version Control:**
- Configuration files tracked in git
- Separate branch for each deployment environment
- Secrets (API keys) stored in environment variables or secret manager

## Open Source Documentation

### Repository Structure

```
james-robot/
├── README.md
├── LICENSE
├── docs/
│   ├── architecture.md
│   ├── setup.md
│   ├── hardware_assembly.md
│   ├── software_installation.md
│   ├── calibration.md
│   ├── troubleshooting.md
│   └── diagrams/
│       ├── system_architecture.excalidraw
│       ├── electrical_wiring.excalidraw
│       └── software_architecture.excalidraw
├── platform/
│   ├── controller/          # ESP32 platform controller
│   │   ├── src/
│   │   ├── include/
│   │   └── platformio.ini
│   └── remote/              # ESP32 remote control
│       ├── src/
│       ├── include/
│       └── platformio.ini
├── ros2_ws/
│   └── src/
│       ├── james_bringup/
│       ├── james_navigation/
│       ├── james_perception/
│       ├── james_manipulation/
│       ├── james_voice/
│       └── james_description/
├── models/
│   ├── yolov8_household.pt
│   └── llama3_8b_q4.gguf
├── scripts/
│   ├── setup_james.sh
│   ├── calibrate_cameras.py
│   └── test_system.py
└── tests/
    ├── unit/
    ├── integration/
    └── system/
```

### Documentation Requirements

**Architecture Documentation:**
- System overview with diagrams
- Hardware component specifications
- Software architecture and data flow
- Communication protocols
- Safety systems

**Setup Documentation:**
- Bill of materials (BOM)
- Hardware assembly instructions with photos
- Electrical wiring diagrams
- Software installation steps
- Calibration procedures

**Usage Documentation:**
- Quick start guide
- Voice command examples
- Teaching object locations
- Troubleshooting common issues
- FAQ

**Developer Documentation:**
- Code structure and conventions
- Adding new object types
- Extending manipulation capabilities
- Contributing guidelines
- API reference

### Excalidraw Diagrams

**Required Diagrams:**
1. **System Architecture**: High-level component diagram
2. **Electrical Wiring**: ESP32, ODrive, motors, cameras, Jetson connections
3. **Software Architecture**: ROS2 nodes and topics
4. **Data Flow**: How sensor data flows through the system
5. **State Machine**: Robot behavior states and transitions

**Excalidraw Benefits:**
- Web-based, easy to edit
- Version control friendly (JSON format)
- Collaborative editing
- Export to PNG/SVG

## Summary

This design provides a comprehensive architecture for James, balancing complexity with practicality. The modular ROS2-based approach allows incremental development and testing, starting with SLAM and progressing through object recognition, learning, and manipulation. The dual Jetson Nano setup should be sufficient for initial development, with a clear upgrade path if needed. Safety is prioritized through multiple layers of protection, and the system is designed for both autonomous operation and manual override.
