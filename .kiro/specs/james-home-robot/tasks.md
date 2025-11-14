# Implementation Plan

- [x] 1. Project Setup and Infrastructure





  - Create GitHub repository with proper structure (docs/, platform/, ros2_ws/, models/, scripts/, tests/)
  - Set up .gitignore for ROS2, Python, and Arduino projects
  - Create README.md with project overview and quick start guide
  - Set up CI/CD pipeline for automated testing
  - _Requirements: 11.1, 11.2_

- [ ] 2. Platform Controller Firmware Enhancement
- [ ] 2.1 Add Serial Communication with Jetson
  - Implement serial command parser to receive joystick-format commands from Jetson
  - Add command arbitration logic (ESPNOW manual control takes priority over serial autonomous)
  - Implement status reporting back to Jetson (commanded speeds, mode, timestamp)
  - _Requirements: 1.1, 1.2_

- [ ] 2.2 Implement AMOLED Display Interface
  - Initialize LilyGO AMOLED display using manufacturer examples
  - Create status display showing: current velocities, mode (manual/autonomous), connection status
  - Implement display update loop (10 Hz refresh rate)
  - _Requirements: 10.2, 10.3_

- [ ] 2.3 Add Touchscreen Emergency Stop
  - Implement touchscreen input handling
  - Create emergency stop button on display
  - Add mode switching button (manual/autonomous)
  - _Requirements: 5.5, 10.2_

- [ ]* 2.4 Write unit tests for platform controller
  - Test mecanum kinematics calculations with known inputs
  - Test command arbitration logic (manual vs autonomous priority)
  - Test serial communication protocol
  - _Requirements: 7.2_

- [ ] 3. Remote Control Firmware Enhancement
- [x] 3.1 Enhance Remote Control Display





  - Add AMOLED display showing joystick values in real-time
  - Display connection status and signal strength
  - Show battery level if available
  - _Requirements: 10.2_

- [ ]* 3.2 Add remote control configuration interface
  - Implement touchscreen menu for sensitivity adjustment
  - Add dead zone configuration
  - Store settings in EEPROM
  - _Requirements: 7.2_

- [ ] 4. ROS2 Workspace Setup
- [ ] 4.1 Create ROS2 package structure
  - Create james_bringup package for launch files
  - Create james_description package for URDF and meshes
  - Create james_navigation package for Nav2 configuration
  - Create james_perception package for cameras and object detection
  - Create james_manipulation package for arm control
  - Create james_voice package for voice control
  - Create james_platform package for platform controller interface
  - _Requirements: 7.1, 11.2_

- [ ] 4.2 Create URDF model for James
  - Model platform base (45x56cm footprint, 17.5cm wheels)
  - Import AR4-MK3 URDF from Annin Robotics
  - Position arm at 36cm height on platform
  - Add camera mounts at 120cm height, 35cm from front
  - Define TF tree: base_link → arm_base → camera_mount
  - _Requirements: 7.1, 11.1_

- [ ]* 4.3 Create visualization launch file
  - Launch RViz2 with James URDF model
  - Configure joint state publisher for testing
  - Verify all transforms are correct
  - _Requirements: 7.4_

- [ ] 5. Platform Controller ROS2 Integration
- [ ] 5.1 Implement james_platform_controller node
  - Subscribe to /cmd_vel (geometry_msgs/Twist)
  - Convert cmd_vel (vx, vy, omega) to joystick format (-255 to 255)
  - Send commands to ESP32 via serial port
  - Publish /platform/status with commanded speeds and mode
  - _Requirements: 1.2, 7.1_

- [ ] 5.2 Implement command effectiveness monitor
  - Subscribe to /t265/odom for visual odometry
  - Compare commanded velocities with actual T265 displacement
  - Detect poor traction (effectiveness < 50%)
  - Publish warnings and adjust velocity limits dynamically
  - _Requirements: 1.3, 5.1_

- [ ]* 5.3 Write integration tests for platform control
  - Test cmd_vel to joystick conversion
  - Test serial communication with mock ESP32
  - Test command effectiveness monitoring logic
  - _Requirements: 7.2_

- [ ] 6. Camera Integration
- [ ] 6.1 Set up RealSense D435 camera
  - Install librealsense2 and ROS2 wrapper
  - Configure camera parameters (resolution, frame rate, depth settings)
  - Launch camera node and verify RGB and depth streams
  - Calibrate camera intrinsics if needed
  - _Requirements: 2.1, 3.1, 11.2_

- [ ] 6.2 Set up RealSense T265 tracking camera
  - Install T265 ROS2 wrapper
  - Configure odometry output
  - Verify pose estimation accuracy
  - Integrate with TF tree (publish odom → base_link transform)
  - _Requirements: 2.2, 11.2_

- [ ]* 6.3 Create camera calibration tools
  - Write script to capture calibration images
  - Perform stereo calibration for D435
  - Verify depth accuracy at various distances
  - _Requirements: 11.2_

- [ ] 7. SLAM Implementation
- [ ] 7.1 Configure RTAB-Map for James
  - Set up RTAB-Map node with D435 RGB-D input
  - Integrate T265 odometry as prior
  - Configure memory management for Jetson Nano
  - Enable loop closure detection
  - Tune parameters for indoor environments
  - _Requirements: 2.1, 2.2, 2.3, 2.4_

- [ ] 7.2 Create SLAM launch file
  - Launch RTAB-Map with camera inputs
  - Launch RViz2 with map visualization
  - Configure map saving and loading
  - _Requirements: 2.4, 7.1_

- [ ] 7.3 Test SLAM in home environment
  - Drive James manually through home to build initial map
  - Verify loop closure works correctly
  - Test relocalization after restart
  - Save map for navigation testing
  - _Requirements: 2.1, 2.3, 2.4_

- [ ]* 7.4 Implement map quality monitoring
  - Monitor loop closure success rate
  - Detect map degradation
  - Trigger map optimization when needed
  - _Requirements: 2.4_

- [ ] 8. Navigation System
- [ ] 8.1 Configure Nav2 stack
  - Set up global costmap with RTAB-Map
  - Configure local costmap with D435 depth data
  - Set robot footprint (45x56cm)
  - Configure inflation radius (0.40m) for door navigation
  - Set min_obstacle_height to 0.10m for traversable obstacles
  - Configure velocity limits (max 0.3 m/s)
  - _Requirements: 3.3, 5.1, 5.2, 5.3_

- [ ] 8.2 Implement navigation behaviors
  - Configure DWB local planner for mecanum wheels
  - Set up recovery behaviors (spin, backup, wait)
  - Configure goal tolerance parameters
  - _Requirements: 3.3, 5.4_

- [ ] 8.3 Create navigation launch file
  - Launch Nav2 stack with James configuration
  - Launch RViz2 with navigation visualization
  - Set up 2D goal pose tool for testing
  - _Requirements: 3.3, 7.1_

- [ ] 8.4 Test navigation in mapped environment
  - Send navigation goals via RViz2
  - Verify obstacle avoidance works correctly
  - Test navigation through 55-60cm doors
  - Test driving over yoga mats and carpets
  - Verify manual override interrupts autonomous navigation
  - _Requirements: 1.1, 3.3, 5.1, 5.2, 5.3_

- [ ]* 8.5 Implement navigation monitoring and diagnostics
  - Monitor path planning success rate
  - Detect navigation failures and stuck conditions
  - Log navigation metrics for analysis
  - _Requirements: 7.4, 7.5_

- [ ] 9. Object Detection System
- [ ] 9.1 Set up YOLOv8 for household objects
  - Install ultralytics package and dependencies
  - Download pre-trained YOLOv8n model
  - Convert model to TensorRT for Jetson optimization
  - Test inference speed on Jetson Nano 2
  - _Requirements: 3.1, 3.5_

- [ ] 9.2 Implement object detection node
  - Subscribe to /camera/color/image_raw from D435
  - Subscribe to /camera/aligned_depth_to_color/image_raw
  - Run YOLO inference on RGB images
  - Extract depth for each detected object
  - Project 2D detections to 3D using camera intrinsics
  - Publish DetectedObjectArray with 3D poses
  - _Requirements: 3.1, 3.2, 3.4, 3.5_

- [ ] 9.3 Implement object tracking
  - Track objects across frames using IOU matching
  - Filter detections (require 3 consecutive frames for confirmation)
  - Maintain object database with unique IDs
  - _Requirements: 3.1, 3.4_

- [ ] 9.4 Test object detection in home environment
  - Test detection of common objects (toys, glasses, books)
  - Measure detection accuracy and false positive rate
  - Verify 3D position accuracy using depth data
  - _Requirements: 3.1, 3.5_

- [ ]* 9.5 Implement cloud-based object recognition fallback
  - Set up API client for OpenAI Vision or Google Cloud Vision
  - Call cloud API when local confidence < 0.7
  - Cache results to avoid repeated API calls
  - _Requirements: 3.2_

- [ ] 10. Object Location Learning System
- [ ] 10.1 Create object location database
  - Design SQLite schema for object types and locations
  - Implement database access layer
  - Add methods for adding, querying, and updating locations
  - _Requirements: 9.2, 9.3_

- [ ] 10.2 Implement location learning node
  - Subscribe to detected objects and robot pose
  - Provide service to teach new object locations
  - Store object type, location name, 3D pose, and priority
  - Save database persistently
  - _Requirements: 9.1, 9.2, 9.3, 9.5_

- [ ] 10.3 Implement location query service
  - Provide service to query locations for object types
  - Return locations sorted by priority
  - Handle multiple locations per object type
  - _Requirements: 9.4_

- [ ]* 10.4 Create location learning test scenarios
  - Test teaching toy box location
  - Test teaching kitchen cabinet location
  - Test querying locations for different object types
  - Verify persistence across restarts
  - _Requirements: 9.3, 9.5_

- [ ] 11. AR4-MK3 Arm Integration
- [ ] 11.1 Set up AR4-MK3 ROS2 package
  - Clone AR4-MK3 ROS2 repository from Annin Robotics
  - Update URDF to reflect JMC closed-loop motors on first 3 joints
  - Configure joint controllers
  - Test arm movement with joint trajectory controller
  - _Requirements: 4.1, 4.2, 11.2_

- [ ] 11.2 Configure MoveIt2 for AR4-MK3
  - Generate MoveIt2 configuration using setup assistant
  - Configure OMPL planner with RRTConnect
  - Set up planning scene with RTAB-Map point cloud
  - Configure collision checking
  - _Requirements: 4.1, 4.4_

- [ ] 11.3 Implement custom gripper controller
  - Create ROS2 controller for custom gripper
  - Implement open/close commands
  - Add force feedback if available
  - Configure grip strength per object type
  - _Requirements: 4.2, 4.4_

- [ ] 11.4 Test arm motion planning
  - Send test poses to MoveIt2
  - Verify collision avoidance with environment
  - Test joint limits and singularity handling
  - Measure planning and execution time
  - _Requirements: 4.1, 4.4_

- [ ]* 11.5 Implement arm diagnostics and monitoring
  - Monitor joint positions and velocities
  - Detect arm collisions and excessive forces
  - Log arm movements for analysis
  - _Requirements: 7.4, 7.5_

- [ ] 12. Grasp Planning and Visual Servoing
- [ ] 12.1 Implement grasp pose generation
  - Segment object point cloud from scene
  - Compute object principal axes using PCA
  - Generate candidate grasp poses along axes
  - Score grasps based on reachability and stability
  - _Requirements: 4.3_

- [ ] 12.2 Implement visual servoing node
  - Subscribe to camera feed and detected objects
  - Compute image-space error between gripper and target
  - Convert image error to Cartesian correction
  - Publish correction commands to arm controller
  - Implement convergence detection
  - _Requirements: 4.2_

- [ ] 12.3 Implement pick-and-place pipeline
  - Navigate to object location
  - Detect and localize target object
  - Plan grasp approach trajectory
  - Execute visual servoing for precision
  - Close gripper and verify grasp
  - Plan transport trajectory
  - Navigate to destination
  - Open gripper to release object
  - _Requirements: 4.2, 4.3, 4.4, 4.5_

- [ ] 12.4 Test pick-and-place with common objects
  - Test picking toys from floor
  - Test picking glasses from table
  - Test picking books from various surfaces
  - Measure grasp success rate (target >80%)
  - _Requirements: 4.2, 4.5_

- [ ]* 12.5 Implement grasp failure recovery
  - Detect grasp failures using force/visual feedback
  - Retry with adjusted pose (±5cm, ±10°)
  - Try alternative grasp poses
  - Request user assistance after 3 failed attempts
  - _Requirements: 4.5_

- [ ] 13. Safety System Implementation
- [ ] 13.1 Implement human detection
  - Implement depth-based human detection (1.2-2.0m tall, 0.3-0.6m wide)
  - Run at 15 Hz for real-time response
  - Publish detected humans with 3D positions
  - _Requirements: 5.2_

- [ ] 13.2 Implement safety zone monitoring
  - Define safety zones: critical (0.3m), warning (0.5m), caution (1.0m)
  - Monitor distance to detected humans
  - Publish emergency stop if human within 0.3m
  - Reduce velocity if human within 0.5m
  - _Requirements: 5.1, 5.2, 5.3_

- [ ] 13.3 Implement emergency stop system
  - Handle emergency stop from remote control
  - Handle emergency stop from platform controller touchscreen
  - Handle emergency stop from safety monitoring
  - Require manual reset after emergency stop
  - _Requirements: 5.5_

- [ ] 13.4 Test safety system
  - Test human detection accuracy
  - Test emergency stop from all sources
  - Test velocity reduction in warning zones
  - Verify manual override always works
  - _Requirements: 5.1, 5.2, 5.3, 5.4, 5.5_

- [ ]* 13.5 Implement safety system diagnostics
  - Log all emergency stop events
  - Monitor safety system health
  - Detect sensor failures
  - _Requirements: 7.4, 7.5_

- [ ] 14. Voice Control and LLM Integration
- [ ] 14.1 Set up speech-to-text
  - Install Whisper.cpp with quantized model
  - Implement audio capture from microphone
  - Test speech recognition accuracy
  - Measure latency (target < 2 seconds)
  - _Requirements: 6.1_

- [ ] 14.2 Set up LLM for task planning
  - Install llama.cpp with Llama 3.1 8B quantized model
  - Create prompt template for task parsing
  - Define available actions (navigate_to, pick_object, place_object, search_object)
  - Test LLM task parsing with example commands
  - _Requirements: 6.2, 6.3_

- [ ] 14.3 Implement voice control node
  - Capture voice input continuously
  - Send audio to speech-to-text
  - Send transcript to LLM for task parsing
  - Parse LLM response into task steps
  - Publish task to behavior tree
  - _Requirements: 6.1, 6.2, 6.5_

- [ ] 14.4 Implement text-to-speech feedback
  - Install piper-tts for local TTS
  - Implement speech synthesis for confirmations
  - Provide feedback for task understanding
  - Request clarification for ambiguous commands
  - _Requirements: 6.4_

- [ ] 14.5 Test voice control with common commands
  - Test "Pick up toys from living room and put them in toy box"
  - Test "Bring dirty glasses to the kitchen"
  - Test "Find my book and bring it to me"
  - Verify task parsing accuracy
  - _Requirements: 6.1, 6.2, 6.3, 6.5_

- [ ]* 14.6 Implement cloud LLM fallback
  - Set up API client for OpenAI GPT-4 or Claude
  - Use cloud LLM for complex reasoning
  - Fall back to cloud if local LLM fails
  - _Requirements: 6.2_

- [ ] 15. Behavior Tree and Task Execution
- [ ] 15.1 Implement behavior tree framework
  - Set up BehaviorTree.CPP or py_trees
  - Define behavior tree nodes for: navigate, detect_object, pick_object, place_object
  - Implement task queue management
  - _Requirements: 6.2, 7.1_

- [ ] 15.2 Implement task execution node
  - Subscribe to task commands from voice control
  - Execute behavior tree for each task
  - Monitor task progress and status
  - Handle task failures and recovery
  - Publish task status updates
  - _Requirements: 6.5, 7.1, 10.3_

- [ ] 15.3 Test end-to-end task execution
  - Test complete toy cleanup task
  - Test complete glass retrieval task
  - Test search and retrieve task
  - Measure task success rate
  - _Requirements: 6.2, 6.3, 6.5_

- [ ]* 15.4 Implement task monitoring and logging
  - Log all task executions with timestamps
  - Monitor task success/failure rates
  - Detect common failure patterns
  - _Requirements: 7.4, 7.5_

- [ ] 16. Web Dashboard
- [ ] 16.1 Set up ROS2 web bridge
  - Install rosbridge_suite for ROS2
  - Configure WebSocket server on port 9090
  - Test connection from web browser
  - _Requirements: 10.1, 10.6_

- [ ] 16.2 Create web dashboard frontend
  - Set up React project with Three.js
  - Create layout with camera feeds, map view, status panels
  - Implement WebSocket connection to rosbridge
  - _Requirements: 10.1, 10.2, 10.3, 10.4, 10.5_

- [ ] 16.3 Implement camera feed streaming
  - Subscribe to camera topics via rosbridge
  - Display D435 RGB feed
  - Display D435 depth feed (colorized)
  - Implement JPEG compression for bandwidth
  - _Requirements: 10.4_

- [ ] 16.4 Implement 3D map visualization
  - Subscribe to /rtabmap/map and /rtabmap/cloud_map
  - Render occupancy grid in Three.js
  - Display robot position and orientation
  - Overlay detected objects on map
  - _Requirements: 10.2, 10.5_

- [ ] 16.5 Implement status panels
  - Display current task and progress
  - Display task queue
  - Display ROS2 node status
  - Display system health (CPU, GPU, memory, temperature)
  - Display robot mode (manual/autonomous)
  - _Requirements: 10.2, 10.3_

- [ ] 16.6 Deploy web dashboard
  - Set up nginx on Jetson Nano 1
  - Configure to serve on port 8080
  - Test access from mobile devices
  - _Requirements: 10.1, 10.6_

- [ ]* 16.7 Add manual control interface to dashboard
  - Implement emergency stop button
  - Add mode switching controls
  - Add manual navigation goal setting
  - _Requirements: 10.2_

- [ ] 17. System Integration and Testing
- [ ] 17.1 Create main launch file
  - Launch all ROS2 nodes in correct order
  - Set up node dependencies and parameters
  - Configure logging and diagnostics
  - _Requirements: 7.1, 7.3_

- [ ] 17.2 Implement system health monitoring
  - Monitor CPU, GPU, memory usage on both Jetsons
  - Monitor temperature and throttling
  - Monitor ROS2 node health
  - Publish system diagnostics
  - _Requirements: 7.4, 7.5_

- [ ] 17.3 Test complete system integration
  - Start all nodes and verify connectivity
  - Test manual control via remote
  - Test autonomous navigation
  - Test object detection and tracking
  - Test pick-and-place operations
  - Test voice control
  - Test web dashboard
  - _Requirements: 7.1, 7.2, 7.3, 7.4_

- [ ] 17.4 Perform stress testing
  - Run continuous operation for 2+ hours
  - Test with multiple objects in cluttered environment
  - Test with dynamic obstacles (people moving)
  - Monitor for memory leaks and performance degradation
  - _Requirements: 7.3, 7.5_

- [ ]* 17.5 Implement automatic software updates
  - Create update script that pulls from git
  - Run automated tests on new code
  - Build and install if tests pass
  - Restart affected nodes
  - Implement rollback on failure
  - _Requirements: 7.3_

- [ ] 18. Documentation and Open Source Preparation
- [ ] 18.1 Create architecture diagrams
  - Create system architecture diagram in Excalidraw
  - Create electrical wiring diagram in Excalidraw
  - Create software architecture diagram in Excalidraw
  - Create data flow diagram in Excalidraw
  - Export diagrams to PNG for documentation
  - _Requirements: 11.1, 11.5_

- [ ] 18.2 Write hardware assembly documentation
  - Document bill of materials (BOM) with part numbers and sources
  - Write step-by-step assembly instructions with photos
  - Document electrical connections and wiring
  - Create troubleshooting guide for hardware issues
  - _Requirements: 11.1, 11.2_

- [ ] 18.3 Write software installation documentation
  - Document Jetson Nano setup (Ubuntu 24.04, ROS2 Jazzy)
  - Document ESP32 firmware flashing procedure
  - Document ROS2 workspace build and installation
  - Document camera calibration procedure
  - Create quick start guide
  - _Requirements: 11.2_

- [ ] 18.4 Write usage documentation
  - Document voice command examples
  - Document teaching object locations
  - Document web dashboard usage
  - Create FAQ for common questions
  - Document troubleshooting procedures
  - _Requirements: 11.1_

- [ ] 18.5 Prepare GitHub repository for public release
  - Review all code for sensitive information
  - Add LICENSE file (choose appropriate open source license)
  - Update README.md with comprehensive project description
  - Add CONTRIBUTING.md with contribution guidelines
  - Add CODE_OF_CONDUCT.md
  - Create GitHub Issues templates
  - Set up GitHub Actions for CI/CD
  - _Requirements: 11.1, 11.2, 11.3, 11.4, 11.5_

- [ ]* 18.6 Create demo videos and tutorials
  - Record video of James performing toy cleanup
  - Record video of James performing glass retrieval
  - Create tutorial video for setup and installation
  - Upload to YouTube and link from README
  - _Requirements: 11.1_

- [ ] 19. MCP Server Integration (Optional)
- [ ]* 19.1 Set up filesystem MCP server
  - Install and configure filesystem MCP server
  - Configure access to James logs and configuration files
  - Test file access from MCP client
  - _Requirements: 7.4_

- [ ]* 19.2 Set up shell MCP server
  - Install and configure shell MCP server
  - Configure safe command execution
  - Test ROS2 command execution
  - _Requirements: 7.4_

- [ ]* 19.3 Create custom James MCP server
  - Implement MCP server for James-specific operations
  - Add tools: get_robot_state, send_navigation_goal, query_object_locations, get_task_status
  - Test MCP tools from client
  - Document MCP server usage
  - _Requirements: 7.4_
