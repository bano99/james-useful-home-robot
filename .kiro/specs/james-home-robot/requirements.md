# Requirements Document

## Introduction

James is an autonomous home cleanup robot designed to perform simple tasks such as picking up objects from floors and tables and returning them to their proper locations. The system integrates a mobile mecanum-wheel platform with an AR4-MK3 robotic arm, stereoscopic cameras, and ROS2-based software architecture. James must operate autonomously while maintaining safety awareness and accepting manual override commands.

## Glossary

- **James**: The autonomous home cleanup robot system
- **Platform Controller**: ESP32 microcontroller managing the mecanum wheel platform
- **Remote Control**: Manual control device using ESP32 with ESPNOW communication
- **Main Brain**: Primary computing system (Jetson Nano units) running ROS2 and decision-making logic
- **AR4-MK3 Arm**: 6-DOF robotic manipulator from Annin Robotics with custom gripper
- **ODrive Controller**: Motor controller managing individual wheel motors (2 motors per ODrive)
- **RealSense D435**: Intel stereoscopic camera for object recognition and depth perception
- **RealSense T265**: Intel tracking camera providing native odometry data
- **SLAM**: Simultaneous Localization and Mapping
- **ESPNOW**: ESP32 wireless communication protocol
- **Mecanum Wheels**: Omnidirectional wheels enabling lateral movement
- **LLM**: Large Language Model for natural language processing

## Requirements

### Requirement 1: Platform Mobility and Control

**User Story:** As a robot operator, I want James to navigate autonomously through the home environment while allowing manual override, so that the robot can reach objects safely and respond to emergency situations.

#### Acceptance Criteria

1. WHEN the Remote Control sends steering commands via ESPNOW, THE Platform Controller SHALL execute the manual commands with priority over autonomous navigation commands.
2. WHEN the Main Brain sends autonomous navigation commands to the Platform Controller, THE Platform Controller SHALL execute these commands only if no manual override is active.
3. WHILE operating on slippery surfaces, THE Main Brain SHALL use visual feedback from stereoscopic cameras to compensate for wheel slip and verify actual platform displacement.
4. THE Platform Controller SHALL communicate with two ODrive Controllers via UART interface to control all four mecanum wheels independently.
5. WHEN receiving movement commands, THE Platform Controller SHALL translate them into appropriate mecanum wheel velocities for omnidirectional movement.

### Requirement 2: SLAM and Navigation

**User Story:** As a robot operator, I want James to build and maintain a map of the home environment, so that the robot can navigate autonomously and locate objects.

#### Acceptance Criteria

1. THE Main Brain SHALL perform SLAM using data from the RealSense D435 and RealSense T265 cameras to create and update a 3D map of the environment.
2. THE Main Brain SHALL integrate native tracking data from the RealSense T265 to improve localization accuracy.
3. WHEN navigating, THE Main Brain SHALL use the SLAM map to plan collision-free paths to target locations.
4. THE Main Brain SHALL update the SLAM map continuously during operation to reflect environmental changes.
5. WHILE the platform is 110cm tall at camera height, THE Main Brain SHALL account for this sensor position when mapping floor-level obstacles.

### Requirement 3: Object Recognition and Classification

**User Story:** As a robot operator, I want James to identify and classify objects in the environment, so that the robot can determine which items are displaced and where they belong.

#### Acceptance Criteria

1. THE Main Brain SHALL process RealSense D435 camera data to detect and classify objects within the environment.
2. WHERE cloud-based object recognition services are required, THE Main Brain SHALL send image data via WiFi API calls and receive classification results.
3. THE Main Brain SHALL maintain a database of object types and their designated storage locations.
4. WHEN an object is detected, THE Main Brain SHALL determine if the object is displaced from its designated location.
5. THE Main Brain SHALL generate 3D position coordinates for detected objects using depth data from the RealSense D435.

### Requirement 4: Robotic Arm Manipulation

**User Story:** As a robot operator, I want James to grasp and manipulate objects using the AR4-MK3 arm, so that the robot can pick up items and place them in designated locations.

#### Acceptance Criteria

1. THE Main Brain SHALL control the AR4-MK3 Arm via ROS2 interfaces to position the gripper at target object locations.
2. WHILE the AR4-MK3 Arm has mechanical backlash, THE Main Brain SHALL use visual feedback from cameras to verify and correct gripper positioning.
3. WHEN approaching an object, THE Main Brain SHALL calculate appropriate grasp poses based on object geometry and gripper capabilities.
4. THE Main Brain SHALL command the custom gripper to close with appropriate force to secure objects without damage.
5. WHEN transporting objects, THE Main Brain SHALL maintain stable arm configurations to prevent dropping items.

### Requirement 5: Safety and Collision Avoidance

**User Story:** As a home occupant, I want James to operate safely around people, furniture, and fragile objects, so that the robot does not cause injury or property damage.

#### Acceptance Criteria

1. WHEN obstacles are detected within a safety threshold distance, THE Main Brain SHALL stop platform movement and replan the navigation path.
2. THE Main Brain SHALL detect human presence in the environment and maintain a minimum safe distance of 50cm from detected persons.
3. WHILE the combined platform and arm weight is 50-60kg, THE Main Brain SHALL limit maximum velocity to prevent damage during potential collisions.
4. WHEN operating the arm, THE Main Brain SHALL verify that the planned trajectory does not intersect with mapped obstacles or detected humans.
5. IF the Remote Control sends an emergency stop command, THEN THE Platform Controller SHALL immediately halt all platform motors.

### Requirement 6: Voice Control and Task Instructions

**User Story:** As a robot operator, I want to instruct James using natural language voice commands, so that I can easily assign cleanup tasks without technical interfaces.

#### Acceptance Criteria

1. THE Main Brain SHALL integrate voice recognition capabilities to capture spoken commands from operators.
2. WHEN a voice command is received, THE Main Brain SHALL process the command using an LLM to extract task parameters and intent.
3. THE Main Brain SHALL support task instructions such as "bring the glass to the kitchen" and "pick up toys from the floor."
4. WHEN a task instruction is ambiguous, THE Main Brain SHALL request clarification from the operator via voice synthesis.
5. THE Main Brain SHALL confirm task understanding and execution plan to the operator before beginning autonomous operation.

### Requirement 7: System Architecture and Modularity

**User Story:** As a system developer, I want James to have a modular architecture with independently testable components, so that I can develop, update, and debug subsystems efficiently.

#### Acceptance Criteria

1. THE Main Brain SHALL implement a ROS2-based architecture with separate nodes for SLAM, object recognition, manipulation planning, and navigation.
2. EACH ROS2 node SHALL expose standard interfaces allowing independent testing and validation.
3. THE Main Brain SHALL support automatic software updates with verification mechanisms that do not require manual intervention.
4. THE Main Brain SHALL provide diagnostic interfaces for monitoring system state and verifying robot behavior during operation.
5. THE Main Brain SHALL log all sensor data, commands, and state transitions for debugging and performance analysis.

### Requirement 8: Hardware Configuration and Compute Resources

**User Story:** As a system architect, I want to determine the optimal compute hardware configuration, so that James can process all required tasks with acceptable latency and cost.

#### Acceptance Criteria

1. THE Main Brain SHALL evaluate whether two Jetson Nano units provide sufficient compute resources for SLAM, object recognition, manipulation planning, and navigation.
2. WHERE local compute resources are insufficient, THE Main Brain SHALL offload specific tasks to cloud services via WiFi API calls.
3. THE Main Brain SHALL prioritize local processing for real-time safety-critical functions such as collision avoidance and emergency stops.
4. WHEN compute performance is inadequate, THE system architecture SHALL document requirements for upgraded hardware such as Jetson Orin Nano or Jetson Orin NX.
5. THE Main Brain SHALL minimize cloud service dependencies to reduce long-term operational costs.

### Requirement 9: Learning and Object Location Memory

**User Story:** As a robot operator, I want to teach James where different types of objects belong, so that the robot can autonomously return displaced items to their correct locations.

#### Acceptance Criteria

1. THE Main Brain SHALL provide an interface for operators to designate storage locations for specific object types.
2. WHEN an operator places an object in a location and confirms it via voice command, THE Main Brain SHALL associate that object type with the designated location.
3. THE Main Brain SHALL store object-location associations persistently across system restarts.
4. WHEN multiple storage locations exist for an object type, THE Main Brain SHALL select the nearest available location.
5. THE Main Brain SHALL update object-location associations when operators provide corrective feedback.

### Requirement 10: Web-Based Status Monitoring

**User Story:** As a robot operator, I want to view James's current status and activities through a web interface, so that I can monitor the robot's operations from any device on the network.

#### Acceptance Criteria

1. THE Main Brain SHALL run a web server accessible via local network that displays real-time robot status.
2. THE web interface SHALL display current ROS2 system status including active nodes, topics, and services.
3. THE web interface SHALL display James's current task, execution progress, and task queue.
4. THE web interface SHALL display live camera feeds from the RealSense cameras.
5. THE web interface SHALL display the SLAM map with robot position and detected objects overlaid.
6. WHEN a client connects to the web interface, THE Main Brain SHALL provide real-time updates without requiring page refresh.

### Requirement 11: Development and Documentation

**User Story:** As an open-source contributor, I want comprehensive architecture documentation and setup procedures, so that I can understand, replicate, and contribute to the James robot project.

#### Acceptance Criteria

1. THE project repository SHALL include architecture diagrams documenting hardware connections, software components, and data flows.
2. THE project repository SHALL include setup documentation for configuring the Platform Controller, ODrive Controllers, cameras, and Main Brain.
3. THE project repository SHALL include source code for the ESP32 Platform Controller under /platform/controller directory.
4. THE project repository SHALL include source code for the ESP32 Remote Control under /platform/remote directory.
5. THE project repository SHALL use Excalidraw format for system diagrams to enable collaborative editing.
