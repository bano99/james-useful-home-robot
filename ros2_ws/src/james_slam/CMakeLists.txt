cmake_minimum_required(VERSION 3.5)
project(james_slam)

# Find dependencies
find_package(ament_cmake REQUIRED)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/tf_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
