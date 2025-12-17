# Task 3 Testing Guide - Windows

## Prerequisites
- ROS2 Jazzy installed on Windows
- Python 3.8+ with pip
- Serial library: `pip install pyserial`

## Step 1: Build the Package

### Commands to Execute:
```bash
cd ros2_ws
colcon build --packages-select james_manipulation
```

### Expected Output (Success):
```
Starting >>> james_manipulation
Finished <<< james_manipulation [2.34s]

Summary: 1 package finished [2.45s]
```

### Expected Output (Failure Examples):
```
# Missing dependencies:
--- stderr: james_manipulation
ModuleNotFoundError: No module named 'serial'

# Syntax errors:
--- stderr: james_manipulation  
  File "james_manipulation/platform_serial_bridge.py", line 45
    syntax error here
                    ^
SyntaxError: invalid syntax

# Missing files:
--- stderr: james_manipulation
FileNotFoundError: [Errno 2] No such file or directory: 'config/arm_cartesian_params.yaml'
```

## Step 2: Source the Workspace

### Commands to Execute:
```bash
# Windows Command Prompt
call install\setup.bat

# Windows PowerShell  
install\setup.ps1

# Git Bash / WSL
source install/setup.bash
```

### Expected Output:
- No output = success
- Error messages indicate sourcing problems

## Step 3: Verify Package Installation

### Commands to Execute:
```bash
ros2 pkg list | findstr james_manipulation
```

### Expected Output:
```
james_manipulation
```

## Step 4: Check Executables

### Commands to Execute:
```bash
ros2 run james_manipulation platform_serial_bridge --help
ros2 run james_manipulation arm_cartesian_controller --help  
ros2 run james_manipulation teensy_serial_bridge --help
```

### Expected Output:
```
usage: platform_serial_bridge [-h] [--ros-args ...]
```

### Expected Errors (if not built correctly):
```
Package 'james_manipulation' not found
No executable found
```

## Step 5: Test Launch File

### Commands to Execute:
```bash
ros2 launch james_manipulation arm_cartesian_control.launch.py --help
```

### Expected Output:
```
usage: arm_cartesian_control.launch.py [-h] [--show-args] [--show-all-subprocesses-output] ...

Launch file for James Robot Arm Cartesian Control

Arguments (pass arguments as '<name>:=<value>'):
    'config_file':
        Path to the configuration file
        (default: [path]/james_manipulation/config/arm_cartesian_params.yaml)
    'use_sim_time':
        Use simulation time
        (default: 'false')
    'log_level':
        Log level for all nodes
        (default: 'info')
```

## Step 6: Test Individual Nodes (Dry Run)

### Test Platform Serial Bridge:
```bash
ros2 run james_manipulation platform_serial_bridge
```

### Expected Output (Success):
```
[INFO] [platform_serial_bridge]: Platform Serial Bridge started on /dev/ttyUSB0
[WARN] [platform_serial_bridge]: Failed to connect to /dev/ttyUSB0: [Errno 2] No such file or directory: '/dev/ttyUSB0'
```
**Note:** Warning is expected on Windows since `/dev/ttyUSB0` doesn't exist. The node should start and keep trying to connect.

### Test Arm Cartesian Controller:
```bash
ros2 run james_manipulation arm_cartesian_controller
```

### Expected Output (Success):
```
[INFO] [arm_cartesian_controller]: Arm Cartesian Controller started
[INFO] [arm_cartesian_controller]: Velocity scale: 0.001 m/s per unit
[INFO] [arm_cartesian_controller]: Rotation scale: 0.01 rad/s per unit
```

### Test Teensy Serial Bridge:
```bash
ros2 run james_manipulation teensy_serial_bridge
```

### Expected Output (Success):
```
[INFO] [teensy_serial_bridge]: Teensy Serial Bridge started on /dev/ttyACM0
[WARN] [teensy_serial_bridge]: Failed to connect to /dev/ttyACM0: [Errno 2] No such file or directory: '/dev/ttyACM0'
```

## Step 7: Test Launch File (Dry Run)

### Commands to Execute:
```bash
ros2 launch james_manipulation arm_cartesian_control.launch.py
```

### Expected Output (Success):
```
[INFO] [launch]: All log files can be found below [log directory]
[INFO] [launch]: Default logging verbosity is set to INFO
[INFO] [platform_serial_bridge-1]: process started with pid [1234]
[INFO] [arm_cartesian_controller-2]: process started with pid [1235]  
[INFO] [teensy_serial_bridge-3]: process started with pid [1236]
[INFO] [platform_serial_bridge]: Platform Serial Bridge started on /dev/ttyUSB0
[INFO] [arm_cartesian_controller]: Arm Cartesian Controller started
[INFO] [teensy_serial_bridge]: Teensy Serial Bridge started on /dev/ttyACM0
```

**Press Ctrl+C to stop all nodes**

## Step 8: Verify ROS2 Topics

### While launch file is running, open another terminal:
```bash
ros2 topic list
```

### Expected Topics:
```
/arm/joint_commands
/arm/manual_cartesian_cmd
/arm/status
/joint_states
/parameter_events
/platform_bridge/status
/rosout
/teensy_bridge/status
```

### Test Topic Communication:
```bash
# Check if topics are publishing (should show data rate)
ros2 topic hz /arm/status
ros2 topic hz /platform_bridge/status
ros2 topic hz /teensy_bridge/status

# Check topic content
ros2 topic echo /arm/status --once
ros2 topic echo /platform_bridge/status --once
```

## Step 9: Test Configuration Loading

### Commands to Execute:
```bash
ros2 param list | findstr james_manipulation
```

### Expected Output:
```
/arm_cartesian_controller.control_rate
/arm_cartesian_controller.velocity_scale
/platform_serial_bridge.baud_rate
/platform_serial_bridge.serial_port
/teensy_serial_bridge.baud_rate
/teensy_serial_bridge.serial_port
... (more parameters)
```

### Check Parameter Values:
```bash
ros2 param get /arm_cartesian_controller velocity_scale
ros2 param get /platform_serial_bridge serial_port
```

### Expected Output:
```
Double value is: 0.001
String value is: /dev/ttyUSB0
```

## Common Issues and Solutions

### Issue 1: Build Fails - Missing pyserial
**Error:** `ModuleNotFoundError: No module named 'serial'`
**Solution:** 
```bash
pip install pyserial
```

### Issue 2: Build Fails - Syntax Error
**Error:** `SyntaxError: invalid syntax`
**Solution:** Check Python files for syntax errors, ensure proper indentation

### Issue 3: Executables Not Found
**Error:** `No executable found`
**Solution:** 
1. Rebuild package: `colcon build --packages-select james_manipulation`
2. Re-source workspace: `call install\setup.bat`

### Issue 4: Launch File Fails
**Error:** `Package 'james_manipulation' not found`
**Solution:** Ensure package is built and workspace is sourced

### Issue 5: Nodes Start But Crash
**Check logs:**
```bash
ros2 launch james_manipulation arm_cartesian_control.launch.py --show-all-subprocesses-output
```

## Success Criteria Checklist

- [ ] Package builds without errors
- [ ] All three executables are available
- [ ] Launch file shows help correctly
- [ ] Individual nodes start (even with connection warnings)
- [ ] Launch file starts all nodes simultaneously
- [ ] ROS2 topics are created and publishing
- [ ] Parameters load from config file
- [ ] Nodes can be stopped cleanly with Ctrl+C

## Windows-Specific Notes

1. **Serial Ports:** Use `COM1`, `COM2`, etc. instead of `/dev/ttyUSB0`
2. **Path Separators:** Windows uses `\` but ROS2 handles this automatically
3. **Permissions:** No special permissions needed for serial ports on Windows
4. **Firewall:** Windows Defender may ask about Python network access (allow it)

## Next Steps After Successful Testing

1. **Modify config for Windows:** Update serial ports to `COM1`, `COM2`
2. **Test with actual hardware:** Connect Platform Controller and Teensy
3. **Proceed to Task 4:** Hardware integration testing
4. **Monitor logs:** Use `ros2 log` commands for debugging

## Expected Timeline

- **Build and basic testing:** 10-15 minutes
- **Topic and parameter verification:** 5-10 minutes  
- **Troubleshooting (if needed):** 15-30 minutes
- **Total:** 30-60 minutes depending on issues encountered