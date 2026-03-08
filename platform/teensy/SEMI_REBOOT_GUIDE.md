# Teensy Semi-Reboot Feature

## Overview
The semi-reboot feature allows the Teensy to save its current joint positions to non-volatile EEPROM memory and perform a system reset. Upon reboot, the saved positions are automatically restored, allowing the robot to continue operation from where it left off without needing recalibration.

## Purpose
This feature addresses firmware drift issues by providing a clean reset while preserving the robot's calibrated state and current position.

## How It Works

### 1. Save and Reset
When triggered, the system:
- Saves all 9 joint step positions (J1StepM through J9StepM) to EEPROM
- Saves the calibration flag
- Writes a magic number to verify data integrity
- Sends acknowledgment: `POSITIONS_SAVED_RESETTING`
- Triggers a system reset via `SCB_AIRCR = 0x05FA0004`

### 2. Restore on Boot
During `setup()`, the system:
- Checks for the magic number in EEPROM
- If found, restores all joint positions
- Restores the calibration flag
- Clears the magic number (one-time restore)
- Sends confirmation: `POSITIONS_RESTORED_FROM_EEPROM`

## Usage

### Serial Command
Send the command `SR` (Semi-Reboot) via serial to trigger the save and reset:

```
SR\n
```

### Example from Python
```python
import serial

ser = serial.Serial('/dev/ttyACM0', 115200)
ser.write(b'SR\n')
response = ser.readline()
print(response)  # Should print: POSITIONS_SAVED_RESETTING
```

### Integration with ROS2
You can add this to your teensy_serial_bridge.py or create a service:

```python
def trigger_semi_reboot(self):
    """Trigger Teensy semi-reboot to recover from drift"""
    self.get_logger().info('Triggering Teensy semi-reboot...')
    self.serial_port.write(b'SR\n')
    response = self.serial_port.readline().decode().strip()
    self.get_logger().info(f'Response: {response}')
    
    # Wait for reboot and reconnection
    time.sleep(2)
    
    # Check for restore confirmation
    response = self.serial_port.readline().decode().strip()
    if 'POSITIONS_RESTORED' in response:
        self.get_logger().info('Positions successfully restored after reboot')
```

## EEPROM Memory Layout

| Address | Size | Content |
|---------|------|---------|
| 0       | 4    | Magic number (0xAB12CD34) |
| 4       | 4    | J1StepM |
| 8       | 4    | J2StepM |
| 12      | 4    | J3StepM |
| 16      | 4    | J4StepM |
| 20      | 4    | J5StepM |
| 24      | 4    | J6StepM |
| 28      | 4    | J7StepM |
| 32      | 4    | J8StepM |
| 36      | 4    | J9StepM |
| 40      | 1    | Calibration flag |

## Safety Considerations

1. **Only use when robot is stationary** - Do not trigger during motion
2. **Verify positions before use** - Ensure the robot is in a safe position
3. **One-time restore** - The magic number is cleared after restore to prevent unintended position loading on subsequent boots
4. **Normal boot behavior** - If no saved data exists, the system boots normally and requires calibration

## When to Use

- After detecting position drift
- When firmware becomes unresponsive but serial communication works
- As a recovery mechanism before full recalibration
- During long operation sessions to refresh the system state

## Limitations

- Does not save velocity, acceleration, or other runtime parameters
- Does not save tool offsets or DH parameters (these are code constants)
- Requires serial communication to be functional
- Brief interruption in operation during reset (~2 seconds)
