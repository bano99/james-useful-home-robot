# AMOLED Display Implementation - Landscape Mode

## Task 2.2 Completion Summary

This document describes the implementation of the AMOLED display interface for the James platform controller, optimized for landscape viewing from a low mounting position.

## Implementation Details

### Display Initialization
- Display powered on via GPIO 38
- QSPI interface configured for LilyGO T-Display S3 AMOLED V2
- **Landscape orientation** (rotation = 1) for 536x240 layout
- Double buffering using Arduino_Canvas for smooth updates
- Initialization occurs first in setup() before other components

### Status Display Components (Landscape Layout)

1. **Connection Status Circle (LEFT)** - Large 60px radius circle
   - GREEN when connected (MANUAL or AUTO)
   - RED when offline
   - Label below shows connection type

2. **Movement Indicator (CENTER)** - Large 80px radius circle
   - Arrow shows movement direction and magnitude
   - Rotating arc shows rotation (Cyan=CW, Magenta=CCW)
   - Crosshair for reference

3. **Battery Status (RIGHT)** - Optimized for distance viewing
   - Small battery icon (30x50px) for status at a glance
   - LARGE voltage text (size 5, ~150px wide) - highly visible
   - Color-coded voltage: Green (>50%), Yellow (20-50%), Red (<20%)
   - Medium percentage text (size 3)
   - Configured for 1S LiPo (3.0V - 4.2V range)

4. **Top Bar** - System name and mode indicator

5. **Bottom Bar** - Individual wheel velocities (FL, FR, BL, BR)

### Display Update Loop

- Update rate: 10 Hz (100ms interval) as specified
- Non-blocking implementation using millis() timing
- Only updates when display is initialized
- Efficient partial screen updates to minimize overhead

### Battery Monitoring

- ADC pin: GPIO 1 (built-in on LilyGO T-Display S3 AMOLED)
- 12-bit resolution (0-4095)
- Voltage divider ratio: 2.0 (built-in on board)
- Battery type: 1S LiPo (3.0V - 4.2V range)
- Real-time voltage and percentage calculation
- Debug output every 2 seconds to serial monitor

### Color Coding

- **GREEN**: Connected / Good battery / Forward movement
- **RED**: Disconnected / Low battery / Critical
- **YELLOW**: Warning / Medium battery
- **CYAN**: Manual mode / Clockwise rotation
- **MAGENTA**: Autonomous mode / Counter-clockwise rotation

## Design Rationale

### Why Landscape Mode?
- Display mounted low on platform
- Easier to read from above/distance
- More horizontal space for side-by-side indicators

### Why Large Elements?
- Viewing from distance while robot operates
- Quick status check at a glance
- High visibility in various lighting conditions

### Why Minimal Text?
- Graphics communicate faster than text
- Language-independent
- Better visibility from distance

## Requirements Met

✅ Initialize LilyGO AMOLED display using manufacturer examples
✅ Create status display showing current velocities
✅ Create status display showing mode (manual/autonomous)
✅ Create status display showing connection status
✅ Implement display update loop at 10 Hz refresh rate
✅ **BONUS**: Added battery voltage monitoring and display

## Testing Recommendations

1. Upload firmware to LilyGO T-Display S3 AMOLED V2
2. Verify display powers on in landscape mode
3. Connect remote control and verify:
   - Connection circle turns GREEN
   - Label shows "MANUAL"
   - Movement arrow appears when joystick moved
   - Rotation arc appears when rotating
4. Check battery voltage display (adjust divider ratio if needed)
5. Verify connection status changes when remote disconnects
6. Test from various viewing angles and distances
7. Monitor serial output for initialization messages

## Configuration Notes

### Battery Voltage Configuration

**Current Setup: 1S LiPo (3.0V - 4.2V)**

The LilyGO T-Display S3 AMOLED has built-in battery monitoring:
```cpp
#define BATTERY_PIN 1  // Built-in ADC pin on LilyGO board
#define BATTERY_DIVIDER_RATIO 2.0  // Built-in voltage divider
```

Battery voltage range in `drawBatteryIndicator()`:
```cpp
float minVoltage = 3.0;   // 1S LiPo empty
float maxVoltage = 4.2;   // 1S LiPo full
```

**For different battery types**, adjust the voltage range:
- 1S LiPo: 3.0V - 4.2V (current default)
- 2S LiPo: 6.0V - 8.4V
- 3S LiPo: 9.0V - 12.6V
- 4S LiPo: 12.0V - 16.8V

**Troubleshooting Battery Reading:**
1. Check serial monitor for debug output: "Battery ADC: xxxx, Voltage: x.xxV"
2. If voltage is 0.00V or incorrect:
   - Verify battery is connected
   - Check if USB charging affects reading
   - Adjust `BATTERY_DIVIDER_RATIO` if needed
3. The board may read ~0V when USB is connected and charging
