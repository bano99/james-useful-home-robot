# Platform Controller Display - Quick Reference (Landscape Mode)

## Display Layout

The display is optimized for viewing from a distance in landscape orientation.

### LEFT: Connection Status Circle (Large)
- **GREEN Circle** = Connected
- **RED Circle** = Offline
- **Label below**:
  - "MANUAL" - Remote control active
  - "AUTO" - Jetson control active
  - "OFFLINE" - No connection

### CENTER: Movement Indicator (Large)
- **Arrow** shows movement direction and magnitude
  - Longer arrow = faster movement
  - Arrow direction = robot movement direction
- **Rotating Arc** shows rotation
  - Cyan arc = rotating right
  - Magenta arc = rotating left
- **Crosshair** shows neutral position

### RIGHT: Battery Status (Optimized for Visibility)
- **Small Battery Icon** (30x50px) shows charge level at a glance
  - Green (>50%) = Good
  - Yellow (20-50%) = Warning
  - Red (<20%) = Critical
- **LARGE Voltage Display** (size 5 text, color-coded)
  - e.g., "3.87V" in GREEN/YELLOW/RED
- **Percentage Display** (size 3 text)
  - e.g., "78%"

### TOP Bar
- **Left**: "JAMES" - System name
- **Right**: Current mode (MANUAL/AUTO/STOP) with color coding

### BOTTOM Bar
- **Wheel Velocities**: FL, FR, BL, BR in m/s

## Update Rate
- 10 Hz (100ms) - Smooth visual feedback

## Hardware
- LilyGO T-Display S3 AMOLED V2
- 1.91" AMOLED (536x240 pixels)
- **Landscape orientation** (rotation = 3, flipped)

## Battery Monitoring
- 1S LiPo battery (3.0V - 4.2V range)
- Built-in voltage divider on GPIO 1
- Debug output to serial monitor every 2 seconds
- Note: May show 0V when USB charging is active
