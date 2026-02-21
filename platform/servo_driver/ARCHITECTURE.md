# Servo Control Architecture

## Before Fix (Not Working)

```
┌─────────────────┐
│  Serial Monitor │
│   or ROS2 Node  │
└────────┬────────┘
         │
         │ "#1P0S500\n" (text)
         ▼
┌─────────────────────────────────┐
│         ESP32 Firmware          │
│                                 │
│  ┌──────────────────────────┐  │
│  │   Serial.available()     │  │
│  │   Read command           │  │
│  └──────────┬───────────────┘  │
│             │                   │
│             │ Raw forwarding    │
│             ▼                   │
│  ┌──────────────────────────┐  │
│  │   Serial1.println(cmd)   │  │
│  │   (GPIO 18/19 UART)      │  │
│  └──────────┬───────────────┘  │
└─────────────┼───────────────────┘
              │
              │ "#1P0S500\n" (raw text bytes)
              ▼
┌─────────────────────────────────┐
│      ST Servo (Feetech)         │
│                                 │
│  ❌ Expects SCServo binary      │
│     protocol, not text          │
│                                 │
│  ❌ Ignores unknown data        │
└─────────────────────────────────┘

Result: Servo doesn't move
```

## After Fix (Working)

```
┌─────────────────┐
│  Serial Monitor │
│   or ROS2 Node  │
└────────┬────────┘
         │
         │ "#1P0S500\n" (text)
         ▼
┌──────────────────────────────────────────────┐
│            ESP32 Firmware                    │
│                                              │
│  ┌────────────────────────────────────────┐ │
│  │      Serial.available()                │ │
│  │      Read command                      │ │
│  └──────────────┬─────────────────────────┘ │
│                 │                            │
│                 │ Check command type         │
│                 ▼                            │
│  ┌────────────────────────────────────────┐ │
│  │  Is it a sensor command? (RD, RO, etc) │ │
│  │  → handleSensorCommands()              │ │
│  └────────────────────────────────────────┘ │
│                 │                            │
│                 │ No, starts with #          │
│                 ▼                            │
│  ┌────────────────────────────────────────┐ │
│  │    SERIAL_PARSER.h                     │ │
│  │    parseAndExecuteServoCommand()       │ │
│  │                                        │ │
│  │  1. Parse: "#1P0S500"                 │ │
│  │     → ID=1, Pos=0, Speed=500          │ │
│  │                                        │ │
│  │  2. Validate:                         │ │
│  │     ✓ ID in range (0-253)             │ │
│  │     ✓ Position in range (0-4095)      │ │
│  │     ✓ Speed in range (0-4000)         │ │
│  │                                        │ │
│  │  3. Execute:                          │ │
│  │     st.WritePosEx(1, 0, 500, 100)     │ │
│  │                                        │ │
│  │  4. Feedback:                         │ │
│  │     "OK: Servo 1 -> Position 0..."    │ │
│  └──────────────┬─────────────────────────┘ │
│                 │                            │
│                 │ SCServo library call       │
│                 ▼                            │
│  ┌────────────────────────────────────────┐ │
│  │      SCServo Library (SMS_STS)         │ │
│  │      Converts to binary protocol       │ │
│  └──────────────┬─────────────────────────┘ │
│                 │                            │
│                 │ Binary protocol bytes      │
│                 ▼                            │
│  ┌────────────────────────────────────────┐ │
│  │   Serial1 (GPIO 18/19 UART)            │ │
│  └──────────────┬─────────────────────────┘ │
└─────────────────┼──────────────────────────┘
                  │
                  │ [FF FF 01 07 03 2A 00 00 ...]
                  │ (SCServo binary protocol)
                  ▼
┌──────────────────────────────────────────────┐
│         ST Servo (Feetech)                   │
│                                              │
│  ✅ Receives proper binary protocol          │
│  ✅ Executes position command                │
│  ✅ Moves to position 0 at speed 500         │
└──────────────────────────────────────────────┘

Result: Servo moves correctly!
```

## Web Interface (Always Worked)

```
┌─────────────────┐
│  Web Browser    │
│  (WiFi)         │
└────────┬────────┘
         │
         │ HTTP POST: {id:1, pos:0, speed:500}
         ▼
┌──────────────────────────────────────────────┐
│            ESP32 Firmware                    │
│                                              │
│  ┌────────────────────────────────────────┐ │
│  │      Web Server Handler                │ │
│  │      Receives JSON data                │ │
│  └──────────────┬─────────────────────────┘ │
│                 │                            │
│                 │ Direct library call        │
│                 ▼                            │
│  ┌────────────────────────────────────────┐ │
│  │   st.WritePosEx(1, 0, 500, 100)        │ │
│  └──────────────┬─────────────────────────┘ │
│                 │                            │
│                 │ SCServo binary protocol    │
│                 ▼                            │
│  ┌────────────────────────────────────────┐ │
│  │   Serial1 (GPIO 18/19 UART)            │ │
│  └──────────────┬─────────────────────────┘ │
└─────────────────┼──────────────────────────┘
                  │
                  ▼
┌──────────────────────────────────────────────┐
│         ST Servo (Feetech)                   │
│  ✅ Moves correctly                          │
└──────────────────────────────────────────────┘

This always worked because it called the library directly!
```

## Command Flow Comparison

### Text Command → Binary Protocol

| Step | Text Command | Binary Protocol (hex) |
|------|--------------|----------------------|
| Input | `#1P0S500` | - |
| Parse | ID=1, Pos=0, Speed=500 | - |
| Library Call | `st.WritePosEx(1, 0, 500, 100)` | - |
| UART Output | - | `FF FF 01 07 03 2A 00 00 F4 01 64 XX` |
| Servo Action | Moves to position 0 | Moves to position 0 |

## Key Components

### SERIAL_PARSER.h
- Parses text commands
- Validates parameters
- Calls SCServo library functions
- Provides user feedback

### SCServo Library (SMS_STS)
- Implements Feetech protocol
- Handles binary packet construction
- Manages checksums and timing
- Communicates with servos

### Serial1 (Hardware UART)
- GPIO 18: RX (receive from servos)
- GPIO 19: TX (transmit to servos)
- Baud: 1000000 (1 Mbps)
- Connected to servo bus

### Serial (USB UART)
- GPIO 1/3 (built-in USB)
- Baud: 115200
- Receives text commands
- Sends feedback to user

## Supported Command Types

```
┌─────────────────────────────────────────────┐
│         Command Router (loop())             │
└──────────┬──────────────────────────────────┘
           │
           ├─→ Sensor Commands (RD, RO, RC, SS, SCAN, INFO)
           │   └─→ handleSensorCommands()
           │       └─→ Read I2C sensors
           │
           ├─→ Servo Commands (#...)
           │   └─→ parseAndExecuteServoCommand()
           │       ├─→ #<ID>P<pos>S<speed> → st.WritePosEx()
           │       ├─→ #<ID>P? → st.ReadPos()
           │       ├─→ #<ID>T<0|1> → st.EnableTorque()
           │       ├─→ #<ID>M<0|3> → setMode()
           │       └─→ #<ID>PING → st.Ping()
           │
           └─→ Help Commands (?, HELP)
               └─→ Print help text
```

## Thread Safety

All servo commands use mutex protection:
```cpp
if (xSemaphoreTake(serial1Mutex, portMAX_DELAY) == pdTRUE) {
    st.WritePosEx(servoID, position, speed, acc);
    xSemaphoreGive(serial1Mutex);
}
```

This prevents conflicts between:
- Serial commands
- Web interface commands
- Background sensor updates
- Display updates

## Error Handling

```
User Input: "#1P5000S1000"
            │
            ▼
Parser: Position validation
        5000 > 4095 (max)
            │
            ▼
Output: "ERROR: Position must be 0-4095"
        (No command sent to servo)
```

All errors are caught before sending to servo, preventing:
- Invalid positions
- Out-of-range speeds
- Wrong servo IDs
- Malformed commands

## Benefits of New Architecture

1. ✅ User-friendly text commands
2. ✅ Parameter validation before execution
3. ✅ Clear error messages
4. ✅ Immediate feedback
5. ✅ Thread-safe operation
6. ✅ Compatible with existing web interface
7. ✅ No changes needed to ROS2 code
8. ✅ Easy to debug and test
