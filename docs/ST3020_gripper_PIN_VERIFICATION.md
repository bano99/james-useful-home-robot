# Waveshare ESP32 Servo Driver - Pin Availability Verification

## CRITICAL ISSUE - Pin Access

**Problem**: GPIO21/22 (I2C) may not be accessible on external headers!

## Board Analysis

### What the Wiki Shows

Looking at https://www.waveshare.com/wiki/Servo_Driver_with_ESP32:

**Internal Connections (NOT accessible):**
- GPIO21/22: Connected to PCA9685 PWM driver (internal I2C)
- GPIO21/22: May also connect to OLED display
- These pins are used by the board itself

**Servo Connections:**
- 16 PWM channels via PCA9685 (not direct GPIO)
- Servo signal pins are NOT direct ESP32 GPIO

**Exposed Pins (Need to verify from schematic):**
- USB port (UART0 - GPIO1/3)
- Possibly some GPIO on headers (need to check)

## Options for Sensor Connection

### Option 1: Share I2C Bus (IF GPIO21/22 are accessible)

**Requirements:**
- GPIO21/22 must be exposed on headers OR
- Solder directly to ESP32 module pins OR
- Tap into existing I2C bus (parallel connection)

**How to verify:**
1. Check if board has GPIO headers
2. Check schematic for exposed pins
3. Physical inspection when board arrives

**If accessible:**
- Connect sensors in parallel with PCA9685 and OLED
- VL53L1X at 0x30, BNO055 at 0x29 (no conflicts)

### Option 2: Use Different I2C Pins (Software I2C)

**Alternative I2C pins on ESP32:**
- Any GPIO can be used for I2C (software I2C)
- Common alternatives: GPIO16/17, GPIO25/26, GPIO32/33

**Pros:**
- Don't need access to GPIO21/22
- Independent I2C bus

**Cons:**
- Need to modify firmware to use different pins
- Software I2C may be slower

**Implementation:**
```cpp
// Instead of default I2C pins
Wire.begin(21, 22);  // Default

// Use alternative pins
Wire.begin(25, 26);  // SDA=25, SCL=26
// or
Wire.begin(32, 33);  // SDA=32, SCL=33
```

### Option 3: Tap into Existing I2C Bus

**If GPIO21/22 are not on headers but accessible on board:**

1. **Locate I2C traces** on PCB
2. **Solder wires** to I2C bus (parallel with PCA9685)
3. **Connect sensors** to these wires

**Advantages:**
- Uses hardware I2C (faster, more reliable)
- No firmware changes needed
- Shares bus with PCA9685 (no conflicts)

**Disadvantages:**
- Requires soldering skills
- May void warranty
- Risk of damaging board

### Option 4: Use OLED Connector (If present)

**If board has OLED connector:**
- OLED typically uses I2C (GPIO21/22)
- Connector exposes VCC, GND, SDA, SCL
- Can connect sensors to this connector

**Check for:**
- 4-pin connector labeled "OLED" or "I2C"
- Usually 0.1" pitch header
- May be labeled SDA/SCL

## Recommended Approach

### Step 1: Wait for Board to Arrive

**Physical inspection needed:**
1. Check for GPIO headers on board
2. Check for OLED/I2C connector
3. Check schematic (should be on wiki or in package)
4. Take photos and we'll analyze together

### Step 2: Identify Available Pins

**Look for:**
- Pin headers labeled with GPIO numbers
- I2C connector (4-pin: VCC, GND, SDA, SCL)
- Expansion headers
- Test points on PCB

### Step 3: Choose Connection Method

**Priority order:**
1. **Best**: Use exposed I2C connector (if present)
2. **Good**: Use exposed GPIO headers for alternative I2C
3. **Acceptable**: Tap into I2C bus on PCB (solder)
4. **Last resort**: Solder directly to ESP32 module

## Modified Pin Assignment (Flexible)

### If GPIO21/22 Available (Ideal)

| Pin | Function | Device |
|-----|----------|--------|
| GPIO21 | I2C SDA | VL53L1X + BNO055 (+ PCA9685 + OLED) |
| GPIO22 | I2C SCL | VL53L1X + BNO055 (+ PCA9685 + OLED) |
| GPIO18 | UART RX | ST3020 Servo |
| GPIO19 | UART TX | ST3020 Servo |

### If GPIO21/22 NOT Available (Alternative)

| Pin | Function | Device | Notes |
|-----|----------|--------|-------|
| GPIO25 | I2C SDA | VL53L1X + BNO055 | Software I2C |
| GPIO26 | I2C SCL | VL53L1X + BNO055 | Software I2C |
| GPIO18 | UART RX | ST3020 Servo | If available |
| GPIO19 | UART TX | ST3020 Servo | If available |

**OR**

| Pin | Function | Device | Notes |
|-----|----------|--------|-------|
| GPIO32 | I2C SDA | VL53L1X + BNO055 | Software I2C |
| GPIO33 | I2C SCL | VL53L1X + BNO055 | Software I2C |
| GPIO16 | UART RX | ST3020 Servo | Alternative UART |
| GPIO17 | UART TX | ST3020 Servo | Alternative UART |

## Firmware Modifications for Alternative Pins

### Current Code (assumes GPIO21/22)

```cpp
// In SENSOR_CTRL.h
void initSensors() {
  // Uses default I2C pins (21/22)
  // Wire.begin() is called in boardDevInit()
  ...
}
```

### Modified Code (for alternative pins)

```cpp
// In SENSOR_CTRL.h
#define SENSOR_I2C_SDA 25  // Change as needed
#define SENSOR_I2C_SCL 26  // Change as needed

void initSensors() {
  // Initialize I2C on alternative pins
  Wire.begin(SENSOR_I2C_SDA, SENSOR_I2C_SCL);
  
  // Rest of initialization...
  if (vl53.begin(VL53L1X_NEW_ADDR, &Wire)) {
    ...
  }
}
```

### For ST3020 Servo (if GPIO18/19 not available)

```cpp
// In STSCTRL.h
#define S_RXD 16  // Change from 18
#define S_TXD 17  // Change from 19

void servoInit(){
  Serial2.begin(1000000, SERIAL_8N1, S_RXD, S_TXD);  // Use Serial2 instead of Serial1
  st.pSerial = &Serial2;
  ...
}
```

## Questions to Answer When Board Arrives

### Visual Inspection

- [ ] Are there any GPIO headers on the board?
- [ ] Is there an OLED connector (4-pin)?
- [ ] Is there an I2C expansion connector?
- [ ] Are any GPIO pins labeled on the PCB?
- [ ] Are there test points for GPIO pins?

### Schematic Review

- [ ] Download schematic from Waveshare wiki
- [ ] Identify which GPIO are connected internally
- [ ] Identify which GPIO are exposed externally
- [ ] Check for expansion headers in schematic

### Continuity Testing

- [ ] Use multimeter to check which pins are accessible
- [ ] Verify GPIO21/22 are connected to PCA9685
- [ ] Check if GPIO21/22 are also on any connector

## Worst Case Scenario

**If NO GPIO pins are exposed:**

### Option A: Use Different Board

Consider using a generic ESP32 DevKit instead:
- All GPIO exposed on headers
- Lower cost (~$10)
- More flexible
- Need external power distribution

### Option B: Modify Waveshare Board

**Soldering required:**
1. Identify GPIO pins on ESP32 module
2. Solder thin wires to pins
3. Route wires to sensors
4. Use hot glue for strain relief

**Pins to access:**
- GPIO25, GPIO26 (or any unused GPIO)
- Or tap into GPIO21/22 I2C bus

### Option C: Use Expansion Board

Create a small PCB or perfboard that:
- Plugs into any available connector
- Breaks out GPIO pins
- Provides sensor connections

## Recommendation

**WAIT for board to arrive before proceeding further.**

Once you have the physical board:
1. Take clear photos of both sides
2. Check for any connectors or headers
3. Download schematic from Waveshare
4. We'll analyze together and determine best approach

**The firmware is ready** - we just need to adjust pin numbers based on what's actually available.

## Likely Scenarios (Ranked by Probability)

### Scenario 1: OLED/I2C Connector Present (70% likely)
- Board has 4-pin connector for I2C devices
- Can connect sensors directly
- No modifications needed
- **Action**: Use this connector

### Scenario 2: Some GPIO Headers Present (20% likely)
- Board has expansion headers with some GPIO
- Need to identify which pins are available
- May need to use alternative I2C pins
- **Action**: Modify firmware for available pins

### Scenario 3: No External Pins (10% likely)
- All GPIO used internally or not exposed
- Need to solder to board or ESP32 module
- **Action**: Tap into I2C bus or use alternative pins

## Next Steps

1. ✅ Firmware is ready (can adjust pins easily)
2. ⏳ Wait for board to arrive
3. ⏳ Physical inspection
4. ⏳ Determine available pins
5. ⏳ Adjust firmware if needed (5 minutes)
6. ⏳ Connect sensors and test

**Don't worry** - ESP32 is very flexible with pin assignments. We can make it work with whatever pins are available!

---

**Status**: Waiting for hardware to verify pin availability
**Risk**: Medium - may need minor firmware adjustments
**Mitigation**: Firmware designed to be easily reconfigured for different pins
