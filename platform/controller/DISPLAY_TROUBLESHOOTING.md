# Display Troubleshooting Guide

## Common Issues and Solutions

### Display Not Turning On

**Symptom**: Black screen, no display output

**Solutions**:
1. Check GPIO 38 is HIGH (display power)
   ```cpp
   pinMode(38, OUTPUT);
   digitalWrite(38, HIGH);
   ```
2. Verify QSPI pin connections (CS=6, SCK=47, D0=18, D1=7, D2=48, D3=5)
3. Check RST pin (GPIO 17) is connected
4. Monitor serial output for "Display initialized!" message
5. Try different rotation values (0, 1, 2, 3) if display is blank

### Display Shows Wrong Orientation

**Symptom**: Display is rotated incorrectly

**Solution**: Change rotation parameter in display initialization:
```cpp
Arduino_GFX *gfx = new Arduino_RM67162(bus, 17 /* RST */, 1 /* rotation */);
```
- 0 = Portrait
- 1 = Landscape (default for platform)
- 2 = Portrait inverted
- 3 = Landscape inverted

### Battery Voltage Reading Incorrect

**Symptom**: Battery shows wrong voltage or percentage

**Solutions**:
1. Verify voltage divider circuit
2. Adjust `BATTERY_DIVIDER_RATIO` constant:
   ```cpp
   #define BATTERY_DIVIDER_RATIO 2.0  // Adjust this value
   ```
3. Check ADC pin (default GPIO 4)
4. Verify battery voltage range in code matches your battery type

**Calculation**:
```
Measured Voltage = (ADC_Value / 4095) × 3.3V × DIVIDER_RATIO
```

### Connection Status Always Shows Offline

**Symptom**: Connection circle stays RED even when remote is connected

**Solutions**:
1. Verify ESP-NOW is receiving data (check serial output)
2. Check `lastManualCommandTime` is being updated in `onDataReceive()`
3. Verify `CONNECTION_TIMEOUT_MS` (default 1000ms) is appropriate
4. Ensure remote control is paired and sending data

### Movement Indicator Not Showing

**Symptom**: Center circle shows no arrow or movement

**Solutions**:
1. Verify joystick data is being received (check serial output)
2. Check `joystickData.x` and `joystickData.y` values
3. Ensure deadzone threshold (10) is appropriate for your joystick
4. Verify coordinate mapping in `drawMovementIndicator()`

### Display Updates Slowly or Freezes

**Symptom**: Display lags or stops updating

**Solutions**:
1. Check `DISPLAY_UPDATE_MS` is set to 100 (10 Hz)
2. Verify main loop is not blocked by long operations
3. Ensure `updateDisplay()` is called in `loop()`
4. Check for excessive serial printing in main loop
5. Monitor CPU usage - reduce update rate if needed

### Text or Graphics Appear Corrupted

**Symptom**: Display shows garbled text or graphics

**Solutions**:
1. Verify double buffering is working (`gfx2->flush()` is called)
2. Check canvas size matches screen dimensions (536x240)
3. Ensure text doesn't overflow screen boundaries
4. Try reducing text size or graphic complexity
5. Verify Arduino_GFX library is up to date

### Colors Look Wrong

**Symptom**: Colors don't match expected values

**Solutions**:
1. Check color definitions at top of file:
   ```cpp
   #define COLOR_GOOD     GREEN
   #define COLOR_ERROR    RED
   #define COLOR_WARNING  YELLOW
   ```
2. Verify display color mode (RGB565)
3. Try different color values if needed
4. Check display brightness/contrast settings

## Hardware Checks

### Power Supply
- Verify 5V supply to ESP32
- Check current capacity (display can draw 100-200mA)
- Ensure stable power (no brownouts)

### Pin Connections
```
Display Power: GPIO 38 → HIGH
QSPI CS:       GPIO 6
QSPI SCK:      GPIO 47
QSPI D0:       GPIO 18
QSPI D1:       GPIO 7
QSPI D2:       GPIO 48
QSPI D3:       GPIO 5
Display RST:   GPIO 17
Battery ADC:   GPIO 4 (optional)
```

### I2C Conflicts
- Display uses QSPI, not I2C
- I2C (SDA=3, SCL=2) is for ODrive communication
- These should not conflict

## Debug Tips

### Enable Verbose Logging
Add debug prints in display functions:
```cpp
void updateDisplay() {
  Serial.println("Display update");
  // ... rest of function
}
```

### Test Display Independently
Create minimal test sketch:
```cpp
void loop() {
  gfx2->fillScreen(RED);
  gfx2->flush();
  delay(1000);
  gfx2->fillScreen(GREEN);
  gfx2->flush();
  delay(1000);
}
```

### Check Memory Usage
- Large canvas (536x240) uses significant RAM
- Monitor free heap: `Serial.println(ESP.getFreeHeap());`
- Reduce canvas size if memory issues occur

## Performance Optimization

If display updates are too slow:
1. Increase `DISPLAY_UPDATE_MS` (reduce update rate)
2. Reduce graphic complexity (smaller circles, simpler shapes)
3. Update only changed regions instead of full screen
4. Reduce text size or amount of text

## Getting Help

If issues persist:
1. Check serial monitor for error messages
2. Verify Arduino_GFX library version
3. Test with official LilyGO examples first
4. Check LilyGO GitHub for known issues
5. Verify board selection in Arduino IDE

## Useful Serial Debug Commands

Add to `loop()` for debugging:
```cpp
if (Serial.available()) {
  char cmd = Serial.read();
  if (cmd == 'd') {
    Serial.printf("Display: %s\n", displayInitialized ? "OK" : "FAIL");
    Serial.printf("Battery: %.2fV\n", readBatteryVoltage());
    Serial.printf("Mode: %d\n", currentMode);
    Serial.printf("Last Manual: %lu\n", lastManualCommandTime);
  }
}
```
