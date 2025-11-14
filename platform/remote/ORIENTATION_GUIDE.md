# Display Orientation Guide

## Overview

The LilyGO T-Display S3 AMOLED V2 has a native resolution of 536x240 pixels. However, the orientation differs between the remote control and platform controller based on their physical mounting and usage.

## Remote Control - Portrait Mode

**Configuration**: 240x536 pixels (Portrait)  
**Rotation**: 0 (or 2 for inverted)  
**Physical Orientation**: Held vertically like a smartphone  
**Reason**: Natural handheld position for joystick control

```
     ┌─────────┐
     │  JAMES  │  ← 240 pixels wide
     │ REMOTE  │
     │         │
     │ Fwd/Bck │
     │  ┌───┐  │
     │  │░░░│  │
     │  │███│  │
     │  │░░░│  │
     │  └───┘  │
     │         │
     │ L/R     │
     │  ┌───┐  │
     │  │░░░│  │
     │  │███│  │
     │  │░░░│  │
     │  └───┘  │
     │         │
     │ Rot     │
     │  ┌───┐  │
     │  │░░░│  │
     │  │█░░│  │
     │  │░░░│  │
     │  └───┘  │
     │         │
     │ Status  │
     │ Signal  │
     │ Battery │
     └─────────┘
        ↑
    536 pixels tall
```

### Code Configuration:
```cpp
#define SCREEN_WIDTH 240
#define SCREEN_HEIGHT 536
tft.setRotation(0);  // Portrait mode
```

### UI Layout:
- **Vertical bars**: Show joystick values with up/down movement
- **Stacked layout**: Elements arranged top to bottom
- **Compact width**: Fits in hand comfortably
- **Tall display**: Allows multiple status items

## Platform Controller - Landscape Mode

**Configuration**: 536x240 pixels (Landscape)  
**Rotation**: 1 (or 3 for inverted)  
**Physical Orientation**: Mounted horizontally at front of robot  
**Reason**: Wide field of view for status monitoring, matches robot's forward direction

```
┌──────────────────────────────────────────────────────────────┐
│  JAMES PLATFORM CONTROLLER                                   │  ← 536 pixels wide
│                                                              │
│  Status: Autonomous    Speed: 1.2 m/s    Battery: 85%       │
│                                                              │
│  FL: 2.5  FR: 2.5     ┌────────────────┐   Mode: Auto      │
│  BL: 2.5  BR: 2.5     │   Robot View   │   Obstacles: 0    │
│                       │   (Optional)   │   Humans: 1       │
│                       └────────────────┘                     │
└──────────────────────────────────────────────────────────────┘
                           ↑
                    240 pixels tall
```

### Code Configuration:
```cpp
#define SCREEN_WIDTH 536
#define SCREEN_HEIGHT 240
tft.setRotation(1);  // Landscape mode
```

### UI Layout:
- **Horizontal layout**: Wide status bar at top
- **Side-by-side elements**: Motor speeds, status indicators
- **Wide display**: Shows more information at once
- **Compact height**: Fits on robot platform

## Rotation Values

The `setRotation()` function accepts values 0-3:

| Value | Orientation | Remote | Platform |
|-------|-------------|--------|----------|
| 0 | Portrait | ✅ Default | ❌ |
| 1 | Landscape | ❌ | ✅ Default |
| 2 | Portrait (inverted) | Optional | ❌ |
| 3 | Landscape (inverted) | ❌ | Optional |

## Why Different Orientations?

### Remote Control (Portrait):
1. **Ergonomics**: Natural to hold vertically with one hand
2. **Joystick position**: Thumb naturally rests on joystick
3. **Viewing angle**: Easy to glance at while controlling
4. **Vertical space**: Allows stacking multiple status items
5. **Compact**: Fits in hand, pocket, or mount

### Platform Controller (Landscape):
1. **Robot mounting**: Mounted horizontally at front of robot
2. **Forward direction**: Wide display matches robot's forward view
3. **Status dashboard**: Wide layout shows more information
4. **Visibility**: Easier to see from a distance
5. **Professional**: Looks like a dashboard/control panel

## Implementation Notes

### TFT_eSPI Configuration

The `User_Setup.h` file defines the native resolution:
```cpp
#define TFT_WIDTH  240   // Native width
#define TFT_HEIGHT 536   // Native height
```

The rotation is set in code:
```cpp
// Remote control
tft.setRotation(0);  // Portrait: 240x536

// Platform controller  
tft.setRotation(1);  // Landscape: 536x240
```

### LVGL Configuration

For remote control (`lv_conf.h`):
```cpp
#define LV_HOR_RES_MAX 240
#define LV_VER_RES_MAX 536
```

For platform controller (separate config):
```cpp
#define LV_HOR_RES_MAX 536
#define LV_VER_RES_MAX 240
```

### Display Driver

The `rm67162.cpp` driver handles rotation internally:
```cpp
void lcd_setRotation(uint8_t rotation) {
  switch (rotation) {
    case 0:  // Portrait
      lcd_send_data(0x00);
      lcd_width = 240;
      lcd_height = 536;
      break;
    case 1:  // Landscape
      lcd_send_data(0x60);
      lcd_width = 536;
      lcd_height = 240;
      break;
    // ... cases 2 and 3
  }
}
```

## Testing Orientation

### Visual Check:
1. Upload firmware
2. Power on device
3. Check if text is readable
4. Verify bars move in correct direction
5. Confirm layout matches expected orientation

### Code Check:
```cpp
Serial.print("Display: ");
Serial.print(SCREEN_WIDTH);
Serial.print("x");
Serial.println(SCREEN_HEIGHT);
```

Expected output:
- Remote: `Display: 240x536`
- Platform: `Display: 536x240`

## Common Issues

### Text Appears Sideways
- **Problem**: Wrong rotation value
- **Solution**: Change `setRotation()` value (0↔1 or 2↔3)

### Layout Doesn't Fit
- **Problem**: Width/height swapped
- **Solution**: Verify `SCREEN_WIDTH` and `SCREEN_HEIGHT` match rotation

### Bars Move Wrong Direction
- **Problem**: Horizontal bars in portrait or vice versa
- **Solution**: Change bar drawing from horizontal to vertical

### Touch Input Inverted
- **Problem**: Touch coordinates don't match rotation
- **Solution**: Adjust touch calibration for rotation

## Summary

| Aspect | Remote Control | Platform Controller |
|--------|----------------|---------------------|
| **Orientation** | Portrait | Landscape |
| **Resolution** | 240x536 | 536x240 |
| **Rotation** | 0 | 1 |
| **Bars** | Vertical | Horizontal |
| **Layout** | Stacked | Side-by-side |
| **Use Case** | Handheld | Dashboard |
| **Mounting** | Vertical | Horizontal |

Both configurations use the same hardware but optimize the display orientation for their specific use case.
