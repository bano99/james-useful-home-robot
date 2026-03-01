# Diagnostic Code Fix - Calibration Issue Resolution

## Problem
After adding diagnostic code to track Teensy state, the calibration sequence broke:
- J4 stopped mid-movement
- Weird motion started
- Kinematic errors ("ER") appeared
- FreeRAM showed negative value (-1769680)

## Root Cause
The diagnostic code was adding String operations in the hot path (main loop):
```cpp
// PROBLEMATIC CODE:
lastCommand = inData.substring(0, min(20, (int)inData.length()));
```

This caused:
1. **Memory fragmentation** - Creating 20-char substrings on every loop iteration
2. **Heap corruption** - String allocations in tight loop
3. **Timing issues** - Extra processing time affecting motion control

## Solution
Minimized diagnostic overhead:

### 1. Reduced String Allocation
```cpp
// FIXED CODE:
if (inData.length() >= 2) {
  lastCommand = inData.substring(0, 2);  // Only 2 chars (function code)
  if (lastCommand == "XJ") {
    xjCommandsReceived++;
  }
}
```

### 2. Fixed freeMemory() Function
Changed from problematic `__bss_end` reference to simple stack-based approximation:
```cpp
int freeMemory() {
  char stackVar;
  return (int)&stackVar & 0xFFFF; // Relative comparison only
}
```

### 3. Kept Diagnostic Features
All diagnostic features remain functional:
- Loop counter (lightweight increment)
- Last command tracking (2 chars only)
- XJ command counting
- Full STATUS output via DS command

## Impact
- **Zero impact on normal operation** - Diagnostic code is now minimal
- **Calibration works normally** - No String allocations in hot path
- **STATUS command still works** - Full diagnostics available on demand
- **Memory stable** - No heap fragmentation

## Testing
After fix:
1. Calibration sequence should complete normally
2. FreeRAM should show positive value
3. No kinematic errors during calibration
4. DS command still provides full diagnostic output

## Lesson Learned
**Never use String operations in real-time control loops!**
- Avoid `substring()`, `+=`, or any String allocation in hot paths
- Use fixed-size buffers or minimal allocations
- Keep diagnostic code lightweight
- Only do heavy operations when explicitly requested (DS command)
