# Map Quality Monitoring Architecture

## System Overview

```
┌─────────────────────────────────────────────────────────────────┐
│                        RTAB-Map SLAM                             │
│                                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │   Visual     │  │     Loop     │  │     Map      │          │
│  │   Odometry   │  │   Closure    │  │ Optimization │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└────────────┬──────────────┬──────────────┬─────────────────────┘
             │              │              │
             │ /odom        │ /rtabmap/    │ /rtabmap/
             │              │ info         │ mapData
             ▼              ▼              ▼
┌─────────────────────────────────────────────────────────────────┐
│                   Map Quality Monitor                            │
│                                                                   │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                    Loop Closure Tracker                   │  │
│  │  - Sliding window of recent attempts                      │  │
│  │  - Success/failure classification                         │  │
│  │  - Success rate calculation                               │  │
│  └──────────────────────────────────────────────────────────┘  │
│                              │                                   │
│                              ▼                                   │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                 Degradation Detector                      │  │
│  │  - Compare rate vs threshold                              │  │
│  │  - Check for stale data                                   │  │
│  │  - Classify severity (OK/WARN/ERROR)                      │  │
│  └──────────────────────────────────────────────────────────┘  │
│                              │                                   │
│                              ▼                                   │
│  ┌──────────────────────────────────────────────────────────┐  │
│  │                 Optimization Trigger                      │  │
│  │  - Periodic optimization (5 min)                          │  │
│  │  - On-demand (severe degradation)                         │  │
│  │  - Service call to RTAB-Map                               │  │
│  └──────────────────────────────────────────────────────────┘  │
└────────────┬──────────────┬──────────────┬─────────────────────┘
             │              │              │
             │ /diagnostics │ /rtabmap/    │ logs
             │              │ trigger_new_ │
             │              │ map          │
             ▼              ▼              ▼
┌─────────────────────────────────────────────────────────────────┐
│                         Outputs                                  │
│                                                                   │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────┐          │
│  │ Diagnostics  │  │ Optimization │  │   Console    │          │
│  │   System     │  │   Service    │  │    Logs      │          │
│  │ (rqt_monitor)│  │              │  │              │          │
│  └──────────────┘  └──────────────┘  └──────────────┘          │
└─────────────────────────────────────────────────────────────────┘
```

## Data Flow

### 1. Loop Closure Detection

```
RTAB-Map detects potential loop closure
         │
         ▼
Publishes Info message with:
  - loop_closure_id > 0
  - ref_id (reference node)
  - transform + covariance
         │
         ▼
Monitor receives Info message
         │
         ▼
Checks covariance[0]:
  > 0 → Success ✅
  = 0 → Failure ❌
         │
         ▼
Adds to sliding window
```

### 2. Quality Assessment

```
Monitor maintains sliding window
         │
         ▼
Calculates success rate:
  successes / total_attempts
         │
         ▼
Compares to threshold:
  ≥ 50% → OK ✅
  30-50% → WARN ⚠️
  < 30% → ERROR ❌
         │
         ▼
Publishes diagnostic status
```

### 3. Optimization Trigger

```
Periodic timer (5 min) OR
Severe degradation (< 30%)
         │
         ▼
Check optimization service available
         │
         ▼
Call /rtabmap/trigger_new_map
         │
         ▼
RTAB-Map performs graph optimization
         │
         ▼
Log optimization result
```

## Component Interactions

### Monitor ↔ RTAB-Map

**Subscriptions:**
- `/rtabmap/info`: Loop closure statistics
- `/rtabmap/mapData`: Map graph data

**Service Calls:**
- `/rtabmap/trigger_new_map`: Request optimization

### Monitor ↔ Diagnostic System

**Publications:**
- `/diagnostics`: DiagnosticArray messages containing:
  - Status level (OK/WARN/ERROR)
  - Status message
  - Key-value metrics:
    - Total loop closures
    - Successful loop closures
    - Recent success rate
    - Overall success rate
    - Map nodes count
    - Time since optimization

### Monitor ↔ User

**Console Logs:**
- INFO: Loop closure successes
- WARN: Loop closure failures, degradation detected
- ERROR: Severe degradation, stale data

## State Machine

```
┌─────────────┐
│   STARTUP   │
│ (No data)   │
└──────┬──────┘
       │
       ▼
┌─────────────┐
│  COLLECTING │◄──────────────┐
│   (< 20     │               │
│  attempts)  │               │
└──────┬──────┘               │
       │                      │
       ▼                      │
┌─────────────┐               │
│   NORMAL    │               │
│ (≥ 50%      │               │
│  success)   │               │
└──────┬──────┘               │
       │                      │
       ▼                      │
┌─────────────┐               │
│  DEGRADED   │               │
│ (30-50%     │               │
│  success)   │               │
└──────┬──────┘               │
       │                      │
       ▼                      │
┌─────────────┐               │
│   SEVERE    │               │
│ (< 30%      │───────────────┘
│  success)   │ Trigger
│             │ Optimization
└─────────────┘
```

## Timing Diagram

```
Time →

RTAB-Map:     [Loop]─────[Loop]────────[Loop]─────[Loop]
                │          │            │          │
                ▼          ▼            ▼          ▼
Monitor:      [Track]───[Track]──────[Track]───[Track]
                │          │            │          │
                ▼          ▼            ▼          ▼
Diagnostics:  [Pub]────[Pub]────────[Pub]─────[Pub]
                                       │
                                       ▼
                                  [Check Degradation]
                                       │
                                       ▼
                                  [Optimization?]

Periodic:     ─────────────[5 min]─────────────[5 min]─────
                            │                    │
                            ▼                    ▼
                      [Optimize]           [Optimize]
```

## Configuration Flow

```
Launch File
    │
    ├─ loop_closure_threshold: 0.5
    ├─ window_size: 20
    ├─ optimization_interval: 300.0
    └─ degradation_check_interval: 60.0
         │
         ▼
    Monitor Node
         │
         ├─ Creates timers
         ├─ Sets thresholds
         └─ Initializes tracking
              │
              ▼
         Runtime Operation
```

## Error Handling

```
Error Condition          → Detection              → Response
─────────────────────────────────────────────────────────────
No info messages         → Timeout (30s)          → Log ERROR
Service unavailable      → Service check fails    → Log WARN
Optimization fails       → Future exception       → Log ERROR
High failure rate        → Rate < threshold       → Trigger optimize
Stale data              → No updates (30s)       → Log ERROR
```

## Performance Considerations

### CPU Usage
- Minimal: Only processes info messages (~1 Hz)
- Degradation checks: Every 60s
- No heavy computation

### Memory Usage
- Sliding window: ~20 entries × 8 bytes = 160 bytes
- Diagnostic messages: ~1 KB each
- Total: < 1 MB

### Network Usage
- Subscribes: ~1 Hz (info messages)
- Publishes: ~1 Hz (diagnostics)
- Service calls: ~0.003 Hz (every 5 min)
- Total: < 10 KB/s

## Integration Points

### With Navigation (Nav2)
```python
# Monitor map quality before navigation
if map_quality.level == ERROR:
    cancel_navigation()
    wait_for_optimization()
    retry_navigation()
```

### With Web Dashboard
```javascript
// Display map quality status
const mapQuality = diagnostics.find('SLAM Map Quality');
updateStatusIndicator(mapQuality.level);
updateMetrics(mapQuality.values);
```

### With Task Planner
```python
# Consider map quality in task planning
if map_quality.recent_rate < 0.7:
    prefer_known_areas()
else:
    allow_exploration()
```

## Testing Architecture

```
┌─────────────────────┐
│  Test Simulator     │
│                     │
│  - Generates fake   │
│  - Info messages    │
│  - Controlled rate  │
└──────────┬──────────┘
           │ /rtabmap/info
           ▼
┌─────────────────────┐
│  Monitor (DUT)      │
│                     │
│  - Under test       │
│  - Real logic       │
└──────────┬──────────┘
           │ /diagnostics
           ▼
┌─────────────────────┐
│  Test Validator     │
│                     │
│  - Checks output    │
│  - Asserts correct  │
└─────────────────────┘
```
