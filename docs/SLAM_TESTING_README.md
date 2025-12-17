# SLAM Testing Documentation

This directory contains comprehensive documentation for testing the SLAM (Simultaneous Localization and Mapping) system on James home robot.

## Overview

Task 7.3 from the implementation plan requires testing SLAM in a home environment to verify:
- Map building capability
- Loop closure detection
- Relocalization after restart
- Map persistence and loading

These tests validate Requirements 2.1, 2.3, and 2.4 from the requirements document.

## Documentation Files

### 1. SLAM Testing Guide (`slam_testing_guide.md`)
**Purpose:** Comprehensive step-by-step testing procedure

**Contents:**
- Prerequisites checklist
- Phase 1: Initial map building (30-45 min)
- Phase 2: Map quality verification (10-15 min)
- Phase 3: Loop closure verification (15-20 min)
- Phase 4: Relocalization after restart (15-20 min)
- Phase 5: Map persistence and loading (5-10 min)
- Troubleshooting guide
- Validation checklist

**When to use:** First time testing SLAM, or when you need detailed instructions

### 2. SLAM Testing Quick Reference (`slam_testing_quick_reference.md`)
**Purpose:** Quick command reference for experienced users

**Contents:**
- Quick start commands
- Common checks
- Troubleshooting quick fixes
- Map management commands
- Success criteria

**When to use:** When you know the procedure and just need command syntax

### 3. SLAM Test Report Template (`slam_test_report_template.md`)
**Purpose:** Standardized template for documenting test results

**Contents:**
- Test environment details
- Hardware configuration
- Test results for all phases
- Performance metrics
- Requirements validation
- Issues and recommendations

**When to use:** After completing testing to document results

## Quick Start

### Option 1: Interactive Helper (Recommended)

```bash
cd ~/james-useful-home-robot
chmod +x scripts/slam-test-helper.sh
./scripts/slam-test-helper.sh
```

This provides an interactive menu for:
- Quick system checks
- Monitoring loop closures
- Map backup and management
- Viewing maps
- Accessing documentation

### Option 2: Manual Testing

Follow the comprehensive guide:

```bash
# View the guide
less docs/slam_testing_guide.md

# Or open in your preferred editor
nano docs/slam_testing_guide.md
```

## Testing Workflow

```
┌─────────────────────────────────────────────────────────────┐
│                    SLAM Testing Workflow                     │
└─────────────────────────────────────────────────────────────┘

1. Preparation
   ├── Check hardware (cameras, battery, remote)
   ├── Source ROS2 workspace
   └── Review testing guide

2. Initial Map Building (Phase 1)
   ├── Start D435 camera
   ├── Start RTAB-Map
   ├── Drive through home environment
   ├── Monitor loop closures
   └── Save map

3. Map Quality Verification (Phase 2)
   ├── View map in database viewer
   ├── Check 3D geometry
   ├── Measure dimensional accuracy
   └── Export map for analysis

4. Loop Closure Testing (Phase 3)
   ├── Restart SLAM with existing map
   ├── Drive new loops
   ├── Verify loop closure detection
   └── Check map consistency

5. Relocalization Testing (Phase 4)
   ├── Stop all nodes
   ├── Move robot to new location
   ├── Restart in localization mode
   ├── Verify relocalization
   └── Test from multiple locations

6. Map Persistence Testing (Phase 5)
   ├── Create multiple backups
   ├── Load specific maps
   ├── Verify map statistics
   └── Test map loading

7. Documentation
   ├── Fill out test report template
   ├── Save map files
   ├── Document issues
   └── Make recommendations
```

## Success Criteria

### Map Building
- ✅ Map covers all target areas
- ✅ Walls are straight and continuous
- ✅ Room dimensions accurate within 5%
- ✅ No duplicate geometry

### Loop Closure
- ✅ At least 5 loop closures detected
- ✅ Loop closures at expected locations
- ✅ Map quality improves after loop closure
- ✅ No false loop closures

### Relocalization
- ✅ Localizes within 60 seconds
- ✅ Position accuracy within 20 cm
- ✅ Works from multiple starting locations
- ✅ Success rate > 66%

### Map Persistence
- ✅ Map saves automatically
- ✅ Map loads correctly
- ✅ Backups work properly
- ✅ Map statistics are reasonable

## Common Issues and Solutions

### Issue: Tracking Lost Frequently
**Solution:** Drive slower, improve lighting, point camera at textured surfaces

### Issue: No Loop Closures
**Solution:** Drive more overlapping paths, return to start multiple times

### Issue: Poor Relocalization
**Solution:** Start from distinctive location, rotate 360°, drive to landmark

### Issue: Out of Memory
**Solution:** Close other applications, reduce features in config, enable swap

See the full troubleshooting guide in `slam_testing_guide.md` for more details.

## Helper Scripts

### `scripts/slam-test-helper.sh`
Interactive menu for all testing operations:
- Quick system checks
- ROS2 topic monitoring
- Loop closure monitoring
- Map backup and management
- Map viewing and export

### `scripts/slam-backup-map.sh`
Backup current map with timestamp:
```bash
./scripts/slam-backup-map.sh
```

### `scripts/slam-view-map.sh`
View saved map in database viewer:
```bash
./scripts/slam-view-map.sh [map_file]
```

### `scripts/slam-start-*.sh`
Quick start scripts for cameras and RTAB-Map:
- `slam-start-d435.sh` - Start D435 camera
- `slam-start-rtabmap-d435only.sh` - Start RTAB-Map with D435 only
- `slam-start-rviz.sh` - Start RViz for visualization

See `scripts/SLAM_SCRIPTS_README.md` for complete list.

## Related Documentation

### ROS2 Package Documentation
- `ros2_ws/src/james_slam/README.md` - RTAB-Map configuration and usage
- `ros2_ws/src/james_slam/config/rtabmap_d435_only.yaml` - RTAB-Map parameters

### Requirements and Design
- `.kiro/specs/james-home-robot/requirements.md` - Requirements 2.1, 2.3, 2.4
- `.kiro/specs/james-home-robot/design.md` - SLAM architecture and design

### Implementation Plan
- `.kiro/specs/james-home-robot/tasks.md` - Task 7.3: Test SLAM in home environment

## Estimated Time

**First-time testing:** 2-3 hours
- Preparation: 15 minutes
- Initial mapping: 45 minutes
- Quality verification: 15 minutes
- Loop closure testing: 20 minutes
- Relocalization testing: 20 minutes
- Map persistence testing: 10 minutes
- Documentation: 30 minutes

**Subsequent testing:** 1-2 hours
- Skip detailed verification
- Focus on specific areas
- Quick validation checks

## Prerequisites

### Hardware
- [ ] James robot assembled
- [ ] D435 camera mounted at 120cm
- [ ] Remote control paired
- [ ] Battery charged (>80%)
- [ ] Clear path through home

### Software
- [ ] ROS2 Jazzy installed
- [ ] RTAB-Map package built
- [ ] Workspace sourced
- [ ] Scripts executable

### Environment
- [ ] Good lighting
- [ ] Textured surfaces visible
- [ ] Obstacles removed
- [ ] Route planned

## Next Steps

After successful SLAM testing:

1. **Save final map**
   ```bash
   ./scripts/slam-backup-map.sh
   ```

2. **Document results**
   - Fill out test report template
   - Note any issues or recommendations

3. **Proceed to navigation**
   - Task 8: Navigation System
   - Use saved map for Nav2 testing

4. **Optional improvements**
   - Remap areas with poor quality
   - Tune RTAB-Map parameters
   - Add more coverage

## Support

### Getting Help
- Review troubleshooting section in testing guide
- Check SLAM package README
- Review RTAB-Map documentation
- Open GitHub issue if needed

### Reporting Issues
When reporting issues, include:
- Test report (use template)
- Map file (if relevant)
- ROS2 logs
- System specifications
- Steps to reproduce

## Version History

- **v1.0** (2024-11-26): Initial testing documentation
  - Comprehensive testing guide
  - Quick reference card
  - Test report template
  - Interactive helper script

---

**Last Updated:** 2024-11-26  
**Maintainer:** James Robot Team  
**Status:** Active
