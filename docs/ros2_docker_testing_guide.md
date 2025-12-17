# ROS2 Docker Testing Guide - Quick Start

## Immediate Testing Steps

### 1. Install jetson-containers (5 minutes)

```bash
# On original Jetson Nano
cd ~
git clone https://github.com/dusty-nv/jetson-containers
bash jetson-containers/install.sh

# Verify installation
jetson-containers --help
```

### 2. Test Basic Container (10 minutes)

```bash
# Pull and run basic PyTorch container
jetson-containers run $(autotag l4t-pytorch)

# Inside container, verify CUDA
python3 -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')"

# Exit container
exit
```

### 3. Test ROS2 Humble Container (15 minutes)

```bash
# Build or pull ROS2 Humble container
jetson-containers run dustynv/ros:humble-desktop-l4t-r32.7.1

# Inside container, test ROS2
source /opt/ros/humble/setup.bash
ros2 --version
ros2 run demo_nodes_cpp talker

# In another terminal (outside container)
# Test cross-version communication with native Foxy
source /opt/ros/foxy/setup.bash
ros2 run demo_nodes_cpp listener
```

### 4. Test Camera Access (20 minutes)

```bash
# Run container with USB device access
jetson-containers run --device=/dev/bus/usb dustynv/ros:humble-desktop-l4t-r32.7.1

# Inside container, install RealSense (if not included)
sudo apt update
sudo apt install ros-humble-realsense2-camera

# Test camera detection
rs-enumerate-devices

# Test ROS2 camera launch
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py device_type:=d435
```

### 5. Performance Benchmark (30 minutes)

```bash
# Test 1: Native Foxy Performance
# Terminal 1
source /opt/ros/foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py device_type:=d435

# Terminal 2
ros2 topic hz /camera/color/image_raw
ros2 topic bw /camera/color/image_raw

# Record: FPS and bandwidth

# Test 2: Docker Humble Performance
jetson-containers run --device=/dev/bus/usb dustynv/ros:humble-desktop-l4t-r32.7.1
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_launch.py device_type:=d435

# In another terminal (outside container)
ros2 topic hz /camera/color/image_raw
ros2 topic bw /camera/color/image_raw

# Compare: FPS and bandwidth vs native
```

## Expected Results

### Success Indicators ✅
- [ ] jetson-containers installs without errors
- [ ] Basic PyTorch container runs with CUDA support
- [ ] ROS2 Humble container starts successfully
- [ ] Cross-version communication works (Foxy ↔ Humble)
- [ ] RealSense cameras detected in Docker container
- [ ] Camera streaming works from container
- [ ] Performance overhead < 15% vs native

### Potential Issues ⚠️

**Issue**: Container fails to start
```bash
# Check Docker daemon
sudo systemctl status docker

# Check available space
df -h

# Check memory
free -h
```

**Issue**: No CUDA in container
```bash
# Verify nvidia-docker runtime
docker info | grep nvidia

# Check NVIDIA drivers
nvidia-smi
```

**Issue**: Camera not detected
```bash
# Check USB permissions
ls -la /dev/bus/usb/

# Run with privileged mode (testing only)
jetson-containers run --privileged dustynv/ros:humble-desktop-l4t-r32.7.1
```

**Issue**: ROS2 communication fails
```bash
# Check network configuration
export ROS_DOMAIN_ID=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Check multicast
ping 224.0.0.1
```

## Quick Performance Test Script

```bash
#!/bin/bash
# save as test_performance.sh

echo "=== Testing Native Foxy Performance ==="
source /opt/ros/foxy/setup.bash
ros2 launch realsense2_camera rs_launch.py device_type:=d435 &
FOXY_PID=$!
sleep 10

echo "Measuring Foxy performance..."
timeout 30s ros2 topic hz /camera/color/image_raw > foxy_hz.txt &
timeout 30s ros2 topic bw /camera/color/image_raw > foxy_bw.txt &
wait

kill $FOXY_PID
sleep 5

echo "=== Testing Docker Humble Performance ==="
jetson-containers run --device=/dev/bus/usb dustynv/ros:humble-desktop-l4t-r32.7.1 \
    bash -c "source /opt/ros/humble/setup.bash && ros2 launch realsense2_camera rs_launch.py device_type:=d435" &
HUMBLE_PID=$!
sleep 15

echo "Measuring Humble performance..."
timeout 30s ros2 topic hz /camera/color/image_raw > humble_hz.txt &
timeout 30s ros2 topic bw /camera/color/image_raw > humble_bw.txt &
wait

kill $HUMBLE_PID

echo "=== Results ==="
echo "Foxy FPS:"
cat foxy_hz.txt | grep "average rate"
echo "Humble FPS:"
cat humble_hz.txt | grep "average rate"

echo "Foxy Bandwidth:"
cat foxy_bw.txt | tail -1
echo "Humble Bandwidth:"
cat humble_bw.txt | tail -1
```

## Documentation Template

After testing, document results:

```markdown
# ROS2 Docker Testing Results

## Test Environment
- **Device**: Jetson Nano 4GB
- **OS**: Ubuntu 20.04
- **Native ROS2**: Foxy
- **Container ROS2**: Humble
- **Camera**: RealSense D435

## Results

### Installation ✅/❌
- jetson-containers: ✅
- Basic container: ✅
- ROS2 container: ✅

### Functionality ✅/❌
- CUDA support: ✅
- Cross-version communication: ✅
- Camera access: ✅
- Camera streaming: ✅

### Performance
| Metric | Native Foxy | Docker Humble | Overhead |
|--------|-------------|---------------|----------|
| FPS | XX.X | XX.X | X.X% |
| Bandwidth | XX MB/s | XX MB/s | X.X% |
| CPU Usage | XX% | XX% | X.X% |
| Memory | XX MB | XX MB | X.X% |

### Issues Encountered
- Issue 1: Description and solution
- Issue 2: Description and solution

### Recommendation
Based on testing: ✅ Proceed with migration / ❌ Need more investigation
```

## Next Steps After Testing

If testing is successful:
1. **Update migration plan** with actual performance data
2. **Set up Jetson Orin Nano** with ROS2 Humble
3. **Create custom containers** for James-specific packages
4. **Plan gradual migration** starting with least critical systems

If testing reveals issues:
1. **Document specific problems**
2. **Research alternative approaches**
3. **Consider hybrid native/container approach**
4. **Evaluate staying on Foxy longer**

This testing phase is crucial for validating the entire migration strategy!