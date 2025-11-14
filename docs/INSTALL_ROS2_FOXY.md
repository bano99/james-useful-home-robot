# Quick Start: ROS2 Foxy Installation on Jetson Nano

## TL;DR

```bash
cd ~/james-useful-home-robot/scripts
chmod +x jetson-ros2-foxy-install.sh
./jetson-ros2-foxy-install.sh
```

## What This Does

The installation script will:

1. ✅ Install ROS2 Foxy Desktop (full installation)
2. ✅ **Preserve CUDA-optimized OpenCV** (blocks python3-opencv package)
3. ✅ Install cv_bridge and vision packages
4. ✅ Set up development tools (colcon, rosdep, vcstool)
5. ✅ Configure environment variables
6. ✅ Verify CUDA support remains active

## Why This Matters

Your Q-engineering image has **CUDA-optimized OpenCV 4.5.x** that provides:
- 5-6x faster image processing on GPU
- CUDA modules for computer vision
- TensorRT integration

The standard `python3-opencv` package would **overwrite this** with a CPU-only version.

## Installation Time

Expect **15-20 minutes** for the full installation.

## After Installation

```bash
# Reload environment
source ~/.bashrc

# Verify ROS2
ros2 --version

# Verify CUDA OpenCV is still active
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep -i cuda

# Test ROS2
ros2 run demo_nodes_cpp talker
```

## What Gets Installed

### ROS2 Packages
- `ros-foxy-desktop` - Full desktop installation
- `ros-foxy-cv-bridge` - OpenCV ↔ ROS2 bridge
- `ros-foxy-image-transport` - Image transport
- `ros-foxy-camera-info-manager` - Camera calibration
- `ros-foxy-vision-opencv` - Vision utilities

### Development Tools
- `python3-colcon-common-extensions` - Build tool
- `python3-rosdep` - Dependency management
- `python3-vcstool` - Version control tools

### What Does NOT Get Installed
- ❌ `python3-opencv` - Blocked to preserve CUDA version

## Verification Checklist

After installation, verify everything works:

```bash
# 1. ROS2 is installed
ros2 --version
# Expected: ros2 cli version: 0.9.x

# 2. CUDA OpenCV is active
python3 -c "import cv2; print(cv2.__version__)"
# Expected: 4.5.x or higher

python3 -c "import cv2; print(cv2.getBuildInformation())" | grep "CUDA"
# Expected: NVIDIA CUDA: YES (ver 10.2, ...)

# 3. cv_bridge works
python3 -c "from cv_bridge import CvBridge; print('cv_bridge OK')"
# Expected: cv_bridge OK

# 4. PyTorch still has CUDA
python3 -c "import torch; print(f'PyTorch {torch.__version__}, CUDA: {torch.cuda.is_available()}')"
# Expected: PyTorch 1.13.0, CUDA: True
```

## Troubleshooting

### Issue: "python3-opencv is already installed"

If you already have python3-opencv installed:

```bash
# Check if it's the CUDA version
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep CUDA

# If CUDA: NO, remove the package
sudo apt remove python3-opencv

# Verify CUDA version is back
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep CUDA
# Should show: CUDA: YES

# Then run the installation script
./jetson-ros2-foxy-install.sh
```

### Issue: "cv_bridge import error"

```bash
# Install cv_bridge
sudo apt install ros-foxy-cv-bridge ros-foxy-vision-opencv

# Source ROS2
source /opt/ros/foxy/setup.bash

# Test again
python3 -c "from cv_bridge import CvBridge; print('OK')"
```

### Issue: "Lost CUDA support after installation"

```bash
# Check which OpenCV Python is using
python3 -c "import cv2; print(cv2.__file__)"

# Should show: /usr/local/lib/python3.8/site-packages/cv2/...
# If it shows /usr/lib/..., the apt package was installed

# Remove apt package
sudo apt remove python3-opencv

# Verify CUDA is back
python3 -c "import cv2; print(cv2.getBuildInformation())" | grep CUDA
```

## Next Steps

After successful installation:

1. **Test ROS2 communication**:
   ```bash
   # Terminal 1
   ros2 run demo_nodes_cpp talker
   
   # Terminal 2
   ros2 run demo_nodes_cpp listener
   ```

2. **Install additional packages** (optional):
   ```bash
   # Navigation
   sudo apt install ros-foxy-navigation2 ros-foxy-nav2-bringup
   
   # SLAM
   sudo apt install ros-foxy-slam-toolbox
   
   # RealSense
   sudo apt install ros-foxy-realsense2-camera
   ```

3. **Create your workspace**:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws
   colcon build
   source install/setup.bash
   ```

## References

- [ROS2 Foxy Documentation](https://docs.ros.org/en/foxy/)
- [OpenCV CUDA Setup Details](ros2_foxy_opencv_setup.md)
- [Q-engineering Image Info](qengineering_image_info.md)
- [Hardware Setup Guide](hardware_setup.md)

