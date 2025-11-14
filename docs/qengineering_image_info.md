# Q-engineering Jetson Nano Ubuntu 20.04 Image

## Overview

This project uses the Q-engineering optimized Ubuntu 20.04 image for Jetson Nano:
https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image

This image comes with many pre-installed and optimized packages that are beneficial for our robotics project.

## Pre-installed Software

### System Components

**Operating System:**
- Ubuntu 20.04.6 LTS (Focal Fossa)
- Kernel: 4.9.337-tegra
- L4T: 32.7.4 (JetPack 4.6.4 equivalent)

**Compilers & Build Tools:**
- **GCC 9.4.0** (default)
- **GCC 11.4.0** (available via update-alternatives)
- **CMake 3.16.3**
- **Make 4.2.1**
- **Python 3.8.10**

**Important**: The image has GCC 11 available, which is newer than the default Ubuntu 20.04 GCC 9. This is beneficial for building modern C++ code.

### CUDA & Deep Learning

**CUDA:**
- CUDA 10.2.300
- cuDNN 8.2.1
- TensorRT 8.2.1.8

**Deep Learning Frameworks:**
- **PyTorch 1.13.0** (pre-built for Jetson with CUDA support)
- **TorchVision 0.14.0**
- **ONNX Runtime 1.13.1** (with CUDA and TensorRT support)
- **OpenCV 4.8.0** (with CUDA, cuDNN, and GStreamer support)

**Note**: These are already optimized for Jetson Nano and don't need reinstallation!

### Computer Vision & Media

**OpenCV 4.8.0** with:
- CUDA acceleration
- cuDNN support
- GStreamer support
- Python bindings

**Additional Libraries:**
- **Eigen 3.3.7** (linear algebra)
- **NumPy 1.24.4** (optimized with OpenBLAS)
- **Pillow 10.0.1** (image processing)

### Python Packages

Pre-installed Python packages:
- numpy 1.24.4
- opencv-python 4.8.0
- torch 1.13.0
- torchvision 0.14.0
- onnxruntime-gpu 1.13.1
- pillow 10.0.1
- matplotlib 3.7.3
- scipy 1.10.1

## What This Means for James Project

### ✅ Already Available (No Need to Install)

1. **PyTorch** - For YOLOv8 object detection
2. **OpenCV** - For image processing (already CUDA-accelerated!)
3. **ONNX Runtime** - For optimized model inference
4. **CUDA/cuDNN** - For GPU acceleration
5. **GCC 11** - Modern C++ compiler for ROS2

### ⚠️ Still Need to Install

1. **ROS2 Humble** - Not included in the image
2. **RTAB-Map** - SLAM package
3. **Nav2** - Navigation stack
4. **MoveIt2** - Manipulation planning
5. **RealSense SDK** - Camera drivers

### 🎯 Optimization Opportunities

1. **Use Pre-built PyTorch** - Don't reinstall from pip
2. **Use Pre-built OpenCV** - Already optimized with CUDA
3. **Use GCC 11** - Better C++17/20 support for ROS2
4. **TensorRT Optimization** - Convert YOLO models to TensorRT for 3-5x speedup

## Compiler Configuration

### Switching to GCC 11 (Recommended for ROS2)

```bash
# Check available GCC versions
update-alternatives --list gcc

# Switch to GCC 11
sudo update-alternatives --set gcc /usr/bin/gcc-11
sudo update-alternatives --set g++ /usr/bin/g++-11

# Verify
gcc --version  # Should show 11.4.0
g++ --version  # Should show 11.4.0
```

### Why GCC 11 for ROS2?

- Better C++17 support (ROS2 Humble uses C++17)
- Improved optimization
- Better compatibility with modern libraries
- Faster compilation times

## Python Environment

### System Python

```bash
python3 --version  # Python 3.8.10
pip3 --version     # pip 20.0.2
```

### Important Notes

1. **Don't use pip for PyTorch/OpenCV** - They're already optimized
2. **Use apt for system packages** - Better integration
3. **Use pip only for pure Python packages** - Like pytest, flake8, etc.

## Verification Commands

### Check Pre-installed Software

```bash
# CUDA
nvcc --version

# PyTorch
python3 -c "import torch; print(f'PyTorch: {torch.__version__}, CUDA: {torch.cuda.is_available()}')"

# OpenCV
python3 -c "import cv2; print(f'OpenCV: {cv2.__version__}, CUDA: {cv2.cuda.getCudaEnabledDeviceCount() > 0}')"

# ONNX Runtime
python3 -c "import onnxruntime as ort; print(f'ONNX Runtime: {ort.__version__}, Providers: {ort.get_available_providers()}')"

# GCC
gcc --version
g++ --version

# CMake
cmake --version
```

### Expected Output

```
CUDA: 10.2.300
PyTorch: 1.13.0, CUDA: True
OpenCV: 4.8.0, CUDA: True
ONNX Runtime: 1.13.1, Providers: ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
GCC: 11.4.0
CMake: 3.16.3
```

## Performance Optimizations

### 1. Use TensorRT for YOLO

```python
# Convert YOLOv8 to TensorRT
from ultralytics import YOLO

model = YOLO('yolov8n.pt')
model.export(format='engine')  # Creates yolov8n.engine

# Use TensorRT model
model = YOLO('yolov8n.engine')
results = model(image)  # 3-5x faster than PyTorch!
```

### 2. Use CUDA-accelerated OpenCV

```python
import cv2

# Upload image to GPU
gpu_img = cv2.cuda_GpuMat()
gpu_img.upload(cpu_img)

# Process on GPU
gpu_gray = cv2.cuda.cvtColor(gpu_img, cv2.COLOR_BGR2GRAY)

# Download result
cpu_gray = gpu_gray.download()
```

### 3. Enable Jetson Power Mode

```bash
# Set to maximum performance (10W mode)
sudo nvpmodel -m 0

# Enable max clock speeds
sudo jetson_clocks
```

## Disk Space

The Q-engineering image uses about 8GB of the SD card. Recommended:
- **Minimum**: 32GB SD card
- **Recommended**: 64GB SD card
- **Optimal**: 128GB SD card (for datasets and models)

## Network Configuration

The image comes with:
- SSH server enabled by default
- NetworkManager for easy WiFi setup
- Avahi for mDNS (jetson-nano.local)

## Known Issues & Workarounds

### Issue 1: pip externally-managed-environment

**Problem**: Ubuntu 20.04 prevents pip from installing system-wide packages.

**Solution**: Use `--break-system-packages` flag or virtual environments:
```bash
# Option 1: Use flag (not recommended)
pip3 install package --break-system-packages

# Option 2: Use venv (recommended)
python3 -m venv ~/james_env
source ~/james_env/bin/activate
pip install package
```

### Issue 2: CUDA Out of Memory

**Problem**: Jetson Nano only has 4GB RAM (shared with GPU).

**Solution**: 
- Use smaller models (YOLOv8n instead of YOLOv8x)
- Enable swap space (8GB recommended)
- Close unnecessary applications
- Use TensorRT for lower memory usage

### Issue 3: Slow apt update

**Problem**: Default Ubuntu mirrors can be slow.

**Solution**: Use local mirrors or Q-engineering's mirror:
```bash
# Backup original sources
sudo cp /etc/apt/sources.list /etc/apt/sources.list.backup

# Use faster mirror (example: your country's mirror)
# Edit /etc/apt/sources.list
```

## Additional Resources

- **Q-engineering GitHub**: https://github.com/Qengineering
- **Jetson Nano Guide**: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html
- **Deep Learning Guide**: https://qengineering.eu/deep-learning-with-pytorch-on-jetson-nano.html
- **OpenCV Guide**: https://qengineering.eu/install-opencv-on-jetson-nano.html

## Summary for James Project

### Pre-installed ✅
- PyTorch 1.13.0 with CUDA
- OpenCV 4.8.0 with CUDA
- ONNX Runtime with TensorRT
- CUDA 10.2 + cuDNN 8.2
- GCC 11.4.0
- Python 3.8.10

### Need to Install 📦
- ROS2 Humble
- RTAB-Map
- Nav2
- MoveIt2
- RealSense SDK
- Additional Python packages (pytest, etc.)

### Optimization Tips 🚀
1. Use GCC 11 for building ROS2 packages
2. Convert YOLO models to TensorRT
3. Use pre-installed PyTorch/OpenCV (don't reinstall!)
4. Enable maximum performance mode
5. Add 8GB swap space

This image gives us a significant head start with optimized deep learning frameworks already configured! 🎉
