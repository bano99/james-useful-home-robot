#!/bin/bash

# Fix cv_bridge to work with CUDA-optimized OpenCV
# This script rebuilds cv_bridge from source against the Q-engineering OpenCV

set -e

echo "=========================================="
echo "Rebuilding cv_bridge for CUDA OpenCV"
echo "=========================================="
echo ""
echo "This will rebuild cv_bridge to work with your CUDA-optimized OpenCV"
echo ""

# Check if ROS2 is installed
if [ ! -d "/opt/ros/foxy" ]; then
    echo "Error: ROS2 Foxy not found. Please install ROS2 first."
    exit 1
fi

# Source ROS2
source /opt/ros/foxy/setup.bash

# Create workspace
echo "[1/6] Creating workspace..."
mkdir -p ~/ros2_cv_bridge_ws/src
cd ~/ros2_cv_bridge_ws/src

# Clone vision_opencv (contains cv_bridge)
echo ""
echo "[2/6] Cloning vision_opencv repository..."
if [ ! -d "vision_opencv" ]; then
    git clone https://github.com/ros-perception/vision_opencv.git -b foxy
else
    echo "vision_opencv already cloned"
fi

cd ~/ros2_cv_bridge_ws

# Install dependencies
echo ""
echo "[3/6] Installing dependencies..."
sudo apt install -y \
    python3-colcon-common-extensions \
    libboost-python-dev \
    libboost-all-dev

# Check OpenCV version
echo ""
echo "[4/6] Checking OpenCV configuration..."
CV_VERSION=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null)
echo "Python OpenCV version: $CV_VERSION"

# Find OpenCV installation
if [ -d "/usr/local/lib/python3.8/site-packages/cv2" ]; then
    echo "CUDA OpenCV found at: /usr/local/lib/python3.8/site-packages/cv2"
    export OpenCV_DIR=/usr/local/share/opencv4
    export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
elif [ -d "/usr/local/lib/python3.8/dist-packages/cv2" ]; then
    echo "CUDA OpenCV found at: /usr/local/lib/python3.8/dist-packages/cv2"
    export OpenCV_DIR=/usr/local/share/opencv4
    export CMAKE_PREFIX_PATH=/usr/local:$CMAKE_PREFIX_PATH
else
    echo "Warning: CUDA OpenCV location not found in standard paths"
fi

# Build cv_bridge
echo ""
echo "[5/6] Building cv_bridge (this may take 5-10 minutes)..."
colcon build --packages-select cv_bridge --cmake-args \
    -DCMAKE_BUILD_TYPE=Release \
    -DPYTHON_EXECUTABLE=/usr/bin/python3 \
    -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \
    -DPYTHON_LIBRARY=/usr/lib/aarch64-linux-gnu/libpython3.8.so

# Test the build
echo ""
echo "[6/6] Testing cv_bridge..."
source ~/ros2_cv_bridge_ws/install/setup.bash

if python3 -c "from cv_bridge import CvBridge; import cv2; print('cv_bridge OK')" 2>/dev/null; then
    echo "✓ cv_bridge is working with CUDA OpenCV!"
else
    echo "⚠ cv_bridge test failed. Checking details..."
    python3 -c "from cv_bridge import CvBridge; import cv2; print('cv_bridge OK')"
fi

# Add to bashrc
echo ""
echo "Adding cv_bridge workspace to ~/.bashrc..."
if ! grep -q "ros2_cv_bridge_ws" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Custom cv_bridge built for CUDA OpenCV" >> ~/.bashrc
    echo "source ~/ros2_cv_bridge_ws/install/setup.bash" >> ~/.bashrc
    echo "✓ Added to ~/.bashrc"
else
    echo "✓ Already in ~/.bashrc"
fi

echo ""
echo "=========================================="
echo "cv_bridge Rebuild Complete!"
echo "=========================================="
echo ""
echo "cv_bridge has been rebuilt against your CUDA-optimized OpenCV."
echo ""
echo "To use it in this terminal:"
echo "  source ~/ros2_cv_bridge_ws/install/setup.bash"
echo ""
echo "To use it in new terminals:"
echo "  source ~/.bashrc"
echo ""
echo "Test it:"
echo "  python3 -c 'from cv_bridge import CvBridge; import cv2; print(\"OK\")'"
echo ""

