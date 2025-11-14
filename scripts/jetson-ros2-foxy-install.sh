#!/bin/bash

# ROS2 Foxy Installation Script for Jetson Nano (Ubuntu 20.04 Q-engineering Image)
# This script preserves CUDA-optimized OpenCV while installing ROS2 Foxy

set -e  # Exit on error

echo "=========================================="
echo "ROS2 Foxy Installation for Jetson Nano"
echo "=========================================="
echo ""
echo "This script will install ROS2 Foxy while preserving:"
echo "  - CUDA-optimized OpenCV"
echo "  - PyTorch with GPU support"
echo "  - TensorRT optimizations"
echo ""
read -p "Press Enter to continue or Ctrl+C to cancel..."

# Check Ubuntu version
echo ""
echo "[1/8] Checking Ubuntu version..."
UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" != "20.04" ]; then
    echo "Error: This script requires Ubuntu 20.04"
    echo "Your version: $UBUNTU_VERSION"
    exit 1
fi
echo "✓ Ubuntu $UBUNTU_VERSION detected"

# Verify CUDA OpenCV before installation
echo ""
echo "[2/8] Verifying CUDA-enabled OpenCV..."
if python3 -c "import cv2; print(cv2.getBuildInformation())" 2>/dev/null | grep -q "CUDA.*YES"; then
    CV_VERSION=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null)
    echo "✓ CUDA-enabled OpenCV $CV_VERSION detected"
else
    echo "⚠ Warning: CUDA support not detected in OpenCV"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# Set locale
echo ""
echo "[3/8] Setting up locale..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "✓ Locale configured"

# Add ROS2 repository
echo ""
echo "[4/8] Adding ROS2 Foxy repository..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
echo "✓ ROS2 repository added"

# Create dummy python3-opencv package to satisfy ROS2 dependencies
echo ""
echo "[5/8] Creating dummy python3-opencv package..."
mkdir -p /tmp/python3-opencv-dummy/DEBIAN
cat > /tmp/python3-opencv-dummy/DEBIAN/control <<EOF
Package: python3-opencv
Version: 9999.0.0
Architecture: arm64
Maintainer: Local User
Depends: 
Provides: python3-opencv
Conflicts: 
Description: Dummy package - CUDA OpenCV is installed manually
 This dummy package satisfies ROS2 dependencies while preserving
 the Q-engineering CUDA-optimized OpenCV installation.
EOF

# Build and install dummy package
dpkg-deb --build /tmp/python3-opencv-dummy
sudo dpkg -i /tmp/python3-opencv-dummy.deb
rm -rf /tmp/python3-opencv-dummy /tmp/python3-opencv-dummy.deb
echo "✓ Dummy package created"

# Install ROS2 Foxy Desktop
echo ""
echo "[6/8] Installing ROS2 Foxy Desktop (this may take 15-20 minutes)..."
echo "Note: Using CUDA-optimized OpenCV instead of python3-opencv"
sudo apt install -y ros-foxy-desktop

# Install additional ROS2 packages
echo ""
echo "[7/8] Installing additional ROS2 packages..."
sudo apt install -y \
    ros-foxy-cv-bridge \
    ros-foxy-image-transport \
    ros-foxy-camera-info-manager \
    ros-foxy-vision-opencv \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool

# Initialize rosdep
echo ""
echo "[8/8] Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
echo "✓ rosdep initialized"

# Configure environment
echo ""
echo "Configuring ROS2 environment..."
if ! grep -q "source /opt/ros/foxy/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Foxy setup" >> ~/.bashrc
    echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
    echo "" >> ~/.bashrc
    echo "# Ensure ROS2 uses CUDA-optimized OpenCV" >> ~/.bashrc
    echo "export PYTHONPATH=/usr/local/lib/python3.8/site-packages:\$PYTHONPATH" >> ~/.bashrc
    echo "✓ Added ROS2 setup to ~/.bashrc"
else
    echo "✓ ROS2 already configured in ~/.bashrc"
fi

# Verify CUDA OpenCV after installation
echo ""
echo "Verifying CUDA-enabled OpenCV after installation..."
if python3 -c "import cv2; print(cv2.getBuildInformation())" 2>/dev/null | grep -q "CUDA.*YES"; then
    CV_VERSION=$(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null)
    echo "✓ CUDA-enabled OpenCV $CV_VERSION is still active"
else
    echo "⚠ Warning: CUDA support not detected in OpenCV after installation"
    echo "You may need to manually configure cv_bridge to use the CUDA version"
fi

# Check Q-engineering optimizations
echo ""
echo "Checking Q-engineering image optimizations..."
if python3 -c "import torch" 2>/dev/null; then
    TORCH_VERSION=$(python3 -c "import torch; print(torch.__version__)" 2>/dev/null)
    CUDA_AVAILABLE=$(python3 -c "import torch; print(torch.cuda.is_available())" 2>/dev/null)
    echo "✓ PyTorch $TORCH_VERSION (CUDA: $CUDA_AVAILABLE)"
else
    echo "! PyTorch not found"
fi

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "ROS2 Foxy has been successfully installed on your Jetson Nano."
echo ""
echo "Preserved optimizations:"
python3 -c "import cv2; info = cv2.getBuildInformation(); cuda = 'YES' if 'CUDA' in info and 'YES' in info else 'NO'; print(f'  - OpenCV {cv2.__version__} (CUDA: {cuda})')" 2>/dev/null || echo "  - OpenCV: Error checking"
python3 -c "import torch; print(f'  - PyTorch {torch.__version__} (CUDA: {torch.cuda.is_available()})')" 2>/dev/null || echo "  - PyTorch: Not found"
echo ""
echo "Next steps:"
echo "1. Source your bashrc: source ~/.bashrc"
echo "2. Verify installation: ros2 --version"
echo "3. Test with demo: ros2 run demo_nodes_cpp talker"
echo "4. Test cv_bridge with CUDA OpenCV:"
echo "   python3 -c 'from cv_bridge import CvBridge; import cv2; print(\"cv_bridge OK\")'"
echo ""
echo "To start using ROS2 in this terminal:"
echo "  source /opt/ros/foxy/setup.bash"
echo ""
echo "Important notes:"
echo "  - python3-opencv was NOT installed to preserve CUDA support"
echo "  - cv_bridge will use your CUDA-optimized OpenCV"
echo "  - If you encounter cv_bridge issues, see: docs/qengineering_image_info.md"
echo ""

