#!/bin/bash

# ROS2 Humble Installation Script for Jetson Nano (Ubuntu 20.04)
# This script automates the installation of ROS2 Humble and required packages

set -e  # Exit on error

echo "=========================================="
echo "ROS2 Humble Installation for Jetson Nano"
echo "=========================================="
echo ""
echo "This script will install:"
echo "  - ROS2 Humble Hawksbill"
echo "  - RTAB-Map for SLAM"
echo "  - Nav2 for navigation"
echo "  - MoveIt2 for manipulation"
echo "  - RealSense ROS2 wrapper"
echo "  - Development tools"
echo ""
read -p "Press Enter to continue or Ctrl+C to cancel..."

# Check Ubuntu version
echo ""
echo "[1/10] Checking Ubuntu version..."
UBUNTU_VERSION=$(lsb_release -rs)
if [ "$UBUNTU_VERSION" != "20.04" ]; then
    echo "Warning: This script is designed for Ubuntu 20.04"
    echo "Your version: $UBUNTU_VERSION"
    read -p "Continue anyway? (y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi
echo "✓ Ubuntu $UBUNTU_VERSION detected"

# Set locale
echo ""
echo "[2/10] Setting up locale..."
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8
echo "✓ Locale configured"

# Add ROS2 repository
echo ""
echo "[3/10] Adding ROS2 repository..."
sudo apt install -y software-properties-common
sudo add-apt-repository universe -y
sudo apt update
sudo apt install -y curl gnupg lsb-release

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
echo "✓ ROS2 repository added"

# Install ROS2 Humble Desktop
echo ""
echo "[4/10] Installing ROS2 Humble Desktop (this may take 10-15 minutes)..."
sudo apt install -y ros-humble-desktop
echo "✓ ROS2 Humble Desktop installed"

# Install development tools
echo ""
echo "[5/10] Installing development tools..."
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    python3-pip \
    build-essential \
    git
echo "✓ Development tools installed"

# Initialize rosdep
echo ""
echo "[6/10] Initializing rosdep..."
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    sudo rosdep init
fi
rosdep update
echo "✓ rosdep initialized"

# Install RTAB-Map
echo ""
echo "[7/10] Installing RTAB-Map for SLAM..."
sudo apt install -y ros-humble-rtabmap-ros
echo "✓ RTAB-Map installed"

# Install Nav2
echo ""
echo "[8/10] Installing Nav2 for navigation..."
sudo apt install -y \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-robot-localization
echo "✓ Nav2 installed"

# Install MoveIt2
echo ""
echo "[9/10] Installing MoveIt2 for manipulation..."
sudo apt install -y ros-humble-moveit
echo "✓ MoveIt2 installed"

# Install RealSense packages
echo ""
echo "[10/10] Installing RealSense ROS2 wrapper..."
sudo apt install -y \
    ros-humble-realsense2-camera \
    ros-humble-realsense2-description
echo "✓ RealSense packages installed"

# Configure environment
echo ""
echo "Configuring ROS2 environment..."
if ! grep -q "source /opt/ros/humble/setup.bash" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# ROS2 Humble setup" >> ~/.bashrc
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
    echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc
    echo "✓ Added ROS2 setup to ~/.bashrc"
else
    echo "✓ ROS2 already configured in ~/.bashrc"
fi

# Install additional Python packages
echo ""
echo "Installing additional Python packages..."
pip3 install --user \
    pytest \
    pytest-cov \
    numpy \
    opencv-python

echo ""
echo "=========================================="
echo "Installation Complete!"
echo "=========================================="
echo ""
echo "ROS2 Humble has been successfully installed on your Jetson Nano."
echo ""
echo "Next steps:"
echo "1. Source your bashrc: source ~/.bashrc"
echo "2. Verify installation: ros2 --version"
echo "3. Test with demo: ros2 run demo_nodes_cpp talker"
echo ""
echo "To start using ROS2 in this terminal:"
echo "  source /opt/ros/humble/setup.bash"
echo ""
echo "System information:"
ros2 --version 2>/dev/null || echo "  (restart terminal to use ros2 command)"
echo ""
echo "For next steps, see: docs/hardware_setup.md"
echo ""
