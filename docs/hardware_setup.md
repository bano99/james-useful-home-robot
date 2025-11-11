# Hardware Setup Guide

This guide provides detailed instructions for setting up the hardware components of James, including Ubuntu installation on Jetson Nano, system configuration, module installation, and MCP server setup.

## Table of Contents

1. [Jetson Nano Setup](#jetson-nano-setup)
2. [Ubuntu 24.04 Installation](#ubuntu-2404-installation)
3. [Network Configuration](#network-configuration)
4. [SSH and Remote Access](#ssh-and-remote-access)
5. [ROS2 Jazzy Installation](#ros2-jazzy-installation)
6. [RealSense SDK Installation](#realsense-sdk-installation)
7. [CUDA and TensorRT Setup](#cuda-and-tensorrt-setup)
8. [Python Environment Setup](#python-environment-setup)
9. [MCP Server Installation](#mcp-server-installation)
10. [ESP32 Development Environment](#esp32-development-environment)
11. [System Optimization](#system-optimization)
12. [Verification and Testing](#verification-and-testing)

---

## 1. Jetson Nano Setup

### Hardware Requirements

- 2x NVIDIA Jetson Nano (4GB recommended)
- 2x 64GB+ microSD cards (Class 10 or UHS-1)
- 2x 5V 4A power supplies with barrel jack
- Ethernet cable or WiFi adapter
- USB keyboard and mouse (for initial setup)
- HDMI monitor
- Jumper cap (to enable barrel jack power)

### Initial Preparation

1. **Enable Barrel Jack Power**
   - Locate J48 jumper pins near the camera connector
   - Place jumper cap on J48 pins to enable 5V 4A barrel jack power
   - This is required for stable operation with peripherals

2. **Prepare microSD Cards**
   - Use high-quality microSD cards (Samsung EVO or SanDisk Extreme recommended)
   - Format cards as exFAT or FAT32 before flashing

---

## 2. Ubuntu 24.04 Installation

### Download Ubuntu Image

```bash
# On your development PC
wget https://cdimage.ubuntu.com/releases/24.04/release/ubuntu-24.04-preinstalled-server-arm64+raspi.img.xz

# Extract the image
xz -d ubuntu-24.04-preinstalled-server-arm64+raspi.img.xz
```

### Flash Image to microSD Card

**Using Balena Etcher (Recommended):**

1. Download Etcher from https://www.balena.io/etcher/
2. Insert microSD card into your PC
3. Select the Ubuntu image file
4. Select the microSD card as target
5. Click "Flash!"

**Using dd (Linux/Mac):**

```bash
# Find the device name
lsblk

# Flash the image (replace /dev/sdX with your SD card device)
sudo dd if=ubuntu-24.04-preinstalled-server-arm64+raspi.img of=/dev/sdX bs=4M status=progress
sudo sync
```

### First Boot Configuration

1. Insert microSD card into Jetson Nano
2. Connect HDMI monitor, keyboard, and Ethernet cable
3. Connect power supply
4. Wait for first boot (takes 2-3 minutes)

**Initial Login:**
- Username: `ubuntu`
- Password: `ubuntu`
- You will be prompted to change the password immediately

**Set New Password:**
```bash
# Follow the prompts to set a secure password
```

**Update Hostname:**
```bash
# For Jetson Nano 1 (Navigation & Platform Control)
sudo hostnamectl set-hostname james-nav

# For Jetson Nano 2 (Perception & Manipulation)
sudo hostnamectl set-hostname james-perception

# Edit hosts file
sudo nano /etc/hosts
# Change 127.0.1.1 line to match new hostname
```

### System Update

```bash
# Update package lists
sudo apt update

# Upgrade all packages
sudo apt upgrade -y

# Install essential tools
sudo apt install -y build-essential git curl wget vim nano htop net-tools

# Reboot
sudo reboot
```

---

## 3. Network Configuration

### Static IP Configuration (Recommended)

Edit netplan configuration:

```bash
sudo nano /etc/netplan/50-cloud-init.yaml
```

Add the following configuration:

```yaml
network:
  version: 2
  renderer: networkd
  ethernets:
    eth0:
      dhcp4: no
      addresses:
        - 192.168.1.100/24  # For james-nav
        # - 192.168.1.101/24  # For james-perception
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 8.8.4.4]
```

Apply configuration:

```bash
sudo netplan apply
```

### WiFi Configuration (Optional)

If using WiFi adapter:

```bash
# Install network manager
sudo apt install -y network-manager

# List available networks
nmcli device wifi list

# Connect to WiFi
sudo nmcli device wifi connect "SSID" password "PASSWORD"
```

### Verify Network

```bash
# Check IP address
ip addr show

# Test internet connectivity
ping -c 4 google.com

# Check DNS resolution
nslookup google.com
```

---

## 4. SSH and Remote Access

### Enable and Configure SSH

```bash
# SSH should be enabled by default, verify:
sudo systemctl status ssh

# If not enabled:
sudo systemctl enable ssh
sudo systemctl start ssh
```

### Configure SSH for Key-Based Authentication

**On your development PC:**

```bash
# Generate SSH key (if you don't have one)
ssh-keygen -t ed25519 -C "your_email@example.com"

# Copy public key to Jetson
ssh-copy-id ubuntu@192.168.1.100  # james-nav
ssh-copy-id ubuntu@192.168.1.101  # james-perception
```

**On Jetson Nano:**

```bash
# Disable password authentication (optional, more secure)
sudo nano /etc/ssh/sshd_config

# Set the following:
# PasswordAuthentication no
# PubkeyAuthentication yes

# Restart SSH
sudo systemctl restart ssh
```

### Set Up Hostname Resolution

**On your development PC** (add to `/etc/hosts` or `C:\Windows\System32\drivers\etc\hosts`):

```
192.168.1.100  james-nav
192.168.1.101  james-perception
```

Now you can SSH using:
```bash
ssh ubuntu@james-nav
ssh ubuntu@james-perception
```

### Install and Configure tmux (Recommended)

```bash
# Install tmux for persistent sessions
sudo apt install -y tmux

# Create tmux configuration
cat > ~/.tmux.conf << 'EOF'
# Enable mouse support
set -g mouse on

# Set prefix to Ctrl-a
unbind C-b
set -g prefix C-a

# Split panes using | and -
bind | split-window -h
bind - split-window -v

# Reload config
bind r source-file ~/.tmux.conf
EOF
```

---

## 5. ROS2 Jazzy Installation

### Add ROS2 Repository

```bash
# Set locale
sudo apt update && sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS2 GPG key
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install -y curl
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add repository to sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
```

### Install ROS2 Jazzy

```bash
# Update package lists
sudo apt update

# Install ROS2 Jazzy Desktop (includes RViz, demos, tutorials)
sudo apt install -y ros-jazzy-desktop

# Install development tools
sudo apt install -y ros-dev-tools

# Install colcon build tool
sudo apt install -y python3-colcon-common-extensions
```

### Configure ROS2 Environment

```bash
# Add to .bashrc
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
echo "export ROS_LOCALHOST_ONLY=0" >> ~/.bashrc

# For james-nav
echo "export ROS_NAMESPACE=james_nav" >> ~/.bashrc

# For james-perception
# echo "export ROS_NAMESPACE=james_perception" >> ~/.bashrc

# Source the file
source ~/.bashrc
```

### Verify ROS2 Installation

```bash
# Check ROS2 version
ros2 --version

# Run demo (in separate terminals)
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener
```

---

## 6. RealSense SDK Installation

### Install Dependencies

```bash
sudo apt install -y \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev
```

### Install librealsense2

```bash
# Add Intel RealSense repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

# Update and install
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev
```

### Install ROS2 RealSense Wrapper

```bash
sudo apt install -y \
    ros-jazzy-realsense2-camera \
    ros-jazzy-realsense2-description
```

### Configure USB Permissions

```bash
# Add user to video group
sudo usermod -a -G video $USER

# Apply udev rules
sudo cp /usr/local/lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger

# Reboot to apply changes
sudo reboot
```

### Verify RealSense Installation

```bash
# List connected RealSense devices
realsense-viewer

# Test with ROS2
ros2 launch realsense2_camera rs_launch.py
```

---

## 7. CUDA and TensorRT Setup

### Install NVIDIA JetPack Components

```bash
# Add NVIDIA repository
sudo apt install -y nvidia-jetpack
```

### Verify CUDA Installation

```bash
# Check CUDA version
nvcc --version

# Check GPU status
nvidia-smi  # May not work on Jetson Nano

# Alternative GPU check
sudo /usr/bin/tegrastats
```

### Install PyTorch for Jetson

```bash
# Install dependencies
sudo apt install -y python3-pip libopenblas-base libopenmpi-dev

# Install PyTorch (pre-built for Jetson)
wget https://nvidia.box.com/shared/static/mp164asf3sceb570wvjsrezk1p4ftj8t.whl -O torch-2.0.0-cp310-cp310-linux_aarch64.whl
pip3 install torch-2.0.0-cp310-cp310-linux_aarch64.whl

# Install torchvision
sudo apt install -y libjpeg-dev zlib1g-dev
git clone --branch v0.15.0 https://github.com/pytorch/vision torchvision
cd torchvision
python3 setup.py install --user
cd ..
rm -rf torchvision
```

### Install TensorRT (Included in JetPack)

```bash
# Verify TensorRT installation
dpkg -l | grep TensorRT

# Install Python bindings
pip3 install pycuda
```

---

## 8. Python Environment Setup

### Install Python Development Tools

```bash
# Install pip and venv
sudo apt install -y python3-pip python3-venv python3-dev

# Upgrade pip
pip3 install --upgrade pip
```

### Create Virtual Environment (Optional but Recommended)

```bash
# Create virtual environment
python3 -m venv ~/james_env

# Activate virtual environment
source ~/james_env/bin/activate

# Add to .bashrc for auto-activation
echo "source ~/james_env/bin/activate" >> ~/.bashrc
```

### Install Python Dependencies

```bash
# Navigate to project directory
cd ~/james-home-robot

# Install requirements
pip3 install -r requirements.txt

# Install additional ROS2 Python tools
pip3 install rosdep vcstool
```

### Initialize rosdep

```bash
# Initialize rosdep
sudo rosdep init
rosdep update
```

---

## 9. MCP Server Installation

### Install UV Package Manager

```bash
# Install uv (Python package manager)
curl -LsSf https://astral.sh/uv/install.sh | sh

# Add to PATH
echo 'export PATH="$HOME/.cargo/bin:$PATH"' >> ~/.bashrc
source ~/.bashrc

# Verify installation
uv --version
```

### Configure MCP Servers

Create workspace MCP configuration:

```bash
# Create Kiro settings directory
mkdir -p ~/james-useful-home-robot/.kiro/settings

# Create MCP configuration
cat > ~/james-useful-home-robot/.kiro/settings/mcp.json << 'EOF'
{
  "mcpServers": {
    "filesystem": {
      "command": "uvx",
      "args": ["mcp-server-filesystem", "/home/ubuntu/james-useful-home-robot"],
      "env": {
        "FASTMCP_LOG_LEVEL": "ERROR"
      },
      "disabled": false,
      "autoApprove": ["read_file", "list_directory"]
    },
    "git": {
      "command": "uvx",
      "args": ["mcp-server-git"],
      "env": {
        "FASTMCP_LOG_LEVEL": "ERROR"
      },
      "disabled": false,
      "autoApprove": ["git_status", "git_diff"]
    },
    "ros2-tools": {
      "command": "uvx",
      "args": ["mcp-server-ros2"],
      "env": {
        "FASTMCP_LOG_LEVEL": "ERROR",
        "ROS_DOMAIN_ID": "42"
      },
      "disabled": false,
      "autoApprove": ["ros2_node_list", "ros2_topic_list"]
    }
  }
}
EOF
```

### Install Common MCP Servers

```bash
# The MCP servers will be automatically downloaded and run by uvx
# when first accessed. No manual installation needed.

# Test MCP server availability
uvx mcp-server-filesystem --help
uvx mcp-server-git --help
```

### Create Custom ROS2 MCP Server (Optional)

```bash
# Create custom MCP server for ROS2 operations
mkdir -p ~/mcp-servers/ros2
cd ~/mcp-servers/ros2

# Create server script
cat > server.py << 'EOF'
#!/usr/bin/env python3
"""MCP Server for ROS2 operations"""

import subprocess
import json
from typing import Any

def run_ros2_command(args: list[str]) -> dict[str, Any]:
    """Execute ROS2 command and return output"""
    try:
        result = subprocess.run(
            ["ros2"] + args,
            capture_output=True,
            text=True,
            timeout=10
        )
        return {
            "success": True,
            "stdout": result.stdout,
            "stderr": result.stderr,
            "returncode": result.returncode
        }
    except Exception as e:
        return {
            "success": False,
            "error": str(e)
        }

# MCP server implementation would go here
# This is a simplified example
EOF

chmod +x server.py
```

---

## 10. ESP32 Development Environment

### Install PlatformIO

```bash
# Install PlatformIO Core
pip3 install platformio

# Verify installation
pio --version

# Update PlatformIO
pio upgrade
```

### Install Arduino CLI (Alternative)

```bash
# Download and install Arduino CLI
curl -fsSL https://raw.githubusercontent.com/arduino/arduino-cli/master/install.sh | sh

# Move to PATH
sudo mv bin/arduino-cli /usr/local/bin/

# Initialize configuration
arduino-cli config init

# Add ESP32 board support
arduino-cli config add board_manager.additional_urls https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

# Update index
arduino-cli core update-index

# Install ESP32 platform
arduino-cli core install esp32:esp32
```

### Configure USB Permissions for ESP32

```bash
# Add user to dialout group
sudo usermod -a -G dialout $USER

# Create udev rule for ESP32
sudo bash -c 'cat > /etc/udev/rules.d/99-esp32.rules << EOF
SUBSYSTEMS=="usb", ATTRS{idVendor}=="303a", ATTRS{idProduct}=="1001", MODE="0666"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE="0666"
EOF'

# Reload udev rules
sudo udevadm control --reload-rules && sudo udevadm trigger

# Reboot to apply group changes
sudo reboot
```

### Test ESP32 Connection

```bash
# List connected devices
pio device list

# Or with Arduino CLI
arduino-cli board list
```

---

## 11. System Optimization

### Increase Swap Space

```bash
# Disable existing swap
sudo swapoff /swapfile

# Create larger swap file (8GB recommended)
sudo dd if=/dev/zero of=/swapfile bs=1M count=8192
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

# Make permanent
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab
```

### Enable Maximum Performance Mode

```bash
# Set to maximum performance
sudo nvpmodel -m 0

# Set maximum clock speeds
sudo jetson_clocks

# Make persistent (add to /etc/rc.local)
sudo bash -c 'cat > /etc/rc.local << EOF
#!/bin/bash
nvpmodel -m 0
jetson_clocks
exit 0
EOF'

sudo chmod +x /etc/rc.local
```

### Disable GUI (Save Resources)

```bash
# If you don't need desktop environment
sudo systemctl set-default multi-user.target

# To re-enable GUI
# sudo systemctl set-default graphical.target
```

### Configure Automatic Fan Control

```bash
# Install fan control utility
sudo apt install -y python3-smbus

# Create fan control script
sudo bash -c 'cat > /usr/local/bin/jetson-fan-control.py << EOF
#!/usr/bin/env python3
import time
import subprocess

def get_temp():
    result = subprocess.run(["cat", "/sys/devices/virtual/thermal/thermal_zone0/temp"], 
                          capture_output=True, text=True)
    return int(result.stdout.strip()) / 1000

def set_fan_speed(speed):
    # Speed: 0-255
    subprocess.run(["echo", str(speed), ">", "/sys/devices/pwm-fan/target_pwm"])

while True:
    temp = get_temp()
    if temp > 60:
        set_fan_speed(255)
    elif temp > 50:
        set_fan_speed(180)
    elif temp > 40:
        set_fan_speed(120)
    else:
        set_fan_speed(80)
    time.sleep(5)
EOF'

sudo chmod +x /usr/local/bin/jetson-fan-control.py
```

---

## 12. Verification and Testing

### System Health Check

```bash
# Check CPU usage
htop

# Check memory usage
free -h

# Check disk usage
df -h

# Check temperature
cat /sys/devices/virtual/thermal/thermal_zone0/temp

# Check GPU usage
sudo /usr/bin/tegrastats
```

### ROS2 Verification

```bash
# Source ROS2
source /opt/ros/jazzy/setup.bash

# Check ROS2 environment
printenv | grep ROS

# List available packages
ros2 pkg list

# Test communication between Jetsons
# On james-nav:
ros2 topic pub /test std_msgs/msg/String "data: Hello from nav"

# On james-perception:
ros2 topic echo /test
```

### RealSense Verification

```bash
# List devices
rs-enumerate-devices

# Test D435
realsense-viewer

# Test with ROS2
ros2 launch realsense2_camera rs_launch.py

# Check topics
ros2 topic list | grep camera
```

### Network Verification

```bash
# Test SSH from development PC
ssh ubuntu@james-nav
ssh ubuntu@james-perception

# Test inter-Jetson communication
# From james-nav:
ping james-perception

# From james-perception:
ping james-nav
```

### Create System Info Script

```bash
cat > ~/system-info.sh << 'EOF'
#!/bin/bash
echo "=== System Information ==="
echo "Hostname: $(hostname)"
echo "IP Address: $(hostname -I)"
echo "Ubuntu Version: $(lsb_release -d | cut -f2)"
echo "Kernel: $(uname -r)"
echo ""
echo "=== ROS2 ==="
echo "ROS_DISTRO: $ROS_DISTRO"
ros2 --version
echo ""
echo "=== CUDA ==="
nvcc --version 2>/dev/null || echo "CUDA not found"
echo ""
echo "=== Python ==="
python3 --version
pip3 --version
echo ""
echo "=== RealSense ==="
rs-enumerate-devices 2>/dev/null || echo "No RealSense devices connected"
echo ""
echo "=== System Resources ==="
echo "Temperature: $(cat /sys/devices/virtual/thermal/thermal_zone0/temp | awk '{print $1/1000}')°C"
free -h
df -h /
EOF

chmod +x ~/system-info.sh
```

Run verification:
```bash
~/system-info.sh
```

---

## Troubleshooting

### Common Issues

**Issue: SSH connection refused**
```bash
sudo systemctl status ssh
sudo systemctl restart ssh
```

**Issue: RealSense not detected**
```bash
# Check USB connection
lsusb | grep Intel

# Reinstall udev rules
sudo cp /usr/local/lib/udev/rules.d/99-realsense-libusb.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
```

**Issue: ROS2 nodes can't communicate**
```bash
# Check ROS_DOMAIN_ID matches on both Jetsons
echo $ROS_DOMAIN_ID

# Check firewall
sudo ufw status
sudo ufw allow from 192.168.1.0/24
```

**Issue: Out of memory**
```bash
# Check swap
free -h

# Increase swap (see System Optimization section)
```

**Issue: Jetson overheating**
```bash
# Check temperature
cat /sys/devices/virtual/thermal/thermal_zone0/temp

# Ensure fan is working
# Add heatsink if not present
# Improve ventilation
```

---

## Next Steps

After completing this hardware setup:

1. Clone the James repository: `git clone https://github.com/bano99/james-useful-home-robot.git ~/james-useful-home-robot`
2. Build the ROS2 workspace: `cd ~/james-useful-home-robot/ros2_ws && colcon build`
3. Flash ESP32 firmware (see platform/controller and platform/remote READMEs)
4. Proceed with software configuration and calibration
5. Run integration tests

For detailed software setup, refer to the main README.md.
