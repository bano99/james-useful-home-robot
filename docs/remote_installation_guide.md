# Remote Installation Guide for Jetson Nano

This guide explains how to remotely install ROS2 and required packages on your Jetson Nano from your Windows PC.

## Prerequisites

- Jetson Nano with Ubuntu 20.04 installed and booted
- Jetson Nano connected to same network as your Windows PC
- SSH access to Jetson Nano

## Step 1: Set Up SSH Access

### On Jetson Nano (with monitor/keyboard):

```bash
# Install SSH server (usually pre-installed)
sudo apt update
sudo apt install -y openssh-server

# Enable and start SSH
sudo systemctl enable ssh
sudo systemctl start ssh

# Check SSH status
sudo systemctl status ssh

# Find your IP address
ip addr show
# Look for "inet" under eth0 or wlan0
# Example: inet 192.168.1.100/24
```

### On Windows PC:

**Option A: Using PowerShell (Built-in)**

```powershell
# Connect to Jetson Nano
ssh username@192.168.1.100
# Replace 'username' with your Jetson username
# Replace '192.168.1.100' with your Jetson IP address

# Enter password when prompted
```

**Option B: Using PuTTY (GUI)**

1. Download PuTTY: https://www.putty.org/
2. Open PuTTY
3. Enter Jetson IP address in "Host Name"
4. Port: 22
5. Connection type: SSH
6. Click "Open"
7. Enter username and password

## Step 2: Transfer Installation Script

### Method 1: Using Git (Recommended)

On Jetson Nano (via SSH):

```bash
# Clone the repository
cd ~
git clone https://github.com/bano99/james-useful-home-robot.git
cd james-useful-home-robot

# Make script executable
chmod +x scripts/jetson-ros2-install.sh
```

### Method 2: Using SCP (Secure Copy)

On Windows PC (PowerShell):

```powershell
# Navigate to your project directory
cd C:\Users\denis\Documents\James_Useful_Home_Robot

# Copy script to Jetson
scp scripts/jetson-ros2-install.sh username@192.168.1.100:~/
```

On Jetson Nano (via SSH):

```bash
# Make script executable
chmod +x ~/jetson-ros2-install.sh
```

### Method 3: Manual Copy-Paste

1. Open the script file on Windows: `scripts/jetson-ros2-install.sh`
2. Copy all contents
3. On Jetson Nano (via SSH):
```bash
nano ~/jetson-ros2-install.sh
# Paste the contents
# Press Ctrl+X, then Y, then Enter to save

chmod +x ~/jetson-ros2-install.sh
```

## Step 3: Run Installation Script

On Jetson Nano (via SSH):

```bash
# Run the installation script
./jetson-ros2-install.sh

# Or if you cloned the repo:
cd ~/james-useful-home-robot
./scripts/jetson-ros2-install.sh
```

The script will:
1. Check Ubuntu version
2. Set up locale
3. Add ROS2 repository
4. Install ROS2 Humble Desktop (~10-15 minutes)
5. Install development tools
6. Initialize rosdep
7. Install RTAB-Map for SLAM
8. Install Nav2 for navigation
9. Install MoveIt2 for manipulation
10. Install RealSense packages
11. Configure environment

**Note**: The installation will take 15-30 minutes depending on internet speed.

## Step 4: Verify Installation

After installation completes:

```bash
# Source ROS2 environment
source ~/.bashrc

# Check ROS2 version
ros2 --version
# Should show: ros2 cli version: 0.18.x

# List available packages
ros2 pkg list | head -20

# Test with demo (open two SSH sessions)
# Terminal 1:
ros2 run demo_nodes_cpp talker

# Terminal 2:
ros2 run demo_nodes_cpp listener
```

If you see messages being sent and received, ROS2 is working correctly!

## Step 5: Install RealSense SDK

The RealSense ROS2 wrapper is installed, but you also need the SDK:

```bash
# Add Intel RealSense repository
sudo mkdir -p /etc/apt/keyrings
curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null

echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo $(lsb_release -cs) main" | \
sudo tee /etc/apt/sources.list.d/librealsense.list

# Install SDK
sudo apt update
sudo apt install -y librealsense2-dkms librealsense2-utils librealsense2-dev

# Test (with camera connected)
realsense-viewer
```

## Troubleshooting

### SSH Connection Refused

```bash
# On Jetson Nano
sudo systemctl status ssh
sudo systemctl restart ssh

# Check firewall
sudo ufw status
sudo ufw allow 22/tcp
```

### Installation Script Fails

```bash
# Check internet connection
ping -c 4 google.com

# Check disk space
df -h
# Need at least 5GB free

# Check for package conflicts
sudo apt update
sudo apt upgrade -y
```

### ROS2 Command Not Found

```bash
# Source ROS2 manually
source /opt/ros/humble/setup.bash

# Check if installed
ls /opt/ros/humble

# If not installed, run script again
./jetson-ros2-install.sh
```

## Alternative: Manual Step-by-Step Installation

If the script doesn't work, you can follow the manual installation steps in `docs/hardware_setup.md`.

## Using tmux for Persistent Sessions

To keep your installation running even if SSH disconnects:

```bash
# Install tmux
sudo apt install -y tmux

# Start tmux session
tmux new -s install

# Run installation
./jetson-ros2-install.sh

# Detach from session: Press Ctrl+B, then D
# Reattach later: tmux attach -t install
```

## Next Steps

After successful installation:

1. **Install Python dependencies**:
```bash
cd ~/james-useful-home-robot
pip3 install -r requirements.txt
```

2. **Build ROS2 workspace**:
```bash
cd ~/james-useful-home-robot/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

3. **Configure network** (see `docs/hardware_setup.md` Section 3)

4. **Set up second Jetson Nano** (repeat this process)

5. **Test RealSense cameras** (see `docs/hardware_setup.md` Section 6)

## Remote Development Tips

### VS Code Remote SSH

1. Install "Remote - SSH" extension in VS Code
2. Press F1, type "Remote-SSH: Connect to Host"
3. Enter: `username@192.168.1.100`
4. Open folder: `/home/username/james-useful-home-robot`
5. Edit code directly on Jetson Nano

### File Transfer

```powershell
# Upload file to Jetson
scp local_file.txt username@192.168.1.100:~/remote_path/

# Download file from Jetson
scp username@192.168.1.100:~/remote_file.txt C:\local\path\

# Upload directory
scp -r local_dir username@192.168.1.100:~/remote_path/
```

### Port Forwarding for Web Dashboard

```powershell
# Forward Jetson port 8080 to local port 8080
ssh -L 8080:localhost:8080 username@192.168.1.100

# Access dashboard at: http://localhost:8080
```

## Security Best Practices

1. **Change default password**:
```bash
passwd
```

2. **Set up SSH keys** (no password needed):
```powershell
# On Windows
ssh-keygen -t ed25519
ssh-copy-id username@192.168.1.100
```

3. **Disable password authentication** (after SSH keys work):
```bash
sudo nano /etc/ssh/sshd_config
# Set: PasswordAuthentication no
sudo systemctl restart ssh
```

4. **Set up firewall**:
```bash
sudo ufw enable
sudo ufw allow 22/tcp  # SSH
sudo ufw allow 8080/tcp  # Web dashboard
```

## Summary

You now have:
- ✅ SSH access to Jetson Nano
- ✅ Automated installation script
- ✅ ROS2 Humble installed
- ✅ All required packages installed
- ✅ Remote development setup

You can now proceed with the rest of the hardware setup and start developing James! 🤖
