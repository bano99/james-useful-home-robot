# Jetson Nano Won't Boot - Quick Fix

## Problem: Stuck at NVIDIA Logo

If your Jetson Nano shows only the NVIDIA logo and doesn't boot, you likely flashed the wrong image.

### What Went Wrong

- ❌ **Wrong**: Ubuntu Raspberry Pi image (`ubuntu-24.04-preinstalled-server-arm64+raspi.img`)
- ✅ **Correct**: NVIDIA JetPack image for Jetson Nano

Jetson Nano uses a different ARM architecture (ARM Cortex-A57) than Raspberry Pi and requires NVIDIA's custom kernel and drivers.

## Quick Solution

### Step 1: Download Correct Image

**Option A: Official NVIDIA Image (Recommended)**

1. Go to: https://developer.nvidia.com/embedded/jetpack
2. Scroll to "Jetson Nano Developer Kit"
3. Download: **JetPack 4.6.4 SD Card Image**
4. File will be named something like: `jetson-nano-jp464-sd-card-image.zip`
5. Extract the .zip to get the .img file

**Option B: Direct Download Link**

```bash
# Download JetPack 4.6.4 (Ubuntu 18.04 based)
wget https://developer.nvidia.com/embedded/l4t/r32_release_v7.4/jp_4.6.4_b31_sd_card/jetson-nano/jetson-nano-jp464-sd-card-image.zip

# Extract
unzip jetson-nano-jp464-sd-card-image.zip
```

### Step 2: Flash Correct Image

**Using Balena Etcher (Easiest):**

1. Download Etcher: https://www.balena.io/etcher/
2. Insert your microSD card
3. Open Etcher
4. Select the JetPack .img file (not the Raspberry Pi image!)
5. Select your SD card
6. Click "Flash!"
7. Wait for completion and verification

**Using Raspberry Pi Imager (Alternative):**

1. Download: https://www.raspberrypi.com/software/
2. Open Raspberry Pi Imager
3. Click "Choose OS" → "Use custom"
4. Select the JetPack .img file
5. Choose your SD card
6. Click "Write"

### Step 3: First Boot

1. Insert the newly flashed SD card into Jetson Nano
2. Connect:
   - HDMI monitor
   - USB keyboard and mouse
   - Ethernet cable (recommended)
   - 5V 4A power supply (barrel jack)
3. Power on
4. Wait 5-10 minutes for first boot
5. Follow the on-screen setup wizard

## What to Expect

### First Boot Timeline:
- **0-2 min**: NVIDIA logo, loading kernel
- **2-5 min**: Ubuntu boot messages
- **5-10 min**: Initial setup wizard appears
- **10-20 min**: System configuration and installation

### Setup Wizard Steps:
1. Accept license
2. Select language
3. Select keyboard layout
4. Select timezone
5. Create user account (username: `james` recommended)
6. Set password
7. Partition configuration (use defaults)
8. Wait for installation

## Verification

After setup completes and system reboots:

```bash
# Check Ubuntu version (should be 18.04)
lsb_release -a

# Check JetPack version
sudo apt-cache show nvidia-jetpack

# Check CUDA (should be installed)
nvcc --version

# Check GPU
sudo /usr/bin/tegrastats
```

## Common Issues

### Issue: Still stuck at NVIDIA logo after flashing JetPack

**Solutions:**
1. **Check power supply**: Must be 5V 4A via barrel jack (not micro-USB!)
2. **Check jumper**: J48 jumper must be installed to enable barrel jack
3. **Try different SD card**: Use high-quality card (Samsung EVO, SanDisk Extreme)
4. **Re-flash**: Flash again, ensure verification passes
5. **Check HDMI**: Try different HDMI cable or monitor

### Issue: "No bootable device" error

**Solution:**
- SD card not properly flashed
- Re-flash with Etcher, ensure verification passes
- Try a different SD card

### Issue: Boots but no display

**Solutions:**
1. Wait longer (first boot can take 10+ minutes)
2. Try different HDMI port on monitor
3. Connect monitor before powering on
4. Check HDMI cable

### Issue: Setup wizard doesn't appear

**Solution:**
- System might be booting to console instead of GUI
- Press Ctrl+Alt+F1 to switch to GUI
- Or wait for login prompt and login via console

## Next Steps

Once Jetson Nano boots successfully:

1. Complete the setup wizard
2. Update the system:
```bash
sudo apt update
sudo apt upgrade -y
```

3. Follow the main hardware setup guide: `docs/hardware_setup.md`
4. Install ROS2 Foxy (compatible with Ubuntu 18.04)

## Need More Help?

- NVIDIA Jetson Forums: https://forums.developer.nvidia.com/c/agx-autonomous-machines/jetson-embedded-systems/jetson-nano/76
- JetPack Documentation: https://docs.nvidia.com/jetson/jetpack/
- Our troubleshooting guide: `docs/hardware_setup.md`

## Summary

**Key Points:**
- ✅ Use JetPack 4.6.4 SD Card Image from NVIDIA
- ❌ Don't use Raspberry Pi Ubuntu images
- ⚡ Use 5V 4A barrel jack power (with J48 jumper)
- ⏱️ First boot takes 10-20 minutes
- 💾 Use quality 64GB+ microSD card

Good luck! 🚀
