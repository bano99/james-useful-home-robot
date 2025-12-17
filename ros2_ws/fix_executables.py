#!/usr/bin/env python3
"""
Fix script for james_manipulation executable issues
"""

import os
import subprocess
import sys

def run_command(cmd, cwd=None):
    """Run command and return success status"""
    try:
        result = subprocess.run(cmd, shell=True, cwd=cwd, capture_output=True, text=True)
        print(f"Command: {cmd}")
        if result.stdout:
            print(f"Output: {result.stdout}")
        if result.stderr:
            print(f"Error: {result.stderr}")
        return result.returncode == 0
    except Exception as e:
        print(f"Exception running {cmd}: {e}")
        return False

def main():
    print("🔧 Fixing james_manipulation executable issues...")
    print("=" * 60)
    
    # Step 1: Clean build
    print("1. Cleaning previous build...")
    if os.path.exists("build"):
        run_command("rmdir /s /q build")
    if os.path.exists("install"):
        run_command("rmdir /s /q install")
    if os.path.exists("log"):
        run_command("rmdir /s /q log")
    
    # Step 2: Install dependencies
    print("\n2. Installing Python dependencies...")
    run_command("pip install pyserial")
    
    # Step 3: Build package
    print("\n3. Building package...")
    success = run_command("colcon build --packages-select james_manipulation --cmake-args -DCMAKE_BUILD_TYPE=Release")
    
    if not success:
        print("❌ Build failed! Check error messages above.")
        return 1
    
    # Step 4: Source workspace
    print("\n4. Sourcing workspace...")
    print("💡 Run this command manually: call install\\setup.bat")
    
    # Step 5: Test executables
    print("\n5. Testing executables...")
    print("💡 After sourcing, test with:")
    print("   ros2 run james_manipulation platform_serial_bridge --help")
    
    print("\n✅ Fix script completed!")
    print("📋 Next steps:")
    print("   1. Run: call install\\setup.bat")
    print("   2. Test: ros2 run james_manipulation platform_serial_bridge --help")
    
    return 0

if __name__ == "__main__":
    sys.exit(main())