#!/usr/bin/env python3
"""
Diagnostic script to troubleshoot ROS2 executable issues
"""

import os
import sys
import subprocess

def run_command(cmd):
    """Run a command and return output"""
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True)
        return result.returncode, result.stdout, result.stderr
    except Exception as e:
        return -1, "", str(e)

def check_package_built():
    """Check if package was built successfully"""
    print("🔍 Checking if package was built...")
    
    # Check for install directory
    if os.path.exists("install/james_manipulation"):
        print("✅ install/james_manipulation directory exists")
        
        # Check for setup files
        setup_files = [
            "install/setup.bat",
            "install/setup.ps1", 
            "install/local_setup.bat"
        ]
        
        for setup_file in setup_files:
            if os.path.exists(setup_file):
                print(f"✅ {setup_file} exists")
            else:
                print(f"❌ {setup_file} missing")
        
        # Check for package files
        pkg_dir = "install/james_manipulation"
        if os.path.exists(f"{pkg_dir}/lib/james_manipulation"):
            print("✅ Package executables directory exists")
            
            # List executables
            exec_dir = f"{pkg_dir}/lib/james_manipulation"
            if os.path.exists(exec_dir):
                executables = os.listdir(exec_dir)
                print(f"📁 Executables found: {executables}")
            else:
                print("❌ No executables directory")
        else:
            print("❌ Package executables directory missing")
            
    else:
        print("❌ install/james_manipulation directory missing - package not built")
        return False
    
    return True

def check_workspace_sourced():
    """Check if workspace is properly sourced"""
    print("\n🔍 Checking workspace sourcing...")
    
    # Check ROS2 environment
    returncode, stdout, stderr = run_command("ros2 pkg list")
    if returncode == 0:
        if "james_manipulation" in stdout:
            print("✅ james_manipulation package found in ROS2 package list")
        else:
            print("❌ james_manipulation package NOT found in ROS2 package list")
            print("💡 Try: call install\\setup.bat")
            return False
    else:
        print("❌ ROS2 not available or workspace not sourced")
        print(f"Error: {stderr}")
        return False
    
    return True

def check_executables():
    """Check if executables are available"""
    print("\n🔍 Checking executables...")
    
    executables = [
        "platform_serial_bridge",
        "arm_cartesian_controller", 
        "teensy_serial_bridge"
    ]
    
    all_good = True
    for exe in executables:
        returncode, stdout, stderr = run_command(f"ros2 run james_manipulation {exe} --help")
        if returncode == 0:
            print(f"✅ {exe}: Available")
        else:
            print(f"❌ {exe}: Not available")
            print(f"   Error: {stderr.strip()}")
            all_good = False
    
    return all_good

def check_python_imports():
    """Check if Python modules can be imported"""
    print("\n🔍 Checking Python imports...")
    
    modules = [
        "rclpy",
        "std_msgs.msg", 
        "sensor_msgs.msg",
        "geometry_msgs.msg",
        "serial"
    ]
    
    all_good = True
    for module in modules:
        try:
            __import__(module)
            print(f"✅ {module}: Available")
        except ImportError as e:
            print(f"❌ {module}: Missing - {e}")
            if module == "serial":
                print("   💡 Install with: pip install pyserial")
            all_good = False
    
    return all_good

def main():
    """Main diagnostic function"""
    print("🔧 ROS2 james_manipulation Package Diagnostics")
    print("=" * 60)
    
    # Change to ros2_ws directory if not already there
    if not os.path.exists("src/james_manipulation"):
        if os.path.exists("ros2_ws/src/james_manipulation"):
            os.chdir("ros2_ws")
            print("📁 Changed to ros2_ws directory")
        else:
            print("❌ Cannot find james_manipulation package")
            return 1
    
    # Run checks
    checks = [
        ("Package Built", check_package_built),
        ("Python Imports", check_python_imports),
        ("Workspace Sourced", check_workspace_sourced),
        ("Executables Available", check_executables)
    ]
    
    results = []
    for check_name, check_func in checks:
        try:
            result = check_func()
            results.append((check_name, result))
        except Exception as e:
            print(f"❌ {check_name}: Exception - {e}")
            results.append((check_name, False))
    
    # Summary
    print("\n" + "=" * 60)
    print("📊 SUMMARY:")
    all_passed = True
    for check_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"   {status}: {check_name}")
        if not result:
            all_passed = False
    
    if all_passed:
        print("\n🎉 All checks passed! Package should work correctly.")
    else:
        print("\n🔧 Some checks failed. Follow the suggestions above to fix issues.")
        print("\n💡 Common solutions:")
        print("   1. Rebuild: colcon build --packages-select james_manipulation")
        print("   2. Source workspace: call install\\setup.bat")
        print("   3. Install missing Python packages: pip install pyserial")
    
    return 0 if all_passed else 1

if __name__ == "__main__":
    sys.exit(main())