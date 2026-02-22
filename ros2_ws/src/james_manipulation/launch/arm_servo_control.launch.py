import sys

def generate_launch_description():
    print("\n" + "="*80)
    print("  ERROR: WRONG LAUNCH FILE!")
    print("="*80)
    print("\n  This launch file (arm_servo_control.launch.py) is DEPRECATED.")
    print("\n  Please use the correct launch file instead:")
    print("\n    ros2 launch james_manipulation arm_cartesian_control.launch.py")
    print("\n" + "="*80 + "\n")
    
    sys.exit(1)
