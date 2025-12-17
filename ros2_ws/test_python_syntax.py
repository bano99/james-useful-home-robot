#!/usr/bin/env python3
"""
Test script to check Python syntax of james_manipulation nodes
"""

import ast
import sys
import os

def check_syntax(file_path):
    """Check if a Python file has valid syntax"""
    try:
        with open(file_path, 'r') as f:
            source = f.read()
        
        # Try to parse the AST
        ast.parse(source)
        print("✅ " + file_path + ": Syntax OK")
        return True
        
    except SyntaxError as e:
        print("❌ " + file_path + ": Syntax Error at line " + str(e.lineno) + ": " + e.msg)
        return False
    except Exception as e:
        print("❌ " + file_path + ": Error: " + str(e))
        return False

def main():
    """Test all Python files in james_manipulation package"""
    base_path = "src/james_manipulation/james_manipulation"
    
    files_to_check = [
        base_path + "/platform_serial_bridge.py",
        base_path + "/arm_cartesian_controller.py", 
        base_path + "/teensy_serial_bridge.py"
    ]
    
    print("Checking Python syntax for james_manipulation nodes...")
    print("=" * 60)
    
    all_good = True
    for file_path in files_to_check:
        if os.path.exists(file_path):
            if not check_syntax(file_path):
                all_good = False
        else:
            print("❌ " + file_path + ": File not found")
            all_good = False
    
    print("=" * 60)
    if all_good:
        print("✅ All files have valid Python syntax!")
    else:
        print("❌ Some files have syntax errors - fix them before building")
    
    return 0 if all_good else 1

if __name__ == "__main__":
    sys.exit(main())