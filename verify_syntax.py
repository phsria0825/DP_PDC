#!/usr/bin/env python3
"""Verify Python syntax without running the code."""

import py_compile
import sys
from pathlib import Path

def check_syntax(filepath):
    """Check if a Python file has valid syntax."""
    try:
        py_compile.compile(filepath, doraise=True)
        print(f"✓ {filepath.name}: Syntax OK")
        return True
    except py_compile.PyCompileError as e:
        print(f"✗ {filepath.name}: Syntax Error")
        print(f"  {e}")
        return False

def main():
    root = Path(__file__).parent
    python_files = [
        root / "rhc_eco_driving.py",
        root / "test_eco_driving.py",
        root / "example_eco_driving.py",
    ]

    print("Checking Python syntax...")
    print("=" * 60)

    all_ok = True
    for filepath in python_files:
        if filepath.exists():
            ok = check_syntax(filepath)
            all_ok = all_ok and ok
        else:
            print(f"✗ {filepath.name}: File not found")
            all_ok = False

    print("=" * 60)
    if all_ok:
        print("All files have valid syntax!")
        return 0
    else:
        print("Some files have errors.")
        return 1

if __name__ == "__main__":
    sys.exit(main())
