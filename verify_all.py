#!/usr/bin/env python3
"""Comprehensive syntax verification for all Python files."""

import py_compile
import sys
from pathlib import Path

def check_syntax(filepath):
    """Check if a Python file has valid syntax."""
    try:
        py_compile.compile(filepath, doraise=True)
        return True, None
    except py_compile.PyCompileError as e:
        return False, str(e)

def main():
    root = Path(__file__).parent

    # All Python files to check
    python_files = [
        "rhc_eco_driving.py",
        "test_eco_driving.py",
        "example_eco_driving.py",
        "simulator.py",
        "run_simulation.py",
        "analyze_results.py",
        "verify_syntax.py",
        "verify_all.py",
    ]

    print("=" * 70)
    print("Comprehensive Python Syntax Verification")
    print("=" * 70)
    print()

    all_ok = True
    results = []

    for filename in python_files:
        filepath = root / filename
        if filepath.exists():
            ok, error = check_syntax(filepath)
            results.append((filename, ok, error))
            all_ok = all_ok and ok
        else:
            results.append((filename, False, "File not found"))
            all_ok = False

    # Display results
    max_filename_len = max(len(f) for f, _, _ in results)

    for filename, ok, error in results:
        status = "✓ OK" if ok else "✗ ERROR"
        color = "\033[92m" if ok else "\033[91m"
        reset = "\033[0m"

        print(f"{filename:{max_filename_len}} : {color}{status}{reset}")
        if not ok and error:
            print(f"  → {error}")

    print()
    print("=" * 70)

    if all_ok:
        print("✓ All files have valid syntax!")
        print("=" * 70)
        return 0
    else:
        print("✗ Some files have errors.")
        print("=" * 70)
        return 1

if __name__ == "__main__":
    sys.exit(main())
